import math
import threading
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node


MID360_ROBOT_ODOM_TOPIC = "/lio/robo/odom"
MID360_LIDAR_ODOM_TOPIC = "/lio/odom"


ENTRANCE_X = 2.92
ENTRANCE_Y = 0.92
PRE_ENTRANCE_X = 1.80
PRE_ENTRANCE_Y = 0.957
WEAPON_TARGETS = {
    1: (-0.32, 4.12),
    2: (-0.12, 4.12),
    3: (0.074, 4.12),
    4: (0.275, 4.12),
    5: (0.483, 4.12),
    6: (0.681, 4.12),
}
WEAPON_RETREAT_STOP_Y = 4.00


IMU_TO_LIDAR_X = -0.011  # m, /lio/odom(imu) 到 lidar 的x方向外参
IMU_TO_LIDAR_Y = -0.02329  # m, /lio/odom(imu) 到 lidar 的y方向外参
IMU_TO_LIDAR_Z = 0.04412  # m, /lio/odom(imu) 到 lidar 的z方向外参
T_IMU_TO_LIDAR = np.array([
    [1.0, 0.0, 0.0, IMU_TO_LIDAR_X],
    [0.0, 1.0, 0.0, IMU_TO_LIDAR_Y],
    [0.0, 0.0, 1.0, IMU_TO_LIDAR_Z],
    [0.0, 0.0, 0.0, 1.0],
], dtype=float)


LIDAR_TO_BASE_X = -0.191  # m, 雷达到机器人中心的x方向安装偏移
LIDAR_TO_BASE_Y = -0.202  # m, 雷达到机器人中心的y方向安装偏移
LIDAR_TO_BASE_Z = 0.0  # m, 雷达到机器人中心的z方向安装偏移
T_LIDAR_TO_BASE = np.array([
    [1.0, 0.0, 0.0, LIDAR_TO_BASE_X],
    [0.0, 1.0, 0.0, LIDAR_TO_BASE_Y],
    [0.0, 0.0, 1.0, LIDAR_TO_BASE_Z],
    [0.0, 0.0, 0.0, 1.0],
], dtype=float)


LIDAR_TO_WEAPON_X = 0.302  # m, 雷达到武器/夹爪的x方向安装偏移
LIDAR_TO_WEAPON_Y = -0.545  # m, 雷达到武器/夹爪的y方向安装偏移
LIDAR_TO_WEAPON_Z = 0.0  # m, 雷达到武器/夹爪的z方向安装偏移
T_LIDAR_TO_WEAPON = np.array([
    [1.0, 0.0, 0.0, LIDAR_TO_WEAPON_X],
    [0.0, 1.0, 0.0, LIDAR_TO_WEAPON_Y],
    [0.0, 0.0, 1.0, LIDAR_TO_WEAPON_Z],
    [0.0, 0.0, 0.0, 1.0],
], dtype=float)


def get_t_lidar_to_base():
    """
    返回 雷达->机器人中心 的固定齐次变换矩阵(4x4)。
    """
    return T_LIDAR_TO_BASE.copy()


def get_t_imu_to_lidar():
    """
    返回 /lio/odom(imu) -> lidar 的固定齐次变换矩阵(4x4)。
    """
    return T_IMU_TO_LIDAR.copy()


def get_t_lidar_to_weapon():
    """
    返回 雷达->武器/夹爪 的固定齐次变换矩阵(4x4)。
    """
    return T_LIDAR_TO_WEAPON.copy()


def quaternion_to_rotation_matrix(qx, qy, qz, qw):
    """
    将四元数转换为3x3旋转矩阵。
    """
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm < 1e-12:
        return np.eye(3, dtype=float)

    qx /= norm
    qy /= norm
    qz /= norm
    qw /= norm

    return np.array([
        [1 - 2 * (qy ** 2 + qz ** 2), 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy)],
        [2 * (qx * qy + qw * qz), 1 - 2 * (qx ** 2 + qz ** 2), 2 * (qy * qz - qw * qx)],
        [2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 1 - 2 * (qx ** 2 + qy ** 2)],
    ], dtype=float)


def rotation_matrix_to_yaw(rotation_matrix: np.ndarray):
    """
    从旋转矩阵中提取map平面下的yaw角，单位rad。
    """
    return math.atan2(rotation_matrix[1, 0], rotation_matrix[0, 0])


def radians_to_degrees(angle_rad):
    """
    弧度转角度。
    """
    return math.degrees(angle_rad)


def transform_to_matrix(transform_msg):
    """
    将ROS2 TransformStamped消息转换为4x4齐次变换矩阵。
    """
    tx = transform_msg.transform.translation.x
    ty = transform_msg.transform.translation.y
    tz = transform_msg.transform.translation.z

    qx = transform_msg.transform.rotation.x
    qy = transform_msg.transform.rotation.y
    qz = transform_msg.transform.rotation.z
    qw = transform_msg.transform.rotation.w

    T = np.eye(4, dtype=float)
    T[0:3, 0:3] = quaternion_to_rotation_matrix(qx, qy, qz, qw)
    T[0:3, 3] = [tx, ty, tz]
    return T


def _stamp_to_sec(stamp_msg):
    """
    将ROS2时间戳转换为秒。
    """
    return float(stamp_msg.sec) + float(stamp_msg.nanosec) * 1e-9


def _stamp_to_ros_time(stamp_msg):
    """
    将ROS2时间戳消息转换为rclpy Time对象。
    """
    return rclpy.time.Time.from_msg(stamp_msg)


def _lidar_pose_from_matrices(T_map_odom: np.ndarray, T_odom_base: np.ndarray, stamp_sec=None, age_sec=None):
    """
    由 map->odom 与 odom->base 两段矩阵拼出雷达在map下的位姿。

    这里保留 odin 中的命名和返回字段，后续 mid360 订阅层可以直接复用。
    """
    T_map_lidar = T_map_odom @ T_odom_base
    x = float(T_map_lidar[0, 3])
    y = float(T_map_lidar[1, 3])
    z = float(T_map_lidar[2, 3])
    yaw = math.atan2(T_map_lidar[1, 0], T_map_lidar[0, 0])

    return {
        "x": x,
        "y": y,
        "z": z,
        "yaw": yaw,
        "T_map_lidar": T_map_lidar,
        "stamp_sec": stamp_sec,
        "age_sec": age_sec,
    }


def _lidar_pose_from_matrix(T_map_lidar: np.ndarray, stamp_sec=None, age_sec=None, frame_id="world"):
    """
    由 world/map -> lidar 矩阵生成雷达位姿字典。
    """
    x = float(T_map_lidar[0, 3])
    y = float(T_map_lidar[1, 3])
    z = float(T_map_lidar[2, 3])
    yaw = math.atan2(T_map_lidar[1, 0], T_map_lidar[0, 0])

    return {
        "x": x,
        "y": y,
        "z": z,
        "yaw": yaw,
        "T_map_lidar": T_map_lidar,
        "stamp_sec": stamp_sec,
        "age_sec": age_sec,
        "frame_id": frame_id,
    }


def pose_to_matrix(pose_msg):
    """
    将ROS2 Pose消息转换为4x4齐次变换矩阵。
    """
    tx = pose_msg.position.x
    ty = pose_msg.position.y
    tz = pose_msg.position.z

    qx = pose_msg.orientation.x
    qy = pose_msg.orientation.y
    qz = pose_msg.orientation.z
    qw = pose_msg.orientation.w

    T = np.eye(4, dtype=float)
    T[0:3, 0:3] = quaternion_to_rotation_matrix(qx, qy, qz, qw)
    T[0:3, 3] = [tx, ty, tz]
    return T


def _robot_pose_from_matrix(T_map_robot: np.ndarray, stamp_sec=None, age_sec=None, frame_id="world"):
    """
    由 world/map -> robot 矩阵生成机器人位姿字典。
    """
    x = float(T_map_robot[0, 3])
    y = float(T_map_robot[1, 3])
    z = float(T_map_robot[2, 3])
    yaw = math.atan2(T_map_robot[1, 0], T_map_robot[0, 0])

    return {
        "x": x,
        "y": y,
        "z": z,
        "yaw": yaw,
        "T_map_robot": T_map_robot,
        "stamp_sec": stamp_sec,
        "age_sec": age_sec,
        "frame_id": frame_id,
    }


class TfCacheNode(Node):
    """
    mid360 relocation 位姿缓存节点。

    为了减少调用层修改，类名沿用 odin 的 TfCacheNode；但这里不订阅 /tf。
    - /lio/robo/odom: robot frame 在 world 下的位姿
    - /lio/odom: imu/lidar系统在 world 下的位姿，用于计算 lidar 位姿
    """

    def __init__(
        self,
        base_frame="robot",
        tf_topic=MID360_ROBOT_ODOM_TOPIC,
        lidar_topic=MID360_LIDAR_ODOM_TOPIC,
        update_hz=50.0,
        max_pending_age_sec=0.25,
    ):
        super().__init__("mid360_odom_cache_node")
        if update_hz <= 0.0:
            raise ValueError(f"update_hz must be > 0, got {update_hz}")

        self.base_frame = base_frame
        self.tf_topic = tf_topic
        self.topic = tf_topic
        self.robot_topic = tf_topic
        self.lidar_topic = lidar_topic
        self.period = 1.0 / max(1e-6, float(update_hz))
        self.max_pending_age_sec = float(max_pending_age_sec)

        self._lock = threading.Lock()
        self._latest_robot_odom = None
        self._latest_lidar_odom = None
        self._robot_stamp = 0.0
        self._lidar_stamp = 0.0

        self._robot_odom_sub = self.create_subscription(
            Odometry,
            self.robot_topic,
            self._robot_odom_callback,
            50,
        )
        self._lidar_odom_sub = self.create_subscription(
            Odometry,
            self.lidar_topic,
            self._lidar_odom_callback,
            50,
        )

    def _robot_odom_callback(self, msg: Odometry):
        with self._lock:
            self._latest_robot_odom = msg
            self._robot_stamp = time.time()

    def _lidar_odom_callback(self, msg: Odometry):
        with self._lock:
            self._latest_lidar_odom = msg
            self._lidar_stamp = time.time()

    def update_once(self):
        with self._lock:
            return self._latest_robot_odom is not None or self._latest_lidar_odom is not None

    def get_latest(self):
        return self.get_latest_robot()

    def get_latest_robot(self):
        with self._lock:
            return self._latest_robot_odom, self._robot_stamp

    def get_latest_lidar(self):
        with self._lock:
            return self._latest_lidar_odom, self._lidar_stamp


def spin_tf_cache(node: TfCacheNode, stop_event: threading.Event):
    """
    启动 mid360 odom 缓存节点的 spin 线程。

    函数名沿用 odin，方便 start_position_thread 一类调用层少改。
    """
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        while rclpy.ok() and not stop_event.is_set():
            try:
                executor.spin_once(timeout_sec=0.01)
            except ExternalShutdownException:
                break
            time.sleep(node.period)
    finally:
        executor.remove_node(node)
        executor.shutdown()


def tf_pair_synced(tf_cache_node: TfCacheNode):
    """
    兼容 odin 的同名接口。

    mid360 不依赖 /tf 的 map->odom 与 odom->base 配对。这里用 /lio/odom 生成
    一个等价可组合矩阵对：(I, T_map_lidar)，使 odin 风格调用能继续得到
    T_map_lidar = I @ T_map_lidar。
    """
    odom_msg, cache_stamp = tf_cache_node.get_latest_lidar()
    if odom_msg is None:
        return None, None, None, None

    T_map_imu = pose_to_matrix(odom_msg.pose.pose)
    T_map_lidar = T_map_imu @ get_t_imu_to_lidar()
    T_map_odom = np.eye(4, dtype=float)
    T_odom_base = T_map_lidar
    return T_map_odom, T_odom_base, odom_msg.header.stamp, cache_stamp


def tf_map2odom_sub(tf_cache_node: TfCacheNode):
    """
    兼容 odin 的同名接口，mid360 下返回伪 T_map_odom=I。
    """
    T_map_odom, _, _, _ = tf_pair_synced(tf_cache_node)
    return T_map_odom


def tf_odom2base_sub(tf_cache_node: TfCacheNode):
    """
    兼容 odin 的同名接口，mid360 下返回伪 T_odom_base=T_map_lidar。
    """
    _, T_odom_base, _, _ = tf_pair_synced(tf_cache_node)
    return T_odom_base


def get_robot_pose_in_map_synced(
    tf_cache_node: TfCacheNode,
    max_age_sec=0.10,
    clock=None,
):
    """
    获取机器人在 world/map 坐标系下的最新位姿。

    mid360 的 /lio/robo/odom 已经是 robot frame 在 world 下的位姿，
    因此这里不再应用 雷达->机器人中心 外参。
    """
    odom_msg, cache_stamp = tf_cache_node.get_latest_robot()
    if odom_msg is None:
        return None

    age_sec = time.time() - cache_stamp if cache_stamp else None
    if age_sec is None:
        if clock is None:
            clock = rclpy.clock.Clock()
        stamp_ros = _stamp_to_ros_time(odom_msg.header.stamp)
        now_ros = clock.now()
        age_sec = (now_ros - stamp_ros).nanoseconds * 1e-9

    if age_sec < 0.0 or age_sec > float(max_age_sec):
        return None

    T_map_robot = pose_to_matrix(odom_msg.pose.pose)
    stamp_sec = _stamp_to_sec(odom_msg.header.stamp)
    return _robot_pose_from_matrix(
        T_map_robot=T_map_robot,
        stamp_sec=stamp_sec,
        age_sec=age_sec,
        frame_id=odom_msg.header.frame_id,
    )


def get_lidar_pose_in_map_synced(
    tf_cache_node: TfCacheNode,
    max_age_sec=0.10,
    clock=None,
):
    """
    获取雷达在 world/map 坐标系下的位姿。

    /lio/odom 发布的是 imu/lidar系统在 world 下的位姿。当前外参旋转为单位阵，
    因此雷达与机器人绝对偏航角一致；严格雷达中心位置通过 T_world_imu *
    T_imu_lidar 得到。
    """
    odom_msg, cache_stamp = tf_cache_node.get_latest_lidar()
    if odom_msg is None:
        return None

    age_sec = time.time() - cache_stamp if cache_stamp else None
    if age_sec is None:
        if clock is None:
            clock = rclpy.clock.Clock()
        stamp_ros = _stamp_to_ros_time(odom_msg.header.stamp)
        now_ros = clock.now()
        age_sec = (now_ros - stamp_ros).nanoseconds * 1e-9

    if age_sec < 0.0 or age_sec > float(max_age_sec):
        return None

    T_map_imu = pose_to_matrix(odom_msg.pose.pose)
    T_map_lidar = T_map_imu @ get_t_imu_to_lidar()
    stamp_sec = _stamp_to_sec(odom_msg.header.stamp)
    return _lidar_pose_from_matrix(
        T_map_lidar=T_map_lidar,
        stamp_sec=stamp_sec,
        age_sec=age_sec,
        frame_id=odom_msg.header.frame_id,
    )


def get_lidar_pose_in_map_latest(tf_cache_node: TfCacheNode, max_cache_age_sec=1.0):
    """
    使用缓存中的最新 /lio/odom，计算雷达在 world/map 下的位姿。
    """
    return get_lidar_pose_in_map_synced(
        tf_cache_node=tf_cache_node,
        max_age_sec=max_cache_age_sec,
        clock=tf_cache_node.get_clock(),
    )


def get_weapon_pose_in_map_synced(
    tf_cache_node: TfCacheNode,
    max_age_sec=0.10,
    clock=None,
):
    """
    实时获取 weapon 在 world/map 坐标系下的位姿。

    先从 /lio/odom 获取 lidar 位姿，再通过 lidar->weapon 外参转换。
    """
    lidar_pose = get_lidar_pose_in_map_synced(
        tf_cache_node=tf_cache_node,
        max_age_sec=max_age_sec,
        clock=clock,
    )
    if lidar_pose is None:
        return None

    weapon_pose = cal_weapon_position(lidar_pose["T_map_lidar"])
    weapon_pose["stamp_sec"] = lidar_pose.get("stamp_sec")
    weapon_pose["age_sec"] = lidar_pose.get("age_sec")
    weapon_pose["frame_id"] = lidar_pose.get("frame_id")
    return weapon_pose


def cal_robot_position(T_map_lidar: np.ndarray):
    """
    由雷达在 world/map 下位姿，计算机器人中心在 world/map 下位姿。

    语义保持和 odin 一致：输入必须是 T_map_lidar，而不是 /lio/robo/odom
    直接给出的 T_map_robot。
    """
    T_map_robot = T_map_lidar @ get_t_lidar_to_base()
    return _robot_pose_from_matrix(T_map_robot=T_map_robot)


def cal_target_yaw_deg(target_x, target_y, robot_x, robot_y):
    """
    根据目标点与机器人当前位置，计算目标方向在 world/map 下的绝对航向角（度）。
    """
    dx = float(target_x) - float(robot_x)
    dy = float(target_y) - float(robot_y)
    yaw_deg = math.degrees(math.atan2(dy, dx))

    while yaw_deg >= 180.0:
        yaw_deg -= 360.0
    while yaw_deg < -180.0:
        yaw_deg += 360.0

    return yaw_deg


def cal_weapon_position(T_map_lidar: np.ndarray):
    """
    由雷达在 world/map 下位姿，计算武器/夹爪在 world/map 下位姿。

    输入:
      T_map_lidar: 4x4, 雷达在 world/map 坐标系下的齐次矩阵

    返回:
      dict:
      {
        "x": float,
        "y": float,
        "z": float,
        "yaw": float,           # rad
        "T_map_weapon": np.ndarray(4x4)
      }
    """
    T_map_weapon = T_map_lidar @ get_t_lidar_to_weapon()

    x = float(T_map_weapon[0, 3])
    y = float(T_map_weapon[1, 3])
    z = float(T_map_weapon[2, 3])
    yaw = math.atan2(T_map_weapon[1, 0], T_map_weapon[0, 0])

    return {
        "x": x,
        "y": y,
        "z": z,
        "yaw": yaw,
        "T_map_weapon": T_map_weapon,
    }
