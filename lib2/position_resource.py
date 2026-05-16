import threading
import time

from lib2 import position_backend
from lib2 import tools

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node


ODOM_TOPIC = "/odin1/odometry_highfreq"
position_lib = position_backend.get_position_backend()

ENTRANCE_X = 2.92
ENTRANCE_Y = 0.92
PRE_ENTRANCE_X = 1.80
PRE_ENTRANCE_Y = 0.957
STAIR_SIDE_LENGTH = 1.2 #m


def get_position_lib():
    return position_lib


def get_entrance_x():
    return float(getattr(position_lib, "ENTRANCE_X", ENTRANCE_X))


def get_entrance_y():
    return float(getattr(position_lib, "ENTRANCE_Y", ENTRANCE_Y))


def get_pre_entrance_x():
    return float(getattr(position_lib, "PRE_ENTRANCE_X", PRE_ENTRANCE_X))


def get_pre_entrance_y():
    return float(getattr(position_lib, "PRE_ENTRANCE_Y", PRE_ENTRANCE_Y))


def get_stair_side_length():
    return float(getattr(position_lib, "STAIR_SIDE_LENGTH", STAIR_SIDE_LENGTH))


def build_stair_height_relation_matrix():
    """
    按当前 position 后端的入口坐标生成台阶关系矩阵。

    每行格式:
    [阶梯编号, 前方关系, +90deg方向关系, -90deg方向关系, row, col]
    关系值: 0=该方向没有衔接台阶, 1=该方向台阶比当前台阶高, 2=该方向台阶比当前台阶低。
    方向语义: 1=前方, 2=+90deg, 3=-90deg。
    """
    entrance_x = get_entrance_x()
    entrance_y = get_entrance_y()
    pre_entrance_x = get_pre_entrance_x()
    pre_entrance_y = get_pre_entrance_y()
    side = get_stair_side_length()

    return [
        [-1, 1, 0, 0, entrance_x - side, entrance_y],
        [1, 2, 0, 2, entrance_x, entrance_y + side],
        [2, 1, 1, 1, entrance_x, entrance_y],
        [3, 1, 2, 0, entrance_x, entrance_y - side],
        [4, 1, 0, 1, entrance_x + side, entrance_y + side],
        [5, 1, 2, 1, entrance_x + side, entrance_y],
        [6, 2, 0, 2, entrance_x + side, entrance_y - side],
        [7, 2, 0, 1, entrance_x + 2 * side, entrance_y + side],
        [8, 2, 2, 2, entrance_x + 2 * side, entrance_y],
        [9, 2, 1, 0, entrance_x + 2 * side, entrance_y - side],
        [10, 2, 2, 1, entrance_x + 3 * side, entrance_y + side],
        [11, 0, 2, 2, entrance_x + 3 * side, entrance_y],
        [12, 2, 1, 2, entrance_x + 3 * side, entrance_y - side],
        [13, 0, 0, 0, entrance_x + 4 * side, entrance_y + side],
        [15, 0, 0, 2, entrance_x + 4 * side, entrance_y - side],
    ]


def get_stair_matrix():
    return build_stair_height_relation_matrix()


# 兼容旧代码的初始快照；主流程应调用 get_stair_matrix() 获取当前雷达后端矩阵。
STAIR_HEIGHT_RELATION_MATRIX = build_stair_height_relation_matrix()


def configure_position_backend(lidar_type):
    """
    设置并刷新当前位姿后端。

    resource.py 缓存 position_lib；切换雷达类型时调用本函数刷新资源层状态。
    """
    position_backend.set_lidar_type(lidar_type)
    backend = position_backend.get_position_backend()

    global position_lib, STAIR_HEIGHT_RELATION_MATRIX
    position_lib = backend
    STAIR_HEIGHT_RELATION_MATRIX = build_stair_height_relation_matrix()
    return backend


def get_stair_matrix_row(stair_id):
    stair_matrix = get_stair_matrix()
    stair_matrix_index = tools.stair_id_to_matrix_index(
        stair_id,
        stair_matrix=stair_matrix,
    )
    return stair_matrix_index, stair_matrix[stair_matrix_index]


def get_stair_xy(stair_id):
    _, stair_row = get_stair_matrix_row(stair_id)
    return float(stair_row[4]), float(stair_row[5])


def get_stair_height_relation(stair_id, direction):
    """
    根据台阶编号和方向返回该方向的高低关系。

    返回值沿用 STAIR_HEIGHT_RELATION_MATRIX:
      0: 不相邻/无衔接
      1: 该方向台阶比当前台阶高
      2: 该方向台阶比当前台阶低
    """
    direction = int(direction)
    if direction not in (1, 2, 3, 4):
        raise ValueError(f"direction must be 1, 2, 3 or 4, got {direction}")

    _, stair_row = get_stair_matrix_row(stair_id)
    if direction in (1, 2, 3):
        return int(stair_row[direction])

    current_x = float(stair_row[4])
    current_y = float(stair_row[5])
    stair_matrix = get_stair_matrix()
    stair_side_length = get_stair_side_length()
    expected_x = current_x - stair_side_length
    expected_y = current_y
    tolerance = stair_side_length * 0.2

    for candidate in stair_matrix:
        candidate_x = float(candidate[4])
        candidate_y = float(candidate[5])
        if (
            abs(candidate_x - expected_x) <= tolerance
            and abs(candidate_y - expected_y) <= tolerance
        ):
            candidate_front_relation = int(candidate[1])
            if candidate_front_relation == 1:
                return 2
            if candidate_front_relation == 2:
                return 1
            return 0
    return 0


class OdometrySubscriber(Node):
    def __init__(self, topic=ODOM_TOPIC):
        super().__init__("odometry_subscriber_node")
        self.topic = topic

        self._lock = threading.Lock()
        self._odom_msg = None
        self._recv_wall_time = 0.0

        self._sub = self.create_subscription(
            Odometry,
            self.topic,
            self._callback,
            50,
        )

    def _callback(self, msg: Odometry):
        with self._lock:
            self._odom_msg = msg
            self._recv_wall_time = time.time()

    def get_latest_msg(self):
        with self._lock:
            return self._odom_msg, self._recv_wall_time


def spin_odometry(node: OdometrySubscriber, stop_event: threading.Event):
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        while rclpy.ok() and not stop_event.is_set():
            try:
                executor.spin_once(timeout_sec=0.1)
            except ExternalShutdownException:
                break
    finally:
        executor.remove_node(node)
        executor.shutdown()


def _stamp_to_sec(stamp_msg):
    return float(stamp_msg.sec) + float(stamp_msg.nanosec) * 1e-9


def extract_odometry_params(odom_msg: Odometry):
    pose = odom_msg.pose.pose
    twist = odom_msg.twist.twist

    return {
        "header": odom_msg.header,
        "stamp_sec": _stamp_to_sec(odom_msg.header.stamp),
        "frame_id": odom_msg.header.frame_id,
        "child_frame_id": odom_msg.child_frame_id,
        "position_x": float(pose.position.x),
        "position_y": float(pose.position.y),
        "position_z": float(pose.position.z),
        "orientation_x": float(pose.orientation.x),
        "orientation_y": float(pose.orientation.y),
        "orientation_z": float(pose.orientation.z),
        "orientation_w": float(pose.orientation.w),
        "linear_x": float(twist.linear.x),
        "linear_y": float(twist.linear.y),
        "linear_z": float(twist.linear.z),
        "angular_x": float(twist.angular.x),
        "angular_y": float(twist.angular.y),
        "angular_z": float(twist.angular.z),
        "pose_covariance": list(odom_msg.pose.covariance),
        "twist_covariance": list(odom_msg.twist.covariance),
        "pose": odom_msg.pose,
        "twist": odom_msg.twist,
    }


def get_latest_odometry(node: OdometrySubscriber, max_age_sec=0.25):
    odom_msg, recv_wall_time = node.get_latest_msg()
    if odom_msg is None:
        return None

    age_sec = time.time() - recv_wall_time
    if age_sec < 0.0 or age_sec > float(max_age_sec):
        return None

    data = extract_odometry_params(odom_msg)
    data["age_sec"] = age_sec
    return data


class PositionRuntime:
    def __init__(self, sender, tf_node, tf_thread, tf_stop_event, position_thread, position_stop_event):
        self.sender = sender
        self.tf_node = tf_node
        self.tf_thread = tf_thread
        self.tf_stop_event = tf_stop_event
        self.position_thread = position_thread
        self.position_stop_event = position_stop_event
        self._lock = threading.Lock()
        self._latest_lidar_pose = None
        self._latest_robot_pose = None
        self._latest_yaw_deg = None
        self._latest_yaw_i16 = None
        self._latest_update_time = None

    def update(self, lidar_pose, robot_pose, yaw_deg, yaw_i16):
        with self._lock:
            self._latest_lidar_pose = None if lidar_pose is None else dict(lidar_pose)
            self._latest_robot_pose = None if robot_pose is None else dict(robot_pose)
            self._latest_yaw_deg = None if yaw_deg is None else float(yaw_deg)
            self._latest_yaw_i16 = None if yaw_i16 is None else int(yaw_i16)
            self._latest_update_time = time.time()

    def get_lidar_pose(self):
        with self._lock:
            return None if self._latest_lidar_pose is None else dict(self._latest_lidar_pose)

    def get_robot_pose(self):
        with self._lock:
            return None if self._latest_robot_pose is None else dict(self._latest_robot_pose)

    def get_current_yaw_deg(self):
        with self._lock:
            return self._latest_yaw_deg

    def get_current_yaw_i16(self):
        with self._lock:
            return self._latest_yaw_i16

    def get_current_position(self):
        with self._lock:
            if self._latest_robot_pose is None:
                return None
            return {
                "x": float(self._latest_robot_pose["x"]),
                "y": float(self._latest_robot_pose["y"]),
                "z": float(self._latest_robot_pose["z"]),
            }

    def get_weapon_pose(self, max_tf_age_sec=0.25):
        """
        实时读取最新同步 TF，并计算 weapon 在 map 坐标系下的位姿。

        该方法不使用 PositionRuntime 的位姿缓存，也不会写入缓存。
        """
        return position_lib.get_weapon_pose_in_map_synced(
            tf_cache_node=self.tf_node,
            max_age_sec=max_tf_age_sec,
            clock=self.tf_node.get_clock(),
        )

    def get_latest_update_time(self):
        with self._lock:
            return self._latest_update_time

    def get_tf_node(self):
        return self.tf_node

    def get_threads(self):
        return {
            "tf_node": self.tf_node,
            "tf_thread": self.tf_thread,
            "tf_stop_event": self.tf_stop_event,
            "position_thread": self.position_thread,
            "position_stop_event": self.position_stop_event,
        }


class OdomRuntime:
    def __init__(self, odom_node, odom_thread, odom_stop_event, max_age_sec=0.25):
        self.odom_node = odom_node
        self.odom_thread = odom_thread
        self.odom_stop_event = odom_stop_event
        self.max_age_sec = float(max_age_sec)

    def get_odometry(self, max_age_sec=None):
        if max_age_sec is None:
            max_age_sec = self.max_age_sec
        return get_latest_odometry(self.odom_node, max_age_sec=max_age_sec)

    def get_velocity(self, max_age_sec=None):
        odometry = self.get_odometry(max_age_sec=max_age_sec)
        if odometry is None:
            return None
        return {
            "linear_x": float(odometry["linear_x"]),
            "linear_y": float(odometry["linear_y"]),
            "linear_z": float(odometry["linear_z"]),
            "angular_x": float(odometry["angular_x"]),
            "angular_y": float(odometry["angular_y"]),
            "angular_z": float(odometry["angular_z"]),
        }

    def get_linear_speed_mps(self, max_age_sec=None):
        odometry = self.get_odometry(max_age_sec=max_age_sec)
        if odometry is None:
            return None
        linear_x = float(odometry["linear_x"])
        linear_y = float(odometry["linear_y"])
        return (linear_x * linear_x + linear_y * linear_y) ** 0.5

    def get_angular_z_rad(self, max_age_sec=None):
        odometry = self.get_odometry(max_age_sec=max_age_sec)
        if odometry is None:
            return None
        return float(odometry["angular_z"])

    def get_threads(self):
        return {
            "odom_node": self.odom_node,
            "odom_thread": self.odom_thread,
            "odom_stop_event": self.odom_stop_event,
        }


def _position_loop(
    runtime,
    update_hz=50.0,
    max_tf_age_sec=0.25,
):
    if update_hz <= 0.0:
        raise ValueError(f"update_hz must be > 0, got {update_hz}")

    period = 1.0 / float(update_hz)
    while not runtime.position_stop_event.is_set():
        if position_backend.is_mid360():
            robot_pose = position_lib.get_robot_pose_in_map_synced(
                tf_cache_node=runtime.tf_node,
                max_age_sec=max_tf_age_sec,
                clock=runtime.tf_node.get_clock(),
            )
            lidar_pose = None
        else:
            lidar_pose = position_lib.get_lidar_pose_in_map_synced(
                tf_cache_node=runtime.tf_node,
                max_age_sec=max_tf_age_sec,
                clock=runtime.tf_node.get_clock(),
            )
            robot_pose = None if lidar_pose is None else position_lib.cal_robot_position(
                lidar_pose["T_map_lidar"]
            )

        if robot_pose is not None:
            yaw_deg = position_lib.radians_to_degrees(robot_pose["yaw"])
            yaw_i16 = tools.yaw_deg_to_i16(yaw_deg)
            runtime.sender.set_current_yaw_i16(yaw_i16)
            runtime.update(
                lidar_pose=lidar_pose,
                robot_pose=robot_pose,
                yaw_deg=yaw_deg,
                yaw_i16=yaw_i16,
            )
        time.sleep(period)


def start_position_thread(
    sender,
    base_frame="odin1_base_link",
    tf_update_hz=50.0,
    yaw_update_hz=50.0,
    max_tf_age_sec=0.25,
):
    """
    启动位置相关后台模块：
    - 创建并启动 TfCacheNode 的 spin 线程
    - 创建并启动一个 yaw 更新线程
    - 持续从 TF 缓存中取当前位置，计算机器人实际航向角，并写入 sender

    返回:
      PositionRuntime
    """
    tf_node = position_lib.TfCacheNode(
        base_frame=base_frame,
        update_hz=tf_update_hz,
    )
    tf_stop_event = threading.Event()
    tf_thread = threading.Thread(
        target=position_lib.spin_tf_cache,
        args=(tf_node, tf_stop_event),
        daemon=True,
        name="tf_cache_thread",
    )
    tf_thread.start()

    position_stop_event = threading.Event()
    runtime = PositionRuntime(
        sender=sender,
        tf_node=tf_node,
        tf_thread=tf_thread,
        tf_stop_event=tf_stop_event,
        position_thread=None,
        position_stop_event=position_stop_event,
    )
    position_thread = threading.Thread(
        target=_position_loop,
        args=(runtime,),
        kwargs={
            "update_hz": yaw_update_hz,
            "max_tf_age_sec": max_tf_age_sec,
        },
        daemon=True,
        name="position_thread",
    )
    runtime.position_thread = position_thread
    position_thread.start()

    return runtime


def position_thread(*args, **kwargs):
    return start_position_thread(*args, **kwargs)


def start_odometry_thread(
    topic=ODOM_TOPIC,
    max_age_sec=0.25,
):
    """
    启动 odom_highfreq 订阅后台线程。

    返回:
      OdomRuntime
    """
    if not rclpy.ok():
        rclpy.init()

    odom_node = OdometrySubscriber(topic=topic)
    odom_stop_event = threading.Event()
    odom_thread = threading.Thread(
        target=spin_odometry,
        args=(odom_node, odom_stop_event),
        daemon=True,
        name="odometry_thread",
    )
    odom_thread.start()
    return OdomRuntime(
        odom_node=odom_node,
        odom_thread=odom_thread,
        odom_stop_event=odom_stop_event,
        max_age_sec=max_age_sec,
    )


def odometry_thread(*args, **kwargs):
    return start_odometry_thread(*args, **kwargs)
