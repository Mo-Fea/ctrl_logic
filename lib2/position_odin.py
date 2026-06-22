import time
import threading
import traceback
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from tf2_msgs.msg import TFMessage
import math

from lib2 import position_backend


ENTRANCE_X_RED = 0.587
ENTRANCE_Y_RED = -1.725
PRE_ENTRANCE_X_RED = -0.500
PRE_ENTRANCE_Y_RED = -1.776
WEAPON_TARGETS_RED = {
    1: (-2.176, 1.120),
    2: (-1.998, 1.122),
    3: (-1.784, 1.116),
    4: (-1.588, 1.110),
    5: (-1.378, 1.113),
    6: (-1.723, 1.150),
}
WEAPON_RETREAT_STOP_Y_RED = 4.00


# Left field placeholders. Fill these with measured coordinates from the real field.
ENTRANCE_X_BLUE = 0.55
ENTRANCE_Y_BLUE = 1.55
PRE_ENTRANCE_X_BLUE = -0.500
PRE_ENTRANCE_Y_BLUE = 1.776
WEAPON_TARGETS_BLUE = {
    1: (-1.840, -1.312),
    2: (-1.902, -1.342),
    3: (-2.265, -1.33),
    4: (-2.191, -1.150),
    5: (-1.996, -1.150),
    6: (-1.793, -1.150),
}
WEAPON_RETREAT_STOP_Y_BLUE = -4.00


# Backward-compatible defaults: current main flow still uses red/right field.
ENTRANCE_X = ENTRANCE_X_RED
ENTRANCE_Y = ENTRANCE_Y_RED
PRE_ENTRANCE_X = PRE_ENTRANCE_X_RED
PRE_ENTRANCE_Y = PRE_ENTRANCE_Y_RED
WEAPON_TARGETS = WEAPON_TARGETS_RED
WEAPON_RETREAT_STOP_Y = WEAPON_RETREAT_STOP_Y_RED


def _select_field_value(red_value, blue_value):
    if position_backend.is_blue_field():
        return blue_value
    return red_value


def get_entrance_x():
    return float(_select_field_value(ENTRANCE_X_RED, ENTRANCE_X_BLUE))


def get_entrance_y():
    return float(_select_field_value(ENTRANCE_Y_RED, ENTRANCE_Y_BLUE))


def get_pre_entrance_x():
    return float(_select_field_value(PRE_ENTRANCE_X_RED, PRE_ENTRANCE_X_BLUE))


def get_pre_entrance_y():
    return float(_select_field_value(PRE_ENTRANCE_Y_RED, PRE_ENTRANCE_Y_BLUE))


def get_weapon_targets():
    return dict(_select_field_value(WEAPON_TARGETS_RED, WEAPON_TARGETS_BLUE))


def get_weapon_retreat_stop_y():
    return float(_select_field_value(WEAPON_RETREAT_STOP_Y_RED, WEAPON_RETREAT_STOP_Y_BLUE))

LIDAR_TO_BASE_X = - 0.236  # m
LIDAR_TO_BASE_Y = - 0.206  # m  
LIDAR_TO_BASE_Z = 0.0
T_LIDAR_TO_BASE = np.array([
    [1.0, 0.0, 0.0, LIDAR_TO_BASE_X],
    [0.0, 1.0, 0.0, LIDAR_TO_BASE_Y],
    [0.0, 0.0, 1.0, LIDAR_TO_BASE_Z],
    [0.0, 0.0, 0.0, 1.0],
], dtype=float)


LIDAR_TO_WEAPON_X = -0.730  # m
LIDAR_TO_WEAPON_Y = -0.206  # m
LIDAR_TO_WEAPON_Z = 0.0  # m
T_LIDAR_TO_WEAPON = np.array([
    [1.0, 0.0, 0.0, LIDAR_TO_WEAPON_X],
    [0.0, 1.0, 0.0, LIDAR_TO_WEAPON_Y],
    [0.0, 0.0, 1.0, LIDAR_TO_WEAPON_Z],
    [0.0, 0.0, 0.0, 1.0],
], dtype=float)

#-----------------------------------------------------------------------------------------------------
# 获取 雷达->机器人中心 的固定齐次变换矩阵(4x4)
def get_t_lidar_to_base():
    """
    返回 雷达->机器人中心 的固定齐次变换矩阵(4x4)。
    """ 
    return T_LIDAR_TO_BASE.copy()

# 获取 雷达->武器 的固定齐次变换矩阵(4x4)
def get_t_lidar_to_weapon():
    """
    返回 雷达->武器 的固定齐次变换矩阵(4x4)。
    """
    return T_LIDAR_TO_WEAPON.copy()
#-----------------------------------------------------------------------------------------------------
# 订阅/读取 TF
class TfCacheNode(Node):
    def __init__(
        self,
        base_frame="odin1_base_link",
        tf_topic="/tf",
        update_hz=50.0,
        max_pending_age_sec=0.25,
    ):
        super().__init__("tf_cache_node")
        self.base_frame = base_frame
        self.tf_topic = tf_topic
        if update_hz <= 0.0:
            raise ValueError(f"update_hz must be > 0, got {update_hz}")
        self.period = 1.0 / max(1e-6, float(update_hz))

        self._lock = threading.Lock()
        self._latest_synced_pair = None
        self._stamp = 0.0
        self._pending_pairs = {}
        self._max_pending_pairs = 128
        self._max_pending_age_sec = float(max_pending_age_sec)
        self._latest_tf_stamp_sec = None

        self._tf_sub = self.create_subscription(
            TFMessage,
            self.tf_topic,
            self._tf_callback,
            100,
        )

    @staticmethod
    def _stamp_key(stamp_msg):
        return (int(stamp_msg.sec), int(stamp_msg.nanosec))

    @staticmethod
    def _key_to_sec(key):
        sec, nanosec = key
        return float(sec) + float(nanosec) * 1e-9

    def _prune_pending_pairs_locked(self):
        if self._latest_tf_stamp_sec is None:
            return

        expired_keys = []
        for key in self._pending_pairs:
            stamp_sec = self._key_to_sec(key)
            if (self._latest_tf_stamp_sec - stamp_sec) > self._max_pending_age_sec:
                expired_keys.append(key)

        for key in expired_keys:
            self._pending_pairs.pop(key, None)

        if len(self._pending_pairs) <= self._max_pending_pairs:
            return

        sorted_keys = sorted(self._pending_pairs.keys())
        remove_count = len(sorted_keys) - self._max_pending_pairs
        for key in sorted_keys[:remove_count]:
            self._pending_pairs.pop(key, None)

    def _try_finalize_pair_locked(self, key):
        pair = self._pending_pairs.get(key)
        if pair is None:
            return

        tf_odom_base = pair.get("tf_odom_base")
        tf_odom_map = pair.get("tf_odom_map")
        if tf_odom_base is None or tf_odom_map is None:
            return

        T_odom_base = transform_to_matrix(tf_odom_base)
        T_odom_map = transform_to_matrix(tf_odom_map)
        T_map_odom = np.linalg.inv(T_odom_map)

        self._latest_synced_pair = {
            "tf_odom_base": tf_odom_base,
            "tf_odom_map": tf_odom_map,
            "T_odom_base": T_odom_base,
            "T_map_odom": T_map_odom,
            "stamp_msg": tf_odom_base.header.stamp,
        }
        self._stamp = time.time()
        self._pending_pairs.pop(key, None)

    def _tf_callback(self, msg: TFMessage):
        with self._lock:
            for transform in msg.transforms:
                parent = transform.header.frame_id
                child = transform.child_frame_id

                if parent != "odom":
                    continue

                field_name = None
                if child == self.base_frame:
                    field_name = "tf_odom_base"
                elif child == "map":
                    field_name = "tf_odom_map"
                else:
                    continue

                key = self._stamp_key(transform.header.stamp)
                stamp_sec = self._key_to_sec(key)
                if self._latest_tf_stamp_sec is None or stamp_sec > self._latest_tf_stamp_sec:
                    self._latest_tf_stamp_sec = stamp_sec
                pair = self._pending_pairs.setdefault(key, {})
                pair[field_name] = transform
                self._try_finalize_pair_locked(key)

            self._prune_pending_pairs_locked()

    def update_once(self):
        with self._lock:
            return self._latest_synced_pair is not None

    def get_latest(self):
        with self._lock:
            if self._latest_synced_pair is None:
                return None, None, None, self._stamp
            return (
                self._latest_synced_pair["T_map_odom"].copy(),
                self._latest_synced_pair["T_odom_base"].copy(),
                self._latest_synced_pair["stamp_msg"],
                self._stamp,
            )

def spin_tf_cache(node: TfCacheNode, stop_event: threading.Event):
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

def tf_map2odom_sub(tf_cache_node: TfCacheNode):
    tf_map_odom, _, _, _ = tf_cache_node.get_latest()
    return tf_map_odom


def tf_odom2base_sub(tf_cache_node: TfCacheNode):
    _, tf_odom_base, _, _ = tf_cache_node.get_latest()
    return tf_odom_base

#-----------------------------------------------------------------------------------------------------
#获取旋转矩阵
def quaternion_to_rotation_matrix(qx, qy, qz, qw):
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm < 1e-12:
        return np.eye(3, dtype=float)

    qx /= norm
    qy /= norm
    qz /= norm
    qw /= norm

    return np.array([
        [1 - 2*(qy**2 + qz**2),     2*(qx*qy - qw*qz),     2*(qx*qz + qw*qy)],
        [    2*(qx*qy + qw*qz), 1 - 2*(qx**2 + qz**2),     2*(qy*qz - qw*qx)],
        [    2*(qx*qz - qw*qy),     2*(qy*qz + qw*qx), 1 - 2*(qx**2 + qy**2)]
    ], dtype=float)


def rotation_matrix_to_yaw(rotation_matrix: np.ndarray):
    return math.atan2(rotation_matrix[1, 0], rotation_matrix[0, 0])


def radians_to_degrees(angle_rad):
    return math.degrees(angle_rad)

#将ROS2的TransformStamped消息转换为4x4齐次变换矩阵
def transform_to_matrix(transform_msg):
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

# 将ROS2的时间戳转换为秒（浮点数）
def _stamp_to_sec(stamp_msg):
    return float(stamp_msg.sec) + float(stamp_msg.nanosec) * 1e-9

def _stamp_to_ros_time(stamp_msg):
    return rclpy.time.Time.from_msg(stamp_msg)

def _lidar_pose_from_matrices(T_map_odom: np.ndarray, T_odom_base: np.ndarray, stamp_sec=None, age_sec=None):
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

def get_lidar_pose_in_map_latest(tf_cache_node: TfCacheNode, max_cache_age_sec=1.0):
    """
    使用 /tf 中按同时间戳配对后的最新有效 TF 对，计算雷达在 map 下的位姿。
    """
    T_map_odom, T_odom_base, stamp_msg, cache_stamp = tf_cache_node.get_latest()
    if T_map_odom is None or T_odom_base is None or stamp_msg is None:
        return None

    age_sec = time.time() - cache_stamp
    if age_sec < 0.0 or age_sec > float(max_cache_age_sec):
        return None

    stamp_sec = _stamp_to_sec(stamp_msg)
    return _lidar_pose_from_matrices(T_map_odom, T_odom_base, stamp_sec, age_sec)

# 获取一对同一时间戳的TF: odom->base 和 map->odom
def tf_pair_synced(tf_cache_node: TfCacheNode):
    """
    从 /tf 配对缓存中获取一对同时间戳的可组合TF。

    返回:
      (T_map_odom, T_odom_base, stamp_msg) 或 (None, None, None)
    """
    try:
        T_map_odom, T_odom_base, stamp_msg, cache_stamp = tf_cache_node.get_latest()
        if T_map_odom is None or T_odom_base is None or stamp_msg is None:
            return None, None, None, None
        return T_map_odom, T_odom_base, stamp_msg, cache_stamp
    except Exception:
        print(f"[tf_pair_synced] TF error: {traceback.format_exc()}")
        return None, None, None, None

# 通过两段TF计算雷达在map坐标系下位姿（带年龄检查）
def get_lidar_pose_in_map_synced(
    tf_cache_node: TfCacheNode,
    max_age_sec=0.10,
    clock=None,   # 新增：ROS时钟，可传 node.get_clock()
):
    T_map_odom, T_odom_base, stamp_msg, cache_stamp = tf_pair_synced(
        tf_cache_node=tf_cache_node,
    )
    if T_map_odom is None:
        return None

    # 优先使用本地缓存接收年龄，避免设备侧TF时间戳与本机ROS时钟不在同一时间基准时被误判。
    age_sec = time.time() - cache_stamp if cache_stamp is not None else None

    # 若没有缓存时间，再退回ROS时间戳年龄。
    if age_sec is None:
        if clock is None:
            clock = rclpy.clock.Clock()
        stamp_ros = _stamp_to_ros_time(stamp_msg)
        now_ros = clock.now()
        age_sec = (now_ros - stamp_ros).nanoseconds * 1e-9

    stamp_sec = _stamp_to_sec(stamp_msg)

    # 年龄检查
    if age_sec < 0.0 or age_sec > float(max_age_sec):
        return None

    return _lidar_pose_from_matrices(T_map_odom, T_odom_base, stamp_sec, age_sec)


def get_weapon_pose_in_map_synced(
    tf_cache_node: TfCacheNode,
    max_age_sec=0.10,
    clock=None,
):
    """
    实时获取 weapon 在 map 坐标系下的位姿。

    每次调用都会读取当前 TF cache 中最新的同步 TF pair，先计算雷达在 map
    下位姿，再通过雷达->weapon 外参转换得到 weapon 位姿；不做后台缓存。
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
    return weapon_pose
#-----------------------------------------------------------------------------------------------------
# 由雷达在map下位姿，计算机器人中心在map下位姿
def cal_robot_position(T_map_lidar: np.ndarray):
    """
    由雷达在map下位姿，计算机器人中心在map下位姿

    输入:
      T_map_lidar: 4x4, 雷达在map坐标系下的齐次矩阵

    返回:
      dict:
      {
        "x": float,
        "y": float,
        "z": float,
        "yaw": float,          # rad
        "T_map_robot": np.ndarray(4x4)
      }
    """
    T_map_robot = T_map_lidar @ get_t_lidar_to_base()

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
    }


def cal_target_yaw_deg(target_x, target_y, robot_x, robot_y):
    """
    根据目标点与机器人当前位置，计算目标方向在 map 坐标系下的绝对航向角（度）。

    输入:
      target_x, target_y: 目标点在 map 下坐标
      robot_x, robot_y: 机器人当前位置在 map 下坐标

    返回:
      float: 归一化到 [-180, 180) 的绝对航向角（deg）
    """
    dx = float(target_x) - float(robot_x)
    dy = float(target_y) - float(robot_y)
    yaw_deg = math.degrees(math.atan2(dy, dx))

    while yaw_deg >= 180.0:
        yaw_deg -= 360.0
    while yaw_deg < -180.0:
        yaw_deg += 360.0

    return yaw_deg

# 由雷达在map下位姿，计算夹爪/武器在map下位姿
def cal_weapon_position(T_map_lidar: np.ndarray):
    """
    由雷达在map下位姿，计算夹爪/武器在map下位姿

    输入:
      T_map_lidar: 4x4, 雷达在map坐标系下的齐次矩阵

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
#-----------------------------------------------------------------------------------------------------