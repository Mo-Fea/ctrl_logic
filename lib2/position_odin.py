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
from lib2 import tools


side_9 = 0.54
weapon_side = 0.2

# 里程计模式固定启动点。坐标以雷达的逻辑地图坐标表示。
ODOMETRY_ORIGIN_START_POINT = (0.0, 0.0)
ODOMETRY_SECOND_START_POINT = (3.757, 7.2)
ODOMETRY_DEFAULT_START_POINT = ODOMETRY_ORIGIN_START_POINT

# 0 号选项由交互配置保留给手动输入坐标。
ODOMETRY_START_POINTS = {
    1: ("origin", ODOMETRY_ORIGIN_START_POINT),
    2: ("second_start_point", ODOMETRY_SECOND_START_POINT),
}
_odometry_start_lidar_logic_xy = ODOMETRY_DEFAULT_START_POINT
_odometry_start_reference_odom_base = None
_odometry_start_lock = threading.Lock()



entrance_x0_red = 1.9331
entrance_y0_red = 3.1187

entrance_x90_red = 1.8677
entrance_y90_red = 3.1417

entrance_xneg90_red = 1.9133
entrance_yneg90_red = 3.0718

entrance_x180_red = 1.8493
entrance_y180_red = 3.0828

weapon_x_red = -1.0134
weapon_y_red = -0.140

# 九宫格区域坐标占位（二维坐标，待实测后替换）
pre_entrance9_red = (4.124, 8.2674)
entrance9_red = (4.124, 10.579)
column2_x_red = -0.7680
column2_y_red = 10.0
pre_x_red = 2.8189   
R1climb_red = (3.3189, 9.4231)

#--------------------------------------------------------------------------------------------------------------
# Left field placeholders. Fill these with measured coordinates from the real field.
entrance_x0_blue = 1.9331
entrance_y0_blue = 3.1187

entrance_x90_blue = 1.8677
entrance_y90_blue = 3.1417

entrance_xneg90_blue = 1.7393
entrance_yneg90_blue = 3.0718

entrance_x180_blue = 1.8493
entrance_y180_blue = 3.0828

weapon_x_blue = -1.0134
weapon_y_blue = -0.140

pre_entrance9_blue = (4.124, 8.2674)
entrance9_blue = (4.124, 10.579)
column2_x_blue = -0.7680
column2_y_blue = 10.0
pre_x_blue = 3.3189                   
R1climb_blue = (3.3189, 9.4321)


ENTRANCE_X0_RED = entrance_x0_red
ENTRANCE_X90_RED = entrance_x90_red
ENTRANCE_XNEG90_RED = entrance_xneg90_red
ENTRANCE_X180_RED = entrance_x180_red
ENTRANCE_Y0_RED = entrance_y0_red
ENTRANCE_Y90_RED = entrance_y90_red
ENTRANCE_YNEG90_RED = entrance_yneg90_red
ENTRANCE_Y180_RED = entrance_y180_red
ENTRANCE_X_RED = ENTRANCE_X0_RED
ENTRANCE_Y_RED = ENTRANCE_Y0_RED


weapon_side = 0.2
WEAPON_TARGETS_RED = {
    weapon_id: (weapon_x_red, weapon_y_red + (weapon_id - 1) * weapon_side)
    for weapon_id in range(1, 7)
}
WEAPON_RETREAT_STOP_Y_RED = -4.00

column1_red = (column2_x_red, column2_y_red - side_9)
column2_red = (column2_x_red, column2_y_red)
column3_red = (column2_x_red, column2_y_red + side_9)
pre_column1_red = (pre_x_red, column1_red[1])
pre_column2_red = (pre_x_red, column2_red[1])
pre_column3_red = (pre_x_red, column3_red[1])



ENTRANCE_X0_BLUE = entrance_x0_blue
ENTRANCE_X90_BLUE = entrance_x90_blue
ENTRANCE_XNEG90_BLUE = entrance_xneg90_blue
ENTRANCE_X180_BLUE = entrance_x180_blue
ENTRANCE_Y0_BLUE = entrance_y0_blue
ENTRANCE_Y90_BLUE = entrance_y90_blue
ENTRANCE_YNEG90_BLUE = entrance_yneg90_blue
ENTRANCE_Y180_BLUE = entrance_y180_blue
ENTRANCE_X_BLUE = ENTRANCE_X0_BLUE
ENTRANCE_Y_BLUE = ENTRANCE_Y0_BLUE



WEAPON_TARGETS_BLUE = {
    weapon_id: (weapon_x_blue, weapon_y_blue + (weapon_id - 1) * weapon_side)
    for weapon_id in range(1, 7)
}
WEAPON_RETREAT_STOP_Y_BLUE = 4.00

column1_blue = (column2_x_blue, column2_y_blue - side_9)
column2_blue = (column2_x_blue, column2_y_blue)
column3_blue = (column2_x_blue, column2_y_blue + side_9)
pre_column1_blue = (pre_x_blue, column1_blue[1])
pre_column2_blue = (pre_x_blue, column2_blue[1])
pre_column3_blue = (pre_x_blue, column3_blue[1])





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

LIDAR_TO_BASE_X = - 0.3325  # m
LIDAR_TO_BASE_Y = - 0.040  # m
LIDAR_TO_BASE_Z = 0.0
T_LIDAR_TO_BASE = np.array([
    [1.0, 0.0, 0.0, LIDAR_TO_BASE_X],
    [0.0, 1.0, 0.0, LIDAR_TO_BASE_Y],
    [0.0, 0.0, 1.0, LIDAR_TO_BASE_Z],
    [0.0, 0.0, 0.0, 1.0],
], dtype=float)


LIDAR_TO_WEAPON_X = 0.199  # m
LIDAR_TO_WEAPON_Y = -0.040  # m
LIDAR_TO_WEAPON_Z = 0.0  # m
T_LIDAR_TO_WEAPON = np.array([
    [1.0, 0.0, 0.0, LIDAR_TO_WEAPON_X],
    [0.0, 1.0, 0.0, LIDAR_TO_WEAPON_Y],
    [0.0, 0.0, 1.0, LIDAR_TO_WEAPON_Z],
    [0.0, 0.0, 0.0, 1.0],
], dtype=float)
ODIN_YAW_OFFSET_DEG = 180.0

#-----------------------------------------------------------------------------------------------------
# 获取 雷达->机器人中心 的固定齐次变换矩阵(4x4)
def get_t_lidar_to_base():
    """
    返回 雷达->机器人中心 的固定齐次变换矩阵(4x4)。
    """ 
    return T_LIDAR_TO_BASE.copy()


def reset_odometry_start_pose():
    """恢复里程计模式的雷达启动点为默认逻辑坐标。"""
    global _odometry_start_lidar_logic_xy
    global _odometry_start_reference_odom_base

    with _odometry_start_lock:
        _odometry_start_lidar_logic_xy = tuple(ODOMETRY_DEFAULT_START_POINT)
        _odometry_start_reference_odom_base = None


def set_odometry_start_lidar_pose(x, y):
    """设置里程计模式下雷达的逻辑地图启动坐标。"""
    global _odometry_start_lidar_logic_xy
    global _odometry_start_reference_odom_base

    x = float(x)
    y = float(y)
    if not math.isfinite(x) or not math.isfinite(y):
        raise ValueError(f"odometry start coordinates must be finite, got {(x, y)}")

    with _odometry_start_lock:
        _odometry_start_lidar_logic_xy = (x, y)
        # 新启动点必须以第一帧有效 odom->lidar 作为参考，不能复用旧缓存。
        _odometry_start_reference_odom_base = None

    return {
        "x": x,
        "y": y,
        "reference": "lidar_logic_map",
    }


def configure_odometry_start_point_interactively():
    """在里程计模式下选择预设或手动输入雷达启动点。"""
    print("里程计模式启动点（雷达逻辑坐标）：")
    print("0. 手动输入已知坐标")
    for point_id, (point_name, (x, y)) in sorted(ODOMETRY_START_POINTS.items()):
        print(f"{int(point_id)}. {point_name}: ({float(x):.4f}, {float(y):.4f})")

    while True:
        raw_choice = input("请输入启动点编号：").strip()
        try:
            choice = int(raw_choice)
        except ValueError:
            print("错误输入，请输入有效启动点编号")
            continue

        if choice == 0:
            try:
                x = float(input("请输入启动点雷达x坐标（m）：").strip())
                y = float(input("请输入启动点雷达y坐标（m）：").strip())
            except ValueError:
                print("错误输入，x/y必须是数值")
                continue
            try:
                result = set_odometry_start_lidar_pose(x, y)
            except ValueError as exc:
                print(f"错误输入，{exc}")
                continue
            print(f"里程计启动点已设置为：({result['x']:.4f}, {result['y']:.4f})")
            return result

        point = ODOMETRY_START_POINTS.get(choice)
        if point is None:
            print("错误输入，请输入有效启动点编号")
            continue

        point_name, (x, y) = point
        result = set_odometry_start_lidar_pose(x, y)
        result["name"] = str(point_name)
        print(
            f"里程计启动点已设置为 {point_name}："
            f"({result['x']:.4f}, {result['y']:.4f})"
        )
        return result


def _build_odometry_start_map_transform(T_odom_base):
    """构造使雷达起始输出等于手动逻辑坐标的 map<-odom 变换。"""
    global _odometry_start_reference_odom_base

    with _odometry_start_lock:
        if _odometry_start_reference_odom_base is None:
            _odometry_start_reference_odom_base = np.array(
                T_odom_base,
                dtype=float,
                copy=True,
            )

        T_initial_lidar = _odometry_start_reference_odom_base.copy()
        start_x, start_y = _odometry_start_lidar_logic_xy

    # 蓝场输出会在 _transform_y_for_field() 中取反，因此这里先转回原始 map y。
    raw_start_y = -start_y if position_backend.is_blue_field() else start_y
    T_target_lidar = T_initial_lidar.copy()
    T_target_lidar[0, 3] = float(start_x)
    T_target_lidar[1, 3] = float(raw_start_y)
    return T_target_lidar @ np.linalg.inv(T_initial_lidar)

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
        self._latest_odom_base = None
        self._latest_odom_base_stamp_sec = None
        self._latest_odom_base_cache_stamp = 0.0
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

                if field_name == "tf_odom_base":
                    if (
                        self._latest_odom_base_stamp_sec is None
                        or stamp_sec >= self._latest_odom_base_stamp_sec
                    ):
                        self._latest_odom_base = {
                            "T_odom_base": transform_to_matrix(transform),
                            "stamp_msg": transform.header.stamp,
                        }
                        self._latest_odom_base_stamp_sec = stamp_sec
                        self._latest_odom_base_cache_stamp = time.time()

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

    def get_latest_odom_to_base(self):
        """
        获取最新 odom -> base_frame 变换，不要求同时存在 odom -> map。

        返回 (T_odom_base, stamp_msg, cache_stamp)。调用方使用 cache_stamp
        按完整 map TF 缓存相同的方式检查有效年龄。
        """
        with self._lock:
            if self._latest_odom_base is None:
                return None, None, self._latest_odom_base_cache_stamp
            return (
                self._latest_odom_base["T_odom_base"].copy(),
                self._latest_odom_base["stamp_msg"],
                self._latest_odom_base_cache_stamp,
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


def normalize_yaw_deg(yaw_deg):
    yaw_deg = float(yaw_deg)
    while yaw_deg >= 180.0:
        yaw_deg -= 360.0
    while yaw_deg < -180.0:
        yaw_deg += 360.0
    return yaw_deg


def _mirror_yaw_deg_for_blue_field(yaw_deg):
    return normalize_yaw_deg(-float(yaw_deg))


def _transform_y_for_field(y):
    y = float(y)
    if position_backend.is_blue_field():
        return -y
    return y


def radians_to_degrees(angle_rad):
    yaw_deg = normalize_yaw_deg(math.degrees(angle_rad) + ODIN_YAW_OFFSET_DEG)
    if position_backend.is_blue_field():
        yaw_deg = _mirror_yaw_deg_for_blue_field(yaw_deg)
    return yaw_deg


def radians_to_control_degrees(angle_rad):
    return normalize_yaw_deg(math.degrees(angle_rad) + ODIN_YAW_OFFSET_DEG)


def logic_yaw_to_control_yaw_deg(logic_yaw_deg):
    logic_yaw_deg = normalize_yaw_deg(float(logic_yaw_deg))
    if position_backend.is_blue_field():
        return _mirror_yaw_deg_for_blue_field(logic_yaw_deg)
    return logic_yaw_deg


def transform_odometry_for_field(odometry):
    """
    按当前红/蓝半场调整 odom 输出。

    红场保持原值；蓝场将雷达输出转换到当前逻辑坐标系:
      x -> x
      y -> -y
      yaw -> -yaw
      linear_x -> linear_x
      linear_y -> -linear_y
      angular_z -> -angular_z
    """
    if odometry is None or not position_backend.is_blue_field():
        return odometry

    transformed = dict(odometry)
    transformed["position_x"] = float(transformed["position_x"])
    transformed["position_y"] = -float(transformed["position_y"])
    transformed["yaw_deg"] = _mirror_yaw_deg_for_blue_field(transformed["yaw_deg"])
    transformed["yaw_rad"] = math.radians(transformed["yaw_deg"])
    transformed["linear_x"] = float(transformed["linear_x"])
    transformed["linear_y"] = -float(transformed["linear_y"])
    transformed["angular_z"] = -float(transformed["angular_z"])
    transformed["field_transform"] = "blue_mirror_y"
    return transformed

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

def _lidar_pose_from_matrices(
    T_map_odom: np.ndarray,
    T_odom_base: np.ndarray,
    stamp_sec=None,
    age_sec=None,
    pose_source=None,
):
    T_map_lidar = T_map_odom @ T_odom_base
    x = float(T_map_lidar[0, 3])
    y = _transform_y_for_field(T_map_lidar[1, 3])
    z = float(T_map_lidar[2, 3])
    yaw = math.atan2(T_map_lidar[1, 0], T_map_lidar[0, 0])

    pose = {
        "x": x,
        "y": y,
        "z": z,
        "yaw": yaw,
        "T_map_lidar": T_map_lidar,
        "stamp_sec": stamp_sec,
        "age_sec": age_sec,
    }
    if pose_source is not None:
        pose["pose_source"] = str(pose_source)
    return pose

def get_lidar_pose_in_map_latest(tf_cache_node: TfCacheNode, max_cache_age_sec=1.0):
    """
    获取雷达在当前 map 逻辑坐标系下的最新有效位姿。

    默认使用完整 map TF；里程计模式下将 odom 原点作为 map 原点。
    """
    return get_lidar_pose_in_map_synced(
        tf_cache_node=tf_cache_node,
        max_age_sec=max_cache_age_sec,
    )

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
    if tools.is_odometry_mode():
        T_odom_base, stamp_msg, cache_stamp = tf_cache_node.get_latest_odom_to_base()
        if T_odom_base is None or stamp_msg is None:
            return None
        T_map_odom = _build_odometry_start_map_transform(T_odom_base)
        pose_source = "odom_with_manual_start_point"
    else:
        # 未选择里程计模式时，保持原有完整 map TF 逻辑。
        if T_map_odom is None or T_odom_base is None or stamp_msg is None:
            return None
        pose_source = "relocalization_tf"

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

    return _lidar_pose_from_matrices(
        T_map_odom,
        T_odom_base,
        stamp_sec,
        age_sec,
        pose_source=pose_source,
    )


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
    weapon_pose["pose_source"] = lidar_pose.get("pose_source")
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
    y = _transform_y_for_field(T_map_robot[1, 3])
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
    y = _transform_y_for_field(T_map_weapon[1, 3])
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
