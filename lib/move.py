import threading
import time
import math

from lib import tools

from r2_logic.lib2 import position_odin
import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node


ODOM_TOPIC = "/odin1/odometry_highfreq"
DEFAULT_STOP_DISTANCE = 0.15
DEFAULT_REACHED_SPEED_MPS = 0.05
DEFAULT_REACHED_YAW_RATE_RAD = 0.05
DEFAULT_MOVE_GATE_DEG = 10.0
DEFAULT_OVERSHOOT_NEAR_DISTANCE = 0.5
DEFAULT_OVERSHOOT_DISTANCE_EPS = 0.03
DEFAULT_OVERSHOOT_HEADING_THRESHOLD_DEG = 120.0
DEFAULT_OVERSHOOT_CONFIRM_COUNT = 3
DEFAULT_OVERSHOOT_REVERSE_CMD = -50
DEFAULT_OVERSHOOT_MAX_DURATION_SEC = 2.0
CLIMB_TF_MAX_AGE_SEC = 0.25
CLIMB_YAW_TOLERANCE_DEG = 1.0
CLIMB_FORWARD_DURATION_SEC = 2.0
CLIMB_FORWARD_CMD = 300
CLIMB_SUCCESS_Z_MARGIN = 0.15
CLIMB_MODE_STAIR = 1
CLIMB_STAIR_TRIGGER_ARM_DURATION_SEC = 0.20
CLIMB_STAIR_TRIGGER_FIRE_DURATION_SEC = 0.20
CLIMB_POST_TRIGGER_SETTLE_SEC = 15.0
CLIMB_RECOVERY_FORWARD_CMD = 100
CLIMB_RECOVERY_DURATION_SEC = 1.0
CLIMB_CH_MODE_INDEX = 5
CLIMB_CH_STAIR_TRIGGER_INDEX = 7
CLIMB_TARGET_YAW_MAP = {
    1: 0.01,
    2: 90.0,
    3: 180.0,
    4: -90.0,
}



#----------------------------------------------------------------------------------------------------------------------
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


def normalize_yaw_deg(yaw_deg):
    return tools.yaw_normalization(float(yaw_deg))


def yaw_deg_to_i16(yaw_deg):
    return tools.yaw_deg_to_i16(float(yaw_deg))


def yaw_i16_to_deg(yaw_i16):
    yaw_i16 = max(-18000, min(18000, int(yaw_i16)))
    return float(yaw_i16) / 100.0


def quaternion_to_yaw_rad(qx, qy, qz, qw):
    rotation_matrix = position_odin.quaternion_to_rotation_matrix(qx, qy, qz, qw)
    return position_odin.rotation_matrix_to_yaw(rotation_matrix)


def quaternion_to_yaw_deg(qx, qy, qz, qw):
    yaw_rad = quaternion_to_yaw_rad(qx, qy, qz, qw)
    return normalize_yaw_deg(position_odin.radians_to_degrees(yaw_rad))


def quaternion_to_yaw_i16(qx, qy, qz, qw):
    yaw_rad = quaternion_to_yaw_rad(qx, qy, qz, qw)
    yaw_deg = normalize_yaw_deg(position_odin.radians_to_degrees(yaw_rad))
    return yaw_deg_to_i16(yaw_deg)


def get_current_yaw_deg(odometry_data):
    if odometry_data is None:
        return None

    return quaternion_to_yaw_deg(
        odometry_data["orientation_x"],
        odometry_data["orientation_y"],
        odometry_data["orientation_z"],
        odometry_data["orientation_w"],
    )


def heading_error_deg(current_yaw_deg, target_yaw_deg):
    return normalize_yaw_deg(float(target_yaw_deg) - float(current_yaw_deg))


def is_heading_reached(current_yaw_deg, target_yaw_deg, tolerance_deg=2.0):
    return abs(heading_error_deg(current_yaw_deg, target_yaw_deg)) <= float(tolerance_deg)


def encode_current_yaw_i16(current_yaw_deg):
    return yaw_deg_to_i16(current_yaw_deg)


def encode_target_yaw_i16(target_yaw_deg):
    target_yaw_i16 = yaw_deg_to_i16(target_yaw_deg)
    if target_yaw_i16 == 0:
        return 1
    return target_yaw_i16


def compose_chassis_channels(
    lateral_cmd=0,
    forward_cmd=0,
    rotation_cmd=0,
    channel_count=tools.CHANNEL_COUNT,
):
    return tools.compose_channels(
        lateral_cmd=lateral_cmd,
        forward_cmd=forward_cmd,
        rotation_cmd=rotation_cmd,
        channel_count=channel_count,
    )


def clamp_chassis_cmd(cmd, limit=600):
    return max(-int(limit), min(int(limit), int(round(cmd))))


def set_target_yaw_with_motion(
    target_yaw_deg,
    odometry_data=None,
    lateral_cmd=0,
    forward_cmd=0,
    channel_count=tools.CHANNEL_COUNT,
    tolerance_deg=2.0,
    current_yaw_deg=None,
):
    """
    生成“设置目标航向角”的非阻塞控制参数。

    说明:
    - 当前航向角来自 odometry
    - 目标航向角通过 des_yaw_i16 发送给下位机
    - des_yaw_i16 == 0 在电控中表示“停止旋转”，因此目标角 0° 会编码为 1
    - ch3 已废弃，因此 rotation_cmd 固定为 0
    - lateral_cmd / forward_cmd 可同时给定，保证航向与移动逻辑互不阻塞
    """
    if current_yaw_deg is None:
        current_yaw_deg = get_current_yaw_deg(odometry_data)
    if current_yaw_deg is None:
        return None

    target_yaw_deg = normalize_yaw_deg(target_yaw_deg)
    yaw_i16 = encode_current_yaw_i16(current_yaw_deg)
    des_yaw_i16 = encode_target_yaw_i16(target_yaw_deg)

    channels = compose_chassis_channels(
        lateral_cmd=lateral_cmd,
        forward_cmd=forward_cmd,
        rotation_cmd=0,
        channel_count=channel_count,
    )

    return {
        "channels": channels,
        "yaw_i16": yaw_i16,
        "des_yaw_i16": des_yaw_i16,
        "current_yaw_deg": current_yaw_deg,
        "target_yaw_deg": target_yaw_deg,
        "heading_error_deg": heading_error_deg(current_yaw_deg, target_yaw_deg),
        "heading_reached": is_heading_reached(
            current_yaw_deg,
            target_yaw_deg,
            tolerance_deg=tolerance_deg,
        ),
    }


def rotate_to_target_yaw(
    target_yaw_deg,
    odometry_data=None,
    channel_count=tools.CHANNEL_COUNT,
    tolerance_deg=2.0,
    current_yaw_deg=None,
):
    """
    原地自旋到指定航向角。

    说明:
    - ch0 = 0
    - ch2 = 0
    - ch3 = 0
    - 实际自旋由下位机根据 yaw_i16 / des_yaw_i16 闭环完成
    """
    return set_target_yaw_with_motion(
        odometry_data=odometry_data,
        target_yaw_deg=target_yaw_deg,
        lateral_cmd=0,
        forward_cmd=0,
        channel_count=channel_count,
        tolerance_deg=tolerance_deg,
        current_yaw_deg=current_yaw_deg,
    )


class MoveController:
    def __init__(self, odom_node, tf_cache_node):
        self.odom_node = odom_node
        self.tf_cache_node = tf_cache_node
        self._last_odometry_data = None
        self._last_odometry_wall_time = 0.0
        self._last_map_pose = None
        self._last_map_pose_wall_time = 0.0
        self._move_target_key = None
        self._best_distance_xy = None
        self._overshoot_confirm_count = 0
        self._overshoot_recovery_active = False
        self._overshoot_recovery_start_wall_time = 0.0

    def _reset_move_tracking(self, target_key=None):
        self._move_target_key = target_key
        self._best_distance_xy = None
        self._overshoot_confirm_count = 0
        self._overshoot_recovery_active = False
        self._overshoot_recovery_start_wall_time = 0.0

    def _ensure_move_tracking(self, target_key):
        if self._move_target_key != target_key:
            self._reset_move_tracking(target_key=target_key)

    def des_move(
        self,
        x,
        y,
        z,
        max_odom_age_sec=0.25,
        max_tf_age_sec=0.25,
        stop_distance=DEFAULT_STOP_DISTANCE,
        move_gate_deg=DEFAULT_MOVE_GATE_DEG,
        yaw_tolerance_deg=2.0,
        max_forward_cmd=600,
        min_forward_cmd=120,
        linear_speed_limit_mps=3.5,
        speed_damping_gain=90.0,
        heading_brake_deg=20.0,
        reached_speed_mps=DEFAULT_REACHED_SPEED_MPS,
        reached_yaw_rate_rad=DEFAULT_REACHED_YAW_RATE_RAD,
    ):
        """
        朝目标点移动的非阻塞控制方法。

        控制逻辑:
        1) 计算目标点与机器人当前点的向量
        2) 计算该向量在地图/世界坐标系下的绝对航向角
        3) 若当前航向偏差 > +-30deg，则原地对准该角度
        4) 若偏差 <= +-30deg，则边调整航向边向前移动
        5) 每次调用都会重新读取 odometry 并重新计算目标航向

        说明:
        - 输入目标点坐标为 map 坐标系
        - 当前位姿通过 position_odin 中的同步 TF 结果获取
        - 速度反馈使用 odometry
        """
        odometry_data = get_latest_odometry(self.odom_node, max_age_sec=max_odom_age_sec)
        if odometry_data is not None:
            self._last_odometry_data = odometry_data
            self._last_odometry_wall_time = time.time()
        elif (
            self._last_odometry_data is not None
            and (time.time() - self._last_odometry_wall_time) <= float(max_odom_age_sec)
        ):
            odometry_data = self._last_odometry_data
        else:
            return None

        map_pose = position_odin.get_lidar_pose_in_map_synced(
            tf_cache_node=self.tf_cache_node,
            max_age_sec=max_tf_age_sec,
            clock=self.tf_cache_node.get_clock(),
        )
        if map_pose is not None:
            self._last_map_pose = map_pose
            self._last_map_pose_wall_time = time.time()
        elif (
            self._last_map_pose is not None
            and (time.time() - self._last_map_pose_wall_time) <= float(max_tf_age_sec)
        ):
            map_pose = self._last_map_pose
        else:
            return None

        robot_pose = position_odin.cal_robot_position(map_pose["T_map_lidar"])
        current_x = robot_pose["x"]
        current_y = robot_pose["y"]
        current_z = robot_pose["z"]
        current_yaw_deg = normalize_yaw_deg(position_odin.radians_to_degrees(robot_pose["yaw"]))
        target_key = (float(x), float(y), float(z))
        self._ensure_move_tracking(target_key)

        dx = float(x) - current_x
        dy = float(y) - current_y
        dz = float(z) - current_z
        distance_xy = math.hypot(dx, dy)
        distance_xyz = math.sqrt(dx * dx + dy * dy + dz * dz)
        target_yaw_deg = normalize_yaw_deg(math.degrees(math.atan2(dy, dx))) if distance_xy > 1e-9 else current_yaw_deg
        yaw_error_deg = heading_error_deg(current_yaw_deg, target_yaw_deg)

        linear_x = odometry_data["linear_x"]
        linear_y = odometry_data["linear_y"]
        angular_z = odometry_data["angular_z"]
        linear_speed_mps = math.hypot(linear_x, linear_y)

        common_data = {
            "distance_xy": distance_xy,
            "distance_xyz": distance_xyz,
            "dx": dx,
            "dy": dy,
            "dz": dz,
            "linear_speed_mps": linear_speed_mps,
            "angular_z_rad": angular_z,
            "odometry_data": odometry_data,
            "map_pose": map_pose,
            "robot_pose": robot_pose,
        }

        reached = (
            distance_xy <= float(stop_distance)
            and linear_speed_mps <= float(reached_speed_mps)
            and abs(angular_z) <= float(reached_yaw_rate_rad)
        )
        if reached:
            self._reset_move_tracking(target_key=target_key)
            channels = compose_chassis_channels(
                lateral_cmd=0,
                forward_cmd=0,
                rotation_cmd=0,
            )
            result = {
                "mode": "reached",
                "channels": channels,
                "yaw_i16": encode_current_yaw_i16(current_yaw_deg),
                "des_yaw_i16": 0,
                "current_yaw_deg": current_yaw_deg,
                "target_yaw_deg": target_yaw_deg,
                "heading_error_deg": yaw_error_deg,
                "heading_reached": True,
            }
            result.update(common_data)
            return result

        if self._overshoot_recovery_active:
            recovery_elapsed = time.time() - self._overshoot_recovery_start_wall_time
            if distance_xy <= float(stop_distance):
                self._reset_move_tracking(target_key=target_key)
                channels = compose_chassis_channels(
                    lateral_cmd=0,
                    forward_cmd=0,
                    rotation_cmd=0,
                )
                result = {
                    "mode": "reached",
                    "channels": channels,
                    "yaw_i16": encode_current_yaw_i16(current_yaw_deg),
                    "des_yaw_i16": 0,
                    "current_yaw_deg": current_yaw_deg,
                    "target_yaw_deg": target_yaw_deg,
                    "heading_error_deg": yaw_error_deg,
                    "heading_reached": True,
                    "reached_by_overshoot_recovery": True,
                }
                result.update(common_data)
                return result

            if recovery_elapsed <= DEFAULT_OVERSHOOT_MAX_DURATION_SEC:
                channels = compose_chassis_channels(
                    lateral_cmd=0,
                    forward_cmd=DEFAULT_OVERSHOOT_REVERSE_CMD,
                    rotation_cmd=0,
                )
                result = {
                    "channels": channels,
                    "yaw_i16": encode_current_yaw_i16(current_yaw_deg),
                    "des_yaw_i16": 0,
                    "current_yaw_deg": current_yaw_deg,
                    "target_yaw_deg": 0.0,
                    "heading_error_deg": 0.0,
                    "heading_reached": True,
                }
                result.update({
                    "mode": "overshoot_recovering",
                    "forward_cmd": DEFAULT_OVERSHOOT_REVERSE_CMD,
                    "overshoot_recovery_elapsed_sec": recovery_elapsed,
                })
                result.update(common_data)
                return result

            self._overshoot_recovery_active = False
            self._overshoot_confirm_count = 0
            if self._best_distance_xy is None or distance_xy < self._best_distance_xy:
                self._best_distance_xy = distance_xy

        if self._best_distance_xy is None or distance_xy < self._best_distance_xy:
            self._best_distance_xy = distance_xy
            self._overshoot_confirm_count = 0
        else:
            overshoot_detected = (
                distance_xy <= float(DEFAULT_OVERSHOOT_NEAR_DISTANCE)
                and distance_xy >= (self._best_distance_xy + float(DEFAULT_OVERSHOOT_DISTANCE_EPS))
                and abs(yaw_error_deg) >= float(DEFAULT_OVERSHOOT_HEADING_THRESHOLD_DEG)
            )
            if overshoot_detected:
                self._overshoot_confirm_count += 1
            else:
                self._overshoot_confirm_count = 0

            if self._overshoot_confirm_count >= int(DEFAULT_OVERSHOOT_CONFIRM_COUNT):
                self._overshoot_recovery_active = True
                self._overshoot_recovery_start_wall_time = time.time()
                self._overshoot_confirm_count = 0

                channels = compose_chassis_channels(
                    lateral_cmd=0,
                    forward_cmd=DEFAULT_OVERSHOOT_REVERSE_CMD,
                    rotation_cmd=0,
                )
                result = {
                    "channels": channels,
                    "yaw_i16": encode_current_yaw_i16(current_yaw_deg),
                    "des_yaw_i16": 0,
                    "current_yaw_deg": current_yaw_deg,
                    "target_yaw_deg": 0.0,
                    "heading_error_deg": 0.0,
                    "heading_reached": True,
                }
                result.update({
                    "mode": "overshoot_recovering",
                    "forward_cmd": DEFAULT_OVERSHOOT_REVERSE_CMD,
                    "overshoot_recovery_elapsed_sec": 0.0,
                })
                result.update(common_data)
                return result

        if abs(yaw_error_deg) > float(move_gate_deg):
            result = rotate_to_target_yaw(
                odometry_data=odometry_data,
                target_yaw_deg=target_yaw_deg,
                tolerance_deg=yaw_tolerance_deg,
                current_yaw_deg=current_yaw_deg,
            )
            result.update({"mode": "aligning"})
            result.update(common_data)
            return result

        if distance_xy <= 1.5:
            forward_cmd = 100.0
        else:
            distance_ratio = min(1.0, max(0.0, distance_xy / 1.0))
            forward_cmd = min_forward_cmd + (max_forward_cmd - min_forward_cmd) * distance_ratio

        if abs(yaw_error_deg) > float(heading_brake_deg):
            forward_cmd *= 0.5
        else:
            heading_scale = max(0.2, math.cos(math.radians(abs(yaw_error_deg))))
            forward_cmd *= heading_scale

        # 使用 odometry 速度做阻尼，尽量减小冲过头
        speed_feedback_cmd = (linear_speed_mps / max(1e-6, float(linear_speed_limit_mps))) * float(speed_damping_gain)
        forward_cmd -= speed_feedback_cmd
        forward_cmd = clamp_chassis_cmd(max(0.0, forward_cmd), limit=max_forward_cmd)

        result = set_target_yaw_with_motion(
            odometry_data=odometry_data,
            target_yaw_deg=target_yaw_deg,
            lateral_cmd=0,
            forward_cmd=forward_cmd,
            tolerance_deg=yaw_tolerance_deg,
            current_yaw_deg=current_yaw_deg,
        )
        result.update({
            "mode": "moving",
            "forward_cmd": forward_cmd,
        })
        result.update(common_data)
        return result


def start_odometry_subscription(topic=ODOM_TOPIC):
    if not rclpy.ok():
        rclpy.init()

    node = OdometrySubscriber(topic=topic)
    stop_event = threading.Event()
    spin_thread = threading.Thread(
        target=spin_odometry,
        args=(node, stop_event),
        daemon=True,
    )
    spin_thread.start()

    return node, spin_thread, stop_event


def stop_odometry_subscription(node, spin_thread, stop_event):
    tools.destroy_ros2_thread(
        node=node,
        spin_thread=spin_thread,
        stop_event=stop_event,
        shutdown_rclpy=False,
    )


def _set_channel(channels, index, value):
    channels[int(index)] = int(value)


def _apply_stair_mode(channels, stair_trigger_value=1):
    _set_channel(channels, CLIMB_CH_MODE_INDEX, CLIMB_MODE_STAIR)
    _set_channel(channels, CLIMB_CH_STAIR_TRIGGER_INDEX, stair_trigger_value)


def _get_robot_pose_from_tf_cache(tf_cache_node, max_tf_age_sec=CLIMB_TF_MAX_AGE_SEC):
    map_pose = position_odin.get_lidar_pose_in_map_synced(
        tf_cache_node=tf_cache_node,
        max_age_sec=max_tf_age_sec,
        clock=tf_cache_node.get_clock(),
    )
    if map_pose is None:
        return None, None

    robot_pose = position_odin.cal_robot_position(map_pose["T_map_lidar"])
    current_yaw_deg = normalize_yaw_deg(position_odin.radians_to_degrees(robot_pose["yaw"]))
    return robot_pose, current_yaw_deg


def _send_control_frame(sock, seq, channels, yaw_i16, des_yaw_i16):
    frame = tools.build_frame(
        seq=seq,
        channels=channels,
        yaw_i16=yaw_i16,
        des_yaw_i16=des_yaw_i16,
    )
    sock, ok = tools.send_frame(
        sock=sock,
        frame=frame,
        connect_func=tools.connect,
        tcp_ip=tools.TCP_IP,
        tcp_port=tools.TCP_PORT,
        retry_interval=tools.CONNECT_RETRY_INTERVAL,
    )
    if not ok:
        return sock, seq, False
    return sock, ((seq + 1) & 0xFFFF), True


def climb(direction, sock, seq, tf_cache_node=None):
    direction = int(direction)
    if direction not in CLIMB_TARGET_YAW_MAP:
        raise ValueError("climb(direction) only accepts 1, 2, 3, 4")

    target_yaw_deg = CLIMB_TARGET_YAW_MAP[direction]
    initialized_rclpy = False
    owns_tf_cache_node = False
    tf_node = None
    tf_thread = None
    tf_stop_event = None
    last_status_print = 0.0

    def maybe_print(stage, message):
        nonlocal last_status_print
        now = time.time()
        if (now - last_status_print) >= 0.5:
            print(f"[climb:{stage}] {message}")
            last_status_print = now

    try:
        if not rclpy.ok():
            rclpy.init()
            initialized_rclpy = True

        if tf_cache_node is not None:
            tf_node = tf_cache_node
        else:
            tf_node = position_odin.TfCacheNode(update_hz=50.0)
            tf_stop_event = threading.Event()
            tf_thread = threading.Thread(
                target=position_odin.spin_tf_cache,
                args=(tf_node, tf_stop_event),
                daemon=True,
            )
            tf_thread.start()
            owns_tf_cache_node = True

        start_robot_pose = None
        start_deadline = time.time() + 2.0
        while time.time() < start_deadline and start_robot_pose is None:
            start_robot_pose, _ = _get_robot_pose_from_tf_cache(tf_node)
            if start_robot_pose is None:
                time.sleep(0.02)

        if start_robot_pose is None:
            print("上楼梯出错：无法获取初始 z 坐标")
            return sock, seq, False

        start_z = float(start_robot_pose["z"])
        print(
            f"[climb] start direction={direction} | "
            f"target_yaw={target_yaw_deg:.2f} deg | start_z={start_z:.3f}"
        )

        while True:
            _, current_yaw_deg = _get_robot_pose_from_tf_cache(tf_node)
            if current_yaw_deg is None:
                time.sleep(0.02)
                continue

            result = rotate_to_target_yaw(
                target_yaw_deg=target_yaw_deg,
                current_yaw_deg=current_yaw_deg,
                tolerance_deg=CLIMB_YAW_TOLERANCE_DEG,
            )
            if result is None:
                time.sleep(0.02)
                continue

            sock, seq, ok = _send_control_frame(
                sock=sock,
                seq=seq,
                channels=result["channels"],
                yaw_i16=result["yaw_i16"],
                des_yaw_i16=result["des_yaw_i16"],
            )
            if not ok:
                continue

            maybe_print(
                "ALIGN",
                f"current_yaw={current_yaw_deg:.2f} deg | "
                f"yaw_error={result['heading_error_deg']:.2f} deg"
            )

            if result["heading_reached"]:
                print("[climb:ALIGN] heading reached.")
                break

            time.sleep(0.02)

        forward_deadline = time.time() + CLIMB_FORWARD_DURATION_SEC
        print(
            f"[climb:FORWARD] forward_cmd={CLIMB_FORWARD_CMD} | "
            f"duration={CLIMB_FORWARD_DURATION_SEC:.2f}s"
        )
        while time.time() < forward_deadline:
            _, current_yaw_deg = _get_robot_pose_from_tf_cache(tf_node)
            if current_yaw_deg is None:
                time.sleep(0.02)
                continue

            result = set_target_yaw_with_motion(
                target_yaw_deg=target_yaw_deg,
                forward_cmd=CLIMB_FORWARD_CMD,
                current_yaw_deg=current_yaw_deg,
                tolerance_deg=CLIMB_YAW_TOLERANCE_DEG,
            )
            if result is None:
                time.sleep(0.02)
                continue

            sock, seq, ok = _send_control_frame(
                sock=sock,
                seq=seq,
                channels=result["channels"],
                yaw_i16=result["yaw_i16"],
                des_yaw_i16=result["des_yaw_i16"],
            )
            if not ok:
                continue

            maybe_print(
                "FORWARD",
                f"current_yaw={current_yaw_deg:.2f} deg | "
                f"remaining={max(0.0, forward_deadline - time.time()):.2f}s"
            )

            time.sleep(0.02)

        _, current_yaw_deg = _get_robot_pose_from_tf_cache(tf_node)
        if current_yaw_deg is None:
            current_yaw_deg = normalize_yaw_deg(target_yaw_deg)

        print("[climb:TRIGGER] arm stair trigger.")
        arm_deadline = time.time() + CLIMB_STAIR_TRIGGER_ARM_DURATION_SEC
        while time.time() < arm_deadline:
            channels = compose_chassis_channels(lateral_cmd=0, forward_cmd=0, rotation_cmd=0)
            _apply_stair_mode(channels, stair_trigger_value=1)
            sock, seq, ok = _send_control_frame(
                sock=sock,
                seq=seq,
                channels=channels,
                yaw_i16=encode_current_yaw_i16(current_yaw_deg),
                des_yaw_i16=encode_target_yaw_i16(target_yaw_deg),
            )
            if not ok:
                continue
            time.sleep(0.02)

        print("[climb:TRIGGER] fire stair trigger.")
        fire_deadline = time.time() + CLIMB_STAIR_TRIGGER_FIRE_DURATION_SEC
        while time.time() < fire_deadline:
            channels = compose_chassis_channels(lateral_cmd=0, forward_cmd=0, rotation_cmd=0)
            _apply_stair_mode(channels, stair_trigger_value=3)
            sock, seq, ok = _send_control_frame(
                sock=sock,
                seq=seq,
                channels=channels,
                yaw_i16=encode_current_yaw_i16(current_yaw_deg),
                des_yaw_i16=encode_target_yaw_i16(target_yaw_deg),
            )
            if not ok:
                continue
            time.sleep(0.02)

        print(f"[climb:SETTLE] keep sending blank frames for {CLIMB_POST_TRIGGER_SETTLE_SEC:.2f}s.")
        settle_deadline = time.time() + CLIMB_POST_TRIGGER_SETTLE_SEC
        while time.time() < settle_deadline:
            _, settle_yaw_deg = _get_robot_pose_from_tf_cache(tf_node)
            if settle_yaw_deg is None:
                settle_yaw_deg = current_yaw_deg

            channels = compose_chassis_channels(lateral_cmd=0, forward_cmd=0, rotation_cmd=0)
            sock, seq, ok = _send_control_frame(
                sock=sock,
                seq=seq,
                channels=channels,
                yaw_i16=encode_current_yaw_i16(settle_yaw_deg),
                des_yaw_i16=encode_target_yaw_i16(target_yaw_deg),
            )
            if not ok:
                continue

            maybe_print(
                "SETTLE",
                f"current_yaw={settle_yaw_deg:.2f} deg | "
                f"remaining={max(0.0, settle_deadline - time.time()):.2f}s"
            )
            time.sleep(0.02)

        print(
            f"[climb:RECOVER] send normal forward frames for "
            f"{CLIMB_RECOVERY_DURATION_SEC:.2f}s | ch2={CLIMB_RECOVERY_FORWARD_CMD}"
        )
        recovery_deadline = time.time() + CLIMB_RECOVERY_DURATION_SEC
        while time.time() < recovery_deadline:
            _, recovery_yaw_deg = _get_robot_pose_from_tf_cache(tf_node)
            if recovery_yaw_deg is None:
                recovery_yaw_deg = current_yaw_deg

            channels = compose_chassis_channels(
                lateral_cmd=0,
                forward_cmd=CLIMB_RECOVERY_FORWARD_CMD,
                rotation_cmd=0,
            )
            sock, seq, ok = _send_control_frame(
                sock=sock,
                seq=seq,
                channels=channels,
                yaw_i16=encode_current_yaw_i16(recovery_yaw_deg),
                des_yaw_i16=encode_target_yaw_i16(target_yaw_deg),
            )
            if not ok:
                continue

            maybe_print(
                "RECOVER",
                f"current_yaw={recovery_yaw_deg:.2f} deg | "
                f"remaining={max(0.0, recovery_deadline - time.time()):.2f}s"
            )
            time.sleep(0.02)

        print("[climb:CHECK] recovery finished, continue to next stage.")
        return sock, seq, True

    finally:
        if owns_tf_cache_node:
            if tf_stop_event is not None:
                tf_stop_event.set()
            if tf_thread is not None and tf_thread.is_alive():
                tf_thread.join(timeout=1.0)
            if tf_node is not None:
                try:
                    tf_node.destroy_node()
                except Exception:
                    pass

        if initialized_rclpy and rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass
#----------------------------------------------------------------------------------------------------------------------
