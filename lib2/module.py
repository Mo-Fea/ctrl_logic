import threading
import time
import sys

from lib2 import tools
from lib2 import move as move_lib
from lib2 import position_backend

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
PRE_DESCEND_ADJUST_DISTANCE = 0.0 #m
KFS_SUCTION_CHANNEL_INDEX = 4
KFS_MODE_CHANNEL_INDEX = 5
KFS_POSE_CHANNEL_INDEX = 6
KFS_TRIGGER_CHANNEL_INDEX = 7
KFS_MODE_VALUE = 2
KFS_SUCTION_OFF_VALUE = 1
KFS_SUCTION_ON_VALUE = 3
KFS_TRIGGER_IDLE_VALUE = 1
DEFAULT_KFS_SUCTION_EDGE_ARM_SEC = 0.1
DEFAULT_KFS_SUCTION_EDGE_HOLD_SEC = 0.1
DEFAULT_KFS_SUCTION_HOLD_SEC = 3.0
ACTION_MATRIX_COLUMNS = [
    "from_pos",
    "to_pos",
    "move_dir",
    "height_action",
    "grab_action",
]
ACTION_MATRIX_ROW_SIZE = 5


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
        [-1, 2, 0, 0, pre_entrance_x, pre_entrance_y],
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

    module.py 和 move.py 都会缓存 position_lib；切换雷达类型时需要同时刷新，
    避免 init() 后的位置线程或移动逻辑继续使用旧后端。
    """
    position_backend.set_lidar_type(lidar_type)
    backend = position_backend.get_position_backend()

    global position_lib, STAIR_HEIGHT_RELATION_MATRIX
    position_lib = backend
    move_lib.position_lib = backend
    STAIR_HEIGHT_RELATION_MATRIX = build_stair_height_relation_matrix()
    return backend


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

def init(
    seq=0,
    hz=70.0,
    lidar_type=position_backend.LIDAR_TYPE_ODIN,
    topic='/odin1/flag1',
    tcp_ip=tools.TCP_IP,
    tcp_port=tools.TCP_PORT,
    retry_interval=tools.CONNECT_RETRY_INTERVAL,
    wait_relocalization=True,
    wait_poll_interval=0.01,
    auto_destroy_relocalization_listener=True,
    wait_sender_ready=True,
    sender_ready_timeout_sec=2.0,
):
    """
    一站式初始化:
    - 连接下位机
    - 启动重定位监听
    - 可选等待重定位成功后再返回
    - 可选在重定位成功后自动销毁重定位监听
    - 创建并启动 frame_thread
    - 可选等待 frame_thread 首帧发送成功后再返回

    lidar_type:
      1: odin
      2: mid360

    返回:
      sender: 已启动的 frame_thread
      get_flag: 可调用函数，返回当前重定位flag(bool)
      flag_node: ROS节点对象
      flag_thread: 后台spin线程
      flag_stop_event: 停止事件
    """
    configure_position_backend(lidar_type)
    tools.relocalization_flag = False
    sock = tools.connect(
        tcp_ip=tcp_ip,
        tcp_port=tcp_port,
        retry_interval=retry_interval,
    )

    sender = tools.frame_thread(
        sock=sock,
        seq=seq,
        hz=hz,
        connect_func=tools.connect,
        tcp_ip=tcp_ip,
        tcp_port=tcp_port,
        retry_interval=retry_interval,
    )
    if position_backend.is_mid360():
        # mid360 模式下初始目标航向角使用协议特殊值0，表示不执行旋转控制。
        sender.set_des_yaw_i16(0)
    else:
        # odin 模式下重定位前需要原地旋转；显式设置后不会被 relocalization_flag 自动覆盖。
        sender.set_des_yaw_i16(tools.yaw_deg_to_i16(90.0))

    sender_started = False

    def start_sender_once():
        nonlocal sender_started
        if sender_started:
            return
        sender.start()
        sender_started = True
        if wait_sender_ready:
            ready = sender.wait_until_first_send(timeout_sec=sender_ready_timeout_sec)
            if not ready:
                sender.stop(send_stop=False)
                tools.socket_close(sender.sock)
                raise RuntimeError(
                    "frame_thread did not send first frame before "
                    f"{float(sender_ready_timeout_sec):.2f}s timeout"
                )
        state = sender.get_state()
        print(
            "frame_thread started: "
            f"yaw_i16={state['yaw_i16']} "
            f"des_yaw_i16={state['des_yaw_i16']} "
            f"channels={state['channels']}"
        )

    # odin 需要先持续发帧，利用默认 90deg 目标角在重定位前原地旋转。
    # 因此 odin 下先启动 frame_thread，再启动重定位监听。
    # mid360 保持原逻辑：先等待重定位，再启动发送线程，且初始目标角为0停止旋转控制。
    if (not wait_relocalization) or (not position_backend.is_mid360()):
        start_sender_once()

    get_flag, flag_node, flag_thread, flag_stop_event = tools.relocalization_conformation(
        topic=topic
    )

    if wait_relocalization:
        print(f"Waiting for relocalization flag on {topic}...")
        while not tools.relocalization_flag:
            time.sleep(wait_poll_interval)
        if auto_destroy_relocalization_listener:
            tools.destroy_ros2_thread(
                node=flag_node,
                spin_thread=flag_thread,
                stop_event=flag_stop_event,
                shutdown_rclpy=False,
            )
            flag_node = None
            flag_thread = None
            flag_stop_event = None

    start_sender_once()

    return sender, get_flag, flag_node, flag_thread, flag_stop_event


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


def _remaining_timeout_sec(deadline):
    if deadline is None:
        return None
    return max(0.0, float(deadline) - time.time())


def move_to_des(
    sender,
    position_runtime,
    odom_runtime,
    x,
    y,
    target_deg,
    v=600,
    rotate_tolerance_deg=5.0,
    final_direction_timeout_sec=None,
    move_timeout_sec=None,
    total_timeout_sec=None,
    wait_position_timeout_sec=2.0,
    reference="robot",
):
    """
    组合动作：
    1. 先根据当前 reference 参考点位置计算目标点方向绝对航向角
    2. 设置一次目标航向角
    3. 阻塞旋转到该目标航向角（默认 +-5deg）
    4. 设置一次前进速度起步
    5. 调用 move.move_to_target(...) 阻塞移动到目标点
    6. target_deg 不是 None 时，阻塞等待最终航向进入阈值范围
       target_deg 为 None 时，到点后停止旋转控制，不做最终旋转
    """
    reference = str(reference).lower()
    if reference not in ("robot", "weapon"):
        raise ValueError(f"reference must be 'robot' or 'weapon', got {reference}")

    total_deadline = None if total_timeout_sec is None else (time.time() + float(total_timeout_sec))
    wait_deadline = time.time() + float(wait_position_timeout_sec)
    if total_deadline is not None:
        wait_deadline = min(wait_deadline, total_deadline)
    current_pose = move_lib.get_reference_pose(position_runtime, reference=reference)
    while current_pose is None and time.time() < wait_deadline:
        time.sleep(0.02)
        current_pose = move_lib.get_reference_pose(position_runtime, reference=reference)

    if current_pose is None:
        return None

    initial_target_yaw_deg = position_lib.cal_target_yaw_deg(
        target_x=x,
        target_y=y,
        robot_x=current_pose["x"],
        robot_y=current_pose["y"],
    )

    zero_channels = tools.compose_channels(lateral_cmd=0, forward_cmd=0, rotation_cmd=0)
    sender.set_channels_and_des_yaw_i16(
        zero_channels,
        move_lib.encode_target_yaw_i16(initial_target_yaw_deg),
    )

    rotate_result = move_lib.rotate_to_target_yaw_segmented(
        sender=sender,
        position_runtime=position_runtime,
        target_yaw_deg=initial_target_yaw_deg,
        tolerance_deg=rotate_tolerance_deg,
        timeout_sec=_remaining_timeout_sec(total_deadline),
    )
    if rotate_result is None or rotate_result.get("timed_out"):
        return None

    start_channels = tools.compose_channels(lateral_cmd=0, forward_cmd=int(v), rotation_cmd=0)
    sender.set_channels_and_des_yaw_i16(
        start_channels,
        move_lib.encode_target_yaw_i16(initial_target_yaw_deg),
    )

    move_result = move_lib.move_to_target(
        sender=sender,
        position_runtime=position_runtime,
        odom_runtime=odom_runtime,
        target_x=x,
        target_y=y,
        final_target_yaw_deg=target_deg,
        cruise_forward_cmd=v,
        timeout_sec=(
            _remaining_timeout_sec(total_deadline)
            if total_deadline is not None
            else move_timeout_sec
        ),
        reference=reference,
    )
    if move_result is None or move_result.get("timed_out"):
        return None

    final_direction_result = None
    if target_deg is not None:
        final_direction_result = move_lib.rotate_to_target_yaw_segmented(
            sender=sender,
            position_runtime=position_runtime,
            target_yaw_deg=target_deg,
            tolerance_deg=move_lib.DEFAULT_DIRECTION_THRESHOLD_DEG,
            timeout_sec=(
                _remaining_timeout_sec(total_deadline)
                if total_deadline is not None
                else final_direction_timeout_sec
            ),
        )
        if final_direction_result is None or final_direction_result.get("timed_out"):
            return None

    return {
        "initial_target_yaw_deg": float(initial_target_yaw_deg),
        "reference": reference,
        "rotate_result": rotate_result,
        "move_result": move_result,
        "final_direction_result": final_direction_result,
    }


def move_backward_to_des(
    sender,
    position_runtime,
    odom_runtime,
    x,
    y,
    target_deg,
    v=600,
    rotate_tolerance_deg=5.0,
    final_direction_timeout_sec=None,
    move_timeout_sec=None,
    total_timeout_sec=None,
    wait_position_timeout_sec=2.0,
    reference="robot",
):
    """
    组合式倒退到点动作：
    1. 先根据当前 reference 参考点位置计算目标点方向绝对航向角
    2. 初始航向取目标点方向 + 180deg，使车尾正对目标点
    3. 阻塞旋转到该倒退航向角（默认 +-5deg）
    4. 设置一次负 ch2 起步
    5. 调用 move.move_backward_to_target(...) 阻塞倒退移动到目标点
    6. target_deg 不是 None 时，按原始 target_deg 阻塞旋转到最终航向；
       target_deg 为 None 时，到点后停止旋转控制，不做最终旋转
    """
    reference = str(reference).lower()
    if reference not in ("robot", "weapon"):
        raise ValueError(f"reference must be 'robot' or 'weapon', got {reference}")

    total_deadline = None if total_timeout_sec is None else (time.time() + float(total_timeout_sec))
    wait_deadline = time.time() + float(wait_position_timeout_sec)
    if total_deadline is not None:
        wait_deadline = min(wait_deadline, total_deadline)
    current_pose = move_lib.get_reference_pose(position_runtime, reference=reference)
    while current_pose is None and time.time() < wait_deadline:
        time.sleep(0.02)
        current_pose = move_lib.get_reference_pose(position_runtime, reference=reference)

    if current_pose is None:
        return None

    initial_target_direction_deg = position_lib.cal_target_yaw_deg(
        target_x=x,
        target_y=y,
        robot_x=current_pose["x"],
        robot_y=current_pose["y"],
    )
    initial_backward_yaw_deg = move_lib.normalize_yaw_deg(
        initial_target_direction_deg + 180.0
    )

    zero_channels = tools.compose_channels(lateral_cmd=0, forward_cmd=0, rotation_cmd=0)
    sender.set_channels_and_des_yaw_i16(
        zero_channels,
        move_lib.encode_target_yaw_i16(initial_backward_yaw_deg),
    )

    rotate_result = move_lib.rotate_to_target_yaw_segmented(
        sender=sender,
        position_runtime=position_runtime,
        target_yaw_deg=initial_backward_yaw_deg,
        tolerance_deg=rotate_tolerance_deg,
        timeout_sec=_remaining_timeout_sec(total_deadline),
    )
    if rotate_result is None or rotate_result.get("timed_out"):
        return None

    backward_cmd = -abs(int(v))
    start_channels = tools.compose_channels(lateral_cmd=0, forward_cmd=backward_cmd, rotation_cmd=0)
    sender.set_channels_and_des_yaw_i16(
        start_channels,
        move_lib.encode_target_yaw_i16(initial_backward_yaw_deg),
    )

    move_result = move_lib.move_backward_to_target(
        sender=sender,
        position_runtime=position_runtime,
        odom_runtime=odom_runtime,
        target_x=x,
        target_y=y,
        final_target_yaw_deg=target_deg,
        cruise_backward_cmd=backward_cmd,
        timeout_sec=(
            _remaining_timeout_sec(total_deadline)
            if total_deadline is not None
            else move_timeout_sec
        ),
        reference=reference,
    )
    if move_result is None or move_result.get("timed_out"):
        return None

    final_direction_result = None
    if target_deg is not None:
        final_direction_result = move_lib.rotate_to_target_yaw_segmented(
            sender=sender,
            position_runtime=position_runtime,
            target_yaw_deg=target_deg,
            tolerance_deg=move_lib.DEFAULT_DIRECTION_THRESHOLD_DEG,
            timeout_sec=(
                _remaining_timeout_sec(total_deadline)
                if total_deadline is not None
                else final_direction_timeout_sec
            ),
        )
        if final_direction_result is None or final_direction_result.get("timed_out"):
            return None

    return {
        "initial_target_direction_deg": float(initial_target_direction_deg),
        "initial_backward_yaw_deg": float(initial_backward_yaw_deg),
        "reference": reference,
        "rotate_result": rotate_result,
        "move_result": move_result,
        "final_direction_result": final_direction_result,
    }


def move_weapon_to_des(
    sender,
    position_runtime,
    odom_runtime,
    x,
    y,
    target_deg,
    **kwargs,
):
    """
    以 weapon/夹爪为到点参考的 move_to_des 包装。
    """
    kwargs.pop("reference", None)
    return move_to_des(
        sender=sender,
        position_runtime=position_runtime,
        odom_runtime=odom_runtime,
        x=x,
        y=y,
        target_deg=target_deg,
        reference="weapon",
        **kwargs,
    )


def adjust_position(
    sender,
    position_runtime,
    odom_runtime,
    move_type,
    direction,
    stair_id,
    height_relation,
    adjust_distance=PRE_DESCEND_ADJUST_DISTANCE,
    move_speed=600,
    final_direction_timeout_sec=None,
    move_timeout_sec=None,
    total_timeout_sec=None,
):
    """
    通用位置微调动作。

    move_type:
      1: 前进微调，调用 move_to_des(...)
      2: 后退微调，调用 move_backward_to_des(...)

    stair_id:
      台阶编号。函数会按 STAIR_HEIGHT_RELATION_MATRIX 第一列查找编号，
      再使用该行的 x/y 作为微调起点。

    direction:
      1: current_x + adjust_distance
      2: current_y + adjust_distance
      3: current_y - adjust_distance
      4: current_x - adjust_distance

    height_relation:
      1: 微调方向台阶较高，先旋转到目标航向，再 ch2=200 前进 2s，完成微调。
      2: 微调方向台阶较低，执行原来的按坐标微调逻辑。

    direction 同时通过 tools.direction_int_to_yaw_deg(...) 转成微调过程中的目标航向角。
    """
    move_type = int(move_type)
    direction = int(direction)
    height_relation = int(height_relation)
    if move_type not in (1, 2):
        raise ValueError(f"move_type must be 1(forward) or 2(backward), got {move_type}")
    if direction not in (1, 2, 3, 4):
        raise ValueError(f"direction must be 1, 2, 3 or 4, got {direction}")
    if height_relation not in (1, 2):
        print(f"{adjust_position.__name__}输入错误: height_relation={height_relation}")
        sys.exit(1)

    target_deg = tools.direction_int_to_yaw_deg(direction)
    adjust_distance = float(adjust_distance)
    stair_id = int(stair_id)
    stair_matrix = get_stair_matrix()
    stair_matrix_index = tools.stair_id_to_matrix_index(
        stair_id,
        stair_matrix=stair_matrix,
    )
    stair_row = stair_matrix[stair_matrix_index]
    current_x = float(stair_row[4])
    current_y = float(stair_row[5])
    adjust_x = float(current_x)
    adjust_y = float(current_y)

    if height_relation == 1:
        rotate_result = move_lib.rotate_to_target_yaw_segmented(
            sender=sender,
            position_runtime=position_runtime,
            target_yaw_deg=target_deg,
            timeout_sec=final_direction_timeout_sec,
        )
        forward_channels = tools.compose_channels(
            lateral_cmd=0,
            forward_cmd=200,
            rotation_cmd=0,
        )
        drive_result = move_lib.drive_with_channels_for_duration(
            sender=sender,
            channels=forward_channels,
            duration_sec=2.0,
            target_yaw_deg=target_deg,
            brake_reverse_cmd=0,
            brake_duration_sec=0.0,
        )
        return {
            "move_type": int(move_type),
            "direction": int(direction),
            "height_relation": int(height_relation),
            "stair_id": int(stair_id),
            "stair_matrix_index": int(stair_matrix_index),
            "target_deg": float(target_deg),
            "current_x": float(current_x),
            "current_y": float(current_y),
            "adjust_x": float(adjust_x),
            "adjust_y": float(adjust_y),
            "adjust_distance": float(adjust_distance),
            "rotate_result": rotate_result,
            "drive_result": drive_result,
            "move_result": drive_result,
        }

    if direction == 1:
        adjust_x += adjust_distance
    elif direction == 2:
        adjust_y += adjust_distance
    elif direction == 3:
        adjust_y -= adjust_distance
    elif direction == 4:
        adjust_x -= adjust_distance
    else:
        raise ValueError(f"direction must be 1, 2, 3 or 4, got {direction}")

    if move_type == 1:
        move_result = move_to_des(
            sender=sender,
            position_runtime=position_runtime,
            odom_runtime=odom_runtime,
            x=adjust_x,
            y=adjust_y,
            target_deg=target_deg,
            v=move_speed,
            final_direction_timeout_sec=final_direction_timeout_sec,
            move_timeout_sec=move_timeout_sec,
            total_timeout_sec=total_timeout_sec,
        )
    elif move_type == 2:
        move_result = move_backward_to_des(
            sender=sender,
            position_runtime=position_runtime,
            odom_runtime=odom_runtime,
            x=adjust_x,
            y=adjust_y,
            target_deg=target_deg,
            v=move_speed,
            final_direction_timeout_sec=final_direction_timeout_sec,
            move_timeout_sec=move_timeout_sec,
            total_timeout_sec=total_timeout_sec,
        )
    else:
        raise ValueError(f"move_type must be 1(forward) or 2(backward), got {move_type}")

    return {
        "move_type": int(move_type),
        "direction": int(direction),
        "height_relation": int(height_relation),
        "stair_id": int(stair_id),
        "stair_matrix_index": int(stair_matrix_index),
        "target_deg": float(target_deg),
        "current_x": float(current_x),
        "current_y": float(current_y),
        "adjust_x": float(adjust_x),
        "adjust_y": float(adjust_y),
        "adjust_distance": float(adjust_distance),
        "move_result": move_result,
    }


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


def kfs_pose_id_from_height_relation(height_relation):
    height_relation = int(height_relation)
    if height_relation == 1:
        return 1
    if height_relation == 2:
        return 2
    print(f"{kfs_pose_id_from_height_relation.__name__}输入错误: 高低关系={height_relation}")
    sys.exit(1)


def _kfs_suction_channels(suction_value, pose_id=1, trigger_value=KFS_TRIGGER_IDLE_VALUE):
    channels = tools.compose_channels(lateral_cmd=0, forward_cmd=0, rotation_cmd=0)
    channels[KFS_SUCTION_CHANNEL_INDEX] = int(suction_value)
    channels[KFS_MODE_CHANNEL_INDEX] = KFS_MODE_VALUE
    channels[KFS_POSE_CHANNEL_INDEX] = int(pose_id)
    channels[KFS_TRIGGER_CHANNEL_INDEX] = int(trigger_value)
    return channels


def set_kfs_suction(
    sender,
    suction_on=True,
    pose_id=1,
    edge_arm_sec=DEFAULT_KFS_SUCTION_EDGE_ARM_SEC,
    edge_hold_sec=DEFAULT_KFS_SUCTION_EDGE_HOLD_SEC,
    loop_interval_sec=0.02,
):
    """
    KFS 吸盘吸取/释放边沿控制。

    suction_on=True:  ch4 1 -> 3
    suction_on=False: ch4 3 -> 1
    """
    arm_value = KFS_SUCTION_OFF_VALUE if suction_on else KFS_SUCTION_ON_VALUE
    fire_value = KFS_SUCTION_ON_VALUE if suction_on else KFS_SUCTION_OFF_VALUE

    arm_channels = _kfs_suction_channels(arm_value, pose_id=pose_id)
    arm_deadline = time.time() + float(edge_arm_sec)
    while time.time() < arm_deadline:
        sender.set_channels_and_des_yaw_i16(arm_channels, 0)
        time.sleep(float(loop_interval_sec))

    fire_channels = _kfs_suction_channels(fire_value, pose_id=pose_id)
    fire_deadline = time.time() + float(edge_hold_sec)
    while time.time() < fire_deadline:
        sender.set_channels_and_des_yaw_i16(fire_channels, 0)
        time.sleep(float(loop_interval_sec))

    sender.set_channels_and_des_yaw_i16(fire_channels, 0)
    return {
        "suction_on": bool(suction_on),
        "pose_id": int(pose_id),
        "arm_channels": arm_channels,
        "fire_channels": fire_channels,
        "edge_arm_sec": float(edge_arm_sec),
        "edge_hold_sec": float(edge_hold_sec),
        "completed": True,
    }


def wait_with_kfs_suction(
    sender,
    duration_sec,
    pose_id=1,
    loop_interval_sec=0.02,
):
    channels = _kfs_suction_channels(KFS_SUCTION_ON_VALUE, pose_id=pose_id)
    deadline = time.time() + float(duration_sec)
    while time.time() < deadline:
        sender.set_channels_and_des_yaw_i16(channels, 0)
        time.sleep(float(loop_interval_sec))
    sender.set_channels_and_des_yaw_i16(channels, 0)
    return {
        "channels": channels,
        "duration_sec": float(duration_sec),
        "pose_id": int(pose_id),
        "completed": True,
    }


def fetch_and_store_kfs(
    sender,
    position_runtime,
    odom_runtime,
    stair_id,
    direction,
    adjust_distance=PRE_DESCEND_ADJUST_DISTANCE,
    move_speed=600,
    suction_hold_sec=DEFAULT_KFS_SUCTION_HOLD_SEC,
    loop_interval_sec=0.02,
):
    """
    KFS 方块吸取并存储组合流程。

    1. 按 stair_id + direction 正向微调到吸取位置。
    2. 根据 STAIR_HEIGHT_RELATION_MATRIX 的高低关系选择 1/2 抓取姿态。
    3. 吸取并保持 suction_hold_sec。
    4. 执行 3 过渡态，再执行 4 存储态。
    5. 释放吸盘，再执行 0 态。
    6. 倒退回当前 stair_id 的中心坐标，目标方向为 0。
    """
    stair_id = int(stair_id)
    direction = int(direction)
    stair_x, stair_y = get_stair_xy(stair_id)
    height_relation = get_stair_height_relation(stair_id, direction)

    adjust_result = adjust_position(
        sender=sender,
        position_runtime=position_runtime,
        odom_runtime=odom_runtime,
        move_type=1,
        direction=direction,
        stair_id=stair_id,
        height_relation=height_relation,
        adjust_distance=adjust_distance,
        move_speed=move_speed,
    )
    if adjust_result["move_result"] is None:
        return {
            "completed": False,
            "failed_step": "adjust_position",
            "adjust_result": adjust_result,
        }

    grab_pose_id = kfs_pose_id_from_height_relation(height_relation)

    grab_pose_result = move_lib.control_kfs_pose(
        sender=sender,
        pose_id=grab_pose_id,
        suction_ch4=KFS_SUCTION_OFF_VALUE,
    )

    suction_result = set_kfs_suction(
        sender=sender,
        suction_on=True,
        pose_id=grab_pose_id,
        loop_interval_sec=loop_interval_sec,
    )
    suction_hold_result = wait_with_kfs_suction(
        sender=sender,
        duration_sec=suction_hold_sec,
        pose_id=grab_pose_id,
        loop_interval_sec=loop_interval_sec,
    )

    transition_pose_result = move_lib.control_kfs_pose(
        sender=sender,
        pose_id=3,
        suction_ch4=KFS_SUCTION_ON_VALUE,
    )
    store_pose_result = move_lib.control_kfs_pose(
        sender=sender,
        pose_id=4,
        suction_ch4=KFS_SUCTION_ON_VALUE,
    )

    release_result = set_kfs_suction(
        sender=sender,
        suction_on=False,
        pose_id=4,
        loop_interval_sec=loop_interval_sec,
    )
    zero_pose_result = move_lib.control_kfs_pose(
        sender=sender,
        pose_id=0,
        suction_ch4=KFS_SUCTION_OFF_VALUE,
    )

    return_move_result = move_backward_to_des(
        sender=sender,
        position_runtime=position_runtime,
        odom_runtime=odom_runtime,
        x=stair_x,
        y=stair_y,
        target_deg=0.0,
        v=move_speed,
    )

    return {
        "completed": return_move_result is not None,
        "stair_id": int(stair_id),
        "direction": int(direction),
        "height_relation": int(height_relation),
        "grab_pose_id": int(grab_pose_id),
        "stair_x": float(stair_x),
        "stair_y": float(stair_y),
        "adjust_result": adjust_result,
        "grab_pose_result": grab_pose_result,
        "suction_result": suction_result,
        "suction_hold_result": suction_hold_result,
        "transition_pose_result": transition_pose_result,
        "store_pose_result": store_pose_result,
        "release_result": release_result,
        "zero_pose_result": zero_pose_result,
        "return_move_result": return_move_result,
    }


def fetch_weapon(
    sender,
    position_runtime,
    odom_runtime,
    weapon_id,
    v=600,
    final_target_yaw_deg=0.01,
    weapon_mode_settle_sec=0.3,
    grab_arm_sec=0.3,
    grab_hold_sec=1.0,
    lift_hold_sec=1.0,
    first_rotate_yaw_deg=90.0,
    retreat_forward_cmd=-100,
    retreat_stop_y=None,
    retreat_timeout_sec=None,
    final_rotate_yaw_deg=-90.0,
    retreat_loop_interval_sec=0.02,
):
    """
    根据 weapon_id 选择目标点，移动 weapon/夹爪到目标点后执行夹取并抬起。

    weapon_id 对应目标点从当前 position 后端的 WEAPON_TARGETS 读取。
    retreat_stop_y 默认从当前 position 后端的 WEAPON_RETREAT_STOP_Y 读取；
    调用时显式传入 retreat_stop_y 可临时覆盖。
    """
    weapon_targets = getattr(position_lib, "WEAPON_TARGETS", None)
    if weapon_targets is None:
        raise AttributeError(f"{position_lib.__name__} must define WEAPON_TARGETS")
    if retreat_stop_y is None:
        retreat_stop_y = getattr(position_lib, "WEAPON_RETREAT_STOP_Y", None)
    if retreat_stop_y is None:
        raise AttributeError(f"{position_lib.__name__} must define WEAPON_RETREAT_STOP_Y")

    weapon_id = int(weapon_id)
    if weapon_id not in weapon_targets:
        valid_weapon_ids = sorted(int(key) for key in weapon_targets)
        raise ValueError(f"weapon_id must be one of {valid_weapon_ids}, got {weapon_id}")

    des_weapon = weapon_targets[weapon_id]
    des_x, des_y = des_weapon

    move_result = move_lib.move_to_target(
        sender=sender,
        position_runtime=position_runtime,
        odom_runtime=odom_runtime,
        target_x=des_x,
        target_y=des_y,
        final_target_yaw_deg=final_target_yaw_deg,
        cruise_forward_cmd=v,
        reference="weapon",
    )

    def set_weapon_state(ch1=0, ch4=1, forward_cmd=0, des_yaw_i16=0):
        channels = tools.compose_channels(lateral_cmd=0, forward_cmd=int(forward_cmd), rotation_cmd=0)
        channels[5] = 3
        channels[4] = int(ch4)
        channels[1] = int(ch1)
        sender.set_channels_and_des_yaw_i16(channels, int(des_yaw_i16))
        return channels

    mode_channels = set_weapon_state(ch1=0, ch4=1)
    time.sleep(float(weapon_mode_settle_sec))

    grab_arm_channels = set_weapon_state(ch1=0, ch4=1)
    time.sleep(float(grab_arm_sec))

    grab_fire_channels = set_weapon_state(ch1=0, ch4=3)
    time.sleep(float(grab_hold_sec))

    lift_channels = set_weapon_state(ch1=100, ch4=3)
    time.sleep(float(lift_hold_sec))

    first_rotate_result = move_lib.rotate_to_target_yaw_segmented(
        sender=sender,
        position_runtime=position_runtime,
        target_yaw_deg=first_rotate_yaw_deg,
    )

    retreat_started_at = time.time()
    retreat_deadline = (
        None
        if retreat_timeout_sec is None
        else retreat_started_at + float(retreat_timeout_sec)
    )
    retreat_result = None
    retreat_des_yaw_i16 = move_lib.encode_target_yaw_i16(first_rotate_yaw_deg)

    while True:
        weapon_pose = position_runtime.get_weapon_pose()
        if weapon_pose is not None:
            weapon_y = float(weapon_pose["y"])
            retreat_result = {
                "weapon_y": weapon_y,
                "retreat_reference": "weapon",
                "retreat_stop_y": float(retreat_stop_y),
                "retreat_forward_cmd": int(retreat_forward_cmd),
                "elapsed_sec": float(time.time() - retreat_started_at),
            }
            if weapon_y < float(retreat_stop_y):
                break

        set_weapon_state(
            ch1=100,
            ch4=3,
            forward_cmd=retreat_forward_cmd,
            des_yaw_i16=retreat_des_yaw_i16,
        )

        if retreat_deadline is not None and time.time() >= retreat_deadline:
            if retreat_result is None:
                retreat_result = {
                    "weapon_y": None,
                    "retreat_reference": "weapon",
                    "retreat_stop_y": float(retreat_stop_y),
                    "retreat_forward_cmd": int(retreat_forward_cmd),
                    "elapsed_sec": float(time.time() - retreat_started_at),
                }
            retreat_result["timed_out"] = True
            break

        time.sleep(float(retreat_loop_interval_sec))

    retreat_stop_channels = set_weapon_state(ch1=100, ch4=3, forward_cmd=0)

    final_rotate_result = move_lib.rotate_to_target_yaw_segmented(
        sender=sender,
        position_runtime=position_runtime,
        target_yaw_deg=final_rotate_yaw_deg,
    )
    final_hold_channels = set_weapon_state(ch1=100, ch4=3, forward_cmd=0)

    return {
        "weapon_id": int(weapon_id),
        "des_weapon": {
            "x": float(des_x),
            "y": float(des_y),
        },
        "move_result": move_result,
        "grab_result": {
            "mode_channels": mode_channels,
            "arm_channels": grab_arm_channels,
            "fire_channels": grab_fire_channels,
            "lift_channels": lift_channels,
            "weapon_mode_settle_sec": float(weapon_mode_settle_sec),
            "grab_arm_sec": float(grab_arm_sec),
            "grab_hold_sec": float(grab_hold_sec),
            "lift_hold_sec": float(lift_hold_sec),
            "completed": True,
        },
        "first_rotate_result": first_rotate_result,
        "retreat_result": retreat_result,
        "retreat_stop_channels": retreat_stop_channels,
        "final_rotate_result": final_rotate_result,
        "final_hold_channels": final_hold_channels,
    }


def climb(
    sender,
    position_runtime,
    odom_runtime,
    direction1,
    direction2,
    x,
    y,
    pre_climb_forward_cmd=300,
    pre_climb_duration_sec=3.0,
    move_speed=600,
):
    """
    组合式上楼梯动作：
    1. direction1/direction2 转成 des_deg1/des_deg2
    2. 原地旋转到 des_deg1
    3. ch2=pre_climb_forward_cmd 前进 pre_climb_duration_sec
    4. 调用 move.climb(...) 阻塞执行半自动上楼梯
    5. 调用 move_to_des(...) 移动到目标点 (x, y)，最终朝向 des_deg2
    """
    des_deg1 = tools.direction_int_to_yaw_deg(direction1)
    des_deg2 = tools.direction_int_to_yaw_deg(direction2)

    rotate_result = move_lib.rotate_to_target_yaw_segmented(
        sender=sender,
        position_runtime=position_runtime,
        target_yaw_deg=des_deg1,
    )

    forward_channels = tools.compose_channels(
        lateral_cmd=0,
        forward_cmd=int(pre_climb_forward_cmd),
        rotation_cmd=0,
    )
    pre_climb_drive_result = move_lib.drive_with_channels_for_duration(
        sender=sender,
        channels=forward_channels,
        duration_sec=pre_climb_duration_sec,
        target_yaw_deg=des_deg1,
    )

    climb_result = move_lib.climb(
        sender=sender,
        position_runtime=position_runtime,
    )

    move_result = move_to_des(
        sender=sender,
        position_runtime=position_runtime,
        odom_runtime=odom_runtime,
        x=x,
        y=y,
        target_deg=des_deg2,
        v=move_speed,
    )

    return {
        "des_deg1": float(des_deg1),
        "des_deg2": float(des_deg2),
        "rotate_result": rotate_result,
        "pre_climb_drive_result": pre_climb_drive_result,
        "climb_result": climb_result,
        "move_result": move_result,
    }


def descend(
    sender,
    position_runtime,
    odom_runtime,
    direction1,
    direction2,
    current_x,
    current_y,
    des_x,
    des_y,
    adjust_distance=PRE_DESCEND_ADJUST_DISTANCE,
    trigger_arm_sec=0.1,
    trigger_hold_sec=2.0,
    loop_interval_sec=0.02,
    move_speed=600,
    timeout_sec=None,
):
    """
    组合式下楼梯动作：
    1. direction1 取反方向并转换为下楼前对正角。
    2. 阻塞旋转到该反方向角。
    3. 调用 move.descend(...) 执行底层阻塞式下楼梯控制。
    4. 调用 move_backward_to_des(...) 倒退移动到 (des_x, des_y)，最终朝向 direction2 对应角度。
    5. 如果传入 timeout_sec，则总流程超时后打印“下楼梯错误”并终止程序。
    """
    started_at = time.time()
    deadline = None if timeout_sec is None else (started_at + float(timeout_sec))
    direction1 = int(direction1)
    direction2 = int(direction2)
    if direction1 not in (1, 2, 3, 4):
        raise ValueError(f"direction1 must be 1, 2, 3 or 4, got {direction1}")
    if direction2 not in (1, 2, 3, 4):
        raise ValueError(f"direction2 must be 1, 2, 3 or 4, got {direction2}")

    des_deg1 = tools.direction_int_to_yaw_deg(direction1)
    des_deg2 = tools.direction_int_to_yaw_deg(direction2)
    opposite_direction_map = {
        1: 4,
        2: 3,
        3: 2,
        4: 1,
    }
    descend_align_direction = opposite_direction_map[direction1]
    descend_align_deg = tools.direction_int_to_yaw_deg(descend_align_direction)

    align_result = move_lib.rotate_to_target_yaw_segmented(
        sender=sender,
        position_runtime=position_runtime,
        target_yaw_deg=descend_align_deg,
        timeout_sec=_remaining_timeout_sec(deadline),
    )
    if align_result is None or align_result.get("timed_out"):
        print("下楼梯错误")
        sys.exit(1)
    if deadline is not None and time.time() >= deadline:
        print("下楼梯错误")
        sys.exit(1)

    descend_result = move_lib.descend(
        sender=sender,
        position_runtime=position_runtime,
        trigger_hold_sec=trigger_hold_sec,
        trigger_arm_sec=trigger_arm_sec,
        loop_interval_sec=loop_interval_sec,
    )
    if descend_result is None:
        print("下楼梯错误")
        sys.exit(1)
    if deadline is not None and time.time() >= deadline:
        print("下楼梯错误")
        sys.exit(1)

    move_timeout_sec = None if deadline is None else max(0.0, deadline - time.time())
    move_result = move_backward_to_des(
        sender=sender,
        position_runtime=position_runtime,
        odom_runtime=odom_runtime,
        x=des_x,
        y=des_y,
        target_deg=des_deg2,
        v=move_speed,
        total_timeout_sec=move_timeout_sec,
    )
    if move_result is None:
        print("下楼梯错误")
        sys.exit(1)

    if deadline is not None and time.time() >= deadline:
        print("下楼梯错误")
        sys.exit(1)

    return {
        "direction1": int(direction1),
        "direction2": int(direction2),
        "des_deg1": float(des_deg1),
        "des_deg2": float(des_deg2),
        "descend_align_direction": int(descend_align_direction),
        "descend_align_deg": float(descend_align_deg),
        "align_result": align_result,
        "current_x": float(current_x),
        "current_y": float(current_y),
        "adjust_x": None,
        "adjust_y": None,
        "adjust_distance": float(adjust_distance),
        "adjust_result": None,
        "adjust_move_result": None,
        "descend_result": descend_result,
        "height_result": descend_result,
        "move_result": move_result,
        "timeout_sec": None if timeout_sec is None else float(timeout_sec),
        "elapsed_sec": float(time.time() - started_at),
        "completed": True,
    }


def _action_row_to_list(action_row):
    if hasattr(action_row, "tolist"):
        action_row = action_row.tolist()

    try:
        row_values = list(action_row)
    except TypeError:
        print(f"{execute_action_row.__name__}输入错误: action_row 必须是一行5列数据")
        sys.exit(1)

    if len(row_values) == 1 and isinstance(row_values[0], (list, tuple)):
        row_values = list(row_values[0])

    row_size = len(row_values)
    if row_size != ACTION_MATRIX_ROW_SIZE:
        print(
            f"{execute_action_row.__name__}输入错误: action_row size={row_size}, "
            f"必须等于 {ACTION_MATRIX_ROW_SIZE}"
        )
        sys.exit(1)

    return row_values


def _action_value_to_int(value, column_name):
    try:
        int_value = int(value)
    except (TypeError, ValueError):
        print(f"{execute_action_row.__name__}输入错误: {column_name}={value} 不是整数")
        sys.exit(1)

    if int_value != value and not (
        isinstance(value, float) and value.is_integer()
    ):
        print(f"{execute_action_row.__name__}输入错误: {column_name}={value} 不是整数")
        sys.exit(1)

    return int_value


def execute_action_row(
    sender,
    position_runtime,
    odom_runtime,
    action_row,
):
    """
    解释并执行动作矩阵中的一行。

    当前只完成行格式和 move_dir 校验，并按 move_dir 是否为 0 分流；
    具体动作调用后续继续补充。
    """
    row_values = _action_row_to_list(action_row)
    from_pos, to_pos, move_dir, height_action, grab_action = [
        _action_value_to_int(value, ACTION_MATRIX_COLUMNS[index])
        for index, value in enumerate(row_values)
    ]

    inferred_direction = tools.stair_id_to_direction(
        from_pos,
        to_pos,
        exit_on_error=False,
    )
    if from_pos != to_pos and inferred_direction == 0:
        print(f"{execute_action_row.__name__}输入错误: {from_pos} 与 {to_pos} 不相邻")
        sys.exit(1)

    height_relation = (
        0
        if inferred_direction == 0
        else get_stair_height_relation(from_pos, inferred_direction)
    )

    if move_dir not in (0, 1, 2, 3, 4):
        print(
            f"{execute_action_row.__name__}输入错误: "
            f"{ACTION_MATRIX_COLUMNS[2]}={move_dir}, 必须是 0/1/2/3/4"
        )
        sys.exit(1)

    result = {
        "action_row": [
            int(from_pos),
            int(to_pos),
            int(move_dir),
            int(height_action),
            int(grab_action),
        ],
        "from_pos": int(from_pos),
        "to_pos": int(to_pos),
        "move_dir": int(move_dir),
        "height_action": int(height_action),
        "grab_action": int(grab_action),
        "inferred_direction": int(inferred_direction),
        "height_relation": int(height_relation),
    }

    if move_dir != 0:
        # TODO: 第一段逻辑：有方向动作，后续在这里接入夹取、上下楼梯和移动。
        result["branch"] = "directional"
        result["implemented"] = False
        return result

    # TODO: 第二段逻辑：原地动作，后续在这里接入原地夹取/等待等逻辑。
    result["branch"] = "stationary"
    result["implemented"] = False
    return result
