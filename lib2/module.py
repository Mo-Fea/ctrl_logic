import sys
import threading
import time

from lib2 import tools
from lib2 import move as move_lib
from lib2 import position_backend
from lib2 import position_resource


ODOM_TOPIC = position_resource.ODOM_TOPIC
PRE_DESCEND_ADJUST_DISTANCE = 0.3 #m
STAIR_MOVE_MAX_CMD = 200
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
DEFAULT_KFS_SUCTION_HOLD_SEC = 2.0
ACTION_MATRIX_COLUMNS = [
    "from_pos",
    "to_pos",
    "move_dir",
    "height_action",
    "grab_action",
]
ACTION_MATRIX_ROW_SIZE = 5

CHALLENGE_ACTION_MATRIX = [
    [-1,2,1,1,0],
    [2,5,1,0,1],
    [2,5,1,1,0],
    [5,8,1,0,1],
    [5,8,1,1,0],
    [8,11,1,1,0],
    [11,10,2,1,0],
    [10,13,1,1,0]

]


def __getattr__(name):
    if name in (
        "position_lib",
        "STAIR_HEIGHT_RELATION_MATRIX",
        "PositionRuntime",
        "OdomRuntime",
        "OdometrySubscriber",
    ):
        return getattr(position_resource, name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


def get_position_lib():
    return position_resource.get_position_lib()


def get_entrance_x():
    return position_resource.get_entrance_x()


def get_entrance_y():
    return position_resource.get_entrance_y()


def get_pre_entrance_x():
    return position_resource.get_pre_entrance_x()


def get_pre_entrance_y():
    return position_resource.get_pre_entrance_y()


def get_stair_side_length():
    return position_resource.get_stair_side_length()


def build_stair_height_relation_matrix():
    return position_resource.build_stair_height_relation_matrix()


def get_stair_matrix():
    return position_resource.get_stair_matrix()


def configure_position_backend(lidar_type):
    """
    设置并刷新当前位姿后端。

    position_resource.py 和 move.py 都会缓存 position_lib；切换雷达类型时需要同时刷新，
    避免 init() 后的位置线程或移动逻辑继续使用旧后端。
    """
    backend = position_resource.configure_position_backend(lidar_type)
    move_lib.position_lib = backend
    return backend


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
    return position_resource.start_position_thread(
        sender=sender,
        base_frame=base_frame,
        tf_update_hz=tf_update_hz,
        yaw_update_hz=yaw_update_hz,
        max_tf_age_sec=max_tf_age_sec,
    )


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
    return position_resource.start_odometry_thread(
        topic=topic,
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
    1. 等待当前 reference 参考点位姿可用
    2. 调用 move.move_to_target(...) 以 target_deg 作为移动过程目标航向阻塞移动到目标点
    3. target_deg 不是 None 时，阻塞等待最终航向进入阈值范围
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
        final_direction_result = move_lib.wait_until_direction_reached(
            position_runtime=position_runtime,
            target_yaw_deg=target_deg,
            threshold_deg=move_lib.DEFAULT_DIRECTION_THRESHOLD_DEG,
            timeout_sec=(
                _remaining_timeout_sec(total_deadline)
                if total_deadline is not None
                else final_direction_timeout_sec
            ),
        )
        if final_direction_result is None or final_direction_result.get("timed_out"):
            return None

    return {
        "target_yaw_deg": None if target_deg is None else float(move_lib.normalize_yaw_deg(target_deg)),
        "fixed_yaw_deg": None if target_deg is None else float(move_lib.normalize_yaw_deg(target_deg)),
        "reference": reference,
        "rotate_result": None,
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
    1. 等待当前 reference 参考点位姿可用
    2. 调用 move.move_backward_to_target(...)，移动过程目标航向为 target_deg + 180deg
    3. target_deg 不是 None 时，按原始 target_deg 阻塞等待最终航向；
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

    backward_cmd = -abs(int(v))
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
        final_direction_result = move_lib.wait_until_direction_reached(
            position_runtime=position_runtime,
            target_yaw_deg=target_deg,
            threshold_deg=move_lib.DEFAULT_DIRECTION_THRESHOLD_DEG,
            timeout_sec=(
                _remaining_timeout_sec(total_deadline)
                if total_deadline is not None
                else final_direction_timeout_sec
            ),
        )
        if final_direction_result is None or final_direction_result.get("timed_out"):
            return None

    return {
        "target_yaw_deg": None if target_deg is None else float(move_lib.normalize_yaw_deg(target_deg)),
        "backward_target_yaw_deg": None if target_deg is None else float(move_lib.normalize_yaw_deg(target_deg + 180.0)),
        "fixed_yaw_deg": None if target_deg is None else float(move_lib.normalize_yaw_deg(target_deg)),
        "backward_fixed_yaw_deg": None if target_deg is None else float(move_lib.normalize_yaw_deg(target_deg + 180.0)),
        "reference": reference,
        "rotate_result": None,
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
        drive_result = move_lib.drive_with_channels_for_duration(
            sender=sender,
            duration_sec=2.0,
            forward_cmd=200,
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
    return position_resource.get_stair_matrix_row(stair_id)


def get_stair_xy(stair_id):
    return position_resource.get_stair_xy(stair_id)


def get_stair_height_relation(stair_id, direction):
    return position_resource.get_stair_height_relation(stair_id, direction)


def kfs_pose_id_from_height_relation(height_relation):
    height_relation = int(height_relation)
    if height_relation == 1:
        return 1
    if height_relation == 2:
        return 2
    print(f"{kfs_pose_id_from_height_relation.__name__}输入错误: 高低关系={height_relation}")
    sys.exit(1)


def _kfs_suction_channel_values(suction_value, pose_id=1, trigger_value=KFS_TRIGGER_IDLE_VALUE):
    return {
        KFS_SUCTION_CHANNEL_INDEX: int(suction_value),
        KFS_MODE_CHANNEL_INDEX: KFS_MODE_VALUE,
        KFS_POSE_CHANNEL_INDEX: int(pose_id),
        KFS_TRIGGER_CHANNEL_INDEX: int(trigger_value),
    }


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

    arm_channel_values = _kfs_suction_channel_values(arm_value, pose_id=pose_id)
    arm_deadline = time.time() + float(edge_arm_sec)
    while time.time() < arm_deadline:
        arm_channels = move_lib.set_channel_values(sender, channel_values=arm_channel_values)
        time.sleep(float(loop_interval_sec))

    fire_channel_values = _kfs_suction_channel_values(fire_value, pose_id=pose_id)
    fire_deadline = time.time() + float(edge_hold_sec)
    while time.time() < fire_deadline:
        fire_channels = move_lib.set_channel_values(sender, channel_values=fire_channel_values)
        time.sleep(float(loop_interval_sec))

    fire_channels = move_lib.set_channel_values(sender, channel_values=fire_channel_values)
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
    channel_values = _kfs_suction_channel_values(KFS_SUCTION_ON_VALUE, pose_id=pose_id)
    deadline = time.time() + float(duration_sec)
    while time.time() < deadline:
        channels = move_lib.set_channel_values(sender, channel_values=channel_values)
        time.sleep(float(loop_interval_sec))
    channels = move_lib.set_channel_values(sender, channel_values=channel_values)
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
    final_target_yaw_deg=0.0,
    adjust_distance=PRE_DESCEND_ADJUST_DISTANCE,
    move_speed=600,
    suction_hold_sec=DEFAULT_KFS_SUCTION_HOLD_SEC,
    grab_pose_hold_sec=None,
    transition_pose_hold_sec=None,
    store_pose_hold_sec=None,
    loop_interval_sec=0.02,
):
    """
    KFS 方块吸取并存储组合流程。

    1. 按 stair_id + direction 正向微调到吸取位置。
    2. 根据 STAIR_HEIGHT_RELATION_MATRIX 的高低关系选择 1/2 抓取姿态。
    3. 吸取并保持 suction_hold_sec。
    4. 阻塞执行 3 过渡态，再执行 4 存储态。
    5. 释放吸盘，再执行 0 态。
    6. 显式复位 KFS 相关通道；不再自动倒退回当前 stair_id 的中心坐标。
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
        **({} if grab_pose_hold_sec is None else {"hold_sec": grab_pose_hold_sec}),
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

    reset_kfs_channels = move_lib.set_channel_values(
        sender,
        channel_values={
            KFS_SUCTION_CHANNEL_INDEX: KFS_SUCTION_ON_VALUE,
            KFS_MODE_CHANNEL_INDEX: tools.SAFE_SWITCH_VALUE,
            KFS_POSE_CHANNEL_INDEX: tools.SAFE_SWITCH_VALUE,
            KFS_TRIGGER_CHANNEL_INDEX: tools.SAFE_SWITCH_VALUE,
        },
    )
    post_suction_thread = start_kfs_post_suction_thread(
        sender=sender,
        transition_pose_hold_sec=transition_pose_hold_sec,
        store_pose_hold_sec=store_pose_hold_sec,
        loop_interval_sec=loop_interval_sec,
        reset_kfs_channels=True,
    )

    return {
        "completed": True,
        "post_suction_async": True,
        "stair_id": int(stair_id),
        "direction": int(direction),
        "height_relation": int(height_relation),
        "grab_pose_id": int(grab_pose_id),
        "final_target_yaw_deg": float(final_target_yaw_deg),
        "stair_x": float(stair_x),
        "stair_y": float(stair_y),
        "adjust_result": adjust_result,
        "grab_pose_result": grab_pose_result,
        "suction_result": suction_result,
        "suction_hold_result": suction_hold_result,
        "post_suction_thread": post_suction_thread,
        "transition_pose_result": None,
        "store_pose_result": None,
        "release_result": None,
        "zero_pose_result": None,
        "return_to_center_skipped": True,
        "return_move_result": None,
        "reset_kfs_channels": reset_kfs_channels,
    }


def start_kfs_post_suction_thread(
    sender,
    transition_pose_hold_sec=None,
    store_pose_hold_sec=None,
    loop_interval_sec=0.02,
    reset_kfs_channels=True,
    thread_name="kfs_post_suction_thread",
):
    """
    启动 KFS 吸取保持完成后的异步后续线程。

    默认只执行机械臂/吸盘后续：
    1. pose_id=3 过渡态，保持 ch4=3。
    2. pose_id=4 存储态，保持 ch4=3。
    3. ch4: 3 -> 1 释放吸盘。
    4. pose_id=0 回 0 态。
    5. 可选复位 ch4/ch5/ch6/ch7。

    注意：本线程只管理 KFS 机械臂/吸盘相关通道，不执行回退到台阶中心。
    回中心是底盘动作，应由主流程在线程启动后同步执行，避免线程和主任务
    并发写 ch0/ch2/des_yaw_i16。
    """
    result = {
        "completed": False,
        "running": True,
        "reset_kfs_channels": bool(reset_kfs_channels),
    }
    done_event = threading.Event()
    transition_wait_sec = 3.0 if transition_pose_hold_sec is None else float(transition_pose_hold_sec)
    store_wait_sec = 1.5 if store_pose_hold_sec is None else float(store_pose_hold_sec)

    def repeat_set_channel_values(channel_values, duration_sec):
        deadline = time.time() + float(duration_sec)
        channels = None
        while time.time() < deadline:
            channels = move_lib.set_channel_values(sender, channel_values=channel_values)
            time.sleep(float(loop_interval_sec))
        return move_lib.set_channel_values(sender, channel_values=channel_values)

    def trigger_kfs_pose_with_lock(pose_id, arm_sec, fire_sec, suction_ch4):
        pose_id = int(pose_id)
        with tools.AUTO_TRIGGER_LOCK:
            arm_channel_values = {
                KFS_SUCTION_CHANNEL_INDEX: int(suction_ch4),
                KFS_MODE_CHANNEL_INDEX: KFS_MODE_VALUE,
                KFS_POSE_CHANNEL_INDEX: pose_id,
                KFS_TRIGGER_CHANNEL_INDEX: KFS_TRIGGER_IDLE_VALUE,
            }
            arm_channels = repeat_set_channel_values(arm_channel_values, arm_sec)

            fire_channel_values = {
                KFS_SUCTION_CHANNEL_INDEX: int(suction_ch4),
                KFS_MODE_CHANNEL_INDEX: KFS_MODE_VALUE,
                KFS_POSE_CHANNEL_INDEX: pose_id,
                KFS_TRIGGER_CHANNEL_INDEX: 3,
            }
            fire_channels = repeat_set_channel_values(fire_channel_values, fire_sec)

            idle_channel_values = {
                KFS_SUCTION_CHANNEL_INDEX: int(suction_ch4),
                KFS_MODE_CHANNEL_INDEX: tools.SAFE_SWITCH_VALUE,
                KFS_POSE_CHANNEL_INDEX: tools.SAFE_SWITCH_VALUE,
                KFS_TRIGGER_CHANNEL_INDEX: tools.SAFE_SWITCH_VALUE,
            }
            idle_channels = move_lib.set_channel_values(sender, channel_values=idle_channel_values)

        return {
            "pose_id": pose_id,
            "arm_channels": arm_channels,
            "fire_channels": fire_channels,
            "idle_channels": idle_channels,
            "arm_sec": float(arm_sec),
            "fire_sec": float(fire_sec),
            "suction_ch4": int(suction_ch4),
            "completed": True,
        }

    def release_kfs_suction_with_lock(edge_arm_sec=0.1, edge_hold_sec=0.5):
        with tools.AUTO_TRIGGER_LOCK:
            arm_channels = repeat_set_channel_values(
                {
                    KFS_SUCTION_CHANNEL_INDEX: KFS_SUCTION_ON_VALUE,
                    KFS_MODE_CHANNEL_INDEX: KFS_MODE_VALUE,
                },
                edge_arm_sec,
            )
            fire_channels = repeat_set_channel_values(
                {
                    KFS_SUCTION_CHANNEL_INDEX: KFS_SUCTION_OFF_VALUE,
                    KFS_MODE_CHANNEL_INDEX: KFS_MODE_VALUE,
                },
                edge_hold_sec,
            )
            idle_channel_values = {
                KFS_SUCTION_CHANNEL_INDEX: KFS_SUCTION_OFF_VALUE,
                KFS_MODE_CHANNEL_INDEX: tools.SAFE_SWITCH_VALUE,
                KFS_POSE_CHANNEL_INDEX: tools.SAFE_SWITCH_VALUE,
                KFS_TRIGGER_CHANNEL_INDEX: tools.SAFE_SWITCH_VALUE,
            }
            idle_channels = move_lib.set_channel_values(sender, channel_values=idle_channel_values)
        return {
            "suction_on": False,
            "arm_channels": arm_channels,
            "fire_channels": fire_channels,
            "idle_channels": idle_channels,
            "edge_arm_sec": float(edge_arm_sec),
            "edge_hold_sec": float(edge_hold_sec),
            "ch4_only": False,
            "completed": True,
        }

    def trigger_kfs_zero_return_with_lock(return_sec=0.1):
        with tools.AUTO_TRIGGER_LOCK:
            zero_channel_values = {
                KFS_SUCTION_CHANNEL_INDEX: KFS_SUCTION_OFF_VALUE,
                KFS_MODE_CHANNEL_INDEX: KFS_MODE_VALUE,
                KFS_POSE_CHANNEL_INDEX: 0,
                KFS_TRIGGER_CHANNEL_INDEX: 0,
            }
            zero_channels = repeat_set_channel_values(zero_channel_values, return_sec)
            idle_channel_values = {
                KFS_SUCTION_CHANNEL_INDEX: KFS_SUCTION_OFF_VALUE,
                KFS_MODE_CHANNEL_INDEX: tools.SAFE_SWITCH_VALUE,
                KFS_POSE_CHANNEL_INDEX: tools.SAFE_SWITCH_VALUE,
                KFS_TRIGGER_CHANNEL_INDEX: tools.SAFE_SWITCH_VALUE,
            }
            idle_channels = move_lib.set_channel_values(sender, channel_values=idle_channel_values)

        return {
            "pose_id": 0,
            "pose_name": "zero_return",
            "channels": zero_channels,
            "idle_channels": idle_channels,
            "return_sec": float(return_sec),
            "completed": True,
        }

    def worker():
        try:
            transition_pose_result = trigger_kfs_pose_with_lock(
                pose_id=3,
                arm_sec=0.1,
                fire_sec=0.3,
                suction_ch4=KFS_SUCTION_ON_VALUE,
            )
            time.sleep(transition_wait_sec)

            store_pose_result = trigger_kfs_pose_with_lock(
                pose_id=4,
                arm_sec=0.1,
                fire_sec=0.4,
                suction_ch4=KFS_SUCTION_ON_VALUE,
            )
            time.sleep(store_wait_sec)

            release_result = release_kfs_suction_with_lock()
            time.sleep(2.0)

            zero_pose_result = trigger_kfs_zero_return_with_lock(return_sec=0.1)

            reset_channels = None
            if reset_kfs_channels:
                reset_channels = move_lib.set_channel_values(
                    sender,
                    channel_values={
                        KFS_SUCTION_CHANNEL_INDEX: KFS_SUCTION_OFF_VALUE,
                        KFS_MODE_CHANNEL_INDEX: tools.SAFE_SWITCH_VALUE,
                        KFS_POSE_CHANNEL_INDEX: tools.SAFE_SWITCH_VALUE,
                        KFS_TRIGGER_CHANNEL_INDEX: tools.SAFE_SWITCH_VALUE,
                    },
                )

            result.update({
                "completed": True,
                "running": False,
                "transition_pose_result": transition_pose_result,
                "store_pose_result": store_pose_result,
                "release_result": release_result,
                "zero_pose_result": zero_pose_result,
                "reset_kfs_channels": reset_channels,
                "transition_wait_sec": float(transition_wait_sec),
                "store_wait_sec": float(store_wait_sec),
                "release_wait_sec": 2.0,
            })
        except Exception as exc:
            result.update({
                "completed": False,
                "running": False,
                "exception": repr(exc),
            })
        finally:
            done_event.set()

    thread = threading.Thread(
        target=worker,
        daemon=True,
        name=str(thread_name),
    )
    result["thread_name"] = thread.name
    thread.start()
    return {
        "thread": thread,
        "done_event": done_event,
        "result": result,
    }


def fetch_weapon(
    sender,
    position_runtime,
    odom_runtime,
    weapon_id,
    v=300,
    final_target_yaw_deg=90.0,
    weapon_mode_settle_sec=0.3,
    grab_arm_sec=0.3,
    grab_hold_sec=1.0,
    lift_hold_sec=1.0,
    first_rotate_yaw_deg=90.0,
    weapon_approach_offset_y=2.0,
    intermediate_move_yaw_deg=90.0,
    intermediate_move_x=-2.4,
    intermediate_move_y=-1.2,
    final_move_yaw_deg=-90.0,
    final_move_x=-2.4,
    final_move_y=-2.4,
    release_after_final_wait_sec=5.0,
    release_edge_arm_sec=0.3,
    release_edge_hold_sec=0.3,
):
    """
    根据 weapon_id 选择目标点，先移动 weapon/夹爪到目标点前方，再到目标点后执行夹取并抬起。

    weapon_id 对应目标点从当前 position 后端的 WEAPON_TARGETS 读取。
    """
    position_lib = position_resource.get_position_lib()
    weapon_targets = getattr(position_lib, "WEAPON_TARGETS", None)
    if weapon_targets is None:
        raise AttributeError(f"{position_lib.__name__} must define WEAPON_TARGETS")

    weapon_id = int(weapon_id)
    if weapon_id not in weapon_targets:
        valid_weapon_ids = sorted(int(key) for key in weapon_targets)
        raise ValueError(f"weapon_id must be one of {valid_weapon_ids}, got {weapon_id}")

    des_weapon = weapon_targets[weapon_id]
    des_x, des_y = des_weapon

    approach_x = float(des_x)
    approach_y = float(des_y) - float(weapon_approach_offset_y)

    # 第一段：用 weapon/夹爪参考点，以 90deg 固定航向先移动到 weapon 目标点前方。
    approach_move_result = move_lib.move_to_target(
        sender=sender,
        position_runtime=position_runtime,
        odom_runtime=odom_runtime,
        target_x=approach_x,
        target_y=approach_y,
        final_target_yaw_deg=first_rotate_yaw_deg,
        cruise_forward_cmd=v,
        reference="weapon",
    )
    if approach_move_result is None or approach_move_result.get("timed_out"):
        return {
            "completed": False,
            "failed_step": "weapon_approach_move",
            "weapon_id": int(weapon_id),
            "des_weapon": {
                "x": float(des_x),
                "y": float(des_y),
            },
            "weapon_approach_target": {
                "x": float(approach_x),
                "y": float(approach_y),
                "yaw_deg": float(first_rotate_yaw_deg),
                "offset_y": float(weapon_approach_offset_y),
            },
            "approach_move_result": approach_move_result,
        }

    # 第二段：继续用 weapon/夹爪参考点，以 90deg 固定航向移动到对应 weapon 目标点。
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
    if move_result is None or move_result.get("timed_out"):
        return {
            "completed": False,
            "failed_step": "weapon_target_move",
            "weapon_id": int(weapon_id),
            "des_weapon": {
                "x": float(des_x),
                "y": float(des_y),
            },
            "weapon_approach_target": {
                "x": float(approach_x),
                "y": float(approach_y),
                "yaw_deg": float(first_rotate_yaw_deg),
                "offset_y": float(weapon_approach_offset_y),
            },
            "approach_move_result": approach_move_result,
            "move_result": move_result,
        }

    def enter_weapon_mode(duration_sec=weapon_mode_settle_sec, loop_interval_sec=0.02):
        channel_values = {
            1: 0,
            2: 0,
            4: 1,
            5: 3,
            6: tools.SAFE_SWITCH_VALUE,
            7: tools.SAFE_SWITCH_VALUE,
        }
        deadline = time.time() + float(duration_sec)
        while time.time() < deadline:
            channels = move_lib.set_channel_values(
                sender,
                des_yaw_i16=0,
                channel_values=channel_values,
            )
            time.sleep(float(loop_interval_sec))
        channels = move_lib.set_channel_values(
            sender,
            des_yaw_i16=0,
            channel_values=channel_values,
        )
        return channels

    def set_weapon_state(ch1=0, ch4=1, forward_cmd=0, des_yaw_i16=0):
        return move_lib.set_channel_values(
            sender,
            des_yaw_i16=int(des_yaw_i16),
            channel_values={
                1: int(ch1),
                2: int(forward_cmd),
                4: int(ch4),
                5: 3,
                6: tools.SAFE_SWITCH_VALUE,
                7: tools.SAFE_SWITCH_VALUE,
            },
        )

    mode_channels = enter_weapon_mode()

    grab_arm_channels = set_weapon_state(ch1=0, ch4=1)
    time.sleep(float(grab_arm_sec))

    grab_fire_channels = set_weapon_state(ch1=0, ch4=3)
    time.sleep(float(grab_hold_sec))

    lift_channels = set_weapon_state(ch1=100, ch4=3)
    time.sleep(float(lift_hold_sec))

    # 第二段：保持夹取状态，以 90deg 固定航向正向移动到中间目标点。
    intermediate_move_result = move_lib.move_to_target(
        sender=sender,
        position_runtime=position_runtime,
        odom_runtime=odom_runtime,
        target_x=intermediate_move_x,
        target_y=intermediate_move_y,
        final_target_yaw_deg=intermediate_move_yaw_deg,
        cruise_forward_cmd=v,
        reference="robot",
    )

    intermediate_stop_channels = set_weapon_state(ch1=100, ch4=3, forward_cmd=0)

    # 第三段：保持夹取状态，以 -90deg 固定航向正向移动到最终目标点。
    final_move_result = move_lib.move_to_target(
        sender=sender,
        position_runtime=position_runtime,
        odom_runtime=odom_runtime,
        target_x=final_move_x,
        target_y=final_move_y,
        final_target_yaw_deg=final_move_yaw_deg,
        cruise_forward_cmd=v,
        reference="robot",
    )
    final_hold_channels = set_weapon_state(ch1=100, ch4=3, forward_cmd=0)
    time.sleep(float(release_after_final_wait_sec))

    final_keep_channels = set_weapon_state(ch1=100, ch4=3, forward_cmd=0)
    time.sleep(float(release_edge_hold_sec))

    return {
        "weapon_id": int(weapon_id),
        "des_weapon": {
            "x": float(des_x),
            "y": float(des_y),
        },
        "weapon_approach_target": {
            "x": float(approach_x),
            "y": float(approach_y),
            "yaw_deg": float(first_rotate_yaw_deg),
            "offset_y": float(weapon_approach_offset_y),
        },
        "approach_move_result": approach_move_result,
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
        "intermediate_move_target": {
            "x": float(intermediate_move_x),
            "y": float(intermediate_move_y),
            "yaw_deg": float(intermediate_move_yaw_deg),
        },
        "intermediate_move_result": intermediate_move_result,
        "intermediate_stop_channels": intermediate_stop_channels,
        "final_move_target": {
            "x": float(final_move_x),
            "y": float(final_move_y),
            "yaw_deg": float(final_move_yaw_deg),
        },
        "final_move_result": final_move_result,
        "final_hold_channels": final_hold_channels,
        "keep_result": {
            "wait_sec": float(release_after_final_wait_sec),
            "channels": final_keep_channels,
            "edge_arm_sec": float(release_edge_arm_sec),
            "edge_hold_sec": float(release_edge_hold_sec),
            "completed": True,
        },
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
    2. 调用高位微调逻辑完成上楼前对正和前探
    3. 调用 move.climb(...) 阻塞执行半自动上楼梯
    4. 调用 move_to_des(...) 移动到目标点 (x, y)，最终朝向 des_deg2
    """
    des_deg1 = tools.direction_int_to_yaw_deg(direction1)
    des_deg2 = tools.direction_int_to_yaw_deg(direction2)

    pre_climb_adjust_result = adjust_position(
        sender=sender,
        position_runtime=position_runtime,
        odom_runtime=odom_runtime,
        move_type=1,
        direction=direction1,
        stair_id=-1,
        height_relation=1,
        move_speed=move_speed,
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
        v=STAIR_MOVE_MAX_CMD,
    )

    return {
        "des_deg1": float(des_deg1),
        "des_deg2": float(des_deg2),
        "rotate_result": pre_climb_adjust_result.get("rotate_result"),
        "pre_climb_adjust_result": pre_climb_adjust_result,
        "pre_climb_drive_result": pre_climb_adjust_result.get("drive_result"),
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
    trigger_hold_sec=0.3,
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
        v=STAIR_MOVE_MAX_CMD,
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


def execute_stair_transition(
    sender,
    position_runtime,
    odom_runtime,
    from_x,
    from_y,
    to_x,
    to_y,
    height_relation,
    task_direction,
    final_direction,
):
    """
    根据 from/to 高低关系执行一次上下楼梯动作。

    height_relation:
      1: to 比 from 高，调用 climb(...)
      2: to 比 from 低，调用 descend(...)

    task_direction 是本次上下楼方向，final_direction 是动作完成后的最终朝向。
    """
    height_relation = int(height_relation)
    task_direction = int(task_direction)
    final_direction = int(final_direction)

    if height_relation not in (1, 2):
        print(
            f"{execute_stair_transition.__name__}输入错误: "
            f"height_relation={height_relation}, 必须是 1/2"
        )
        sys.exit(1)
    if task_direction not in (1, 2, 3, 4):
        print(
            f"{execute_stair_transition.__name__}输入错误: "
            f"task_direction={task_direction}, 必须是 1/2/3/4"
        )
        sys.exit(1)
    if final_direction not in (1, 2, 3, 4):
        print(
            f"{execute_stair_transition.__name__}输入错误: "
            f"final_direction={final_direction}, 必须是 1/2/3/4"
        )
        sys.exit(1)

    if height_relation == 1:
        stair_result = climb(
            sender=sender,
            position_runtime=position_runtime,
            odom_runtime=odom_runtime,
            direction1=task_direction,
            direction2=final_direction,
            x=to_x,
            y=to_y,
        )
        return {
            "height_relation": int(height_relation),
            "task_direction": int(task_direction),
            "final_direction": int(final_direction),
            "from_x": float(from_x),
            "from_y": float(from_y),
            "to_x": float(to_x),
            "to_y": float(to_y),
            "stair_action": "climb",
            "stair_result": stair_result,
        }

    stair_result = descend(
        sender=sender,
        position_runtime=position_runtime,
        odom_runtime=odom_runtime,
        direction1=task_direction,
        direction2=final_direction,
        current_x=from_x,
        current_y=from_y,
        des_x=to_x,
        des_y=to_y,
    )
    return {
        "height_relation": int(height_relation),
        "task_direction": int(task_direction),
        "final_direction": int(final_direction),
        "from_x": float(from_x),
        "from_y": float(from_y),
        "to_x": float(to_x),
        "to_y": float(to_y),
        "stair_action": "descend",
        "stair_result": stair_result,
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


def _action_matrix_to_rows(action_matrix):
    if hasattr(action_matrix, "tolist"):
        action_matrix = action_matrix.tolist()

    try:
        rows = list(action_matrix)
    except TypeError:
        print(f"{execute_action_matrix.__name__}输入错误: action_matrix 必须是 n*5 数据")
        sys.exit(1)

    if not rows:
        return []

    if len(rows) == ACTION_MATRIX_ROW_SIZE and not isinstance(rows[0], (list, tuple)):
        rows = [rows]

    normalized_rows = []
    for row_index, row in enumerate(rows):
        if hasattr(row, "tolist"):
            row = row.tolist()
        try:
            row_values = list(row)
        except TypeError:
            print(
                f"{execute_action_matrix.__name__}输入错误: "
                f"第 {row_index} 行不是一行5列数据"
            )
            sys.exit(1)
        normalized_rows.append(row_values)

    return normalized_rows


def execute_action_row(
    sender,
    position_runtime,
    odom_runtime,
    action_row,
    final_direction=1,
    next_from_pose=0,
    next_to_pose=0,
    next_height_action=0,
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
    final_direction = _action_value_to_int(final_direction, "final_direction")
    next_from_pose = _action_value_to_int(next_from_pose, "next_from_pose")
    next_to_pose = _action_value_to_int(next_to_pose, "next_to_pose")
    next_height_action = _action_value_to_int(next_height_action, "next_height_action")
    if final_direction not in (1, 2, 3, 4):
        print(
            f"{execute_action_row.__name__}输入错误: "
            f"final_direction={final_direction}, 必须是 1/2/3/4"
        )
        sys.exit(1)
    if next_height_action not in (0, 1):
        print(
            f"{execute_action_row.__name__}输入错误: "
            f"next_height_action={next_height_action}, 必须是 0/1"
        )
        sys.exit(1)

    inferred_direction = tools.stair_id_to_direction(
        from_pos,
        to_pos,
        exit_on_error=False,
    )
    if from_pos != to_pos and inferred_direction == 0:
        print(f"{execute_action_row.__name__}输入错误: {from_pos} 与 {to_pos} 不相邻")
        sys.exit(1)
    if move_dir != 0 and inferred_direction != 0 and move_dir != inferred_direction:
        print(
            f"{execute_action_row.__name__}输入错误: "
            f"move_dir={move_dir} 与坐标推导方向 inferred_direction={inferred_direction} 不一致"
        )
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
    if move_dir != 0 and height_action == 0 and grab_action == 0:
        print(
            f"{execute_action_row.__name__}输入错误: "
            "当前地图不应出现等高普通移动 "
            f"action_row={[from_pos, to_pos, move_dir, height_action, grab_action]}"
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
        "final_direction": int(final_direction),
        "next_from_pose": int(next_from_pose),
        "next_to_pose": int(next_to_pose),
        "next_height_action": int(next_height_action),
        "inferred_direction": int(inferred_direction),
        "height_relation": int(height_relation),
    }

    if move_dir != 0:
        from_x, from_y = get_stair_xy(from_pos)
        to_x, to_y = get_stair_xy(to_pos)
        result["from_x"] = float(from_x)
        result["from_y"] = float(from_y)
        result["to_x"] = float(to_x)
        result["to_y"] = float(to_y)

        if grab_action == 1:
            fetch_result = fetch_and_store_kfs(
                sender=sender,
                position_runtime=position_runtime,
                odom_runtime=odom_runtime,
                stair_id=from_pos,
                direction=move_dir,
                final_target_yaw_deg=tools.direction_int_to_yaw_deg(final_direction),
            )
            result["branch"] = "directional"
            result["fetch_result"] = fetch_result
            result["return_center_result"] = None
            result["return_center_skipped"] = True
            result["return_center_skip_reason"] = "fetch_failed"
            if not fetch_result.get("completed", False):
                result["implemented"] = True
                return result

            next_inferred_direction = 0
            next_height_relation = 0
            should_skip_return_center = False
            if next_height_action == 1:
                next_inferred_direction = tools.stair_id_to_direction(
                    next_from_pose,
                    next_to_pose,
                    exit_on_error=False,
                )
                if next_inferred_direction == 0:
                    print(
                        f"{execute_action_row.__name__}输入错误: "
                        f"下一行 height_action=1 但 {next_from_pose} 与 {next_to_pose} 不相邻"
                    )
                    sys.exit(1)
                next_height_relation = get_stair_height_relation(
                    next_from_pose,
                    next_inferred_direction,
                )
                should_skip_return_center = (
                    next_height_relation == 1
                    and to_pos == next_to_pose
                )

            result["next_inferred_direction"] = int(next_inferred_direction)
            result["next_height_relation"] = int(next_height_relation)
            if should_skip_return_center:
                result["return_center_skip_reason"] = "next_climb_to_same_target"
                result["implemented"] = True
                return result

            return_center_yaw_deg = tools.direction_int_to_yaw_deg(move_dir)
            return_center_result = move_to_des(
                sender=sender,
                position_runtime=position_runtime,
                odom_runtime=odom_runtime,
                x=from_x,
                y=from_y,
                target_deg=return_center_yaw_deg,
                v=STAIR_MOVE_MAX_CMD,
            )
            result["return_center_result"] = return_center_result
            result["return_center_skipped"] = False
            result["return_center_skip_reason"] = None
            result["return_center_target_yaw_deg"] = float(return_center_yaw_deg)
            result["implemented"] = True
            return result

        if height_action != 0:
            stair_transition_result = execute_stair_transition(
                sender=sender,
                position_runtime=position_runtime,
                odom_runtime=odom_runtime,
                from_x=from_x,
                from_y=from_y,
                to_x=to_x,
                to_y=to_y,
                height_relation=height_relation,
                task_direction=move_dir,
                final_direction=final_direction,
            )
            result["branch"] = "directional"
            result["stair_transition_result"] = stair_transition_result
            result["implemented"] = True
            return result

        # TODO: 第一段逻辑：有方向动作，后续在这里接入夹取、上下楼梯和移动。
        result["branch"] = "directional"
        result["implemented"] = False
        return result

    # TODO: 第二段逻辑：原地动作，后续在这里接入原地夹取/等待等逻辑。
    result["branch"] = "stationary"
    result["implemented"] = False
    return result


def execute_action_matrix(
    sender,
    position_runtime,
    odom_runtime,
    action_matrix,
    final_direction=1,
    stop_on_unimplemented=True,
):
    """
    顺序执行动作矩阵。

    action_matrix: n*5，每行格式同 execute_action_row()。
    final_direction: 每行完成后的最终朝向，当前统一传给每一行。
    stop_on_unimplemented: 遇到 execute_action_row() 返回 implemented=False 时是否终止。
    """
    final_direction = _action_value_to_int(final_direction, "final_direction")
    if final_direction not in (1, 2, 3, 4):
        print(
            f"{execute_action_matrix.__name__}输入错误: "
            f"final_direction={final_direction}, 必须是 1/2/3/4"
        )
        sys.exit(1)

    rows = _action_matrix_to_rows(action_matrix)
    row_count = len(rows)
    results = []
    for row_index, action_row in enumerate(rows):
        row_kwargs = {}
        if row_index + 1 < row_count:
            next_row_values = _action_row_to_list(rows[row_index + 1])
            row_kwargs = {
                "next_from_pose": _action_value_to_int(
                    next_row_values[0],
                    "next_from_pose",
                ),
                "next_to_pose": _action_value_to_int(
                    next_row_values[1],
                    "next_to_pose",
                ),
                "next_height_action": _action_value_to_int(
                    next_row_values[3],
                    "next_height_action",
                ),
            }

        row_result = execute_action_row(
            sender=sender,
            position_runtime=position_runtime,
            odom_runtime=odom_runtime,
            action_row=action_row,
            final_direction=final_direction,
            **row_kwargs,
        )
        row_result["row_index"] = int(row_index)
        results.append(row_result)

        if stop_on_unimplemented and not row_result.get("implemented", False):
            print(
                f"{execute_action_matrix.__name__}输入错误: "
                f"第 {row_index} 行尚未接入真实动作 "
                f"action_row={row_result.get('action_row')}"
            )
            sys.exit(1)

    return {
        "completed": True,
        "row_count": row_count,
        "final_direction": int(final_direction),
        "results": results,
    }
