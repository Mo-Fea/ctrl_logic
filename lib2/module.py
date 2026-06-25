import sys
import time

from lib2 import tools
from lib2 import kfs
from lib2 import move as move_lib
from lib2 import weapon as weapon_lib
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
INITIAL_SUCTION_ROTATION_VALUE = 800
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
    initialize_machine_pose=True,
):
    """
    一站式初始化:
    - 连接下位机
    - 关闭航向 PID，执行 weapon_down 后再执行 weapon_loose，并将 ch9 初始化为 800
    - 完成后将 ch1 归零并退出 weapon 模式，保持夹爪打开 ch4=3 和 ch9=800
    - 启动重定位监听
    - 可选等待重定位成功后再返回
    - 可选在重定位成功后自动销毁重定位监听
    - 创建并启动 frame_thread
    - 可选等待 frame_thread 首帧发送成功后再返回

    lidar_type:
      1: odin
      2: mid360

    initialize_machine_pose:
      True: 连接后依次执行 weapon_down、weapon_loose，并保持 ch4=3、ch9=800。
      False: 跳过机械姿态初始化，供纯通信/位姿调试使用。

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
    # 初始化默认关闭底盘航向 PID，不因雷达后端或重定位状态自动旋转。
    startup_des_yaw_i16 = 0
    sender.set_des_yaw_i16(startup_des_yaw_i16)

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

        try:
            if initialize_machine_pose:
                sender.set_ch9(INITIAL_SUCTION_ROTATION_VALUE)
                weapon_lib.weapon_down(sender)
                weapon_lib.weapon_loose(sender)
                sender.set_channel_values(
                    {
                        weapon_lib.WEAPON_LIFT_CHANNEL_INDEX: 0,
                        weapon_lib.WEAPON_MODE_CHANNEL_INDEX: tools.SAFE_SWITCH_VALUE,
                        weapon_lib.WEAPON_SUBFUNCTION_CHANNEL_INDEX: tools.SAFE_SWITCH_VALUE,
                        weapon_lib.WEAPON_TRIGGER_CHANNEL_INDEX: tools.SAFE_SWITCH_VALUE,
                        9: INITIAL_SUCTION_ROTATION_VALUE,
                    }
                )
            sender.set_des_yaw_i16(startup_des_yaw_i16)
        except Exception:
            sender.stop(send_stop=True)
            tools.socket_close(sender.sock)
            raise

        state = sender.get_state()
        print(
            "frame_thread started and machine pose initialized: "
            f"yaw_i16={state['yaw_i16']} "
            f"des_yaw_i16={state['des_yaw_i16']} "
            f"ch9={state['channels'][9]} "
            f"channels={state['channels']}"
        )

    # 机械姿态初始化需要实际发帧，因此启用时两种雷达后端都先启动发送线程。
    # 初始化完成后保持 des_yaw_i16=0，关闭航向 PID。
    if initialize_machine_pose or (not wait_relocalization) or (not position_backend.is_mid360()):
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
    v=500,
    rotate_tolerance_deg=5.0,
    final_direction_timeout_sec=None,
    move_timeout_sec=None,
    total_timeout_sec=None,
    wait_position_timeout_sec=2.0,
    reference="robot",
    stop_distance=None,
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
        **({} if stop_distance is None else {"stop_distance": stop_distance}),
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
        "completed": True,
        "failed_step": None,
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
    v=500,
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
        "completed": True,
        "failed_step": None,
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


def meilin_prepare(
    sender,
    position_runtime,
    odom_runtime,
    v=500,
    total_timeout_sec=None,
    final_direction_timeout_sec=None,
    move_timeout_sec=None,
    reference="robot",
):
    """
    梅林动作测试准备。

    移动到当前半场编号为 -1 的台阶：
      - 红半场：朝向方向 1，即 90deg。
      - 蓝半场：朝向方向 4，即 -90deg。

    yaw 统一通过 tools.direction_int_to_yaw_deg(...) 获取，避免直接写死
    0/90/180/-90。
    """
    stair_id = -1
    target_direction = 4 if position_backend.is_blue_field() else 1
    target_yaw_deg = tools.direction_int_to_yaw_deg(target_direction)
    target_x, target_y = get_stair_xy(stair_id)

    move_result = move_to_des(
        sender=sender,
        position_runtime=position_runtime,
        odom_runtime=odom_runtime,
        x=target_x,
        y=target_y,
        target_deg=target_yaw_deg,
        v=v,
        final_direction_timeout_sec=final_direction_timeout_sec,
        move_timeout_sec=move_timeout_sec,
        total_timeout_sec=total_timeout_sec,
        reference=reference,
    )

    return {
        "completed": move_result is not None
        and bool(move_result.get("completed", False)),
        "failed_step": None if move_result is not None else "meilin_prepare_move",
        "stair_id": int(stair_id),
        "target_direction": int(target_direction),
        "target_x": float(target_x),
        "target_y": float(target_y),
        "target_yaw_deg": float(target_yaw_deg),
        "move_result": move_result,
    }


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
      红场坐标偏移:
        1: current_y + adjust_distance
        2: current_x - adjust_distance
        3: current_x + adjust_distance
        4: current_y - adjust_distance
      蓝场坐标偏移:
        1: current_y - adjust_distance
        2: current_x + adjust_distance
        3: current_x - adjust_distance
        4: current_y + adjust_distance

    height_relation:
      1: 微调方向台阶较高，先旋转到目标航向，再 ch2=300 前进 1s，完成微调。
      2: 微调方向台阶较低，执行原来的按坐标微调逻辑。

    direction 同时通过 tools.direction_int_to_yaw_deg(...) 转成微调过程中的目标航向角。
    当前实测关系:
      红场 0.01deg=x+，90deg=y+。
      蓝场 0.01deg=x+，-90deg=y+。
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
            duration_sec=1.0,
            forward_cmd=500,
            target_yaw_deg=target_deg,
            brake_reverse_cmd=0,
            brake_duration_sec=0.0,
        )
        rotate_completed = bool(
            rotate_result is not None
            and not rotate_result.get("timed_out", False)
        )
        drive_completed = bool(
            drive_result is not None
            and drive_result.get("completed", False)
        )
        return {
            "completed": rotate_completed and drive_completed,
            "failed_step": (
                None
                if rotate_completed and drive_completed
                else (
                    "rotate_before_adjust"
                    if not rotate_completed
                    else "timed_forward_adjust"
                )
            ),
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

    if position_backend.is_blue_field():
        direction_to_delta = {
            1: (0.0, -adjust_distance),
            2: (adjust_distance, 0.0),
            3: (-adjust_distance, 0.0),
            4: (0.0, adjust_distance),
        }
    else:
        direction_to_delta = {
            1: (0.0, adjust_distance),
            2: (-adjust_distance, 0.0),
            3: (adjust_distance, 0.0),
            4: (0.0, -adjust_distance),
        }

    adjust_dx, adjust_dy = direction_to_delta[direction]
    adjust_x += adjust_dx
    adjust_y += adjust_dy

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
        "completed": move_result is not None,
        "failed_step": None if move_result is not None else "coordinate_adjust_move",
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
        "adjust_dx": float(adjust_dx),
        "adjust_dy": float(adjust_dy),
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

    with tools.AUTO_TRIGGER_LOCK:
        arm_channel_values = _kfs_suction_channel_values(arm_value, pose_id=pose_id)
        arm_deadline = time.time() + float(edge_arm_sec)
        while time.time() < arm_deadline:
            arm_channels = move_lib.set_channel_values(
                sender,
                channel_values=arm_channel_values,
            )
            time.sleep(float(loop_interval_sec))
        arm_channels = move_lib.set_channel_values(
            sender,
            channel_values=arm_channel_values,
        )

        fire_channel_values = _kfs_suction_channel_values(fire_value, pose_id=pose_id)
        fire_deadline = time.time() + float(edge_hold_sec)
        while time.time() < fire_deadline:
            fire_channels = move_lib.set_channel_values(
                sender,
                channel_values=fire_channel_values,
            )
            time.sleep(float(loop_interval_sec))

        fire_channels = move_lib.set_channel_values(
            sender,
            channel_values=fire_channel_values,
        )
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


def side_suck(
    sender,
    position_runtime,
    odom_runtime,
    target_stair_id,
    adjust_distance=0.2,
    move_speed=200,
    lateral_distance=1.1,
    post_move_wait_sec=0.5,
):
    """
    场外侧吸组合动作。

    1. 按原 target_stair_id/suck_count 分支旋转吸盘头：
       target_stair_id=1 时：
       - suck_count=1：将吸盘头旋转到 90deg。
       - suck_count=2：将吸盘头旋转到 -90deg。
       target_stair_id=3 时：
       - suck_count=1：将吸盘头旋转到 -90deg。
       - suck_count=2：将吸盘头旋转到 90deg。
    2. 切换到 side_pose。
    3. 按当前 suck_count 选择气缸。
    4. 执行 move.side_suck_movement(...) 到侧吸准备位，并进行高位前进微调。
    5. 启动已选择气缸吸取。
    6. 执行 move.side_suck_lateral_movement(...) 定时左右平移。
    7. 移动回编号 -1 的台阶中心。

    target_stair_id 只允许 1 或 3；suck_count 由气缸选择方法校验。
    """
    try:
        target_stair_id = int(target_stair_id)
    except (TypeError, ValueError):
        raise ValueError(
            f"target_stair_id must be 1 or 3, got {target_stair_id!r}"
        )
    if target_stair_id not in (1, 3):
        raise ValueError(
            f"target_stair_id must be 1 or 3, got {target_stair_id}"
        )

    suck_count = int(kfs.suck_count)
    if (target_stair_id, suck_count) in ((1, 1), (3, 2)):
        sucker_rotation_result = kfs.sucker_90deg(sender)
    elif (target_stair_id, suck_count) in ((1, 2), (3, 1)):
        sucker_rotation_result = kfs.sucker_neg90deg(sender)
    else:
        return {
            "completed": False,
            "implemented": False,
            "failed_step": "side_suck_branch_not_implemented",
            "target_stair_id": int(target_stair_id),
            "suck_count": int(suck_count),
            "sucker_rotation_result": None,
            "side_pose_result": None,
            "cylinder_selection_result": None,
            "side_suck_movement_result": None,
            "suction_result": None,
            "lateral_move_result": None,
            "post_suction_thread": None,
            "return_move_result": None,
        }

    side_pose_result = kfs.kfs_side_pose(sender)
    cylinder_selection_result = kfs.sucker_select_cylinder(sender)

    side_suck_movement_result = move_lib.side_suck_movement(
        sender=sender,
        position_runtime=position_runtime,
        odom_runtime=odom_runtime,
        target_stair_id=target_stair_id,
        lateral_distance=0.5,
        cruise_forward_cmd=move_speed,
        adjust_forward_cmd=300,
        adjust_duration_sec=1.0,
        reference="robot",
    )
    if not side_suck_movement_result.get("completed", False):
        return {
            "completed": False,
            "implemented": True,
            "failed_step": "side_suck_movement",
            "target_stair_id": int(target_stair_id),
            "suck_count": int(suck_count),
            "sucker_rotation_result": sucker_rotation_result,
            "side_pose_result": side_pose_result,
            "cylinder_selection_result": cylinder_selection_result,
            "side_suck_movement_result": side_suck_movement_result,
            "suction_result": None,
            "lateral_move_result": None,
            "post_suction_thread": None,
            "return_move_result": None,
        }

    suction_result = set_kfs_suction(
        sender=sender,
        suction_on=True,
        pose_id=5,
    )

    target_yaw_deg = side_suck_movement_result.get("target_yaw_deg")
    lateral_move_result = move_lib.side_suck_lateral_movement(
        sender=sender,
        target_stair_id=target_stair_id,
        target_yaw_deg=target_yaw_deg,
    )
    if not lateral_move_result.get("completed", False):
        return {
            "completed": False,
            "implemented": True,
            "failed_step": "side_suck_lateral_movement",
            "target_stair_id": int(target_stair_id),
            "suck_count": int(suck_count),
            "sucker_rotation_result": sucker_rotation_result,
            "cylinder_selection_result": cylinder_selection_result,
            "side_pose_result": side_pose_result,
            "suction_result": suction_result,
            "side_suck_movement_result": side_suck_movement_result,
            "lateral_move_result": lateral_move_result,
            "post_suction_thread": None,
            "return_move_result": None,
        }

    time.sleep(float(post_move_wait_sec))
    post_suction_thread = kfs.start_kfs_post_suction_thread(
        sender=sender,
        reset_kfs_channels=True,
    )
    return_x, return_y = get_stair_xy(-1)
    return_move_result = move_to_des(
        sender=sender,
        position_runtime=position_runtime,
        odom_runtime=odom_runtime,
        x=return_x,
        y=return_y,
        target_deg=target_yaw_deg,
        v=move_speed,
    )
    if return_move_result is None:
        return {
            "completed": False,
            "implemented": True,
            "failed_step": "return_to_stair_minus_one",
            "target_stair_id": int(target_stair_id),
            "suck_count": int(suck_count),
            "sucker_rotation_result": sucker_rotation_result,
            "cylinder_selection_result": cylinder_selection_result,
            "side_pose_result": side_pose_result,
            "suction_result": suction_result,
            "side_suck_movement_result": side_suck_movement_result,
            "lateral_move_result": lateral_move_result,
            "post_move_wait_sec": float(post_move_wait_sec),
            "post_suction_thread": post_suction_thread,
            "return_move_target": {
                "stair_id": -1,
                "x": float(return_x),
                "y": float(return_y),
                "yaw_deg": float(target_yaw_deg),
            },
            "return_move_result": return_move_result,
        }

    return {
        "completed": True,
        "implemented": True,
        "failed_step": None,
        "target_stair_id": int(target_stair_id),
        "suck_count": int(suck_count),
        "sucker_rotation_result": sucker_rotation_result,
        "cylinder_selection_result": cylinder_selection_result,
        "side_pose_result": side_pose_result,
        "suction_result": suction_result,
        "side_suck_movement_result": side_suck_movement_result,
        "lateral_move_result": lateral_move_result,
        "post_move_wait_sec": float(post_move_wait_sec),
        "post_suction_thread": post_suction_thread,
        "return_move_target": {
            "stair_id": -1,
            "x": float(return_x),
            "y": float(return_y),
            "yaw_deg": float(target_yaw_deg),
        },
        "return_move_result": return_move_result,
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
    loop_interval_sec=0.02,
):
    """
    KFS 方块吸取组合流程。

    1. 按 stair_id + direction 正向微调到吸取位置。
    2. 根据 STAIR_HEIGHT_RELATION_MATRIX 的高低关系选择 1/2 抓取姿态。
    3. 吸取并保持 suction_hold_sec。
    4. 启动异步后续：执行 3 过渡态，按 suck_count 预旋转吸盘，再执行 0 态。
    5. 显式复位 KFS 相关通道；不再自动倒退回当前 stair_id 的中心坐标。
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
    if not adjust_result.get("completed", False):
        return {
            "completed": False,
            "failed_step": "adjust_position",
            "adjust_result": adjust_result,
        }

    grab_pose_id = kfs_pose_id_from_height_relation(height_relation)

    grab_pose_result = kfs.kfs_grab_pose(
        sender=sender,
        pose_id=grab_pose_id,
        loop_interval_sec=loop_interval_sec,
        **({} if grab_pose_hold_sec is None else {"fire_sec": grab_pose_hold_sec}),
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
    post_suction_thread = kfs.start_kfs_post_suction_thread(
        sender=sender,
        transition_pose_hold_sec=transition_pose_hold_sec,
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
        "sucker_rotation_result": None,
        "zero_pose_result": None,
        "return_to_center_skipped": True,
        "return_move_result": None,
        "reset_kfs_channels": reset_kfs_channels,
    }


def fetch_weapon(
    sender,
    position_runtime,
    odom_runtime,
    weapon_id,
    v=300,
    final_target_yaw_deg=None,
    weapon_mode_settle_sec=weapon_lib.DEFAULT_EDGE_ARM_SEC,
    grab_arm_sec=weapon_lib.DEFAULT_EDGE_ARM_SEC,
    grab_hold_sec=weapon_lib.DEFAULT_EDGE_HOLD_SEC,
    lift_hold_sec=weapon_lib.DEFAULT_MOTION_HOLD_SEC,
    first_rotate_yaw_deg=None,
    weapon_approach_offset_y=1.0,
    return_move_yaw_deg=None,
    release_rotate_yaw_deg=None,
    release_after_final_wait_sec=5.0,
    release_edge_arm_sec=0.3,
    release_edge_hold_sec=0.3,
):
    """
    根据 weapon_id 选择目标点，移动到目标点前方，再到目标点执行夹取、抬起、
    返回前方点并旋转到释放方向。函数返回时保持夹取状态，不自动放下或松开。

    weapon_id 对应目标点从当前 position 后端读取。
    """
    position_lib = position_resource.get_position_lib()
    get_weapon_targets = getattr(position_lib, "get_weapon_targets", None)
    weapon_targets = (
        get_weapon_targets()
        if get_weapon_targets is not None
        else getattr(position_lib, "WEAPON_TARGETS", None)
    )
    if weapon_targets is None:
        raise AttributeError(
            f"{position_lib.__name__} must define get_weapon_targets() or WEAPON_TARGETS"
        )

    weapon_id = int(weapon_id)
    if weapon_id not in weapon_targets:
        valid_weapon_ids = sorted(int(key) for key in weapon_targets)
        raise ValueError(f"weapon_id must be one of {valid_weapon_ids}, got {weapon_id}")

    des_weapon = weapon_targets[weapon_id]
    des_x, des_y = des_weapon

    if position_backend.is_blue_field():
        if final_target_yaw_deg is None:
            final_target_yaw_deg = 180.0
        if first_rotate_yaw_deg is None:
            first_rotate_yaw_deg = 180.0
        if return_move_yaw_deg is None:
            return_move_yaw_deg = 180.0
        if release_rotate_yaw_deg is None:
            release_rotate_yaw_deg = 0.01
    else:
        if final_target_yaw_deg is None:
            final_target_yaw_deg = 0.01
        if first_rotate_yaw_deg is None:
            first_rotate_yaw_deg = 0.01
        if return_move_yaw_deg is None:
            return_move_yaw_deg = 0.01
        if release_rotate_yaw_deg is None:
            release_rotate_yaw_deg = 180.0

    approach_offset = abs(float(weapon_approach_offset_y))
    if position_backend.is_blue_field():
        approach_x = float(des_x) - approach_offset
        approach_y = float(des_y)
        approach_offset_direction = -1
    else:
        approach_x = float(des_x) + approach_offset
        approach_y = float(des_y)
        approach_offset_direction = 1

    return_move_x = float(approach_x)
    return_move_y = float(approach_y)
    return_move_reference_weapon_id = int(weapon_id)
    if position_backend.is_blue_field():
        blue_return_weapon_id = 1
        if blue_return_weapon_id not in weapon_targets:
            valid_weapon_ids = sorted(int(key) for key in weapon_targets)
            raise ValueError(
                f"blue return weapon_id {blue_return_weapon_id} is not in {valid_weapon_ids}"
            )
        return_weapon_x, return_weapon_y = weapon_targets[blue_return_weapon_id]
        return_move_x = float(return_weapon_x) - approach_offset
        return_move_y = float(return_weapon_y)
        return_move_reference_weapon_id = int(blue_return_weapon_id)

    # 第一段：用 weapon/夹爪参考点，按当前半场目标航向先移动到 weapon 目标点前方 1m。
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
                "offset_distance": float(approach_offset),
                "offset_axis": "x",
                "offset_direction": int(approach_offset_direction),
            },
            "approach_move_result": approach_move_result,
        }

    # 第二段：用 weapon/夹爪参考点，按当前半场目标航向移动到对应 weapon 目标点。
    move_result = move_lib.move_to_target(
        sender=sender,
        position_runtime=position_runtime,
        odom_runtime=odom_runtime,
        target_x=des_x,
        target_y=des_y,
        final_target_yaw_deg=final_target_yaw_deg,
        cruise_forward_cmd=v,
        stop_distance=0.03,
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
                "offset_distance": float(approach_offset),
                "offset_axis": "x",
                "offset_direction": int(approach_offset_direction),
            },
            "approach_move_result": approach_move_result,
            "move_result": move_result,
        }

    seize_result = weapon_lib.weapon_seize(
        sender=sender,
        edge_arm_sec=grab_arm_sec,
        edge_hold_sec=grab_hold_sec,
    )
    lift_result = weapon_lib.weapon_up(
        sender=sender,
        mode_arm_sec=weapon_mode_settle_sec,
        hold_sec=lift_hold_sec,
    )

    # 第三段：保持夹取状态，以机器人参考点返回指定接近点。
    # 蓝场使用 1 号 weapon 坐标的当前地图 x- 接近点；
    # 当前地图坐标系下，红/蓝场默认返回朝向分别为 0.01/180deg。
    return_move_result = move_lib.move_to_target(
        sender=sender,
        position_runtime=position_runtime,
        odom_runtime=odom_runtime,
        target_x=return_move_x,
        target_y=return_move_y,
        final_target_yaw_deg=return_move_yaw_deg,
        cruise_forward_cmd=v,
        reference="robot",
    )

    return_stop_channels = move_lib.set_motion_channels(
        sender,
        lateral_cmd=0,
        forward_cmd=0,
        rotation_cmd=0,
    )

    # 第四段：按当前半场旋转到释放朝向，不做平移。
    release_rotate_result = move_lib.rotate_to_target_yaw_segmented(
        sender=sender,
        position_runtime=position_runtime,
        target_yaw_deg=release_rotate_yaw_deg,
    )
    release_rotate_hold_channels = move_lib.set_motion_channels(
        sender,
        lateral_cmd=0,
        forward_cmd=0,
        rotation_cmd=0,
    )

    return {
        "completed": True,
        "weapon_id": int(weapon_id),
        "des_weapon": {
            "x": float(des_x),
            "y": float(des_y),
        },
        "weapon_approach_target": {
            "x": float(approach_x),
            "y": float(approach_y),
            "yaw_deg": float(first_rotate_yaw_deg),
            "offset_distance": float(approach_offset),
            "offset_axis": "x",
            "offset_direction": int(approach_offset_direction),
        },
        "approach_move_result": approach_move_result,
        "move_result": move_result,
        "grab_result": {
            "mode_channels": lift_result["arm_channels"],
            "arm_channels": seize_result["arm_channels"],
            "fire_channels": seize_result["fire_channels"],
            "lift_channels": lift_result["fire_channels"],
            "seize_result": seize_result,
            "lift_result": lift_result,
            "weapon_mode_settle_sec": float(weapon_mode_settle_sec),
            "grab_arm_sec": float(grab_arm_sec),
            "grab_hold_sec": float(grab_hold_sec),
            "lift_hold_sec": float(lift_hold_sec),
            "completed": True,
        },
        "return_move_target": {
            "x": float(return_move_x),
            "y": float(return_move_y),
            "yaw_deg": float(return_move_yaw_deg),
            "reference_weapon_id": int(return_move_reference_weapon_id),
        },
        "return_move_result": return_move_result,
        "return_stop_channels": return_stop_channels,
        "release_rotate_target": {
            "yaw_deg": float(release_rotate_yaw_deg),
        },
        "release_rotate_result": release_rotate_result,
        "release_rotate_hold_channels": release_rotate_hold_channels,
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
    return_to_center=True,
):
    """
    组合式上楼梯动作：
    1. direction1/direction2 转成 des_deg1/des_deg2
    2. 调用高位微调逻辑完成上楼前对正和前探
    3. 调用 move.climb(...) 阻塞执行半自动上楼梯
    4. return_to_center=True 时调用 move_to_des(...) 移动到目标点 (x, y)，最终朝向 des_deg2
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

    return_to_center = bool(return_to_center)
    move_result = None
    if return_to_center:
        move_result = move_to_des(
            sender=sender,
            position_runtime=position_runtime,
            odom_runtime=odom_runtime,
            x=x,
            y=y,
            target_deg=des_deg2,
            v=STAIR_MOVE_MAX_CMD,
        )

    pre_climb_completed = bool(
        pre_climb_adjust_result.get("completed", False)
    )
    climb_completed = bool(
        climb_result is not None
        and climb_result.get("success", False)
    )
    move_completed = True if not return_to_center else move_result is not None
    completed = pre_climb_completed and climb_completed and move_completed
    failed_step = None
    if not pre_climb_completed:
        failed_step = "pre_climb_adjust"
    elif not climb_completed:
        failed_step = "climb_trigger"
    elif not move_completed:
        failed_step = "move_to_stair_center"

    return {
        "des_deg1": float(des_deg1),
        "des_deg2": float(des_deg2),
        "rotate_result": pre_climb_adjust_result.get("rotate_result"),
        "pre_climb_adjust_result": pre_climb_adjust_result,
        "pre_climb_drive_result": pre_climb_adjust_result.get("drive_result"),
        "climb_result": climb_result,
        "move_result": move_result,
        "return_to_center": bool(return_to_center),
        "completed": bool(completed),
        "failed_step": failed_step,
    }


def climb_R1(
    sender,
    position_runtime,
    odom_runtime,
    pre_climb_duration_sec=2.0,
):
    """
    R1 专用上楼组合动作。

    1. 面向 R1 爬坡方向：红场 180deg，蓝场 0.01deg。
       按当前四方向函数均使用 direction=2。
    2. 保持该航向，以 ch2=100 前进 pre_climb_duration_sec，默认 2s。
    3. 调用 move.climb() 触发并等待底层上楼完成。
    4. 上楼完成后停车并保持航向等待 4s。
    5. 保持航向，以 ch2=50 前进 1s，随后停车并进入锁轮状态。
    """
    pre_climb_duration_sec = float(pre_climb_duration_sec)
    if pre_climb_duration_sec < 0.0:
        raise ValueError(
            "pre_climb_duration_sec must be >= 0, "
            f"got {pre_climb_duration_sec}"
        )

    climb_direction = 2
    target_yaw_deg = tools.direction_int_to_yaw_deg(climb_direction)

    rotate_result = move_lib.rotate_to_target_yaw_segmented(
        sender=sender,
        position_runtime=position_runtime,
        odom_runtime=odom_runtime,
        target_yaw_deg=target_yaw_deg,
    )
    if rotate_result is None or rotate_result.get("timed_out", False):
        return {
            "completed": False,
            "failed_step": "rotate_to_climb_direction",
            "climb_direction": int(climb_direction),
            "target_yaw_deg": float(target_yaw_deg),
            "pre_climb_duration_sec": float(pre_climb_duration_sec),
            "rotate_result": rotate_result,
        }

    pre_climb_drive_result = move_lib.drive_with_channels_for_duration(
        sender=sender,
        duration_sec=pre_climb_duration_sec,
        forward_cmd=100,
        target_yaw_deg=target_yaw_deg,
        brake_reverse_cmd=0,
        brake_duration_sec=0.0,
    )
    if not pre_climb_drive_result.get("completed", False):
        return {
            "completed": False,
            "failed_step": "pre_climb_forward",
            "climb_direction": int(climb_direction),
            "target_yaw_deg": float(target_yaw_deg),
            "pre_climb_duration_sec": float(pre_climb_duration_sec),
            "rotate_result": rotate_result,
            "pre_climb_drive_result": pre_climb_drive_result,
        }

    climb_result = move_lib.climb(
        sender=sender,
        position_runtime=position_runtime,
    )
    if climb_result is None or not climb_result.get("success", False):
        return {
            "completed": False,
            "failed_step": "climb_trigger",
            "climb_direction": int(climb_direction),
            "target_yaw_deg": float(target_yaw_deg),
            "pre_climb_duration_sec": float(pre_climb_duration_sec),
            "rotate_result": rotate_result,
            "pre_climb_drive_result": pre_climb_drive_result,
            "climb_result": climb_result,
        }

    post_climb_wait_result = move_lib.wait_with_target_yaw(
        sender=sender,
        duration_sec=4.0,
        target_yaw_deg=target_yaw_deg,
    )
    post_climb_drive_result = move_lib.drive_with_channels_for_duration(
        sender=sender,
        duration_sec=1.0,
        forward_cmd=50,
        target_yaw_deg=target_yaw_deg,
        brake_reverse_cmd=0,
        brake_duration_sec=0.0,
    )
    lock_wheel_result = move_lib.lock_wheel(sender)
    stop_yaw_pid_channels = move_lib.set_motion_channels(
        sender,
        des_yaw_i16=0,
    )
    completed = bool(
        post_climb_wait_result.get("completed", False)
        and post_climb_drive_result.get("completed", False)
        and lock_wheel_result.get("completed", False)
    )
    failed_step = None
    if not post_climb_wait_result.get("completed", False):
        failed_step = "post_climb_wait"
    elif not post_climb_drive_result.get("completed", False):
        failed_step = "post_climb_forward"
    elif not lock_wheel_result.get("completed", False):
        failed_step = "lock_wheel"
    return {
        "completed": completed,
        "failed_step": failed_step,
        "climb_direction": int(climb_direction),
        "target_yaw_deg": float(target_yaw_deg),
        "pre_climb_duration_sec": float(pre_climb_duration_sec),
        "pre_climb_forward_cmd": 100,
        "post_climb_wait_sec": 4.0,
        "post_climb_duration_sec": 1.0,
        "post_climb_forward_cmd": 50,
        "rotate_result": rotate_result,
        "pre_climb_drive_result": pre_climb_drive_result,
        "climb_result": climb_result,
        "post_climb_wait_result": post_climb_wait_result,
        "post_climb_drive_result": post_climb_drive_result,
        "lock_wheel_result": lock_wheel_result,
        "stop_yaw_pid_channels": stop_yaw_pid_channels,
    }


def high_score190(
    sender,
    position_runtime,
    odom_runtime,
):
    """
    190 分高分组合流程。

    固定执行顺序：
      1. move.climb_R1_movement(...)
      2. climb_R1(...)
      3. kfs.place_3rd_kfs(...)
      4. kfs.sucker_release_pose(...)
      5. kfs.place_3rd_kfs(...)

    函数不接收额外业务输入，红蓝半场由各子流程根据 position_backend 自动判断。
    任一步失败后立即返回，不继续执行后续硬件动作。
    """
    field_name = "blue" if position_backend.is_blue_field() else "red"
    results = {
        "field_name": field_name,
        "suck_count_before": int(kfs.suck_count),
    }

    if int(kfs.suck_count) != 3:
        return {
            "completed": False,
            "failed_step": "invalid_suck_count_before_high_score190",
            "required_suck_count": 3,
            **results,
        }

    climb_r1_movement_result = move_lib.climb_R1_movement(
        sender=sender,
        position_runtime=position_runtime,
        odom_runtime=odom_runtime,
    )
    results["climb_r1_movement_result"] = climb_r1_movement_result
    if not climb_r1_movement_result.get("completed", False):
        return {
            "completed": False,
            "failed_step": "climb_R1_movement",
            **results,
        }

    climb_r1_result = climb_R1(
        sender=sender,
        position_runtime=position_runtime,
        odom_runtime=odom_runtime,
    )
    results["climb_r1_result"] = climb_r1_result
    if not climb_r1_result.get("completed", False):
        return {
            "completed": False,
            "failed_step": "climb_R1",
            **results,
        }

    first_place_3rd_kfs_result = kfs.place_3rd_kfs(
        sender=sender,
        position_runtime=position_runtime,
    )
    results["first_place_3rd_kfs_result"] = first_place_3rd_kfs_result
    if not first_place_3rd_kfs_result.get("completed", False):
        return {
            "completed": False,
            "failed_step": "first_place_3rd_kfs",
            "suck_count_after": int(kfs.suck_count),
            **results,
        }

    sucker_release_pose_result = kfs.sucker_release_pose(sender)
    results["sucker_release_pose_result"] = sucker_release_pose_result
    if not sucker_release_pose_result.get("completed", False):
        return {
            "completed": False,
            "failed_step": "sucker_release_pose",
            "suck_count_after": int(kfs.suck_count),
            **results,
        }

    second_place_3rd_kfs_result = kfs.place_3rd_kfs(
        sender=sender,
        position_runtime=position_runtime,
    )
    results["second_place_3rd_kfs_result"] = second_place_3rd_kfs_result
    completed = bool(second_place_3rd_kfs_result.get("completed", False))
    return {
        "completed": completed,
        "failed_step": None if completed else "second_place_3rd_kfs",
        "suck_count_after": int(kfs.suck_count),
        **results,
    }


def totally_win(
    sender,
    position_runtime,
    odom_runtime,
):
    """
    全胜组合流程。

    固定执行顺序：
      1. move.move_to_2rd_place(...)
      2. kfs.release_kfs(...)
      3. move.climb_R1_movement(...)
      4. climb_R1(...)
      5. kfs.sucker_release_pose(...)
      6. kfs.place_3rd_kfs(...)

    函数不接收额外业务输入，红蓝半场由各子流程根据 position_backend 自动判断。
    任一步失败后立即返回，不继续执行后续硬件动作。
    """
    field_name = "blue" if position_backend.is_blue_field() else "red"
    results = {
        "field_name": field_name,
        "suck_count_before": int(kfs.suck_count),
    }

    if int(kfs.suck_count) != 3:
        return {
            "completed": False,
            "failed_step": "invalid_suck_count_before_totally_win",
            "required_suck_count": 3,
            **results,
        }

    move_to_column2_result = move_lib.move_to_2rd_place(
        sender=sender,
        position_runtime=position_runtime,
        odom_runtime=odom_runtime,
    )
    results["move_to_column2_result"] = move_to_column2_result
    if not move_to_column2_result.get("completed", False):
        return {
            "completed": False,
            "failed_step": "move_to_column2",
            **results,
        }

    release_kfs_result = kfs.release_kfs(sender)
    results["release_kfs_result"] = release_kfs_result
    if not release_kfs_result.get("completed", False):
        return {
            "completed": False,
            "failed_step": "release_kfs",
            "suck_count_after": int(kfs.suck_count),
            **results,
        }

    climb_r1_movement_result = move_lib.climb_R1_movement(
        sender=sender,
        position_runtime=position_runtime,
        odom_runtime=odom_runtime,
    )
    results["climb_r1_movement_result"] = climb_r1_movement_result
    if not climb_r1_movement_result.get("completed", False):
        return {
            "completed": False,
            "failed_step": "climb_R1_movement",
            "suck_count_after": int(kfs.suck_count),
            **results,
        }

    climb_r1_result = climb_R1(
        sender=sender,
        position_runtime=position_runtime,
        odom_runtime=odom_runtime,
    )
    results["climb_r1_result"] = climb_r1_result
    if not climb_r1_result.get("completed", False):
        return {
            "completed": False,
            "failed_step": "climb_R1",
            "suck_count_after": int(kfs.suck_count),
            **results,
        }

    sucker_release_pose_result = kfs.sucker_release_pose(sender)
    results["sucker_release_pose_result"] = sucker_release_pose_result
    if not sucker_release_pose_result.get("completed", False):
        return {
            "completed": False,
            "failed_step": "sucker_release_pose",
            "suck_count_after": int(kfs.suck_count),
            **results,
        }

    place_3rd_kfs_result = kfs.place_3rd_kfs(
        sender=sender,
        position_runtime=position_runtime,
    )
    results["place_3rd_kfs_result"] = place_3rd_kfs_result
    completed = bool(place_3rd_kfs_result.get("completed", False))
    return {
        "completed": completed,
        "failed_step": None if completed else "place_3rd_kfs",
        "suck_count_after": int(kfs.suck_count),
        **results,
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
    return_to_center=True,
):
    """
    组合式下楼梯动作：
    1. direction1 取反方向并转换为下楼前对正角。
    2. 阻塞旋转到该反方向角。
    3. 调用 move.descend(...) 执行底层阻塞式下楼梯控制。
    4. return_to_center=True 时调用 move_backward_to_des(...) 倒退移动到 (des_x, des_y)，最终朝向 direction2 对应角度。
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

    return_to_center = bool(return_to_center)
    move_result = None
    if return_to_center:
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
        "return_to_center": bool(return_to_center),
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
    return_to_center=True,
    return_center_skip_reason=None,
):
    """
    根据 from/to 高低关系执行一次上下楼梯动作。

    height_relation:
      1: to 比 from 高，调用 climb(...)
      2: to 比 from 低，调用 descend(...)

    task_direction 是本次上下楼方向，final_direction 是动作完成后的最终朝向。
    return_to_center=False 时只执行上下楼，不执行最后到 to_pos 中心的移动。
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
            return_to_center=return_to_center,
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
            "return_to_center": bool(return_to_center),
            "return_center_skip_reason": return_center_skip_reason,
            "stair_result": stair_result,
            "completed": bool(stair_result.get("completed", False)),
            "failed_step": (
                None
                if stair_result.get("completed", False)
                else stair_result.get("failed_step", "climb")
            ),
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
        return_to_center=return_to_center,
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
        "return_to_center": bool(return_to_center),
        "return_center_skip_reason": return_center_skip_reason,
        "stair_result": stair_result,
        "completed": bool(stair_result.get("completed", False)),
        "failed_step": (
            None
            if stair_result.get("completed", False)
            else stair_result.get("failed_step", "descend")
        ),
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

    -1->1/-1->3 优先分流到 side_suck()；其他行完成方向、
    相邻性和高低关系校验后，再分流到 KFS 抓取或上下楼动作。
    """
    row_values = _action_row_to_list(action_row)
    from_pos, to_pos, move_dir, height_action, grab_action = [
        _action_value_to_int(value, ACTION_MATRIX_COLUMNS[index])
        for index, value in enumerate(row_values)
    ]

    # 场外 1/3 号格侧吸是特殊动作，-1 与 1/3 不是普通相邻台阶。
    # 必须在方向、相邻性和高低关系校验前直接分流。
    is_pre_entry_side_suck = (
        from_pos == -1
        and to_pos in (1, 3)
        and move_dir == 1
        and height_action == 0
        and grab_action == 1
    )
    if is_pre_entry_side_suck:
        side_suck_result = side_suck(
            sender=sender,
            position_runtime=position_runtime,
            odom_runtime=odom_runtime,
            target_stair_id=to_pos,
        )
        return {
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
            "branch": "pre_entry_side_suck",
            "side_suck_result": side_suck_result,
            "completed": bool(side_suck_result.get("completed", False)),
            "implemented": bool(side_suck_result.get("implemented", True)),
            "failed_step": side_suck_result.get("failed_step"),
        }

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
                result["completed"] = False
                result["implemented"] = True
                result["failed_step"] = fetch_result.get(
                    "failed_step",
                    "fetch_and_store_kfs",
                )
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
                result["completed"] = True
                result["implemented"] = True
                result["failed_step"] = None
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
            result["completed"] = return_center_result is not None
            result["implemented"] = True
            result["failed_step"] = (
                None if return_center_result is not None else "return_to_stair_center"
            )
            return result

        if height_action != 0:
            return_to_center = True
            return_center_skip_reason = None
            next_stair_inferred_direction = tools.stair_id_to_direction(
                next_from_pose,
                next_to_pose,
                exit_on_error=False,
            )
            next_stair_height_relation = 0
            if next_stair_inferred_direction in (1, 2, 3, 4):
                next_stair_height_relation = get_stair_height_relation(
                    next_from_pose,
                    next_stair_inferred_direction,
                )

                if (
                    height_relation == 1
                    and next_stair_inferred_direction == move_dir
                    and next_stair_height_relation == height_relation
                ):
                    return_to_center = False
                    return_center_skip_reason = "next_same_direction_higher_stair"
                elif (
                    height_relation == 2
                    and next_height_action == 1
                    and next_stair_inferred_direction == move_dir
                    and next_stair_height_relation == height_relation
                ):
                    return_to_center = False
                    return_center_skip_reason = "next_same_direction_descend"

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
                return_to_center=return_to_center,
                return_center_skip_reason=return_center_skip_reason,
            )
            result["branch"] = "directional"
            result["next_stair_inferred_direction"] = int(next_stair_inferred_direction)
            result["next_stair_height_relation"] = int(next_stair_height_relation)
            result["return_to_center"] = bool(return_to_center)
            result["return_center_skip_reason"] = return_center_skip_reason
            result["stair_transition_result"] = stair_transition_result
            result["completed"] = bool(
                stair_transition_result.get("completed", False)
            )
            result["implemented"] = True
            result["failed_step"] = stair_transition_result.get("failed_step")
            return result

        # TODO: 第一段逻辑：有方向动作，后续在这里接入夹取、上下楼梯和移动。
        result["branch"] = "directional"
        result["completed"] = False
        result["implemented"] = False
        result["failed_step"] = "directional_branch_not_implemented"
        return result

    # TODO: 第二段逻辑：原地动作，后续在这里接入原地夹取/等待等逻辑。
    result["branch"] = "stationary"
    result["completed"] = False
    result["implemented"] = False
    result["failed_step"] = "stationary_branch_not_implemented"
    return result


def execute_action_matrix(
    sender,
    position_runtime,
    odom_runtime,
    action_matrix,
    final_direction=None,
    stop_on_unimplemented=True,
    stop_on_failed=True,
):
    """
    顺序执行动作矩阵。

    action_matrix: n*5，每行格式同 execute_action_row()。
    final_direction: 没有下一行有效移动时使用的默认最终朝向。
      None 时使用当前半场下 180deg 对应方向码：红场 2，蓝场 3。
    stop_on_unimplemented: 遇到 execute_action_row() 返回 implemented=False 时是否终止。
    stop_on_failed: 遇到已实现但 completed=False 的动作行时是否终止。
    """
    if final_direction is None:
        final_direction = 3 if position_backend.is_blue_field() else 2
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
    failed_row_index = None
    failure_reason = None
    for row_index, action_row in enumerate(rows):
        row_kwargs = {}
        row_final_direction = final_direction
        if row_index + 1 < row_count:
            next_row_values = _action_row_to_list(rows[row_index + 1])
            next_from_pose = _action_value_to_int(
                next_row_values[0],
                "next_from_pose",
            )
            next_to_pose = _action_value_to_int(
                next_row_values[1],
                "next_to_pose",
            )
            next_inferred_direction = tools.stair_id_to_direction(
                next_from_pose,
                next_to_pose,
                exit_on_error=False,
            )
            next_height_action = _action_value_to_int(
                next_row_values[3],
                "next_height_action",
            )
            next_grab_action = _action_value_to_int(
                next_row_values[4],
                "next_grab_action",
            )
            if next_inferred_direction in (1, 2, 3, 4):
                row_final_direction = next_inferred_direction
                if next_height_action == 1 and next_grab_action == 0:
                    next_height_relation = get_stair_height_relation(
                        next_from_pose,
                        next_inferred_direction,
                    )
                    if next_height_relation == 2:
                        opposite_direction_map = {
                            1: 4,
                            2: 3,
                            3: 2,
                            4: 1,
                        }
                        row_final_direction = opposite_direction_map[
                            next_inferred_direction
                        ]
            row_kwargs = {
                "next_from_pose": next_from_pose,
                "next_to_pose": next_to_pose,
                "next_height_action": next_height_action,
            }

        row_result = execute_action_row(
            sender=sender,
            position_runtime=position_runtime,
            odom_runtime=odom_runtime,
            action_row=action_row,
            final_direction=row_final_direction,
            **row_kwargs,
        )
        row_result["row_index"] = int(row_index)
        row_result["default_final_direction"] = int(final_direction)
        row_result["row_final_direction"] = int(row_final_direction)
        results.append(row_result)

        if stop_on_unimplemented and not row_result.get("implemented", False):
            print(
                f"{execute_action_matrix.__name__}输入错误: "
                f"第 {row_index} 行尚未接入真实动作 "
                f"action_row={row_result.get('action_row')}"
            )
            sys.exit(1)

        if not row_result.get("completed", False):
            if failed_row_index is None:
                failed_row_index = int(row_index)
                failure_reason = row_result.get("failed_step", "action_row_failed")
            if stop_on_failed:
                print(
                    f"{execute_action_matrix.__name__}执行失败: "
                    f"第 {row_index} 行未完成 "
                    f"action_row={row_result.get('action_row')}, "
                    f"failed_step={failure_reason}"
                )
                break

    return {
        "completed": failed_row_index is None and len(results) == row_count,
        "row_count": row_count,
        "executed_row_count": len(results),
        "final_direction": int(final_direction),
        "stopped_early": len(results) < row_count,
        "failed_row_index": failed_row_index,
        "failure_reason": failure_reason,
        "results": results,
    }
