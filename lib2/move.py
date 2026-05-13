import math
import sys

from lib2 import tools
from lib2 import position_backend


position_lib = position_backend.get_position_backend()


# 最终方向到达阈值，当前 yaw 与目标 yaw 的误差小于该值后认为旋转到位，单位 deg。
DEFAULT_DIRECTION_THRESHOLD_DEG = 2.0
# 默认 yaw 到达阈值；旋转和最终方向检测默认共用 DEFAULT_DIRECTION_THRESHOLD_DEG。
DEFAULT_YAW_TOLERANCE_DEG = DEFAULT_DIRECTION_THRESHOLD_DEG
# 方向到达后需要连续保持在阈值内的时间，单位 s；用于避免刚进阈值就退出后又过冲。
DEFAULT_DIRECTION_STABLE_SEC = 1.0
# 原地旋转/方向检测循环周期，单位 s。
DEFAULT_ROTATE_LOOP_INTERVAL_SEC = 0.02
# 原地旋转默认超时时间，单位 s；None 表示不超时。
DEFAULT_ROTATE_TIMEOUT_SEC = 20.0
# 分段旋转每次给下位机的目标航向增量，单位 deg，用于降低一次性大角度目标对PID的冲击。
DEFAULT_SEGMENTED_YAW_STEP_DEG = 10.0
# 分段旋转每个中间目标至少保持的时间，单位 s。
DEFAULT_SEGMENTED_YAW_HOLD_SEC = 0.08
# 定时通道运动和上台阶等待循环周期，单位 s。
DEFAULT_DRIVE_LOOP_INTERVAL_SEC = 0.02
# 移动到点主控制循环周期，单位 s。
DEFAULT_MOVE_LOOP_INTERVAL_SEC = 0.02
# 移动时允许前进的最大航向误差，超过该角度则 ch2=0 只转向，单位 deg。
DEFAULT_MOVE_GATE_DEG = 10.0
# 距离目标较远时的默认前进通道值。
DEFAULT_MOVE_FORWARD_CMD = 500
# 进入近点区后的前进通道值。
DEFAULT_NEAR_FORWARD_CMD = 100
# 近点区距离阈值，距离小于该值后使用 DEFAULT_NEAR_FORWARD_CMD，单位 m。
DEFAULT_NEAR_TARGET_DISTANCE = 1.0
# 临点区前进通道值，用于最后低速靠近目标点。
DEFAULT_FINE_FORWARD_CMD = 50
# 临点区距离阈值，距离小于该值后使用 DEFAULT_FINE_FORWARD_CMD，单位 m。
DEFAULT_FINE_TARGET_DISTANCE = 0.5
# 过点检测距离反弹阈值；当前距离大于历史最小距离加该值时计一次过点，单位 m。
DEFAULT_OVERSHOOT_DISTANCE_EPS = 0.02
# 过点检测只在足够接近目标点时启用，避免远距离 TF 抖动误判，单位 m。
DEFAULT_OVERSHOOT_CHECK_DISTANCE = 0.15
# 连续多少帧满足过点检测后锁定为已经冲过目标点。
DEFAULT_OVERSHOOT_CONFIRM_COUNT = 3
# 到点距离阈值，距离小于该值后认为位置满足到点条件，单位 m。
DEFAULT_STOP_DISTANCE = 0.02
# 到点速度阈值，平面线速度小于该值后认为速度足够低，单位 m/s。
DEFAULT_REACHED_SPEED_MPS = 0.05
# 到点 yaw 角速度阈值，角速度小于该值后认为旋转足够稳定，单位 rad/s。
DEFAULT_REACHED_YAW_RATE_RAD = 0.05
# 过点回退时的 ch2 通道值，负数表示后退。
DEFAULT_REACHED_REVERSE_CMD = -50
# 过点回退最大持续时间，单位 s，防止回退距离条件无法满足时死等。
DEFAULT_REACHED_REVERSE_DURATION_SEC = 1.0
# 回退保护距离反弹阈值；回退期间当前距离大于本次回退最小距离加该值时计一次反弹，单位 m。
DEFAULT_RECOVERY_DISTANCE_EPS = 0.01
# 回退保护确认帧数；连续多少帧距离反弹后停止回退。
DEFAULT_RECOVERY_WORSE_CONFIRM_COUNT = 3
# 上台阶成功高度裕量，current_z - z_margin > start_z 时认为高度条件成功，单位 m。
DEFAULT_CLIMB_SUCCESS_Z_MARGIN = 0.15
# 上台阶成功等待默认超时时间，单位 s。
DEFAULT_CLIMB_SUCCESS_TIMEOUT_SEC = 10.0
# 上楼梯完成所需的最小水平位移，单位 m。
DEFAULT_CLIMB_MIN_DISTANCE = 0.4
# 上楼梯触发通道保持时间，单位 s。
DEFAULT_CLIMB_TRIGGER_HOLD_SEC = 3.0
# 上楼梯触发前用于形成 1 -> 3 上升沿的预置时间，单位 s。
DEFAULT_CLIMB_TRIGGER_ARM_SEC = 0.1
# 上楼梯流程最大允许时间，超过该时间后打印错误并终止程序，单位 s。
DEFAULT_CLIMB_TIMEOUT_SEC = 15.0
# 下楼梯触发通道保持时间，单位 s。
DEFAULT_DESCEND_TRIGGER_HOLD_SEC = 3.0
# 下楼梯触发前用于形成离开3再进入3的预置时间，单位 s。
DEFAULT_DESCEND_TRIGGER_ARM_SEC = 0.1
# 升降模式通道索引，ch5。
CLIMB_MODE_CHANNEL_INDEX = 5
# 升降模式值，ch5 == 1。
CLIMB_MODE_VALUE = 1
# 上/下楼梯触发通道索引，ch7。
CLIMB_TRIGGER_CHANNEL_INDEX = 7
# 触发通道预置值，用于形成 1 -> 3 上升沿。
CLIMB_TRIGGER_ARM_VALUE = 1
# 触发通道触发值。
CLIMB_TRIGGER_FIRE_VALUE = 3
# 触发完成后的回归值；新协议中二段开关空闲值使用 1。
CLIMB_TRIGGER_IDLE_VALUE = 1
# 下楼梯触发通道索引，ch6。
DESCEND_TRIGGER_CHANNEL_INDEX = 6
# 下楼梯触发通道预置值，用于离开3，保证下一次进入3可触发。
DESCEND_TRIGGER_ARM_VALUE = 1
# 下楼梯触发值。
DESCEND_TRIGGER_FIRE_VALUE = 3
# 下楼梯触发完成后的回归值。
DESCEND_TRIGGER_IDLE_VALUE = 1
# KFS/方块模式通道索引，ch5。
KFS_MODE_CHANNEL_INDEX = 5
# KFS/方块模式值，ch5 == 2。
KFS_MODE_VALUE = 2
# KFS 姿态选择通道索引，ch6。
KFS_POSE_CHANNEL_INDEX = 6
# KFS 姿态触发通道索引，ch7。
KFS_TRIGGER_CHANNEL_INDEX = 7
# KFS 姿态触发预置值，ch7 常态。
KFS_TRIGGER_ARM_VALUE = 1
# KFS 姿态触发值，用于形成 1 -> 3 上升沿。
KFS_TRIGGER_FIRE_VALUE = 3
# KFS 回归 0 态触发值。
KFS_TRIGGER_ZERO_VALUE = 0
# KFS 姿态触发前等待时间，单位 s。
DEFAULT_KFS_POSE_ARM_WAIT_SEC = 0.5
# KFS 姿态触发后保持输出等待时间，单位 s。
DEFAULT_KFS_POSE_HOLD_SEC = 2.0
# KFS 回归 0 态阻塞等待时间，单位 s。
DEFAULT_KFS_ZERO_RETURN_WAIT_SEC = 2.0


def normalize_yaw_deg(yaw_deg):
    return tools.yaw_normalization(float(yaw_deg))


def heading_error_deg(current_yaw_deg, target_yaw_deg):
    return normalize_yaw_deg(float(target_yaw_deg) - float(current_yaw_deg))


def is_heading_reached(current_yaw_deg, target_yaw_deg, tolerance_deg=DEFAULT_YAW_TOLERANCE_DEG):
    return abs(heading_error_deg(current_yaw_deg, target_yaw_deg)) <= float(tolerance_deg)


def wait_until_direction_reached(
    position_runtime,
    target_yaw_deg,
    threshold_deg=DEFAULT_DIRECTION_THRESHOLD_DEG,
    stable_sec=DEFAULT_DIRECTION_STABLE_SEC,
    loop_interval_sec=DEFAULT_ROTATE_LOOP_INTERVAL_SEC,
    timeout_sec=None,
):
    """
    阻塞式方向检测。

    只读取 position_runtime 当前朝向，不修改 sender 控制状态。
    当前朝向进入 target_yaw_deg +- threshold_deg 后返回。
    """
    target_yaw_deg = normalize_yaw_deg(target_yaw_deg)
    deadline = None if timeout_sec is None else (tools.time.time() + float(timeout_sec))
    result = None
    stable_since = None

    while True:
        current_yaw_deg = position_runtime.get_current_yaw_deg()
        now = tools.time.time()
        if current_yaw_deg is not None:
            error_deg = heading_error_deg(current_yaw_deg, target_yaw_deg)
            in_threshold = abs(error_deg) <= float(threshold_deg)
            if in_threshold:
                if stable_since is None:
                    stable_since = now
            else:
                stable_since = None
            stable_elapsed_sec = 0.0 if stable_since is None else (now - stable_since)
            reached = in_threshold and stable_elapsed_sec >= float(stable_sec)
            result = {
                "current_yaw_deg": float(current_yaw_deg),
                "target_yaw_deg": float(target_yaw_deg),
                "threshold_deg": float(threshold_deg),
                "stable_sec": float(stable_sec),
                "stable_elapsed_sec": float(stable_elapsed_sec),
                "heading_error_deg": float(error_deg),
                "heading_reached": bool(reached),
            }
            if reached:
                return result

        if deadline is not None and tools.time.time() >= deadline:
            if result is None:
                return None
            result["timed_out"] = True
            return result

        tools.time.sleep(loop_interval_sec)


def encode_target_yaw_i16(target_yaw_deg):
    target_yaw_i16 = tools.yaw_deg_to_i16(float(target_yaw_deg))
    if target_yaw_i16 == 0:
        return 1
    return target_yaw_i16


def rotate_to_target_yaw(
    sender,
    position_runtime,
    target_yaw_deg,
    tolerance_deg=DEFAULT_YAW_TOLERANCE_DEG,
    stable_sec=DEFAULT_DIRECTION_STABLE_SEC,
    loop_interval_sec=DEFAULT_ROTATE_LOOP_INTERVAL_SEC,
    timeout_sec=DEFAULT_ROTATE_TIMEOUT_SEC,
):
    """
    阻塞式原地对正到目标角。

    行为:
    - ch0/ch2/ch3 全部置 0
    - 只设置目标航向角 des_yaw_i16
    - 当前航向角由 position_runtime 后台线程持续更新到 sender
    - 持续等待，直到达到目标角容差范围内才返回
    """
    if float(target_yaw_deg) == 0.0:
        channels = tools.compose_channels(lateral_cmd=0, forward_cmd=0, rotation_cmd=0)
        sender.set_channels_and_des_yaw_i16(channels, 0)
        return {
            "channels": channels,
            "current_yaw_deg": position_runtime.get_current_yaw_deg(),
            "target_yaw_deg": 0.0,
            "des_yaw_i16": 0,
            "heading_error_deg": None,
            "heading_reached": True,
            "rotation_control_stopped": True,
        }

    target_yaw_deg = normalize_yaw_deg(target_yaw_deg)
    des_yaw_i16 = encode_target_yaw_i16(target_yaw_deg)
    channels = tools.compose_channels(lateral_cmd=0, forward_cmd=0, rotation_cmd=0)
    deadline = None if timeout_sec is None else (tools.time.time() + float(timeout_sec))
    stable_since = None

    while True:
        current_yaw_deg = position_runtime.get_current_yaw_deg()
        now = tools.time.time()
        if current_yaw_deg is not None:
            sender.set_channels_and_des_yaw_i16(channels, des_yaw_i16)
            error_deg = heading_error_deg(current_yaw_deg, target_yaw_deg)
            in_threshold = abs(error_deg) <= float(tolerance_deg)
            if in_threshold:
                if stable_since is None:
                    stable_since = now
            else:
                stable_since = None
            stable_elapsed_sec = 0.0 if stable_since is None else (now - stable_since)
            reached = in_threshold and stable_elapsed_sec >= float(stable_sec)
            result = {
                "channels": channels,
                "current_yaw_deg": float(current_yaw_deg),
                "target_yaw_deg": float(target_yaw_deg),
                "des_yaw_i16": int(des_yaw_i16),
                "heading_error_deg": float(error_deg),
                "stable_sec": float(stable_sec),
                "stable_elapsed_sec": float(stable_elapsed_sec),
                "heading_reached": reached,
            }
            if reached:
                return result

        if deadline is not None and tools.time.time() >= deadline:
            if current_yaw_deg is None:
                return None
            result["timed_out"] = True
            return result

        tools.time.sleep(loop_interval_sec)


def rotate_to_target_yaw_segmented(
    sender,
    position_runtime,
    target_yaw_deg,
    segment_step_deg=DEFAULT_SEGMENTED_YAW_STEP_DEG,
    tolerance_deg=DEFAULT_YAW_TOLERANCE_DEG,
    stable_sec=DEFAULT_DIRECTION_STABLE_SEC,
    segment_hold_sec=DEFAULT_SEGMENTED_YAW_HOLD_SEC,
    loop_interval_sec=DEFAULT_ROTATE_LOOP_INTERVAL_SEC,
    timeout_sec=DEFAULT_ROTATE_TIMEOUT_SEC,
):
    """
    分段式原地对正到目标角。

    每轮读取当前 yaw，沿 current -> target 的劣弧方向给一个最多
    segment_step_deg 的中间目标；当剩余误差小于该步长后，直接给最终目标角。
    """
    if float(target_yaw_deg) == 0.0:
        channels = tools.compose_channels(lateral_cmd=0, forward_cmd=0, rotation_cmd=0)
        sender.set_channels_and_des_yaw_i16(channels, 0)
        return {
            "channels": channels,
            "current_yaw_deg": position_runtime.get_current_yaw_deg(),
            "target_yaw_deg": 0.0,
            "des_yaw_deg": None,
            "des_yaw_i16": 0,
            "heading_error_deg": None,
            "heading_reached": True,
            "stable_sec": float(stable_sec),
            "stable_elapsed_sec": 0.0,
            "segment_step_deg": float(segment_step_deg),
            "segment_count": 0,
            "rotation_control_stopped": True,
        }

    target_yaw_deg = normalize_yaw_deg(target_yaw_deg)
    segment_step_deg = abs(float(segment_step_deg))
    if segment_step_deg <= 0.0:
        raise ValueError(f"segment_step_deg must be > 0, got {segment_step_deg}")
    if segment_hold_sec < 0.0:
        raise ValueError(f"segment_hold_sec must be >= 0, got {segment_hold_sec}")

    channels = tools.compose_channels(lateral_cmd=0, forward_cmd=0, rotation_cmd=0)
    deadline = None if timeout_sec is None else (tools.time.time() + float(timeout_sec))
    result = None
    active_des_yaw_deg = None
    last_segment_update_time = 0.0
    segment_count = 0
    stable_since = None

    while True:
        current_yaw_deg = position_runtime.get_current_yaw_deg()
        now = tools.time.time()

        if current_yaw_deg is not None:
            current_yaw_deg = normalize_yaw_deg(current_yaw_deg)
            final_error_deg = heading_error_deg(current_yaw_deg, target_yaw_deg)
            in_threshold = abs(final_error_deg) <= float(tolerance_deg)
            if in_threshold:
                if stable_since is None:
                    stable_since = now
            else:
                stable_since = None
            stable_elapsed_sec = 0.0 if stable_since is None else (now - stable_since)

            if in_threshold and stable_elapsed_sec >= float(stable_sec):
                final_des_yaw_i16 = encode_target_yaw_i16(target_yaw_deg)
                sender.set_channels_and_des_yaw_i16(channels, final_des_yaw_i16)
                return {
                    "channels": channels,
                    "current_yaw_deg": float(current_yaw_deg),
                    "target_yaw_deg": float(target_yaw_deg),
                    "des_yaw_deg": float(target_yaw_deg),
                    "des_yaw_i16": int(final_des_yaw_i16),
                    "heading_error_deg": float(final_error_deg),
                    "heading_reached": True,
                    "stable_sec": float(stable_sec),
                    "stable_elapsed_sec": float(stable_elapsed_sec),
                    "segment_step_deg": float(segment_step_deg),
                    "segment_count": int(segment_count),
                }

            should_update_segment = (
                active_des_yaw_deg is None
                or (now - last_segment_update_time) >= float(segment_hold_sec)
            )
            if should_update_segment:
                if abs(final_error_deg) <= segment_step_deg:
                    active_des_yaw_deg = target_yaw_deg
                else:
                    step = math.copysign(segment_step_deg, final_error_deg)
                    active_des_yaw_deg = normalize_yaw_deg(current_yaw_deg + step)
                last_segment_update_time = now
                segment_count += 1

            des_yaw_i16 = encode_target_yaw_i16(active_des_yaw_deg)
            sender.set_channels_and_des_yaw_i16(channels, des_yaw_i16)
            result = {
                "channels": channels,
                "current_yaw_deg": float(current_yaw_deg),
                "target_yaw_deg": float(target_yaw_deg),
                "des_yaw_deg": float(active_des_yaw_deg),
                "des_yaw_i16": int(des_yaw_i16),
                "heading_error_deg": float(final_error_deg),
                "heading_reached": False,
                "stable_sec": float(stable_sec),
                "stable_elapsed_sec": float(stable_elapsed_sec),
                "segment_step_deg": float(segment_step_deg),
                "segment_count": int(segment_count),
            }

        if deadline is not None and tools.time.time() >= deadline:
            if result is None:
                return None
            result["timed_out"] = True
            return result

        tools.time.sleep(loop_interval_sec)


def wait_with_target_yaw(
    sender,
    duration_sec,
    target_yaw_deg,
    loop_interval_sec=DEFAULT_DRIVE_LOOP_INTERVAL_SEC,
):
    """
    阻塞式等待指定时间。

    等待期间保持 ch0/ch2/ch3 为 0，不执行平移/前进/旋转通道动作；
    同时持续输出输入的目标航向角。
    """
    if duration_sec < 0.0:
        raise ValueError(f"duration_sec must be >= 0, got {duration_sec}")

    target_yaw_deg = normalize_yaw_deg(target_yaw_deg)
    des_yaw_i16 = encode_target_yaw_i16(target_yaw_deg)
    channels = tools.compose_channels(lateral_cmd=0, forward_cmd=0, rotation_cmd=0)
    deadline = tools.time.time() + float(duration_sec)

    while tools.time.time() < deadline:
        sender.set_channels_and_des_yaw_i16(channels, des_yaw_i16)
        tools.time.sleep(loop_interval_sec)

    sender.set_channels_and_des_yaw_i16(channels, des_yaw_i16)
    return {
        "channels": channels,
        "duration_sec": float(duration_sec),
        "target_yaw_deg": float(target_yaw_deg),
        "des_yaw_i16": int(des_yaw_i16),
        "completed": True,
    }


def _compose_kfs_pose_channels(pose_id, trigger_value, suction_ch4=tools.SAFE_SWITCH_VALUE):
    channels = tools.compose_channels(lateral_cmd=0, forward_cmd=0, rotation_cmd=0)
    channels[4] = int(suction_ch4)
    channels[KFS_MODE_CHANNEL_INDEX] = KFS_MODE_VALUE
    channels[KFS_POSE_CHANNEL_INDEX] = int(pose_id)
    channels[KFS_TRIGGER_CHANNEL_INDEX] = int(trigger_value)
    return channels


def control_kfs_pose(
    sender,
    pose_id,
    arm_wait_sec=DEFAULT_KFS_POSE_ARM_WAIT_SEC,
    hold_sec=DEFAULT_KFS_POSE_HOLD_SEC,
    zero_return_wait_sec=DEFAULT_KFS_ZERO_RETURN_WAIT_SEC,
    loop_interval_sec=DEFAULT_DRIVE_LOOP_INTERVAL_SEC,
    suction_ch4=tools.SAFE_SWITCH_VALUE,
):
    """
    阻塞式 KFS 吸盘机械臂姿态控制。

    pose_id:
      0: 回归 0 态，发送 ch7=0 并等待 zero_return_wait_sec 后退出。
      1: 高位方块抓取姿态。
      2: 低位方块抓取姿态。
      3: 过渡态。
      4: 存储方块姿态。

    1~4 态时序：
      1. ch5=2 进入 KFS 模式，ch6=pose_id，ch7=1，底盘通道和目标航向角置 0。
      2. 保持 arm_wait_sec。
      3. ch7 从 1 变为 3 触发姿态动作。
      4. 保持 hold_sec。
      5. ch6/ch7 回归常态，其中 ch7 常态为 1。

    suction_ch4:
      保持 KFS 吸盘通道值；默认 1。吸取后继续调整姿态时应传 3，
      避免姿态控制帧把 ch4 拉回 1 形成释放下降沿。
    """
    pose_id = int(pose_id)
    if pose_id not in (0, 1, 2, 3, 4):
        print("KFS姿态输入错误")
        sys.exit(1)

    if pose_id == 0:
        zero_channels = _compose_kfs_pose_channels(
            pose_id=0,
            trigger_value=KFS_TRIGGER_ZERO_VALUE,
            suction_ch4=suction_ch4,
        )
        deadline = tools.time.time() + float(zero_return_wait_sec)
        while tools.time.time() < deadline:
            sender.set_channels_and_des_yaw_i16(zero_channels, 0)
            tools.time.sleep(float(loop_interval_sec))
        sender.set_channels_and_des_yaw_i16(zero_channels, 0)
        return {
            "pose_id": 0,
            "pose_name": "zero_return",
            "channels": zero_channels,
            "des_yaw_i16": 0,
            "zero_return_wait_sec": float(zero_return_wait_sec),
            "completed": True,
        }

    arm_channels = _compose_kfs_pose_channels(
        pose_id=pose_id,
        trigger_value=KFS_TRIGGER_ARM_VALUE,
        suction_ch4=suction_ch4,
    )
    arm_deadline = tools.time.time() + float(arm_wait_sec)
    while tools.time.time() < arm_deadline:
        sender.set_channels_and_des_yaw_i16(arm_channels, 0)
        tools.time.sleep(float(loop_interval_sec))

    fire_channels = _compose_kfs_pose_channels(
        pose_id=pose_id,
        trigger_value=KFS_TRIGGER_FIRE_VALUE,
        suction_ch4=suction_ch4,
    )
    hold_deadline = tools.time.time() + float(hold_sec)
    while tools.time.time() < hold_deadline:
        sender.set_channels_and_des_yaw_i16(fire_channels, 0)
        tools.time.sleep(float(loop_interval_sec))

    idle_channels = _compose_kfs_pose_channels(
        pose_id=1,
        trigger_value=KFS_TRIGGER_ARM_VALUE,
        suction_ch4=suction_ch4,
    )
    sender.set_channels_and_des_yaw_i16(idle_channels, 0)
    return {
        "pose_id": int(pose_id),
        "arm_channels": arm_channels,
        "fire_channels": fire_channels,
        "idle_channels": idle_channels,
        "des_yaw_i16": 0,
        "arm_wait_sec": float(arm_wait_sec),
        "hold_sec": float(hold_sec),
        "completed": True,
    }


def drive_with_channels_for_duration(
    sender,
    channels,
    duration_sec,
    target_yaw_deg=None,
    loop_interval_sec=DEFAULT_DRIVE_LOOP_INTERVAL_SEC,
    brake_reverse_cmd=-50,
    brake_duration_sec=1.0,
):
    """
    阻塞式按指定通道值持续运动一段时间。

    行为:
    - 持续时间内反复设置 channels
    - 如果给了 target_yaw_deg，则同时持续设置目标航向角
    - 时间到后先以 ch2=brake_reverse_cmd 持续 brake_duration_sec
    - 然后将前进通道清零再返回
    """
    if duration_sec < 0.0:
        raise ValueError(f"duration_sec must be >= 0, got {duration_sec}")

    if len(channels) != tools.CHANNEL_COUNT:
        raise ValueError(f"channels length must be {tools.CHANNEL_COUNT}")

    deadline = tools.time.time() + float(duration_sec)
    des_yaw_i16 = None
    if target_yaw_deg is not None:
        des_yaw_i16 = encode_target_yaw_i16(normalize_yaw_deg(target_yaw_deg))

    while tools.time.time() < deadline:
        if des_yaw_i16 is None:
            sender.set_channels(channels)
        else:
            sender.set_channels_and_des_yaw_i16(channels, des_yaw_i16)
        tools.time.sleep(loop_interval_sec)

    brake_channels = list(channels)
    brake_channels[2] = int(brake_reverse_cmd)
    brake_deadline = tools.time.time() + float(brake_duration_sec)
    while tools.time.time() < brake_deadline:
        if des_yaw_i16 is None:
            sender.set_channels(brake_channels)
        else:
            sender.set_channels_and_des_yaw_i16(brake_channels, des_yaw_i16)
        tools.time.sleep(loop_interval_sec)

    stop_channels = list(brake_channels)
    stop_channels[2] = 0
    if des_yaw_i16 is None:
        sender.set_channels(stop_channels)
    else:
        sender.set_channels_and_des_yaw_i16(stop_channels, des_yaw_i16)

    return {
        "channels": list(channels),
        "duration_sec": float(duration_sec),
        "target_yaw_deg": None if target_yaw_deg is None else float(normalize_yaw_deg(target_yaw_deg)),
        "des_yaw_i16": des_yaw_i16,
        "brake_reverse_cmd": int(brake_reverse_cmd),
        "brake_duration_sec": float(brake_duration_sec),
        "completed": True,
    }


def distance_to_target_xy(current_x, current_y, target_x, target_y):
    dx = float(target_x) - float(current_x)
    dy = float(target_y) - float(current_y)
    return math.hypot(dx, dy), dx, dy


def values_are_finite(*values):
    return all(math.isfinite(float(value)) for value in values)


def is_near_target(
    current_x,
    current_y,
    target_x,
    target_y,
    near_target_distance=DEFAULT_NEAR_TARGET_DISTANCE,
):
    distance_xy, dx, dy = distance_to_target_xy(
        current_x=current_x,
        current_y=current_y,
        target_x=target_x,
        target_y=target_y,
    )
    return {
        "distance_xy": float(distance_xy),
        "dx": float(dx),
        "dy": float(dy),
        "near_target_distance": float(near_target_distance),
        "near": distance_xy <= float(near_target_distance),
    }


def is_target_reached(
    current_x,
    current_y,
    target_x,
    target_y,
    linear_speed_mps,
    angular_z_rad,
    stop_distance=DEFAULT_STOP_DISTANCE,
    reached_speed_mps=DEFAULT_REACHED_SPEED_MPS,
    reached_yaw_rate_rad=DEFAULT_REACHED_YAW_RATE_RAD,
):
    distance_xy, dx, dy = distance_to_target_xy(
        current_x=current_x,
        current_y=current_y,
        target_x=target_x,
        target_y=target_y,
    )
    reached = (
        distance_xy <= float(stop_distance)
        and float(linear_speed_mps) <= float(reached_speed_mps)
        and abs(float(angular_z_rad)) <= float(reached_yaw_rate_rad)
    )
    return {
        "distance_xy": float(distance_xy),
        "dx": float(dx),
        "dy": float(dy),
        "linear_speed_mps": float(linear_speed_mps),
        "angular_z_rad": float(angular_z_rad),
        "reached": reached,
    }


def get_reference_pose(position_runtime, reference="robot"):
    reference = str(reference).lower()
    if reference == "robot":
        return position_runtime.get_robot_pose()
    if reference == "weapon":
        return position_runtime.get_weapon_pose()
    raise ValueError(f"reference must be 'robot' or 'weapon', got {reference}")


def move_to_target(
    sender,
    position_runtime,
    odom_runtime,
    target_x,
    target_y,
    final_target_yaw_deg=0.01,
    move_gate_deg=DEFAULT_MOVE_GATE_DEG,
    cruise_forward_cmd=DEFAULT_MOVE_FORWARD_CMD,
    near_forward_cmd=DEFAULT_NEAR_FORWARD_CMD,
    near_target_distance=DEFAULT_NEAR_TARGET_DISTANCE,
    fine_forward_cmd=DEFAULT_FINE_FORWARD_CMD,
    fine_target_distance=DEFAULT_FINE_TARGET_DISTANCE,
    overshoot_distance_eps=DEFAULT_OVERSHOOT_DISTANCE_EPS,
    overshoot_check_distance=DEFAULT_OVERSHOOT_CHECK_DISTANCE,
    overshoot_confirm_count=DEFAULT_OVERSHOOT_CONFIRM_COUNT,
    stop_distance=DEFAULT_STOP_DISTANCE,
    reached_speed_mps=DEFAULT_REACHED_SPEED_MPS,
    reached_yaw_rate_rad=DEFAULT_REACHED_YAW_RATE_RAD,
    reached_reverse_cmd=DEFAULT_REACHED_REVERSE_CMD,
    reached_reverse_duration_sec=DEFAULT_REACHED_REVERSE_DURATION_SEC,
    recovery_distance_eps=DEFAULT_RECOVERY_DISTANCE_EPS,
    recovery_worse_confirm_count=DEFAULT_RECOVERY_WORSE_CONFIRM_COUNT,
    loop_interval_sec=DEFAULT_MOVE_LOOP_INTERVAL_SEC,
    timeout_sec=None,
    reference="robot",
):
    """
    阻塞式移动到目标点。

    循环逻辑:
    1. 根据 reference 参考点与目标点计算 map 下绝对目标航向角
    2. 若偏差角大于 +-move_gate_deg，则停车，只保持目标角
    3. 若进入近点区，则前进通道改为 near_forward_cmd
    4. 若距离小于 fine_target_distance，则前进通道改为 fine_forward_cmd
    5. 每轮做原版到点判定，并记录历史最小距离
    6. 若足够接近目标点且连续三帧相对历史最小距离明显变大，则判定越过目标点
    7. 正常到点后停车并切到 final_target_yaw_deg 后退出
    8. 过点后目标角先置 0（协议特殊值：停止旋转），后退到距离小于 stop_distance
    9. 回退到点后切到 final_target_yaw_deg 后退出

    reference:
      "robot": 以机器人中心作为到点参考。
      "weapon": 以 weapon/夹爪作为到点参考，底盘 yaw 仍使用机器人 yaw。
    """
    reference = str(reference).lower()
    if reference not in ("robot", "weapon"):
        raise ValueError(f"reference must be 'robot' or 'weapon', got {reference}")
    if not values_are_finite(target_x, target_y):
        raise ValueError(
            "target_x and target_y must be finite, got "
            f"{target_x}, {target_y}"
        )
    if final_target_yaw_deg is not None and not values_are_finite(final_target_yaw_deg):
        raise ValueError(
            "final_target_yaw_deg must be finite or None, got "
            f"{final_target_yaw_deg}"
        )

    deadline = None if timeout_sec is None else (tools.time.time() + float(timeout_sec))
    best_distance_xy = None
    overshoot_hits = 0
    result = None

    while True:
        robot_pose = position_runtime.get_robot_pose()
        reference_pose = get_reference_pose(position_runtime, reference=reference)
        odometry = odom_runtime.get_odometry(max_age_sec=None)

        if robot_pose is not None and reference_pose is not None and odometry is not None:
            current_x = float(reference_pose["x"])
            current_y = float(reference_pose["y"])
            current_z = float(reference_pose["z"])
            current_yaw_deg = normalize_yaw_deg(
                position_lib.radians_to_degrees(float(robot_pose["yaw"]))
            )
            linear_x = float(odometry["linear_x"])
            linear_y = float(odometry["linear_y"])
            angular_z_rad = float(odometry["angular_z"])

            if not values_are_finite(
                current_x,
                current_y,
                current_z,
                current_yaw_deg,
                linear_x,
                linear_y,
                angular_z_rad,
            ):
                stop_channels = tools.compose_channels(lateral_cmd=0, forward_cmd=0, rotation_cmd=0)
                sender.set_channels_and_des_yaw_i16(stop_channels, 0)
                tools.time.sleep(loop_interval_sec)
                continue

            target_yaw_deg = position_lib.cal_target_yaw_deg(
                target_x=target_x,
                target_y=target_y,
                robot_x=current_x,
                robot_y=current_y,
            )
            if not values_are_finite(target_yaw_deg):
                stop_channels = tools.compose_channels(lateral_cmd=0, forward_cmd=0, rotation_cmd=0)
                sender.set_channels_and_des_yaw_i16(stop_channels, 0)
                tools.time.sleep(loop_interval_sec)
                continue

            yaw_error_deg = heading_error_deg(current_yaw_deg, target_yaw_deg)

            near_result = is_near_target(
                current_x=current_x,
                current_y=current_y,
                target_x=target_x,
                target_y=target_y,
                near_target_distance=near_target_distance,
            )
            linear_speed_mps = math.hypot(linear_x, linear_y)

            result = is_target_reached(
                current_x=current_x,
                current_y=current_y,
                target_x=target_x,
                target_y=target_y,
                linear_speed_mps=linear_speed_mps,
                angular_z_rad=angular_z_rad,
                stop_distance=stop_distance,
                reached_speed_mps=reached_speed_mps,
                reached_yaw_rate_rad=reached_yaw_rate_rad,
            )

            current_forward_cmd = int(cruise_forward_cmd)
            if near_result["near"]:
                current_forward_cmd = int(near_forward_cmd)
            if float(result["distance_xy"]) < float(fine_target_distance):
                current_forward_cmd = int(fine_forward_cmd)

            if abs(yaw_error_deg) > float(move_gate_deg):
                current_forward_cmd = 0

            channels = tools.compose_channels(
                lateral_cmd=0,
                forward_cmd=current_forward_cmd,
                rotation_cmd=0,
            )
            sender.set_channels_and_des_yaw_i16(
                channels,
                encode_target_yaw_i16(target_yaw_deg),
            )

            result.update(near_result)
            result.update({
                "current_x": current_x,
                "current_y": current_y,
                "current_z": current_z,
                "current_yaw_deg": current_yaw_deg,
                "target_x": float(target_x),
                "target_y": float(target_y),
                "target_yaw_deg": float(target_yaw_deg),
                "heading_error_deg": float(yaw_error_deg),
                "reference": reference,
                "forward_cmd": int(current_forward_cmd),
                "move_gate_deg": float(move_gate_deg),
                "fine_forward_cmd": int(fine_forward_cmd),
                "fine_target_distance": float(fine_target_distance),
                "channels": channels,
            })

            current_distance_xy = float(result["distance_xy"])
            if best_distance_xy is None or current_distance_xy < best_distance_xy:
                best_distance_xy = current_distance_xy
                overshoot_hits = 0
            else:
                overshoot_detected = (
                    current_distance_xy <= float(overshoot_check_distance)
                    and current_distance_xy > (best_distance_xy + float(overshoot_distance_eps))
                )
                if overshoot_detected:
                    overshoot_hits += 1
                else:
                    overshoot_hits = 0

            overshoot_locked = overshoot_hits >= int(overshoot_confirm_count)
            result.update({
                "best_distance_xy": None if best_distance_xy is None else float(best_distance_xy),
                "overshoot_distance_eps": float(overshoot_distance_eps),
                "overshoot_check_distance": float(overshoot_check_distance),
                "overshoot_hits": int(overshoot_hits),
                "overshoot_locked": bool(overshoot_locked),
            })

            if result["reached"]:
                stop_channels = tools.compose_channels(
                    lateral_cmd=0,
                    forward_cmd=0,
                    rotation_cmd=0,
                )
                final_des_yaw_i16 = (
                    0
                    if final_target_yaw_deg is None
                    else encode_target_yaw_i16(final_target_yaw_deg)
                )
                sender.set_channels_and_des_yaw_i16(
                    stop_channels,
                    final_des_yaw_i16,
                )
                result.update({
                    "channels": stop_channels,
                    "final_target_yaw_deg": (
                        None
                        if final_target_yaw_deg is None
                        else float(normalize_yaw_deg(final_target_yaw_deg))
                    ),
                    "des_yaw_i16": int(final_des_yaw_i16),
                    "completed_by_overshoot_lock": False,
                })
                return result

            if overshoot_locked:
                reverse_channels = tools.compose_channels(
                    lateral_cmd=0,
                    forward_cmd=int(reached_reverse_cmd),
                    rotation_cmd=0,
                )
                reverse_started_at = tools.time.time()
                reverse_deadline = reverse_started_at + float(reached_reverse_duration_sec)
                reverse_result = dict(result)
                best_recovery_distance_xy = None
                recovery_worse_hits = 0
                while True:
                    recovery_pose = get_reference_pose(position_runtime, reference=reference)
                    if recovery_pose is not None:
                        recovery_distance_xy, recovery_dx, recovery_dy = distance_to_target_xy(
                            current_x=recovery_pose["x"],
                            current_y=recovery_pose["y"],
                            target_x=target_x,
                            target_y=target_y,
                        )
                        reverse_result.update({
                            "current_x": float(recovery_pose["x"]),
                            "current_y": float(recovery_pose["y"]),
                            "current_z": float(recovery_pose["z"]),
                            "distance_xy": float(recovery_distance_xy),
                            "dx": float(recovery_dx),
                            "dy": float(recovery_dy),
                            "recovery_distance_xy": float(recovery_distance_xy),
                        })
                        if recovery_distance_xy <= float(stop_distance):
                            break
                        if best_recovery_distance_xy is None or recovery_distance_xy < best_recovery_distance_xy:
                            best_recovery_distance_xy = recovery_distance_xy
                            recovery_worse_hits = 0
                        else:
                            recovery_worse_detected = (
                                recovery_distance_xy
                                > (best_recovery_distance_xy + float(recovery_distance_eps))
                            )
                            if recovery_worse_detected:
                                recovery_worse_hits += 1
                            else:
                                recovery_worse_hits = 0
                        reverse_result.update({
                            "best_recovery_distance_xy": float(best_recovery_distance_xy),
                            "recovery_worse_hits": int(recovery_worse_hits),
                            "recovery_distance_eps": float(recovery_distance_eps),
                            "recovery_worse_confirm_count": int(recovery_worse_confirm_count),
                        })
                        if recovery_worse_hits >= int(recovery_worse_confirm_count):
                            reverse_result["recovery_stopped_by_worse_distance"] = True
                            break

                    sender.set_channels_and_des_yaw_i16(
                        reverse_channels,
                        0,
                    )
                    if tools.time.time() >= reverse_deadline:
                        reverse_result["recovery_timed_out"] = True
                        break
                    if deadline is not None and tools.time.time() >= deadline:
                        reverse_result["timed_out"] = True
                        break
                    tools.time.sleep(loop_interval_sec)

                stop_channels = tools.compose_channels(
                    lateral_cmd=0,
                    forward_cmd=0,
                    rotation_cmd=0,
                )
                final_des_yaw_i16 = (
                    0
                    if final_target_yaw_deg is None
                    else encode_target_yaw_i16(final_target_yaw_deg)
                )
                sender.set_channels_and_des_yaw_i16(
                    stop_channels,
                    final_des_yaw_i16,
                )
                reverse_result.update({
                    "channels": stop_channels,
                    "final_target_yaw_deg": (
                        None
                        if final_target_yaw_deg is None
                        else float(normalize_yaw_deg(final_target_yaw_deg))
                    ),
                    "reached_reverse_cmd": int(reached_reverse_cmd),
                    "reached_reverse_duration_sec": float(tools.time.time() - reverse_started_at),
                    "completed_by_overshoot_lock": True,
                    "des_yaw_i16": int(final_des_yaw_i16),
                })
                return reverse_result

        if deadline is not None and tools.time.time() >= deadline:
            if robot_pose is None or reference_pose is None or odometry is None:
                return None
            if result is None:
                return None
            result["timed_out"] = True
            return result

        tools.time.sleep(loop_interval_sec)


def move_backward_to_target(
    sender,
    position_runtime,
    odom_runtime,
    target_x,
    target_y,
    final_target_yaw_deg=0.01,
    move_gate_deg=DEFAULT_MOVE_GATE_DEG,
    cruise_backward_cmd=-DEFAULT_MOVE_FORWARD_CMD,
    near_backward_cmd=-DEFAULT_NEAR_FORWARD_CMD,
    near_target_distance=DEFAULT_NEAR_TARGET_DISTANCE,
    fine_backward_cmd=-DEFAULT_FINE_FORWARD_CMD,
    fine_target_distance=DEFAULT_FINE_TARGET_DISTANCE,
    overshoot_distance_eps=DEFAULT_OVERSHOOT_DISTANCE_EPS,
    overshoot_check_distance=DEFAULT_OVERSHOOT_CHECK_DISTANCE,
    overshoot_confirm_count=DEFAULT_OVERSHOOT_CONFIRM_COUNT,
    stop_distance=DEFAULT_STOP_DISTANCE,
    reached_speed_mps=DEFAULT_REACHED_SPEED_MPS,
    reached_yaw_rate_rad=DEFAULT_REACHED_YAW_RATE_RAD,
    reached_forward_cmd=abs(DEFAULT_REACHED_REVERSE_CMD),
    reached_forward_duration_sec=DEFAULT_REACHED_REVERSE_DURATION_SEC,
    recovery_distance_eps=DEFAULT_RECOVERY_DISTANCE_EPS,
    recovery_worse_confirm_count=DEFAULT_RECOVERY_WORSE_CONFIRM_COUNT,
    loop_interval_sec=DEFAULT_MOVE_LOOP_INTERVAL_SEC,
    timeout_sec=None,
    reference="robot",
):
    """
    阻塞式倒退移动到目标点。

    逻辑与 move_to_target 基本一致，但底盘保持背向目标点：
    - 目标方向 target_direction_deg 仍由 reference 参考点指向目标点。
    - 实际航向 backward_target_yaw_deg = target_direction_deg + 180deg。
    - 主运动使用负 ch2 倒退到点。
    - 若过点锁定，使用正 ch2 前进修回目标点。

    reference:
      "robot": 以机器人中心作为到点参考。
      "weapon": 以 weapon/夹爪作为到点参考，底盘 yaw 仍使用机器人 yaw。
    """
    reference = str(reference).lower()
    if reference not in ("robot", "weapon"):
        raise ValueError(f"reference must be 'robot' or 'weapon', got {reference}")
    if not values_are_finite(target_x, target_y):
        raise ValueError(
            "target_x and target_y must be finite, got "
            f"{target_x}, {target_y}"
        )
    if final_target_yaw_deg is not None and not values_are_finite(final_target_yaw_deg):
        raise ValueError(
            "final_target_yaw_deg must be finite or None, got "
            f"{final_target_yaw_deg}"
        )

    cruise_backward_cmd = -abs(int(cruise_backward_cmd))
    near_backward_cmd = -abs(int(near_backward_cmd))
    fine_backward_cmd = -abs(int(fine_backward_cmd))
    reached_forward_cmd = abs(int(reached_forward_cmd))

    deadline = None if timeout_sec is None else (tools.time.time() + float(timeout_sec))
    best_distance_xy = None
    overshoot_hits = 0
    result = None

    while True:
        robot_pose = position_runtime.get_robot_pose()
        reference_pose = get_reference_pose(position_runtime, reference=reference)
        odometry = odom_runtime.get_odometry(max_age_sec=None)

        if robot_pose is not None and reference_pose is not None and odometry is not None:
            current_x = float(reference_pose["x"])
            current_y = float(reference_pose["y"])
            current_z = float(reference_pose["z"])
            current_yaw_deg = normalize_yaw_deg(
                position_lib.radians_to_degrees(float(robot_pose["yaw"]))
            )
            linear_x = float(odometry["linear_x"])
            linear_y = float(odometry["linear_y"])
            angular_z_rad = float(odometry["angular_z"])

            if not values_are_finite(
                current_x,
                current_y,
                current_z,
                current_yaw_deg,
                linear_x,
                linear_y,
                angular_z_rad,
            ):
                stop_channels = tools.compose_channels(lateral_cmd=0, forward_cmd=0, rotation_cmd=0)
                sender.set_channels_and_des_yaw_i16(stop_channels, 0)
                tools.time.sleep(loop_interval_sec)
                continue

            target_direction_deg = position_lib.cal_target_yaw_deg(
                target_x=target_x,
                target_y=target_y,
                robot_x=current_x,
                robot_y=current_y,
            )
            backward_target_yaw_deg = normalize_yaw_deg(target_direction_deg + 180.0)
            if not values_are_finite(target_direction_deg, backward_target_yaw_deg):
                stop_channels = tools.compose_channels(lateral_cmd=0, forward_cmd=0, rotation_cmd=0)
                sender.set_channels_and_des_yaw_i16(stop_channels, 0)
                tools.time.sleep(loop_interval_sec)
                continue

            yaw_error_deg = heading_error_deg(current_yaw_deg, backward_target_yaw_deg)

            near_result = is_near_target(
                current_x=current_x,
                current_y=current_y,
                target_x=target_x,
                target_y=target_y,
                near_target_distance=near_target_distance,
            )
            linear_speed_mps = math.hypot(linear_x, linear_y)

            result = is_target_reached(
                current_x=current_x,
                current_y=current_y,
                target_x=target_x,
                target_y=target_y,
                linear_speed_mps=linear_speed_mps,
                angular_z_rad=angular_z_rad,
                stop_distance=stop_distance,
                reached_speed_mps=reached_speed_mps,
                reached_yaw_rate_rad=reached_yaw_rate_rad,
            )

            current_backward_cmd = int(cruise_backward_cmd)
            if near_result["near"]:
                current_backward_cmd = int(near_backward_cmd)
            if float(result["distance_xy"]) < float(fine_target_distance):
                current_backward_cmd = int(fine_backward_cmd)

            if abs(yaw_error_deg) > float(move_gate_deg):
                current_backward_cmd = 0

            channels = tools.compose_channels(
                lateral_cmd=0,
                forward_cmd=current_backward_cmd,
                rotation_cmd=0,
            )
            sender.set_channels_and_des_yaw_i16(
                channels,
                encode_target_yaw_i16(backward_target_yaw_deg),
            )

            result.update(near_result)
            result.update({
                "current_x": current_x,
                "current_y": current_y,
                "current_z": current_z,
                "current_yaw_deg": current_yaw_deg,
                "target_x": float(target_x),
                "target_y": float(target_y),
                "target_direction_deg": float(target_direction_deg),
                "target_yaw_deg": float(backward_target_yaw_deg),
                "backward_target_yaw_deg": float(backward_target_yaw_deg),
                "heading_error_deg": float(yaw_error_deg),
                "reference": reference,
                "forward_cmd": int(current_backward_cmd),
                "backward_cmd": int(current_backward_cmd),
                "move_gate_deg": float(move_gate_deg),
                "fine_backward_cmd": int(fine_backward_cmd),
                "fine_target_distance": float(fine_target_distance),
                "channels": channels,
            })

            current_distance_xy = float(result["distance_xy"])
            if best_distance_xy is None or current_distance_xy < best_distance_xy:
                best_distance_xy = current_distance_xy
                overshoot_hits = 0
            else:
                overshoot_detected = (
                    current_distance_xy <= float(overshoot_check_distance)
                    and current_distance_xy > (best_distance_xy + float(overshoot_distance_eps))
                )
                if overshoot_detected:
                    overshoot_hits += 1
                else:
                    overshoot_hits = 0

            overshoot_locked = overshoot_hits >= int(overshoot_confirm_count)
            result.update({
                "best_distance_xy": None if best_distance_xy is None else float(best_distance_xy),
                "overshoot_distance_eps": float(overshoot_distance_eps),
                "overshoot_check_distance": float(overshoot_check_distance),
                "overshoot_hits": int(overshoot_hits),
                "overshoot_locked": bool(overshoot_locked),
            })

            if result["reached"]:
                stop_channels = tools.compose_channels(
                    lateral_cmd=0,
                    forward_cmd=0,
                    rotation_cmd=0,
                )
                final_des_yaw_i16 = (
                    0
                    if final_target_yaw_deg is None
                    else encode_target_yaw_i16(final_target_yaw_deg)
                )
                sender.set_channels_and_des_yaw_i16(
                    stop_channels,
                    final_des_yaw_i16,
                )
                result.update({
                    "channels": stop_channels,
                    "final_target_yaw_deg": (
                        None
                        if final_target_yaw_deg is None
                        else float(normalize_yaw_deg(final_target_yaw_deg))
                    ),
                    "des_yaw_i16": int(final_des_yaw_i16),
                    "completed_by_overshoot_lock": False,
                })
                return result

            if overshoot_locked:
                forward_channels = tools.compose_channels(
                    lateral_cmd=0,
                    forward_cmd=int(reached_forward_cmd),
                    rotation_cmd=0,
                )
                forward_started_at = tools.time.time()
                forward_deadline = forward_started_at + float(reached_forward_duration_sec)
                forward_result = dict(result)
                best_recovery_distance_xy = None
                recovery_worse_hits = 0
                while True:
                    recovery_pose = get_reference_pose(position_runtime, reference=reference)
                    if recovery_pose is not None:
                        recovery_distance_xy, recovery_dx, recovery_dy = distance_to_target_xy(
                            current_x=recovery_pose["x"],
                            current_y=recovery_pose["y"],
                            target_x=target_x,
                            target_y=target_y,
                        )
                        forward_result.update({
                            "current_x": float(recovery_pose["x"]),
                            "current_y": float(recovery_pose["y"]),
                            "current_z": float(recovery_pose["z"]),
                            "distance_xy": float(recovery_distance_xy),
                            "dx": float(recovery_dx),
                            "dy": float(recovery_dy),
                            "recovery_distance_xy": float(recovery_distance_xy),
                        })
                        if recovery_distance_xy <= float(stop_distance):
                            break
                        if best_recovery_distance_xy is None or recovery_distance_xy < best_recovery_distance_xy:
                            best_recovery_distance_xy = recovery_distance_xy
                            recovery_worse_hits = 0
                        else:
                            recovery_worse_detected = (
                                recovery_distance_xy
                                > (best_recovery_distance_xy + float(recovery_distance_eps))
                            )
                            if recovery_worse_detected:
                                recovery_worse_hits += 1
                            else:
                                recovery_worse_hits = 0
                        forward_result.update({
                            "best_recovery_distance_xy": float(best_recovery_distance_xy),
                            "recovery_worse_hits": int(recovery_worse_hits),
                            "recovery_distance_eps": float(recovery_distance_eps),
                            "recovery_worse_confirm_count": int(recovery_worse_confirm_count),
                        })
                        if recovery_worse_hits >= int(recovery_worse_confirm_count):
                            forward_result["recovery_stopped_by_worse_distance"] = True
                            break

                    sender.set_channels_and_des_yaw_i16(
                        forward_channels,
                        0,
                    )
                    if tools.time.time() >= forward_deadline:
                        forward_result["recovery_timed_out"] = True
                        break
                    if deadline is not None and tools.time.time() >= deadline:
                        forward_result["timed_out"] = True
                        break
                    tools.time.sleep(loop_interval_sec)

                stop_channels = tools.compose_channels(
                    lateral_cmd=0,
                    forward_cmd=0,
                    rotation_cmd=0,
                )
                final_des_yaw_i16 = (
                    0
                    if final_target_yaw_deg is None
                    else encode_target_yaw_i16(final_target_yaw_deg)
                )
                sender.set_channels_and_des_yaw_i16(
                    stop_channels,
                    final_des_yaw_i16,
                )
                forward_result.update({
                    "channels": stop_channels,
                    "final_target_yaw_deg": (
                        None
                        if final_target_yaw_deg is None
                        else float(normalize_yaw_deg(final_target_yaw_deg))
                    ),
                    "reached_forward_cmd": int(reached_forward_cmd),
                    "reached_forward_duration_sec": float(tools.time.time() - forward_started_at),
                    "completed_by_overshoot_lock": True,
                    "des_yaw_i16": int(final_des_yaw_i16),
                })
                return forward_result

        if deadline is not None and tools.time.time() >= deadline:
            if robot_pose is None or reference_pose is None or odometry is None:
                return None
            if result is None:
                return None
            result["timed_out"] = True
            return result

        tools.time.sleep(loop_interval_sec)


def climb(
    sender,
    position_runtime,
    min_distance=DEFAULT_CLIMB_MIN_DISTANCE,
    trigger_hold_sec=DEFAULT_CLIMB_TRIGGER_HOLD_SEC,
    trigger_arm_sec=DEFAULT_CLIMB_TRIGGER_ARM_SEC,
    loop_interval_sec=DEFAULT_DRIVE_LOOP_INTERVAL_SEC,
    timeout_sec=DEFAULT_CLIMB_TIMEOUT_SEC,
):
    """
    阻塞式上楼梯控制。

    逻辑：
    1. 记录进入函数时的 x/y/z。
    2. 输出升降模式 ch5=1，底盘通道 ch0/ch2/ch3 保持 0。
    3. ch7 先置 1，再置 3，形成 1 -> 3 上升沿触发半自动上楼梯。
    4. ch7=3 保持 trigger_hold_sec 后回归 0。
    5. 阻塞等待水平位移大于 min_distance，且 current_z - 0.15 > start_z。
    6. 总流程超过 timeout_sec 后打印“上楼梯错误”并终止程序。
    """
    start_position = position_runtime.get_current_position()
    if start_position is None:
        return None

    start_x = float(start_position["x"])
    start_y = float(start_position["y"])
    start_z = float(start_position["z"])

    def climb_channels(trigger_value):
        channels = tools.compose_channels(lateral_cmd=0, forward_cmd=0, rotation_cmd=0)
        channels[CLIMB_MODE_CHANNEL_INDEX] = CLIMB_MODE_VALUE
        channels[CLIMB_TRIGGER_CHANNEL_INDEX] = int(trigger_value)
        return channels

    arm_channels = climb_channels(CLIMB_TRIGGER_ARM_VALUE)
    arm_deadline = tools.time.time() + float(trigger_arm_sec)
    while tools.time.time() < arm_deadline:
        sender.set_channels_and_des_yaw_i16(arm_channels, 0)
        tools.time.sleep(loop_interval_sec)

    fire_channels = climb_channels(CLIMB_TRIGGER_FIRE_VALUE)
    fire_deadline = tools.time.time() + float(trigger_hold_sec)
    while tools.time.time() < fire_deadline:
        sender.set_channels_and_des_yaw_i16(fire_channels, 0)
        tools.time.sleep(loop_interval_sec)

    idle_channels = climb_channels(CLIMB_TRIGGER_IDLE_VALUE)
    sender.set_channels_and_des_yaw_i16(idle_channels, 0)

    deadline = None if timeout_sec is None else (tools.time.time() + float(timeout_sec))
    result = {
        "start_x": start_x,
        "start_y": start_y,
        "start_z": start_z,
        "min_distance": float(min_distance),
        "z_margin": 0.15,
        "channels": idle_channels,
        "des_yaw_i16": 0,
    }

    while True:
        current_position = position_runtime.get_current_position()
        if current_position is not None:
            current_x = float(current_position["x"])
            current_y = float(current_position["y"])
            current_z = float(current_position["z"])
            distance_xy = math.hypot(current_x - start_x, current_y - start_y)
            height_reached = (current_z - 0.15) > start_z
            distance_reached = distance_xy > float(min_distance)
            result.update({
                "current_x": current_x,
                "current_y": current_y,
                "current_z": current_z,
                "distance_xy": float(distance_xy),
                "distance_reached": bool(distance_reached),
                "height_reached": bool(height_reached),
                "success": bool(distance_reached and height_reached),
            })
            if result["success"]:
                return result

        sender.set_channels_and_des_yaw_i16(idle_channels, 0)

        if deadline is not None and tools.time.time() >= deadline:
            print("上楼梯错误")
            result["timed_out"] = True
            sys.exit(1)

        tools.time.sleep(loop_interval_sec)


def descend(
    sender,
    position_runtime,
    trigger_hold_sec=DEFAULT_DESCEND_TRIGGER_HOLD_SEC,
    trigger_arm_sec=DEFAULT_DESCEND_TRIGGER_ARM_SEC,
    loop_interval_sec=DEFAULT_DRIVE_LOOP_INTERVAL_SEC,
):
    """
    阻塞式下楼梯控制。

    逻辑：
    1. 记录进入函数时的 z。
    2. 输出升降模式 ch5=1，底盘通道 ch0/ch2/ch3 保持 0。
    3. ch6 先置 1，再置 3，触发下位机自动下楼梯。
    4. ch6=3 保持 trigger_hold_sec 后回归 1。
    5. 阻塞等待 start_z - 0.15 > current_z，认为下楼梯完成。
    """
    start_position = position_runtime.get_current_position()
    if start_position is None:
        return None

    start_z = float(start_position["z"])

    def descend_channels(trigger_value):
        channels = tools.compose_channels(lateral_cmd=0, forward_cmd=0, rotation_cmd=0)
        channels[CLIMB_MODE_CHANNEL_INDEX] = CLIMB_MODE_VALUE
        channels[DESCEND_TRIGGER_CHANNEL_INDEX] = int(trigger_value)
        return channels

    arm_channels = descend_channels(DESCEND_TRIGGER_ARM_VALUE)
    arm_deadline = tools.time.time() + float(trigger_arm_sec)
    while tools.time.time() < arm_deadline:
        sender.set_channels_and_des_yaw_i16(arm_channels, 0)
        tools.time.sleep(loop_interval_sec)

    fire_channels = descend_channels(DESCEND_TRIGGER_FIRE_VALUE)
    fire_deadline = tools.time.time() + float(trigger_hold_sec)
    while tools.time.time() < fire_deadline:
        sender.set_channels_and_des_yaw_i16(fire_channels, 0)
        tools.time.sleep(loop_interval_sec)

    idle_channels = descend_channels(DESCEND_TRIGGER_IDLE_VALUE)
    sender.set_channels_and_des_yaw_i16(idle_channels, 0)

    result = {
        "start_z": start_z,
        "z_margin": 0.15,
        "arm_channels": arm_channels,
        "fire_channels": fire_channels,
        "idle_channels": idle_channels,
        "des_yaw_i16": 0,
        "trigger_arm_sec": float(trigger_arm_sec),
        "trigger_hold_sec": float(trigger_hold_sec),
    }

    while True:
        current_position = position_runtime.get_current_position()
        if current_position is not None:
            current_z = float(current_position["z"])
            height_reached = (start_z - 0.15) > current_z
            result.update({
                "current_z": current_z,
                "height_reached": bool(height_reached),
                "success": bool(height_reached),
            })
            if height_reached:
                return result

        sender.set_channels_and_des_yaw_i16(idle_channels, 0)
        tools.time.sleep(loop_interval_sec)


def wait_until_climb_success(
    position_runtime,
    odom_runtime,
    target_x,
    target_y,
    start_z,
    z_margin=DEFAULT_CLIMB_SUCCESS_Z_MARGIN,
    stop_distance=DEFAULT_STOP_DISTANCE,
    reached_speed_mps=DEFAULT_REACHED_SPEED_MPS,
    reached_yaw_rate_rad=DEFAULT_REACHED_YAW_RATE_RAD,
    loop_interval_sec=DEFAULT_DRIVE_LOOP_INTERVAL_SEC,
    timeout_sec=DEFAULT_CLIMB_SUCCESS_TIMEOUT_SEC,
    require_both=False,
):
    """
    阻塞式等待上台阶成功。

    并行检查两类条件：
    1. 高度成功条件：current_z - z_margin > start_z
    2. 到点成功条件：满足 is_target_reached(...)

    默认任一条件成立即返回成功；若 require_both=True，则必须两者都成立。
    """
    deadline = None if timeout_sec is None else (tools.time.time() + float(timeout_sec))

    while True:
        current_position = position_runtime.get_current_position()
        odometry = odom_runtime.get_odometry()
        result = None

        if current_position is not None:
            current_z = float(current_position["z"])
            height_reached = (current_z - float(z_margin)) > float(start_z)
            result = {
                "current_x": float(current_position["x"]),
                "current_y": float(current_position["y"]),
                "current_z": current_z,
                "target_x": float(target_x),
                "target_y": float(target_y),
                "start_z": float(start_z),
                "z_margin": float(z_margin),
                "height_reached": height_reached,
            }

            point_reached = False
            if odometry is not None:
                linear_x = float(odometry["linear_x"])
                linear_y = float(odometry["linear_y"])
                linear_speed_mps = math.hypot(linear_x, linear_y)
                angular_z_rad = float(odometry["angular_z"])

                point_result = is_target_reached(
                    current_x=current_position["x"],
                    current_y=current_position["y"],
                    target_x=target_x,
                    target_y=target_y,
                    linear_speed_mps=linear_speed_mps,
                    angular_z_rad=angular_z_rad,
                    stop_distance=stop_distance,
                    reached_speed_mps=reached_speed_mps,
                    reached_yaw_rate_rad=reached_yaw_rate_rad,
                )
                point_reached = bool(point_result["reached"])
                result.update(point_result)
            else:
                result.update({
                    "distance_xy": None,
                    "dx": None,
                    "dy": None,
                    "linear_speed_mps": None,
                    "angular_z_rad": None,
                })

            result["point_reached"] = point_reached
            result["success"] = (
                (height_reached and point_reached)
                if require_both
                else (height_reached or point_reached)
            )
            result["require_both"] = bool(require_both)

            if result["success"]:
                return result

        if deadline is not None and tools.time.time() >= deadline:
            if result is None:
                return None
            result["timed_out"] = True
            return result
        tools.time.sleep(loop_interval_sec)
