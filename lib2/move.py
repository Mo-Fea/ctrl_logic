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
DEFAULT_SEGMENTED_YAW_STEP_DEG = None
DEFAULT_SEGMENTED_YAW_STEP_DEG_CURRENT_POSITION = 10.0
DEFAULT_SEGMENTED_YAW_STEP_DEG_INPUT_POSITION = 360.0
# 分段旋转每个中间目标至少保持的时间，单位 s。
DEFAULT_SEGMENTED_YAW_HOLD_SEC = 0.08
# 分段旋转时位置保持允许的起点漂移距离，单位 m。
DEFAULT_ROTATE_POSITION_TOLERANCE = 0.05
# 分段旋转时位置保持的最大平移通道值。
DEFAULT_ROTATE_POSITION_MAX_CMD = 600
# 分段旋转时位置保持的距离比例增益，单位 channel/m。
DEFAULT_ROTATE_POSITION_KP = 900.0
# 位置保持/移动向量分解的 yaw 预测控制周期补偿，单位 s。
DEFAULT_YAW_PREDICTION_CONTROL_DELAY_SEC = 1.0 / 70.0
# yaw 预测最大时间窗，单位 s，避免旧数据或瞬时阻塞导致预测过大。
DEFAULT_YAW_PREDICTION_MAX_DT_SEC = 0.10
# 定时通道运动和上台阶等待循环周期，单位 s。
DEFAULT_DRIVE_LOOP_INTERVAL_SEC = 0.02
# 移动到点主控制循环周期，单位 s。
DEFAULT_MOVE_LOOP_INTERVAL_SEC = 0.02
# 移动时允许前进的最大航向误差，超过该角度则 ch2=0 只转向，单位 deg。
DEFAULT_MOVE_GATE_DEG = 2.0
# 距离目标较远时的默认前进通道值。
DEFAULT_MOVE_FORWARD_CMD = 500
# 旧 vector PID 输出最小有效通道值；保留为兼容参数。
DEFAULT_MIN_ACTIVE_MOVE_CMD = 240
# 旧 vector PID 参数；当前主移动逻辑不再使用这些参数计算通道。
DEFAULT_MOVE_PID_KP = 700.0
DEFAULT_MOVE_PID_KI = 0.0
DEFAULT_MOVE_PID_KD = 80.0
DEFAULT_MOVE_PID_INTEGRAL_LIMIT = 1.0
# 底盘通道符号；ch0 符号已按实测翻转。
DEFAULT_LATERAL_CMD_SIGN = -1
DEFAULT_FORWARD_CMD_SIGN = 1
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
        channels = set_motion_channels(sender, des_yaw_i16=0)
        segment_step_result = (
            DEFAULT_SEGMENTED_YAW_STEP_DEG_CURRENT_POSITION
            if segment_step_deg is None
            else float(segment_step_deg)
        )
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
    deadline = None if timeout_sec is None else (tools.time.time() + float(timeout_sec))
    stable_since = None

    while True:
        current_yaw_deg = position_runtime.get_current_yaw_deg()
        now = tools.time.time()
        if current_yaw_deg is not None:
            channels = set_motion_channels(sender, des_yaw_i16=des_yaw_i16)
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
    odom_runtime=None,
    segment_step_deg=DEFAULT_SEGMENTED_YAW_STEP_DEG,
    tolerance_deg=DEFAULT_YAW_TOLERANCE_DEG,
    stable_sec=DEFAULT_DIRECTION_STABLE_SEC,
    des_x=0.0,
    des_y=0.0,
    position_tolerance=DEFAULT_ROTATE_POSITION_TOLERANCE,
    position_hold_kp=DEFAULT_ROTATE_POSITION_KP,
    position_hold_max_cmd=DEFAULT_ROTATE_POSITION_MAX_CMD,
    segment_hold_sec=DEFAULT_SEGMENTED_YAW_HOLD_SEC,
    loop_interval_sec=DEFAULT_ROTATE_LOOP_INTERVAL_SEC,
    timeout_sec=DEFAULT_ROTATE_TIMEOUT_SEC,
):
    """
    带位置保持的分段式对正到目标角。

    每轮读取当前 yaw，沿 current -> target 的劣弧方向给一个最多
    segment_step_deg 的中间目标；当剩余误差小于该步长后，直接给最终目标角。
    未显式传 segment_step_deg 时，des_x/des_y 都为 0 使用 10deg，否则使用 360deg。
    des_x/des_y 都为 0 时，进入函数后记录机器人当前位置；否则使用传入坐标。
    旋转过程中用低速 ch0/ch2 把机器人保持在
    初始点附近；退出条件为 yaw 稳定到位且当前位置回到初始点 position_tolerance 内。
    """
    if float(target_yaw_deg) == 0.0:
        channels = set_motion_channels(sender, des_yaw_i16=0)
        segment_step_result = (
            DEFAULT_SEGMENTED_YAW_STEP_DEG_CURRENT_POSITION
            if segment_step_deg is None
            else float(segment_step_deg)
        )
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
            "segment_step_deg": float(segment_step_result),
            "segment_count": 0,
            "rotation_control_stopped": True,
        }

    target_yaw_deg = normalize_yaw_deg(target_yaw_deg)
    if segment_hold_sec < 0.0:
        raise ValueError(f"segment_hold_sec must be >= 0, got {segment_hold_sec}")
    des_x = float(des_x)
    des_y = float(des_y)
    if segment_step_deg is None:
        segment_step_deg = (
            DEFAULT_SEGMENTED_YAW_STEP_DEG_CURRENT_POSITION
            if des_x == 0.0 and des_y == 0.0
            else DEFAULT_SEGMENTED_YAW_STEP_DEG_INPUT_POSITION
        )
    segment_step_deg = abs(float(segment_step_deg))
    if segment_step_deg <= 0.0:
        raise ValueError(f"segment_step_deg must be > 0, got {segment_step_deg}")
    position_tolerance = float(position_tolerance)
    if position_tolerance < 0.0:
        raise ValueError(f"position_tolerance must be >= 0, got {position_tolerance}")
    position_hold_max_cmd = abs(int(position_hold_max_cmd))
    position_hold_kp = float(position_hold_kp)

    deadline = None if timeout_sec is None else (tools.time.time() + float(timeout_sec))
    if des_x == 0.0 and des_y == 0.0:
        start_position = position_runtime.get_current_position()
        while start_position is None:
            if deadline is not None and tools.time.time() >= deadline:
                return None
            tools.time.sleep(loop_interval_sec)
            start_position = position_runtime.get_current_position()

        start_x = float(start_position["x"])
        start_y = float(start_position["y"])
        hold_position_source = "current_position"
    else:
        start_x = des_x
        start_y = des_y
        hold_position_source = "input"
    result = None
    active_des_yaw_deg = None
    last_segment_update_time = 0.0
    segment_count = 0
    stable_since = None

    while True:
        current_yaw_deg = position_runtime.get_current_yaw_deg()
        current_position = position_runtime.get_current_position()
        latest_update_time = position_runtime.get_latest_update_time()
        now = tools.time.time()

        if current_yaw_deg is not None:
            current_yaw_deg = normalize_yaw_deg(current_yaw_deg)
            yaw_age_sec = 0.0 if latest_update_time is None else max(0.0, now - float(latest_update_time))
            odometry = None if odom_runtime is None else odom_runtime.get_odometry(max_age_sec=None)
            angular_z_rad = 0.0 if odometry is None else float(odometry["angular_z"])
            predicted_yaw_deg, yaw_prediction_dt_sec = predict_yaw_deg(
                current_yaw_deg=current_yaw_deg,
                angular_z_rad=angular_z_rad,
                yaw_age_sec=yaw_age_sec,
            )
            final_error_deg = heading_error_deg(current_yaw_deg, target_yaw_deg)
            position_reached = False
            position_distance_xy = None
            position_dx = None
            position_dy = None
            output_lateral_cmd = 0
            output_forward_cmd = 0
            if current_position is not None:
                current_x = float(current_position["x"])
                current_y = float(current_position["y"])
                motion_result = _calculate_position_hold_motion(
                    current_x=current_x,
                    current_y=current_y,
                    target_x=start_x,
                    target_y=start_y,
                    current_yaw_deg=predicted_yaw_deg,
                    position_hold_kp=position_hold_kp,
                    position_hold_max_cmd=position_hold_max_cmd,
                    stop_distance=position_tolerance,
                )
                position_distance_xy = motion_result["distance_xy"]
                position_dx = motion_result["dx"]
                position_dy = motion_result["dy"]
                position_reached = position_distance_xy <= position_tolerance
                output_lateral_cmd = motion_result["lateral_cmd"]
                output_forward_cmd = motion_result["forward_cmd"]

            in_threshold = abs(final_error_deg) <= float(tolerance_deg)
            if in_threshold:
                if stable_since is None:
                    stable_since = now
            else:
                stable_since = None
            stable_elapsed_sec = 0.0 if stable_since is None else (now - stable_since)

            if in_threshold and stable_elapsed_sec >= float(stable_sec) and position_reached:
                final_des_yaw_i16 = encode_target_yaw_i16(target_yaw_deg)
                channels = set_motion_channels(sender, des_yaw_i16=final_des_yaw_i16)
                return {
                    "channels": channels,
                    "start_x": float(start_x),
                    "start_y": float(start_y),
                    "hold_position_source": hold_position_source,
                    "current_yaw_deg": float(current_yaw_deg),
                    "predicted_yaw_deg": float(predicted_yaw_deg),
                    "angular_z_rad": float(angular_z_rad),
                    "yaw_prediction_dt_sec": float(yaw_prediction_dt_sec),
                    "target_yaw_deg": float(target_yaw_deg),
                    "des_yaw_deg": float(target_yaw_deg),
                    "des_yaw_i16": int(final_des_yaw_i16),
                    "heading_error_deg": float(final_error_deg),
                    "heading_reached": True,
                    "position_distance_xy": (
                        None
                        if position_distance_xy is None
                        else float(position_distance_xy)
                    ),
                    "position_reached": bool(position_reached),
                    "position_tolerance": float(position_tolerance),
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
            channels = set_motion_channels(
                sender,
                lateral_cmd=output_lateral_cmd,
                forward_cmd=output_forward_cmd,
                des_yaw_i16=des_yaw_i16,
            )
            result = {
                "channels": channels,
                "start_x": float(start_x),
                "start_y": float(start_y),
                "hold_position_source": hold_position_source,
                "current_yaw_deg": float(current_yaw_deg),
                "predicted_yaw_deg": float(predicted_yaw_deg),
                "angular_z_rad": float(angular_z_rad),
                "yaw_prediction_dt_sec": float(yaw_prediction_dt_sec),
                "target_yaw_deg": float(target_yaw_deg),
                "des_yaw_deg": float(active_des_yaw_deg),
                "des_yaw_i16": int(des_yaw_i16),
                "heading_error_deg": float(final_error_deg),
                "heading_reached": False,
                "position_distance_xy": (
                    None
                    if position_distance_xy is None
                    else float(position_distance_xy)
                ),
                "position_dx": None if position_dx is None else float(position_dx),
                "position_dy": None if position_dy is None else float(position_dy),
                "position_reached": bool(position_reached),
                "position_tolerance": float(position_tolerance),
                "position_lateral_cmd": int(channels[0]),
                "position_forward_cmd": int(channels[2]),
                "position_hold_kp": float(position_hold_kp),
                "position_hold_max_cmd": int(position_hold_max_cmd),
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
    deadline = tools.time.time() + float(duration_sec)

    while tools.time.time() < deadline:
        channels = set_motion_channels(sender, des_yaw_i16=des_yaw_i16)
        tools.time.sleep(loop_interval_sec)

    channels = set_motion_channels(sender, des_yaw_i16=des_yaw_i16)
    return {
        "channels": channels,
        "duration_sec": float(duration_sec),
        "target_yaw_deg": float(target_yaw_deg),
        "des_yaw_i16": int(des_yaw_i16),
        "completed": True,
    }


def reset_weapon_after_fetch(sender):
    """
    阻塞式 weapon 复位动作。

    1. ch4=1，先释放 weapon 气缸。
    2. 等待 3s。
    3. ch1=0, ch5=1，停止 weapon 抬升并退出 weapon 模式。
    """
    release_channels = set_channel_values(
        sender,
        channel_values={
            4: tools.SAFE_SWITCH_VALUE,
        },
    )
    tools.time.sleep(3.0)
    reset_channels = set_channel_values(
        sender,
        channel_values={
            1: 0,
            5: tools.SAFE_SWITCH_VALUE,
        },
    )
    return {
        "release_channels": release_channels,
        "reset_channels": reset_channels,
        "release_wait_sec": 3.0,
        "completed": True,
    }


def _kfs_pose_channel_values(pose_id, trigger_value, suction_ch4=tools.SAFE_SWITCH_VALUE):
    return {
        4: int(suction_ch4),
        KFS_MODE_CHANNEL_INDEX: KFS_MODE_VALUE,
        KFS_POSE_CHANNEL_INDEX: int(pose_id),
        KFS_TRIGGER_CHANNEL_INDEX: int(trigger_value),
    }


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
        zero_channel_values = _kfs_pose_channel_values(
            pose_id=0,
            trigger_value=KFS_TRIGGER_ZERO_VALUE,
            suction_ch4=suction_ch4,
        )
        deadline = tools.time.time() + float(zero_return_wait_sec)
        while tools.time.time() < deadline:
            zero_channels = set_channel_values(sender, channel_values=zero_channel_values)
            tools.time.sleep(float(loop_interval_sec))
        zero_channels = set_channel_values(sender, channel_values=zero_channel_values)
        return {
            "pose_id": 0,
            "pose_name": "zero_return",
            "channels": zero_channels,
            "des_yaw_i16": 0,
            "zero_return_wait_sec": float(zero_return_wait_sec),
            "completed": True,
        }

    arm_channel_values = _kfs_pose_channel_values(
        pose_id=pose_id,
        trigger_value=KFS_TRIGGER_ARM_VALUE,
        suction_ch4=suction_ch4,
    )
    arm_deadline = tools.time.time() + float(arm_wait_sec)
    while tools.time.time() < arm_deadline:
        arm_channels = set_channel_values(sender, channel_values=arm_channel_values)
        tools.time.sleep(float(loop_interval_sec))

    fire_channel_values = _kfs_pose_channel_values(
        pose_id=pose_id,
        trigger_value=KFS_TRIGGER_FIRE_VALUE,
        suction_ch4=suction_ch4,
    )
    hold_deadline = tools.time.time() + float(hold_sec)
    while tools.time.time() < hold_deadline:
        fire_channels = set_channel_values(sender, channel_values=fire_channel_values)
        tools.time.sleep(float(loop_interval_sec))

    idle_channel_values = _kfs_pose_channel_values(
        pose_id=1,
        trigger_value=KFS_TRIGGER_ARM_VALUE,
        suction_ch4=suction_ch4,
    )
    idle_channels = set_channel_values(sender, channel_values=idle_channel_values)
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
    duration_sec,
    lateral_cmd=0,
    forward_cmd=0,
    rotation_cmd=0,
    target_yaw_deg=None,
    loop_interval_sec=DEFAULT_DRIVE_LOOP_INTERVAL_SEC,
    brake_reverse_cmd=-50,
    brake_duration_sec=1.0,
):
    """
    阻塞式按指定通道值持续运动一段时间。

    行为:
    - 持续时间内反复设置内部通道变量
    - 如果给了 target_yaw_deg，则同时持续设置目标航向角
    - 时间到后先以 ch2=brake_reverse_cmd 持续 brake_duration_sec
    - 然后将前进通道清零再返回
    """
    if duration_sec < 0.0:
        raise ValueError(f"duration_sec must be >= 0, got {duration_sec}")

    deadline = tools.time.time() + float(duration_sec)
    des_yaw_i16 = None
    if target_yaw_deg is not None:
        des_yaw_i16 = encode_target_yaw_i16(normalize_yaw_deg(target_yaw_deg))

    while tools.time.time() < deadline:
        channels = set_motion_channels(
            sender,
            lateral_cmd=lateral_cmd,
            forward_cmd=forward_cmd,
            rotation_cmd=rotation_cmd,
            des_yaw_i16=des_yaw_i16,
        )
        tools.time.sleep(loop_interval_sec)

    brake_deadline = tools.time.time() + float(brake_duration_sec)
    while tools.time.time() < brake_deadline:
        brake_channels = set_motion_channels(
            sender,
            lateral_cmd=lateral_cmd,
            forward_cmd=brake_reverse_cmd,
            rotation_cmd=rotation_cmd,
            des_yaw_i16=des_yaw_i16,
        )
        tools.time.sleep(loop_interval_sec)

    stop_channels = set_motion_channels(
        sender,
        lateral_cmd=lateral_cmd,
        forward_cmd=0,
        rotation_cmd=rotation_cmd,
        des_yaw_i16=des_yaw_i16,
    )

    return {
        "channels": channels if "channels" in locals() else stop_channels,
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


def clamp_value(value, min_value, max_value):
    return max(float(min_value), min(float(max_value), float(value)))


def world_error_to_fixed_body(dx, dy, fixed_yaw_deg):
    """
    将 map/world 下的目标误差转换到 fixed_yaw_deg 对应的车体轴。

    返回:
      lateral_error: fixed_yaw_deg 左侧为正。
      forward_error: fixed_yaw_deg 前方为正。
    """
    yaw_rad = math.radians(float(fixed_yaw_deg))
    cos_yaw = math.cos(yaw_rad)
    sin_yaw = math.sin(yaw_rad)

    forward_error = cos_yaw * float(dx) + sin_yaw * float(dy)
    lateral_error = -sin_yaw * float(dx) + cos_yaw * float(dy)
    return lateral_error, forward_error


def _calculate_position_hold_motion(
    current_x,
    current_y,
    target_x,
    target_y,
    current_yaw_deg,
    position_hold_kp=DEFAULT_ROTATE_POSITION_KP,
    position_hold_max_cmd=DEFAULT_ROTATE_POSITION_MAX_CMD,
    stop_distance=0.0,
    lateral_cmd_sign=DEFAULT_LATERAL_CMD_SIGN,
    forward_cmd_sign=DEFAULT_FORWARD_CMD_SIGN,
):
    """
    当前 yaw 下的位置保持/移动控制核心。

    旋转时 target_x/target_y 是保持点；移动时 target_x/target_y 是目标点。
    """
    max_cmd = abs(int(position_hold_max_cmd))
    distance_xy, dx, dy = distance_to_target_xy(
        current_x=current_x,
        current_y=current_y,
        target_x=target_x,
        target_y=target_y,
    )
    scalar_cmd = clamp_value(float(position_hold_kp) * distance_xy, 0.0, max_cmd)
    if distance_xy <= float(stop_distance):
        scalar_cmd = 0.0

    lateral_error, forward_error = world_error_to_fixed_body(
        dx=dx,
        dy=dy,
        fixed_yaw_deg=current_yaw_deg,
    )
    body_error_norm = math.hypot(lateral_error, forward_error)
    if body_error_norm > 1e-6 and max_cmd > 0:
        lateral_cmd = scalar_cmd * lateral_error / body_error_norm
        forward_cmd = scalar_cmd * forward_error / body_error_norm
    else:
        lateral_cmd = 0.0
        forward_cmd = 0.0

    output_lateral_cmd = round(clamp_value(
        float(lateral_cmd_sign) * lateral_cmd,
        -max_cmd,
        max_cmd,
    ))
    output_forward_cmd = round(clamp_value(
        float(forward_cmd_sign) * forward_cmd,
        -max_cmd,
        max_cmd,
    ))
    return {
        "distance_xy": float(distance_xy),
        "dx": float(dx),
        "dy": float(dy),
        "lateral_error": float(lateral_error),
        "forward_error": float(forward_error),
        "scalar_cmd": float(scalar_cmd),
        "lateral_cmd": int(output_lateral_cmd),
        "forward_cmd": int(output_forward_cmd),
        "position_hold_kp": float(position_hold_kp),
        "position_hold_max_cmd": int(max_cmd),
    }


def predict_yaw_deg(
    current_yaw_deg,
    angular_z_rad=0.0,
    yaw_age_sec=0.0,
    control_delay_sec=DEFAULT_YAW_PREDICTION_CONTROL_DELAY_SEC,
    max_prediction_dt_sec=DEFAULT_YAW_PREDICTION_MAX_DT_SEC,
):
    prediction_dt = max(0.0, float(yaw_age_sec) + float(control_delay_sec))
    prediction_dt = min(prediction_dt, float(max_prediction_dt_sec))
    predicted_yaw_deg = float(current_yaw_deg) + math.degrees(float(angular_z_rad) * prediction_dt)
    return normalize_yaw_deg(predicted_yaw_deg), prediction_dt


def _get_fixed_yaw_for_move(position_runtime, fixed_target_yaw_deg):
    if fixed_target_yaw_deg is not None:
        return normalize_yaw_deg(fixed_target_yaw_deg)

    current_yaw_deg = position_runtime.get_current_yaw_deg()
    if current_yaw_deg is None:
        robot_pose = position_runtime.get_robot_pose()
        if robot_pose is None:
            return None
        current_yaw_deg = position_lib.radians_to_degrees(float(robot_pose["yaw"]))
    return normalize_yaw_deg(current_yaw_deg)


def validate_channel_values(channel_values=None):
    if channel_values is None:
        return {}
    values = {}
    for index, value in dict(channel_values).items():
        index = int(index)
        if index < 0 or index >= tools.CHANNEL_COUNT:
            raise ValueError(f"channel index must be 0..{tools.CHANNEL_COUNT - 1}, got {index}")
        values[index] = int(value)
    return values


def set_channel_values(
    sender,
    des_yaw_i16=None,
    yaw_i16=None,
    channel_values=None,
):
    return sender.set_channel_values(
        validate_channel_values(channel_values),
        yaw_i16=yaw_i16,
        des_yaw_i16=des_yaw_i16,
        reset_channels=False,
    )


def set_motion_channels(
    sender,
    lateral_cmd=0,
    forward_cmd=0,
    rotation_cmd=0,
    des_yaw_i16=None,
):
    values = {
        0: int(lateral_cmd),
        2: int(forward_cmd),
        3: int(rotation_cmd),
    }
    return set_channel_values(
        sender,
        des_yaw_i16=des_yaw_i16,
        channel_values=values,
    )


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
    min_active_move_cmd=DEFAULT_MIN_ACTIVE_MOVE_CMD,
    pid_kp=DEFAULT_MOVE_PID_KP,
    pid_ki=DEFAULT_MOVE_PID_KI,
    pid_kd=DEFAULT_MOVE_PID_KD,
    pid_integral_limit=DEFAULT_MOVE_PID_INTEGRAL_LIMIT,
    lateral_cmd_sign=DEFAULT_LATERAL_CMD_SIGN,
    forward_cmd_sign=DEFAULT_FORWARD_CMD_SIGN,
    position_hold_kp=DEFAULT_ROTATE_POSITION_KP,
    position_hold_max_cmd=None,
    loop_interval_sec=DEFAULT_MOVE_LOOP_INTERVAL_SEC,
    timeout_sec=None,
    reference="robot",
):
    """
    阻塞式位置保持式移动到目标点。

    final_target_yaw_deg 作为移动过程持续下发的目标航向；None 表示以进入函数时
    的当前机器人 yaw 作为目标航向，到点后发送 des_yaw_i16=0 关闭航向控制。
    平移控制沿用 rotate_to_target_yaw_segmented() 的位置保持逻辑：每轮按当前
    yaw 把 target - reference 转到车体轴，再按距离比例分配 ch0/ch2。

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
    target_yaw_deg = _get_fixed_yaw_for_move(position_runtime, final_target_yaw_deg)
    if target_yaw_deg is None:
        return None
    des_yaw_i16 = encode_target_yaw_i16(target_yaw_deg)
    max_cmd = abs(int(cruise_forward_cmd if position_hold_max_cmd is None else position_hold_max_cmd))
    if max_cmd <= 0:
        max_cmd = DEFAULT_MOVE_FORWARD_CMD
    position_hold_kp = float(position_hold_kp)
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
            latest_update_time = position_runtime.get_latest_update_time()
            now = tools.time.time()
            yaw_age_sec = 0.0 if latest_update_time is None else max(0.0, now - float(latest_update_time))
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
                set_motion_channels(sender, des_yaw_i16=0)
                tools.time.sleep(loop_interval_sec)
                continue

            predicted_yaw_deg, yaw_prediction_dt_sec = predict_yaw_deg(
                current_yaw_deg=current_yaw_deg,
                angular_z_rad=angular_z_rad,
                yaw_age_sec=yaw_age_sec,
            )
            yaw_error_deg = heading_error_deg(current_yaw_deg, target_yaw_deg)
            linear_speed_mps = math.hypot(linear_x, linear_y)
            distance_xy, dx, dy = distance_to_target_xy(
                current_x=current_x,
                current_y=current_y,
                target_x=target_x,
                target_y=target_y,
            )

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

            motion_result = _calculate_position_hold_motion(
                current_x=current_x,
                current_y=current_y,
                target_x=target_x,
                target_y=target_y,
                current_yaw_deg=predicted_yaw_deg,
                position_hold_kp=position_hold_kp,
                position_hold_max_cmd=max_cmd,
                stop_distance=stop_distance,
                lateral_cmd_sign=lateral_cmd_sign,
                forward_cmd_sign=forward_cmd_sign,
            )
            output_lateral_cmd = motion_result["lateral_cmd"]
            output_forward_cmd = motion_result["forward_cmd"]
            channels = set_motion_channels(
                sender,
                lateral_cmd=output_lateral_cmd,
                forward_cmd=output_forward_cmd,
                rotation_cmd=0,
                des_yaw_i16=des_yaw_i16,
            )

            result.update({
                "current_x": current_x,
                "current_y": current_y,
                "current_z": current_z,
                "current_yaw_deg": current_yaw_deg,
                "predicted_yaw_deg": float(predicted_yaw_deg),
                "yaw_age_sec": float(yaw_age_sec),
                "yaw_prediction_dt_sec": float(yaw_prediction_dt_sec),
                "angular_z_rad": float(angular_z_rad),
                "target_x": float(target_x),
                "target_y": float(target_y),
                "fixed_yaw_deg": float(target_yaw_deg),
                "target_yaw_deg": float(target_yaw_deg),
                "des_yaw_i16": int(des_yaw_i16),
                "heading_error_deg": float(yaw_error_deg),
                "reference": reference,
                "yaw_gate_active": False,
                "lateral_error": float(motion_result["lateral_error"]),
                "forward_error": float(motion_result["forward_error"]),
                "scalar_cmd": float(motion_result["scalar_cmd"]),
                "lateral_cmd": int(channels[0]),
                "forward_cmd": int(channels[2]),
                "lateral_cmd_sign": int(lateral_cmd_sign),
                "forward_cmd_sign": int(forward_cmd_sign),
                "move_gate_deg": float(move_gate_deg),
                "min_active_move_cmd": int(min_active_move_cmd),
                "position_hold_kp": float(motion_result["position_hold_kp"]),
                "position_hold_max_cmd": int(motion_result["position_hold_max_cmd"]),
                "move_mode": "current_yaw_position_hold",
                "pid": {
                    "kp": float(pid_kp),
                    "ki": float(pid_ki),
                    "kd": float(pid_kd),
                    "integral": 0.0,
                    "derivative": 0.0,
                },
                "channels": channels,
            })

            if result["reached"]:
                final_des_yaw_i16 = (
                    0
                    if final_target_yaw_deg is None
                    else encode_target_yaw_i16(final_target_yaw_deg)
                )
                stop_channels = set_motion_channels(
                    sender,
                    lateral_cmd=0,
                    forward_cmd=0,
                    rotation_cmd=0,
                    des_yaw_i16=final_des_yaw_i16,
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
                    "completed": True,
                })
                return result

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
    min_active_move_cmd=DEFAULT_MIN_ACTIVE_MOVE_CMD,
    pid_kp=DEFAULT_MOVE_PID_KP,
    pid_ki=DEFAULT_MOVE_PID_KI,
    pid_kd=DEFAULT_MOVE_PID_KD,
    pid_integral_limit=DEFAULT_MOVE_PID_INTEGRAL_LIMIT,
    lateral_cmd_sign=DEFAULT_LATERAL_CMD_SIGN,
    forward_cmd_sign=DEFAULT_FORWARD_CMD_SIGN,
    position_hold_kp=DEFAULT_ROTATE_POSITION_KP,
    position_hold_max_cmd=None,
    loop_interval_sec=DEFAULT_MOVE_LOOP_INTERVAL_SEC,
    timeout_sec=None,
    reference="robot",
):
    """
    阻塞式倒退移动到目标点。

    移动过程目标航向取 final_target_yaw_deg + 180deg 后复用 move_to_target 的
    位置保持式移动逻辑；到点后若 final_target_yaw_deg 不为 None，会把目标
    航向字段切回 final_target_yaw_deg，外层可继续执行最终旋转。

    reference:
      "robot": 以机器人中心作为到点参考。
      "weapon": 以 weapon/夹爪作为到点参考，底盘 yaw 仍使用机器人 yaw。
    """
    backward_fixed_yaw_deg = (
        None
        if final_target_yaw_deg is None
        else normalize_yaw_deg(float(final_target_yaw_deg) + 180.0)
    )
    result = move_to_target(
        sender=sender,
        position_runtime=position_runtime,
        odom_runtime=odom_runtime,
        target_x=target_x,
        target_y=target_y,
        final_target_yaw_deg=backward_fixed_yaw_deg,
        move_gate_deg=move_gate_deg,
        cruise_forward_cmd=abs(int(cruise_backward_cmd)),
        near_forward_cmd=abs(int(near_backward_cmd)),
        near_target_distance=near_target_distance,
        fine_forward_cmd=abs(int(fine_backward_cmd)),
        fine_target_distance=fine_target_distance,
        overshoot_distance_eps=overshoot_distance_eps,
        overshoot_check_distance=overshoot_check_distance,
        overshoot_confirm_count=overshoot_confirm_count,
        stop_distance=stop_distance,
        reached_speed_mps=reached_speed_mps,
        reached_yaw_rate_rad=reached_yaw_rate_rad,
        reached_reverse_cmd=-abs(int(reached_forward_cmd)),
        reached_reverse_duration_sec=reached_forward_duration_sec,
        recovery_distance_eps=recovery_distance_eps,
        recovery_worse_confirm_count=recovery_worse_confirm_count,
        min_active_move_cmd=min_active_move_cmd,
        pid_kp=pid_kp,
        pid_ki=pid_ki,
        pid_kd=pid_kd,
        pid_integral_limit=pid_integral_limit,
        lateral_cmd_sign=lateral_cmd_sign,
        forward_cmd_sign=forward_cmd_sign,
        position_hold_kp=position_hold_kp,
        position_hold_max_cmd=(
            abs(int(cruise_backward_cmd))
            if position_hold_max_cmd is None
            else position_hold_max_cmd
        ),
        loop_interval_sec=loop_interval_sec,
        timeout_sec=timeout_sec,
        reference=reference,
    )
    if result is not None:
        final_des_yaw_i16 = (
            0
            if final_target_yaw_deg is None
            else encode_target_yaw_i16(final_target_yaw_deg)
        )
        stop_channels = set_motion_channels(
            sender,
            lateral_cmd=0,
            forward_cmd=0,
            rotation_cmd=0,
            des_yaw_i16=final_des_yaw_i16,
        )
        result.update({
            "channels": stop_channels,
            "final_target_yaw_deg": (
                None
                if final_target_yaw_deg is None
                else float(normalize_yaw_deg(final_target_yaw_deg))
            ),
            "backward_fixed_yaw_deg": (
                None
                if backward_fixed_yaw_deg is None
                else float(backward_fixed_yaw_deg)
            ),
            "des_yaw_i16": int(final_des_yaw_i16),
            "move_mode": "backward_current_yaw_position_hold",
        })
    return result


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

    def climb_channel_values(trigger_value):
        return {
            CLIMB_MODE_CHANNEL_INDEX: CLIMB_MODE_VALUE,
            CLIMB_TRIGGER_CHANNEL_INDEX: int(trigger_value),
        }

    arm_channel_values = climb_channel_values(CLIMB_TRIGGER_ARM_VALUE)
    arm_deadline = tools.time.time() + float(trigger_arm_sec)
    while tools.time.time() < arm_deadline:
        arm_channels = set_channel_values(sender, channel_values=arm_channel_values)
        tools.time.sleep(loop_interval_sec)

    fire_channel_values = climb_channel_values(CLIMB_TRIGGER_FIRE_VALUE)
    fire_deadline = tools.time.time() + float(trigger_hold_sec)
    while tools.time.time() < fire_deadline:
        fire_channels = set_channel_values(sender, channel_values=fire_channel_values)
        tools.time.sleep(loop_interval_sec)

    idle_channel_values = climb_channel_values(CLIMB_TRIGGER_IDLE_VALUE)
    idle_channels = set_channel_values(sender, channel_values=idle_channel_values)

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

        idle_channels = set_channel_values(sender, channel_values=idle_channel_values)

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

    def descend_channel_values(trigger_value):
        return {
            CLIMB_MODE_CHANNEL_INDEX: CLIMB_MODE_VALUE,
            DESCEND_TRIGGER_CHANNEL_INDEX: int(trigger_value),
        }

    arm_channel_values = descend_channel_values(DESCEND_TRIGGER_ARM_VALUE)
    arm_deadline = tools.time.time() + float(trigger_arm_sec)
    while tools.time.time() < arm_deadline:
        arm_channels = set_channel_values(sender, channel_values=arm_channel_values)
        tools.time.sleep(loop_interval_sec)

    fire_channel_values = descend_channel_values(DESCEND_TRIGGER_FIRE_VALUE)
    fire_deadline = tools.time.time() + float(trigger_hold_sec)
    while tools.time.time() < fire_deadline:
        fire_channels = set_channel_values(sender, channel_values=fire_channel_values)
        tools.time.sleep(loop_interval_sec)

    idle_channel_values = descend_channel_values(DESCEND_TRIGGER_IDLE_VALUE)
    idle_channels = set_channel_values(sender, channel_values=idle_channel_values)

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

        idle_channels = set_channel_values(sender, channel_values=idle_channel_values)
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
