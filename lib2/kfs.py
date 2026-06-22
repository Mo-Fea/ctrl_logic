import sys
import threading
import time

from lib2 import tools
from lib2 import move as move_lib


KFS_SUCTION_CHANNEL_INDEX = 4
KFS_MODE_CHANNEL_INDEX = 5
KFS_POSE_CHANNEL_INDEX = 6
KFS_TRIGGER_CHANNEL_INDEX = 7
KFS_SUCKER_ROTATION_CHANNEL_INDEX = 9
KFS_MODE_VALUE = 2
KFS_SUCTION_OFF_VALUE = 1
KFS_SUCTION_ON_VALUE = 3
KFS_TRIGGER_IDLE_VALUE = 1
KFS_SUCKER_0_DEG_VALUE = 800
KFS_SUCKER_NEG_90_DEG_VALUE = -800
KFS_SUCKER_180_DEG_VALUE = -300
KFS_SUCKER_90_DEG_VALUE = 300
current_sucker_deg = 0
suck_count = 1


def _set_sucker_rotation(sender, angle_deg, channel_value):
    global current_sucker_deg

    channels = move_lib.set_channel_values(
        sender,
        channel_values={KFS_SUCKER_ROTATION_CHANNEL_INDEX: int(channel_value)},
    )
    current_sucker_deg = int(angle_deg)
    return {
        "angle_deg": int(angle_deg),
        "channel_value": int(channel_value),
        "channels": channels,
        "completed": True,
    }


def sucker_0deg(sender):
    """设置 KFS 吸盘朝向 0deg，ch9=800。"""
    return _set_sucker_rotation(sender, angle_deg=0, channel_value=KFS_SUCKER_0_DEG_VALUE)


def sucker_neg90deg(sender):
    """设置 KFS 吸盘朝向 -90deg，ch9=-800。"""
    return _set_sucker_rotation(
        sender,
        angle_deg=-90,
        channel_value=KFS_SUCKER_NEG_90_DEG_VALUE,
    )


def sucker_180deg(sender):
    """设置 KFS 吸盘朝向 180deg，ch9=-300。"""
    return _set_sucker_rotation(
        sender,
        angle_deg=180,
        channel_value=KFS_SUCKER_180_DEG_VALUE,
    )


def sucker_90deg(sender):
    """设置 KFS 吸盘朝向 90deg，ch9=300。"""
    return _set_sucker_rotation(
        sender,
        angle_deg=90,
        channel_value=KFS_SUCKER_90_DEG_VALUE,
    )


def sucker_to_direction(sender, direction):
    """按项目方向编号 1/2/3/4 设置吸盘角度，非法输入时终止程序。"""
    try:
        direction = int(direction)
    except (TypeError, ValueError):
        print(f"{sucker_to_direction.__name__}输入错误: direction={direction}")
        sys.exit(1)

    direction_to_action = {
        1: sucker_0deg,
        2: sucker_90deg,
        3: sucker_neg90deg,
        4: sucker_180deg,
    }
    action = direction_to_action.get(direction)
    if action is None:
        print(
            f"{sucker_to_direction.__name__}输入错误: "
            f"direction={direction}, 必须是 1/2/3/4"
        )
        sys.exit(1)

    result = action(sender)
    result["direction"] = direction
    return result


def _select_sucker_cylinder(sender, cylinder_select, selection_name):
    cylinder_select = tools.validate_cylinder_select(cylinder_select)
    sender.set_cylinder_select(cylinder_select)
    return {
        "selection": str(selection_name),
        "cylinder_select": int(cylinder_select),
        "completed": True,
    }


def sucker_select_both(sender):
    """设置 cylinderSelect=0，同时选择 PF2/PF3。"""
    return _select_sucker_cylinder(
        sender,
        cylinder_select=tools.CYLINDER_SELECT_BOTH,
        selection_name="pf2_pf3",
    )


def sucker_select_pf2(sender):
    """设置 cylinderSelect=1，只选择 PF2。"""
    return _select_sucker_cylinder(
        sender,
        cylinder_select=tools.CYLINDER_SELECT_PF2,
        selection_name="pf2",
    )


def sucker_select_pf3(sender):
    """设置 cylinderSelect=2，只选择 PF3。"""
    return _select_sucker_cylinder(
        sender,
        cylinder_select=tools.CYLINDER_SELECT_PF3,
        selection_name="pf3",
    )


def sucker_select_cylinder(sender, cylinder_select=None):
    """按 0/1/2 选择 PF2/PF3；省略参数时使用当前 suck_count。"""
    if cylinder_select is None:
        cylinder_select = suck_count

    try:
        cylinder_select = int(cylinder_select)
    except (TypeError, ValueError):
        print(
            f"{sucker_select_cylinder.__name__}输入错误: "
            f"cylinder_select={cylinder_select}"
        )
        sys.exit(1)

    selection_to_action = {
        tools.CYLINDER_SELECT_BOTH: sucker_select_both,
        tools.CYLINDER_SELECT_PF2: sucker_select_pf2,
        tools.CYLINDER_SELECT_PF3: sucker_select_pf3,
    }
    action = selection_to_action.get(cylinder_select)
    if action is None:
        print(
            f"{sucker_select_cylinder.__name__}输入错误: "
            f"cylinder_select={cylinder_select}, 必须是 0/1/2"
        )
        sys.exit(1)

    return action(sender)


def _repeat_set_channel_values(sender, channel_values, duration_sec, loop_interval_sec=0.02):
    deadline = time.time() + float(duration_sec)
    while time.time() < deadline:
        move_lib.set_channel_values(sender, channel_values=channel_values)
        time.sleep(float(loop_interval_sec))
    return move_lib.set_channel_values(sender, channel_values=channel_values)


def _trigger_kfs_pose_with_lock(
    sender,
    pose_id,
    arm_sec,
    fire_sec,
    loop_interval_sec=0.02,
):
    pose_id = int(pose_id)
    with tools.AUTO_TRIGGER_LOCK:
        arm_channel_values = {
            KFS_MODE_CHANNEL_INDEX: KFS_MODE_VALUE,
            KFS_POSE_CHANNEL_INDEX: pose_id,
            KFS_TRIGGER_CHANNEL_INDEX: KFS_TRIGGER_IDLE_VALUE,
        }
        arm_channels = _repeat_set_channel_values(
            sender,
            arm_channel_values,
            arm_sec,
            loop_interval_sec=loop_interval_sec,
        )

        fire_channel_values = {
            KFS_MODE_CHANNEL_INDEX: KFS_MODE_VALUE,
            KFS_POSE_CHANNEL_INDEX: pose_id,
            KFS_TRIGGER_CHANNEL_INDEX: 3,
        }
        fire_channels = _repeat_set_channel_values(
            sender,
            fire_channel_values,
            fire_sec,
            loop_interval_sec=loop_interval_sec,
        )

        idle_channel_values = {
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
        "completed": True,
    }


def kfs_grab_pose(
    sender,
    pose_id,
    arm_sec=move_lib.DEFAULT_KFS_POSE_ARM_WAIT_SEC,
    fire_sec=move_lib.DEFAULT_KFS_POSE_HOLD_SEC,
    loop_interval_sec=0.02,
):
    """
    KFS 吸取态/抓取态：pose_id 由外部根据高低关系传入。
    suck_count=1 时先将吸盘转到 0deg 并选择 PF2；
    suck_count=2 时先将吸盘转到 180deg 并选择 PF3。
    吸盘旋转和气缸选择后等待 0.5s，再触发吸取姿态。
    完成吸盘方向和气缸预置后，姿态触发阶段只控制 ch5/ch6/ch7，不修改 ch4。
    """
    if suck_count == 1:
        sucker_0deg(sender)
        sucker_select_cylinder(sender, 1)
    elif suck_count == 2:
        sucker_180deg(sender)
        sucker_select_cylinder(sender, 2)
    else:
        print(
            f"{kfs_grab_pose.__name__}输入错误: "
            f"suck_count={suck_count}, 必须是 1 或 2"
        )
        sys.exit(1)

    time.sleep(0.5)

    result = _trigger_kfs_pose_with_lock(
        sender=sender,
        pose_id=pose_id,
        arm_sec=arm_sec,
        fire_sec=fire_sec,
        loop_interval_sec=loop_interval_sec,
    )
    return result


def kfs_transition_pose(
    sender,
    arm_sec=0.1,
    fire_sec=0.3,
    loop_interval_sec=0.02,
):
    """
    KFS 吸取后的过渡态：pose_id=3，不修改吸盘 ch4。
    """
    return _trigger_kfs_pose_with_lock(
        sender=sender,
        pose_id=3,
        arm_sec=arm_sec,
        fire_sec=fire_sec,
        loop_interval_sec=loop_interval_sec,
    )


def kfs_store_pose(
    sender,
    arm_sec=0.1,
    fire_sec=0.4,
    loop_interval_sec=0.02,
):
    """
    KFS 放置姿态：pose_id=4，不修改吸盘 ch4。
    """
    return _trigger_kfs_pose_with_lock(
        sender=sender,
        pose_id=4,
        arm_sec=arm_sec,
        fire_sec=fire_sec,
        loop_interval_sec=loop_interval_sec,
    )


def kfs_side_pose(
    sender,
    arm_sec=0.1,
    fire_sec=1.0,
    loop_interval_sec=0.02,
):
    """KFS 侧吸姿态：pose_id=5，只控制 ch5/ch6/ch7，不修改吸盘 ch4。"""
    return _trigger_kfs_pose_with_lock(
        sender=sender,
        pose_id=5,
        arm_sec=arm_sec,
        fire_sec=fire_sec,
        loop_interval_sec=loop_interval_sec,
    )


def kfs_zero_return_pose(
    sender,
    arm_sec=0.1,
    fire_sec=0.5,
    loop_interval_sec=0.02,
):
    """
    KFS 回归 0 态：pose_id=0，只控制 ch5/ch6/ch7，不修改吸盘 ch4。
    """
    result = _trigger_kfs_pose_with_lock(
        sender=sender,
        pose_id=0,
        arm_sec=arm_sec,
        fire_sec=fire_sec,
        loop_interval_sec=loop_interval_sec,
    )
    result["pose_name"] = "zero_return"
    return result


def _release_kfs_suction_with_lock(
    sender,
    edge_arm_sec=0.1,
    edge_hold_sec=0.5,
    loop_interval_sec=0.02,
    keep_suction_on=False,
):
    with tools.AUTO_TRIGGER_LOCK:
        arm_channels = _repeat_set_channel_values(
            sender,
            {
                KFS_SUCTION_CHANNEL_INDEX: KFS_SUCTION_ON_VALUE,
                KFS_MODE_CHANNEL_INDEX: KFS_MODE_VALUE,
            },
            edge_arm_sec,
            loop_interval_sec=loop_interval_sec,
        )
        fire_suction_value = (
            KFS_SUCTION_ON_VALUE
            if keep_suction_on
            else KFS_SUCTION_OFF_VALUE
        )
        fire_channels = _repeat_set_channel_values(
            sender,
            {
                KFS_SUCTION_CHANNEL_INDEX: fire_suction_value,
                KFS_MODE_CHANNEL_INDEX: KFS_MODE_VALUE,
            },
            edge_hold_sec,
            loop_interval_sec=loop_interval_sec,
        )
        idle_channel_values = {
            KFS_SUCTION_CHANNEL_INDEX: fire_suction_value,
            KFS_MODE_CHANNEL_INDEX: tools.SAFE_SWITCH_VALUE,
            KFS_POSE_CHANNEL_INDEX: tools.SAFE_SWITCH_VALUE,
            KFS_TRIGGER_CHANNEL_INDEX: tools.SAFE_SWITCH_VALUE,
        }
        idle_channels = move_lib.set_channel_values(sender, channel_values=idle_channel_values)
    return {
        "suction_on": bool(keep_suction_on),
        "arm_channels": arm_channels,
        "fire_channels": fire_channels,
        "idle_channels": idle_channels,
        "edge_arm_sec": float(edge_arm_sec),
        "edge_hold_sec": float(edge_hold_sec),
        "ch4_only": False,
        "release_edge_suppressed": bool(keep_suction_on),
        "completed": True,
    }


def start_kfs_post_suction_thread(
    sender,
    transition_pose_hold_sec=None,
    loop_interval_sec=0.02,
    reset_kfs_channels=True,
    thread_name="kfs_post_suction_thread",
):
    """
    启动 KFS 吸取保持完成后的异步后续线程。

    默认只执行机械臂后续并保持吸盘：
    1. pose_id=3 过渡态，不修改 ch4。
    2. suck_count=1 时将吸盘转到 180deg；suck_count=2 时转到 0deg。
    3. 吸盘旋转后等待 0.5s，再执行 pose_id=0 回 0 态。
    4. 回 0 态完成后 suck_count 加 1。
    5. 可选复位 ch5/ch6/ch7，全程保留 ch4=3。

    pose_id=4 是放置姿态，不参与吸取后续流程。

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

    def worker():
        global suck_count

        try:
            transition_pose_result = kfs_transition_pose(
                sender=sender,
                loop_interval_sec=loop_interval_sec,
            )
            time.sleep(transition_wait_sec)

            suck_count_before = int(suck_count)
            if suck_count_before == 1:
                sucker_rotation_result = sucker_180deg(sender)
            elif suck_count_before == 2:
                sucker_rotation_result = sucker_0deg(sender)
            else:
                raise ValueError(
                    f"{start_kfs_post_suction_thread.__name__}输入错误: "
                    f"suck_count={suck_count_before}, 必须是 1 或 2"
                )
            time.sleep(0.5)

            zero_pose_result = kfs_zero_return_pose(
                sender=sender,
                loop_interval_sec=loop_interval_sec,
            )
            suck_count += 1

            reset_channels = None
            if reset_kfs_channels:
                reset_channels = move_lib.set_channel_values(
                    sender,
                    channel_values={
                        KFS_MODE_CHANNEL_INDEX: tools.SAFE_SWITCH_VALUE,
                        KFS_POSE_CHANNEL_INDEX: tools.SAFE_SWITCH_VALUE,
                        KFS_TRIGGER_CHANNEL_INDEX: tools.SAFE_SWITCH_VALUE,
                    },
                )

            result.update({
                "completed": True,
                "running": False,
                "transition_pose_result": transition_pose_result,
                "sucker_rotation_result": sucker_rotation_result,
                "zero_pose_result": zero_pose_result,
                "reset_kfs_channels": reset_channels,
                "transition_wait_sec": float(transition_wait_sec),
                "sucker_rotation_wait_sec": 0.5,
                "suck_count_before": suck_count_before,
                "suck_count_after": int(suck_count),
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
