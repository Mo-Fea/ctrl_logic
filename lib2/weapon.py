import time

from lib2 import tools


WEAPON_LIFT_CHANNEL_INDEX = 1
WEAPON_GRIPPER_CHANNEL_INDEX = 4
WEAPON_MODE_CHANNEL_INDEX = 5
WEAPON_SUBFUNCTION_CHANNEL_INDEX = 6
WEAPON_TRIGGER_CHANNEL_INDEX = 7

WEAPON_MODE_VALUE = 3
WEAPON_UP_VALUE = 100
WEAPON_DOWN_VALUE = -100
# PF1 实机语义：ch4=3 为夹爪打开，ch4=1 为夹爪闭合。
WEAPON_GRIPPER_LOOSE_VALUE = 3
WEAPON_GRIPPER_SEIZE_VALUE = 1

DEFAULT_MOTION_HOLD_SEC = 0.2
DEFAULT_EDGE_ARM_SEC = 0.2
DEFAULT_EDGE_HOLD_SEC = 0.2
DEFAULT_LOOP_INTERVAL_SEC = 0.02


def _validate_duration(value, name):
    value = float(value)
    if value < 0.0:
        raise ValueError(f"{name} must be >= 0, got {value}")
    return value


def _weapon_channel_values(channel_values):
    values = {
        WEAPON_MODE_CHANNEL_INDEX: WEAPON_MODE_VALUE,
        WEAPON_SUBFUNCTION_CHANNEL_INDEX: tools.SAFE_SWITCH_VALUE,
        WEAPON_TRIGGER_CHANNEL_INDEX: tools.SAFE_SWITCH_VALUE,
    }
    values.update(channel_values)
    return values


def _repeat_set_channel_values(
    sender,
    channel_values,
    duration_sec,
    loop_interval_sec=DEFAULT_LOOP_INTERVAL_SEC,
):
    duration_sec = _validate_duration(duration_sec, "duration_sec")
    loop_interval_sec = float(loop_interval_sec)
    if loop_interval_sec <= 0.0:
        raise ValueError(
            f"loop_interval_sec must be > 0, got {loop_interval_sec}"
        )

    deadline = time.time() + duration_sec
    while time.time() < deadline:
        sender.set_channel_values(channel_values)
        time.sleep(loop_interval_sec)
    return sender.set_channel_values(channel_values)


def weapon_up(
    sender,
    value=WEAPON_UP_VALUE,
    hold_sec=DEFAULT_MOTION_HOLD_SEC,
    loop_interval_sec=DEFAULT_LOOP_INTERVAL_SEC,
    mode_arm_sec=DEFAULT_EDGE_ARM_SEC,
):
    """预置 weapon 模式，短时触发 ch1 正值拉起夹爪，再安全退出模式。"""
    value = int(value)
    if value <= 0:
        raise ValueError(f"weapon_up value must be > 0, got {value}")

    with tools.AUTO_TRIGGER_LOCK:
        arm_values = _weapon_channel_values({WEAPON_LIFT_CHANNEL_INDEX: 0})
        arm_channels = _repeat_set_channel_values(
            sender,
            arm_values,
            mode_arm_sec,
            loop_interval_sec=loop_interval_sec,
        )

        fire_values = _weapon_channel_values({WEAPON_LIFT_CHANNEL_INDEX: value})
        fire_channels = _repeat_set_channel_values(
            sender,
            fire_values,
            hold_sec,
            loop_interval_sec=loop_interval_sec,
        )

        reset_channels = sender.set_channel_values(
            {
                WEAPON_LIFT_CHANNEL_INDEX: 0,
                WEAPON_MODE_CHANNEL_INDEX: tools.SAFE_SWITCH_VALUE,
                WEAPON_SUBFUNCTION_CHANNEL_INDEX: tools.SAFE_SWITCH_VALUE,
                WEAPON_TRIGGER_CHANNEL_INDEX: tools.SAFE_SWITCH_VALUE,
            }
        )

    return {
        "action": "weapon_up",
        "value": value,
        "channels": fire_channels,
        "arm_channels": arm_channels,
        "fire_channels": fire_channels,
        "reset_channels": reset_channels,
        "mode_arm_sec": float(mode_arm_sec),
        "hold_sec": float(hold_sec),
        "completed": True,
    }


def weapon_down(
    sender,
    value=WEAPON_DOWN_VALUE,
    hold_sec=DEFAULT_MOTION_HOLD_SEC,
    loop_interval_sec=DEFAULT_LOOP_INTERVAL_SEC,
    mode_arm_sec=DEFAULT_EDGE_ARM_SEC,
):
    """预置 weapon 模式，短时触发 ch1 负值放下夹爪，再安全退出模式。"""
    value = int(value)
    if value >= 0:
        raise ValueError(f"weapon_down value must be < 0, got {value}")

    with tools.AUTO_TRIGGER_LOCK:
        arm_values = _weapon_channel_values({WEAPON_LIFT_CHANNEL_INDEX: 0})
        arm_channels = _repeat_set_channel_values(
            sender,
            arm_values,
            mode_arm_sec,
            loop_interval_sec=loop_interval_sec,
        )

        fire_values = _weapon_channel_values({WEAPON_LIFT_CHANNEL_INDEX: value})
        fire_channels = _repeat_set_channel_values(
            sender,
            fire_values,
            hold_sec,
            loop_interval_sec=loop_interval_sec,
        )

        reset_channels = sender.set_channel_values(
            {
                WEAPON_LIFT_CHANNEL_INDEX: 0,
                WEAPON_MODE_CHANNEL_INDEX: tools.SAFE_SWITCH_VALUE,
                WEAPON_SUBFUNCTION_CHANNEL_INDEX: tools.SAFE_SWITCH_VALUE,
                WEAPON_TRIGGER_CHANNEL_INDEX: tools.SAFE_SWITCH_VALUE,
            }
        )

    return {
        "action": "weapon_down",
        "value": value,
        "channels": fire_channels,
        "arm_channels": arm_channels,
        "fire_channels": fire_channels,
        "reset_channels": reset_channels,
        "mode_arm_sec": float(mode_arm_sec),
        "hold_sec": float(hold_sec),
        "completed": True,
    }


def weapon_seize(
    sender,
    edge_arm_sec=DEFAULT_EDGE_ARM_SEC,
    edge_hold_sec=DEFAULT_EDGE_HOLD_SEC,
    loop_interval_sec=DEFAULT_LOOP_INTERVAL_SEC,
):
    """通过 ch4: 3 -> 1 边沿闭合夹爪，返回时保持 ch4=1。"""
    with tools.AUTO_TRIGGER_LOCK:
        arm_values = _weapon_channel_values(
            {
                WEAPON_LIFT_CHANNEL_INDEX: 0,
                WEAPON_GRIPPER_CHANNEL_INDEX: WEAPON_GRIPPER_LOOSE_VALUE,
            }
        )
        arm_channels = _repeat_set_channel_values(
            sender,
            arm_values,
            edge_arm_sec,
            loop_interval_sec=loop_interval_sec,
        )

        fire_values = _weapon_channel_values(
            {
                WEAPON_LIFT_CHANNEL_INDEX: 0,
                WEAPON_GRIPPER_CHANNEL_INDEX: WEAPON_GRIPPER_SEIZE_VALUE,
            }
        )
        fire_channels = _repeat_set_channel_values(
            sender,
            fire_values,
            edge_hold_sec,
            loop_interval_sec=loop_interval_sec,
        )

        reset_channels = sender.set_channel_values(
            {
                WEAPON_MODE_CHANNEL_INDEX: tools.SAFE_SWITCH_VALUE,
                WEAPON_SUBFUNCTION_CHANNEL_INDEX: tools.SAFE_SWITCH_VALUE,
                WEAPON_TRIGGER_CHANNEL_INDEX: tools.SAFE_SWITCH_VALUE,
            }
        )

    return {
        "action": "weapon_seize",
        "arm_channels": arm_channels,
        "fire_channels": fire_channels,
        "reset_channels": reset_channels,
        "edge_arm_sec": float(edge_arm_sec),
        "edge_hold_sec": float(edge_hold_sec),
        "completed": True,
    }


def weapon_loose(
    sender,
    edge_arm_sec=DEFAULT_EDGE_ARM_SEC,
    edge_hold_sec=DEFAULT_EDGE_HOLD_SEC,
    loop_interval_sec=DEFAULT_LOOP_INTERVAL_SEC,
):
    """通过 ch4: 1 -> 3 边沿打开夹爪，返回时保持 ch4=3。"""
    with tools.AUTO_TRIGGER_LOCK:
        arm_values = _weapon_channel_values(
            {
                WEAPON_LIFT_CHANNEL_INDEX: 0,
                WEAPON_GRIPPER_CHANNEL_INDEX: WEAPON_GRIPPER_SEIZE_VALUE,
            }
        )
        arm_channels = _repeat_set_channel_values(
            sender,
            arm_values,
            edge_arm_sec,
            loop_interval_sec=loop_interval_sec,
        )

        fire_values = _weapon_channel_values(
            {
                WEAPON_LIFT_CHANNEL_INDEX: 0,
                WEAPON_GRIPPER_CHANNEL_INDEX: WEAPON_GRIPPER_LOOSE_VALUE,
            }
        )
        fire_channels = _repeat_set_channel_values(
            sender,
            fire_values,
            edge_hold_sec,
            loop_interval_sec=loop_interval_sec,
        )

        reset_channels = sender.set_channel_values(
            {
                WEAPON_MODE_CHANNEL_INDEX: tools.SAFE_SWITCH_VALUE,
                WEAPON_SUBFUNCTION_CHANNEL_INDEX: tools.SAFE_SWITCH_VALUE,
                WEAPON_TRIGGER_CHANNEL_INDEX: tools.SAFE_SWITCH_VALUE,
            }
        )

    return {
        "action": "weapon_loose",
        "arm_channels": arm_channels,
        "fire_channels": fire_channels,
        "reset_channels": reset_channels,
        "edge_arm_sec": float(edge_arm_sec),
        "edge_hold_sec": float(edge_hold_sec),
        "completed": True,
    }
