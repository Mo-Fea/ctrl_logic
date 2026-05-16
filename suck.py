#!/usr/bin/env python3

import time

from lib2 import move, tools


SEND_HZ = 70.0
FINAL_YAW_DEG = 0.01

KFS_MODE_VALUE = 2
KFS_SUCTION_OFF_VALUE = 1
KFS_SUCTION_ON_VALUE = 3
KFS_TRIGGER_ARM_VALUE = 1
KFS_TRIGGER_FIRE_VALUE = 3
KFS_TRIGGER_ZERO_VALUE = 0

HIGH_GRAB_POSE_ID = 1
TRANSITION_POSE_ID = 3
STORE_POSE_ID = 4

PRE_GRAB_FORWARD_CMD = 200
PRE_GRAB_FORWARD_SEC = 2.0
POSE_ARM_WAIT_SEC = 0.5
GRAB_POSE_HOLD_SEC = 4.0
SUCTION_EDGE_ARM_SEC = 0.5
SUCTION_EDGE_HOLD_SEC = 0.5
SUCTION_HOLD_SEC = 3.0
TRANSITION_POSE_HOLD_SEC = 3.0
STORE_POSE_HOLD_SEC = 2.0
ZERO_RETURN_WAIT_SEC = 2.0
HOLD_AFTER_DONE_SEC = 5.0

LOOP_INTERVAL_SEC = 0.02
CHANNEL_PRINT_INTERVAL_SEC = 0.2


def kfs_channel_values(pose_id, trigger_value, suction_ch4):
    return {
        4: int(suction_ch4),
        5: KFS_MODE_VALUE,
        6: int(pose_id),
        7: int(trigger_value),
    }


def print_state(sender, phase, started_at):
    state = sender.get_state()
    last_send_time = state.get("last_send_time")
    send_age_sec = None if last_send_time is None else time.time() - float(last_send_time)
    print(
        f"[{phase}] "
        f"t={time.time() - started_at:7.2f}s | "
        f"seq={state['seq']:5d} | "
        f"send_ok={state['last_send_ok']} | "
        f"send_age={send_age_sec if send_age_sec is not None else 'None'} | "
        f"yaw_i16={state['yaw_i16']:6d} | "
        f"des_yaw_i16={state['des_yaw_i16']:6d} | "
        f"ch={state['channels']}"
    )


def send_for_duration(
    sender,
    duration_sec,
    phase,
    des_yaw_i16=0,
    lateral_cmd=0,
    forward_cmd=0,
    rotation_cmd=0,
    channel_values=None,
):
    started_at = time.time()
    deadline = started_at + float(duration_sec)
    next_print = 0.0
    while time.time() < deadline:
        if channel_values is not None:
            move.set_channel_values(sender, channel_values=channel_values)
        move.set_motion_channels(
            sender,
            lateral_cmd=lateral_cmd,
            forward_cmd=forward_cmd,
            rotation_cmd=rotation_cmd,
            des_yaw_i16=des_yaw_i16,
        )
        now = time.time()
        if now >= next_print:
            print_state(sender, phase, started_at)
            next_print = now + CHANNEL_PRINT_INTERVAL_SEC
        time.sleep(LOOP_INTERVAL_SEC)
    if channel_values is not None:
        move.set_channel_values(sender, channel_values=channel_values)
    move.set_motion_channels(
        sender,
        lateral_cmd=lateral_cmd,
        forward_cmd=forward_cmd,
        rotation_cmd=rotation_cmd,
        des_yaw_i16=des_yaw_i16,
    )
    print_state(sender, phase + "_done", started_at)


def trigger_pose(sender, pose_id, hold_sec, suction_ch4):
    arm_channel_values = kfs_channel_values(
        pose_id=pose_id,
        trigger_value=KFS_TRIGGER_ARM_VALUE,
        suction_ch4=suction_ch4,
    )
    send_for_duration(
        sender=sender,
        duration_sec=POSE_ARM_WAIT_SEC,
        phase=f"pose_{pose_id}_arm",
        channel_values=arm_channel_values,
    )

    fire_channel_values = kfs_channel_values(
        pose_id=pose_id,
        trigger_value=KFS_TRIGGER_FIRE_VALUE,
        suction_ch4=suction_ch4,
    )
    send_for_duration(
        sender=sender,
        duration_sec=hold_sec,
        phase=f"pose_{pose_id}_fire",
        channel_values=fire_channel_values,
    )

    idle_channel_values = kfs_channel_values(
        pose_id=HIGH_GRAB_POSE_ID,
        trigger_value=KFS_TRIGGER_ARM_VALUE,
        suction_ch4=suction_ch4,
    )
    move.set_channel_values(sender, channel_values=idle_channel_values)
    move.set_motion_channels(sender, des_yaw_i16=0)
    print_state(sender, f"pose_{pose_id}_idle", time.time())


def set_suction(sender, suction_on):
    arm_value = KFS_SUCTION_OFF_VALUE if suction_on else KFS_SUCTION_ON_VALUE
    fire_value = KFS_SUCTION_ON_VALUE if suction_on else KFS_SUCTION_OFF_VALUE
    phase = "suction_on" if suction_on else "suction_off"

    arm_channel_values = kfs_channel_values(
        pose_id=HIGH_GRAB_POSE_ID,
        trigger_value=KFS_TRIGGER_ARM_VALUE,
        suction_ch4=arm_value,
    )
    send_for_duration(
        sender=sender,
        duration_sec=SUCTION_EDGE_ARM_SEC,
        phase=phase + "_arm",
        channel_values=arm_channel_values,
    )

    fire_channel_values = kfs_channel_values(
        pose_id=HIGH_GRAB_POSE_ID,
        trigger_value=KFS_TRIGGER_ARM_VALUE,
        suction_ch4=fire_value,
    )
    send_for_duration(
        sender=sender,
        duration_sec=SUCTION_EDGE_HOLD_SEC,
        phase=phase + "_edge",
        channel_values=fire_channel_values,
    )
    return fire_channel_values


def main():
    sender = None

    try:
        print("Starting direct -1 -> 2 high KFS grab test without localization.")
        sock = tools.connect()
        sender = tools.frame_thread(sock=sock, hz=SEND_HZ)
        sender.set_des_yaw_i16(0)
        sender.start()
        if not sender.wait_until_first_send(timeout_sec=2.0):
            raise RuntimeError("frame_thread did not send first frame within 2s")

        target_yaw_i16 = move.encode_target_yaw_i16(FINAL_YAW_DEG)
        send_for_duration(
            sender=sender,
            duration_sec=PRE_GRAB_FORWARD_SEC,
            phase="pre_grab_forward",
            des_yaw_i16=target_yaw_i16,
            forward_cmd=PRE_GRAB_FORWARD_CMD,
        )

        send_for_duration(
            sender=sender,
            duration_sec=0.3,
            phase="pre_grab_stop",
            des_yaw_i16=target_yaw_i16,
        )

        trigger_pose(
            sender=sender,
            pose_id=HIGH_GRAB_POSE_ID,
            hold_sec=GRAB_POSE_HOLD_SEC,
            suction_ch4=KFS_SUCTION_OFF_VALUE,
        )

        suction_hold_channel_values = set_suction(sender=sender, suction_on=True)
        send_for_duration(
            sender=sender,
            duration_sec=SUCTION_HOLD_SEC,
            phase="suction_hold_ch4_3",
            channel_values=suction_hold_channel_values,
        )

        trigger_pose(
            sender=sender,
            pose_id=TRANSITION_POSE_ID,
            hold_sec=TRANSITION_POSE_HOLD_SEC,
            suction_ch4=KFS_SUCTION_ON_VALUE,
        )

        trigger_pose(
            sender=sender,
            pose_id=STORE_POSE_ID,
            hold_sec=STORE_POSE_HOLD_SEC,
            suction_ch4=KFS_SUCTION_ON_VALUE,
        )

        set_suction(sender=sender, suction_on=False)

        zero_channel_values = kfs_channel_values(
            pose_id=0,
            trigger_value=KFS_TRIGGER_ZERO_VALUE,
            suction_ch4=KFS_SUCTION_OFF_VALUE,
        )
        send_for_duration(
            sender=sender,
            duration_sec=ZERO_RETURN_WAIT_SEC,
            phase="zero_return",
            channel_values=zero_channel_values,
        )

        send_for_duration(
            sender=sender,
            duration_sec=HOLD_AFTER_DONE_SEC,
            phase="hold_after_done",
            channel_values=zero_channel_values,
        )
        print("Direct -1 -> 2 high KFS grab test finished.")

    except KeyboardInterrupt:
        print("\nStopped by user.")

    finally:
        if sender is not None:
            sender.stop(send_stop=True)
            tools.socket_close(sender.sock)


if __name__ == "__main__":
    main()
