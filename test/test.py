#!/usr/bin/env python3

from lib2 import tools


SEND_HZ = 70.0
FORWARD_CMD = 200
FIRST_FORWARD_SEC = 2.0
SECOND_FORWARD_SEC = 5.0
WAIT_AFTER_FIRST_CLIMB_SEC = 10.0
FINAL_HOLD_SEC = 15.0
LOOP_INTERVAL_SEC = 0.02

CLIMB_MODE_CHANNEL_INDEX = 5
CLIMB_MODE_VALUE = 1
CLIMB_TRIGGER_CHANNEL_INDEX = 7
CLIMB_TRIGGER_ARM_VALUE = 1
CLIMB_TRIGGER_FIRE_VALUE = 3
CLIMB_TRIGGER_IDLE_VALUE = 0
CLIMB_TRIGGER_ARM_SEC = 0.1
CLIMB_TRIGGER_HOLD_SEC = 2.0


def set_channels_for_duration(
    sender,
    channels,
    duration_sec,
    des_yaw_i16=0,
    loop_interval_sec=LOOP_INTERVAL_SEC,
):
    deadline = tools.time.time() + float(duration_sec)
    while tools.time.time() < deadline:
        sender.set_channels_and_des_yaw_i16(channels, des_yaw_i16)
        tools.time.sleep(loop_interval_sec)


def stop_chassis(sender, des_yaw_i16=0):
    channels = tools.compose_channels(lateral_cmd=0, forward_cmd=0, rotation_cmd=0)
    sender.set_channels_and_des_yaw_i16(channels, des_yaw_i16)
    return channels


def hold_still(sender, duration_sec, des_yaw_i16=0):
    channels = tools.compose_channels(lateral_cmd=0, forward_cmd=0, rotation_cmd=0)
    set_channels_for_duration(sender, channels, duration_sec, des_yaw_i16=des_yaw_i16)
    return channels


def drive_forward(sender, duration_sec, forward_cmd=FORWARD_CMD):
    channels = tools.compose_channels(
        lateral_cmd=0,
        forward_cmd=int(forward_cmd),
        rotation_cmd=0,
    )
    set_channels_for_duration(sender, channels, duration_sec, des_yaw_i16=0)
    return stop_chassis(sender, des_yaw_i16=0)


def climb_trigger(sender):
    def climb_channels(trigger_value):
        channels = tools.compose_channels(lateral_cmd=0, forward_cmd=0, rotation_cmd=0)
        channels[CLIMB_MODE_CHANNEL_INDEX] = CLIMB_MODE_VALUE
        channels[CLIMB_TRIGGER_CHANNEL_INDEX] = int(trigger_value)
        return channels

    arm_channels = climb_channels(CLIMB_TRIGGER_ARM_VALUE)
    set_channels_for_duration(
        sender,
        arm_channels,
        CLIMB_TRIGGER_ARM_SEC,
        des_yaw_i16=0,
    )

    fire_channels = climb_channels(CLIMB_TRIGGER_FIRE_VALUE)
    set_channels_for_duration(
        sender,
        fire_channels,
        CLIMB_TRIGGER_HOLD_SEC,
        des_yaw_i16=0,
    )

    idle_channels = climb_channels(CLIMB_TRIGGER_IDLE_VALUE)
    sender.set_channels_and_des_yaw_i16(idle_channels, 0)
    return idle_channels


def main():
    sender = None

    try:
        print("Connecting to chassis controller...")
        sock = tools.connect()
        sender = tools.frame_thread(
            sock=sock,
            hz=SEND_HZ,
            connect_func=tools.connect,
            tcp_ip=tools.TCP_IP,
            tcp_port=tools.TCP_PORT,
            retry_interval=tools.CONNECT_RETRY_INTERVAL,
        )
        sender.start()

        print(f"Forward {FIRST_FORWARD_SEC:.1f}s, ch2={FORWARD_CMD}")
        drive_forward(sender, FIRST_FORWARD_SEC, FORWARD_CMD)
        print("Stop after first forward")
        stop_chassis(sender, des_yaw_i16=0)

        print(f"Trigger climb #1 for {CLIMB_TRIGGER_HOLD_SEC:.1f}s")
        climb_trigger(sender)
        print(f"Hold still {WAIT_AFTER_FIRST_CLIMB_SEC:.1f}s after climb #1")
        hold_still(sender, WAIT_AFTER_FIRST_CLIMB_SEC, des_yaw_i16=0)

        print(f"Forward {SECOND_FORWARD_SEC:.1f}s, ch2={FORWARD_CMD}")
        drive_forward(sender, SECOND_FORWARD_SEC, FORWARD_CMD)
        print("Stop after second forward")
        stop_chassis(sender, des_yaw_i16=0)

        print(f"Trigger climb #2 for {CLIMB_TRIGGER_HOLD_SEC:.1f}s")
        climb_trigger(sender)
        print(f"Hold still {FINAL_HOLD_SEC:.1f}s before shutdown")
        hold_still(sender, FINAL_HOLD_SEC, des_yaw_i16=0)

        print("Test finished.")

    except KeyboardInterrupt:
        print("\nStopped by user.")

    finally:
        if sender is not None:
            try:
                stop_chassis(sender, des_yaw_i16=0)
                sender.stop(send_stop=True)
                tools.socket_close(sender.sock)
            except Exception as exc:
                print(f"Cleanup failed: {exc}")


if __name__ == "__main__":
    main()
