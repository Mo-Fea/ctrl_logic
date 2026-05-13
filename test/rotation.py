#!/usr/bin/env python3

from lib2 import tools


SEND_HZ = 70.0
TARGET_YAW_DEG = 90.0
TARGET_YAW_I16 = tools.yaw_deg_to_i16(TARGET_YAW_DEG)
RELOCALIZATION_TOPIC = "/odin1/flag1"


def main():
    sender = None
    flag_node = None
    flag_thread = None
    flag_stop_event = None

    try:
        print("Connecting to chassis controller...")
        sock = tools.connect()
        tools.relocalization_flag = False
        _, flag_node, flag_thread, flag_stop_event = tools.relocalization_conformation(
            topic=RELOCALIZATION_TOPIC
        )
        sender = tools.frame_thread(
            sock=sock,
            hz=SEND_HZ,
            connect_func=tools.connect,
            tcp_ip=tools.TCP_IP,
            tcp_port=tools.TCP_PORT,
            retry_interval=tools.CONNECT_RETRY_INTERVAL,
        )

        channels = tools.compose_channels(
            lateral_cmd=0,
            forward_cmd=0,
            rotation_cmd=0,
        )
        sender.set_channels_and_des_yaw_i16(channels, TARGET_YAW_I16)
        sender.start()

        print(
            f"Sending target yaw continuously: {TARGET_YAW_DEG:.2f} deg "
            f"({TARGET_YAW_I16}). Press Ctrl+C to stop."
        )
        while True:
            tools.time.sleep(1.0)

    except KeyboardInterrupt:
        print("\nStopped by user.")

    finally:
        if sender is not None:
            try:
                sender.set_all_zero(yaw_i16=0, des_yaw_i16=0)
                sender.stop(send_stop=True)
                tools.socket_close(sender.sock)
            except Exception as exc:
                print(f"Cleanup failed: {exc}")
        tools.destroy_ros2_thread(
            node=flag_node,
            spin_thread=flag_thread,
            stop_event=flag_stop_event,
            shutdown_rclpy=True,
        )


if __name__ == "__main__":
    main()
