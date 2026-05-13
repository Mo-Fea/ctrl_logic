#!/usr/bin/env python3

from lib2 import tools


def main():
    sender = None

    try:
        print("Connecting to chassis controller...")
        sock = tools.connect()
        sender = tools.frame_thread(
            sock=sock,
            hz=70.0,
            connect_func=tools.connect,
            tcp_ip=tools.TCP_IP,
            tcp_port=tools.TCP_PORT,
            retry_interval=tools.CONNECT_RETRY_INTERVAL,
        )
        sender.start()
        sender.set_all_zero(yaw_i16=0, des_yaw_i16=0)
        print("Frame thread started with des_yaw_i16=0. Press Ctrl+C to stop.")

        while True:
            tools.time.sleep(1.0)

    except KeyboardInterrupt:
        print("\nStopped by user.")

    finally:
        if sender is not None:
            try:
                sender.stop(send_stop=True)
                tools.socket_close(sender.sock)
            except Exception as exc:
                print(f"Cleanup failed: {exc}")


if __name__ == "__main__":
    main()
