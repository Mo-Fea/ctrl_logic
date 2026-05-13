#!/usr/bin/env python3

import time

from lib2 import tools


FORWARD_CMD = 300
DURATION_SEC = 5.0
SEND_HZ = 70.0


def main():
    sender = None

    try:
        print("Starting forward test...")
        sock = tools.connect()
        sender = tools.frame_thread(sock=sock, hz=SEND_HZ)
        sender.set_channels_and_des_yaw_i16(
            tools.compose_channels(
                lateral_cmd=0,
                forward_cmd=FORWARD_CMD,
                rotation_cmd=0,
            ),
            0,
        )
        sender.start()
        if not sender.wait_until_first_send(timeout_sec=2.0):
            raise RuntimeError("frame_thread did not send first frame within 2s")

        print(f"Forward cmd ch2={FORWARD_CMD} for {DURATION_SEC:.1f}s...")
        time.sleep(DURATION_SEC)

    except KeyboardInterrupt:
        print("\nStopped by user.")

    finally:
        if sender is not None:
            sender.set_all_zero()
            time.sleep(0.1)
            sender.stop(send_stop=True)
            tools.socket_close(sender.sock)
        print("Forward test finished.")


if __name__ == "__main__":
    main()
