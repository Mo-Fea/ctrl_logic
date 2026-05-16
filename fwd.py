#!/usr/bin/env python3

import time

from lib2 import move
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
        sender.set_des_yaw_i16(0)
        sender.start()
        if not sender.wait_until_first_send(timeout_sec=2.0):
            raise RuntimeError("frame_thread did not send first frame within 2s")

        print(f"Forward cmd ch2={FORWARD_CMD} for {DURATION_SEC:.1f}s...")
        deadline = time.time() + DURATION_SEC
        while time.time() < deadline:
            move.set_motion_channels(
                sender,
                lateral_cmd=0,
                forward_cmd=FORWARD_CMD,
                rotation_cmd=0,
                des_yaw_i16=0,
            )
            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\nStopped by user.")

    finally:
        if sender is not None:
            move.set_motion_channels(sender, des_yaw_i16=0)
            time.sleep(0.1)
            sender.stop(send_stop=True)
            tools.socket_close(sender.sock)
        print("Forward test finished.")


if __name__ == "__main__":
    main()
