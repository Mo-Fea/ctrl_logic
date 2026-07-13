#!/usr/bin/env python3

import argparse
import time

from lib2 import kfs
from lib2 import module
from lib2 import position_backend
from lib2 import tools


RELEASE_POSE_WAIT_SEC = 5.0
POST_RELEASE_WAIT_SEC = 5.0


def parse_args():
    parser = argparse.ArgumentParser(
        description="Initialize the robot, then release both KFS sucker cylinders."
    )
    parser.add_argument("--tcp-ip", default=tools.TCP_IP)
    parser.add_argument("--tcp-port", type=int, default=tools.TCP_PORT)
    parser.add_argument("--hz", type=float, default=70.0)
    parser.add_argument("--flag-topic", default="/odin1/flag1")
    return parser.parse_args()


def require_completed(result, step_name):
    if result is None or not result.get("completed", False):
        raise RuntimeError(f"{step_name} failed: {result}")
    return result


def release_once(sender, release_index):
    print(f"{release_index}. Rotate sucker to release pose")
    require_completed(kfs.sucker_release_pose(sender), "sucker_release_pose")

    print(f"{release_index}. Wait {RELEASE_POSE_WAIT_SEC:.1f}s")
    time.sleep(RELEASE_POSE_WAIT_SEC)

    print(f"{release_index}. Release KFS")
    require_completed(kfs.release_kfs(sender), "release_kfs")

    print(f"{release_index}. Wait {POST_RELEASE_WAIT_SEC:.1f}s")
    time.sleep(POST_RELEASE_WAIT_SEC)


def main():
    args = parse_args()
    if args.hz <= 0.0:
        raise ValueError(f"hz must be > 0, got {args.hz}")

    sender = None
    try:
        # module.init blocks until /odin1/flag1 reports either localization mode.
        sender, _, _, _, _ = module.init(
            hz=args.hz,
            lidar_type=position_backend.LIDAR_TYPE_ODIN,
            topic=args.flag_topic,
            tcp_ip=args.tcp_ip,
            tcp_port=args.tcp_port,
            wait_relocalization=True,
            initialize_machine_pose=True,
        )

        print("1. Enable PF2 and PF3 suction; set suck_count to 3")
        require_completed(
            kfs.kfs_suck_preparation(sender, count=2),
            "kfs_suck_preparation",
        )

        release_once(sender, release_index=2)
        release_once(sender, release_index=3)
        print("Sucker release test completed")
    finally:
        if sender is not None:
            sender.stop(send_stop=True)
            tools.socket_close(sender.sock)


if __name__ == "__main__":
    main()
