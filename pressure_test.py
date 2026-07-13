import argparse
import time

from lib2 import kfs
from lib2 import module
from lib2 import move
from lib2 import tools


DEFAULT_PRESSURE_BOUNDARY = -20000.0


def parse_args():
    parser = argparse.ArgumentParser(
        description="Run the PF2 pressure-triggered KFS pose test."
    )
    parser.add_argument(
        "--tcp-ip",
        default=tools.TCP_IP,
        help="Chassis controller TCP server IP.",
    )
    parser.add_argument(
        "--tcp-port",
        type=int,
        default=tools.TCP_PORT,
        help="Chassis controller TCP server port.",
    )
    parser.add_argument(
        "--hz",
        type=float,
        default=70.0,
        help="Control frame send frequency.",
    )
    parser.add_argument(
        "--pressure-boundary",
        type=float,
        default=DEFAULT_PRESSURE_BOUNDARY,
        help="Continue when latest pressure is below this value.",
    )
    return parser.parse_args()


def require_completed(result, step_name):
    if result is None or not result.get("completed", False):
        raise RuntimeError(f"{step_name} failed: {result}")
    return result


def main():
    args = parse_args()
    if args.hz <= 0.0:
        raise ValueError(f"hz must be > 0, got {args.hz}")

    sock = tools.connect(tcp_ip=args.tcp_ip, tcp_port=args.tcp_port)
    sender = tools.frame_thread(
        sock=sock,
        hz=args.hz,
        tcp_ip=args.tcp_ip,
        tcp_port=args.tcp_port,
    )
    sender.start()

    try:
        if not sender.wait_until_first_send(timeout_sec=2.0):
            raise RuntimeError("control frame did not send within 2 seconds")

        print("1. Initialize sucker to 0deg")
        require_completed(kfs.sucker_0deg(sender), "sucker_0deg")

        print("2. KFS zero-return pose")
        require_completed(kfs.kfs_zero_return_pose(sender), "kfs_zero_return_pose")

        print("3. Select PF2")
        require_completed(kfs.sucker_select_pf2(sender), "sucker_select_pf2")

        print("4. Enable PF2 suction")
        require_completed(
            module.set_kfs_suction(sender, suction_on=True, pose_id=0),
            "enable_pf2_suction",
        )

        print(f"5. Wait for pressure < {args.pressure_boundary}")
        pressure_result = move.block_till_pressure(
            sender=sender,
            boundary=args.pressure_boundary,
            mode=0,
        )
        print(
            "Pressure threshold reached: "
            f"pressure={pressure_result['latest_pressure']}"
        )

        print("6. Wait 2 seconds")
        time.sleep(2.0)

        print("7. KFS place pose")
        require_completed(kfs.place_kfs_pose(sender), "place_kfs_pose")

        print("8. Disable PF2 suction")
        require_completed(
            module.set_kfs_suction(sender, suction_on=False, pose_id=4),
            "disable_pf2_suction",
        )
        print("Pressure test completed")
    finally:
        sender.stop(send_stop=True)
        tools.socket_close(sender.sock)


if __name__ == "__main__":
    main()
