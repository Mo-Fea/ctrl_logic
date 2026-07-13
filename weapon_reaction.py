"""Weapon reaction flow driven by D455 QR recognition."""

import argparse
import time

from lib2 import tools
from lib2 import weapon
from utils import challenge_lib


DEFAULT_SCANNER_START_TIMEOUT_SEC = 2.0
DEFAULT_FINAL_OPEN_HOLD_SEC = 5.0


def _wait_until_lock_held(scanner, running_lock, timeout_sec):
    """Wait until the scanner thread has acquired its running lock."""
    deadline = time.monotonic() + float(timeout_sec)
    while time.monotonic() < deadline:
        if running_lock.locked():
            return True
        if scanner.thread is not None and not scanner.thread.is_alive():
            return False
        time.sleep(0.01)
    return bool(running_lock.locked())


def run_weapon_reaction(
    tcp_ip=tools.TCP_IP,
    tcp_port=tools.TCP_PORT,
    sender_hz=70.0,
    image_source=1,
    stable_frame_count=2,
    scanner_start_timeout_sec=DEFAULT_SCANNER_START_TIMEOUT_SEC,
    qr_timeout_sec=None,
    final_open_hold_sec=DEFAULT_FINAL_OPEN_HOLD_SEC,
    show_window=False,
):
    """
    Start QR recognition and perform the weapon reaction sequence.

    1. Connect to the lower controller and start frame transmission.
    2. Start the D455 QR scanner and wait until it owns SCANNER_RUNNING_LOCK.
    3. Close the weapon gripper and raise it while QR recognition continues.
    4. Wait for the scanner to release its lock after a valid QR is recognized.
    5. Open the gripper, wait five seconds, then close it again.
    """
    sender = None
    scanner = None
    scanner_lock_started = False
    qr_lock_released = False
    initial_seize_result = None
    weapon_up_result = None
    weapon_loose_result = None
    final_seize_result = None

    try:
        sock = tools.connect(tcp_ip=tcp_ip, tcp_port=int(tcp_port))
        sender = tools.frame_thread(
            sock=sock,
            hz=float(sender_hz),
            connect_func=tools.connect,
            tcp_ip=tcp_ip,
            tcp_port=int(tcp_port),
        )
        sender.start()
        if not sender.wait_until_first_send(timeout_sec=2.0):
            raise RuntimeError("frame_thread did not send its first control frame")

        scanner = challenge_lib.start_background_qr_scanner(
            stable_frame_count=int(stable_frame_count),
            show_window=bool(show_window),
            stop_after_success=True,
            image_source=int(image_source),
        )
        scanner_lock_started = _wait_until_lock_held(
            scanner=scanner,
            running_lock=challenge_lib.SCANNER_RUNNING_LOCK,
            timeout_sec=scanner_start_timeout_sec,
        )
        if not scanner_lock_started:
            raise RuntimeError(
                "QR scanner did not acquire SCANNER_RUNNING_LOCK: "
                f"{scanner.last_error!r}"
            )

        initial_seize_result = weapon.weapon_seize(sender)
        if not initial_seize_result.get("completed", False):
            raise RuntimeError("initial weapon_seize failed")

        weapon_up_result = weapon.weapon_up(sender)
        if not weapon_up_result.get("completed", False):
            raise RuntimeError("weapon_up failed")

        qr_lock_released = challenge_lib.wait_until_scanner_released(
            running_lock=challenge_lib.SCANNER_RUNNING_LOCK,
            timeout=qr_timeout_sec,
        )
        if not qr_lock_released:
            raise TimeoutError("timed out waiting for QR scanner lock release")

        scanner.join(timeout=1.0)
        if scanner.last_result is None:
            raise RuntimeError(
                "QR scanner released its lock without a valid QR result: "
                f"{scanner.last_error!r}"
            )

        weapon_loose_result = weapon.weapon_loose(sender)
        if not weapon_loose_result.get("completed", False):
            raise RuntimeError("weapon_loose failed")

        time.sleep(float(final_open_hold_sec))

        final_seize_result = weapon.weapon_seize(sender)
        if not final_seize_result.get("completed", False):
            raise RuntimeError("final weapon_seize failed")

        return {
            "completed": True,
            "failed_step": None,
            "scanner_lock_started": scanner_lock_started,
            "qr_lock_released": qr_lock_released,
            "qr_data": scanner.last_qr_data,
            "initial_seize_result": initial_seize_result,
            "weapon_up_result": weapon_up_result,
            "weapon_loose_result": weapon_loose_result,
            "final_open_hold_sec": float(final_open_hold_sec),
            "final_seize_result": final_seize_result,
        }
    except Exception as exc:
        return {
            "completed": False,
            "failed_step": "weapon_reaction",
            "exception": repr(exc),
            "scanner_lock_started": scanner_lock_started,
            "qr_lock_released": qr_lock_released,
            "scanner_error": None if scanner is None else repr(scanner.last_error),
            "initial_seize_result": initial_seize_result,
            "weapon_up_result": weapon_up_result,
            "weapon_loose_result": weapon_loose_result,
            "final_seize_result": final_seize_result,
        }
    finally:
        if scanner is not None:
            scanner.stop()
            scanner.join(timeout=1.0)
        if sender is not None:
            sender.stop(send_stop=True)
            tools.socket_close(sender.sock)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ip", default=tools.TCP_IP)
    parser.add_argument("--port", type=int, default=tools.TCP_PORT)
    parser.add_argument("--qr-timeout", type=float, default=None)
    parser.add_argument("--show-window", action="store_true")
    args = parser.parse_args()

    result = run_weapon_reaction(
        tcp_ip=args.ip,
        tcp_port=args.port,
        qr_timeout_sec=args.qr_timeout,
        show_window=args.show_window,
    )
    print(result)
    if not result["completed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
