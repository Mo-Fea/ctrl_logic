import argparse
import threading
import time

import rclpy

from lib2 import position_odin


def normalize_yaw_deg(yaw_deg):
    while yaw_deg >= 180.0:
        yaw_deg -= 360.0
    while yaw_deg < -180.0:
        yaw_deg += 360.0
    return yaw_deg


def parse_args():
    parser = argparse.ArgumentParser(
        description="Continuously print weapon pose calculated from odin TF."
    )
    parser.add_argument(
        "--base-frame",
        default="odin1_base_link",
        help="Odin base frame used by TF cache.",
    )
    parser.add_argument(
        "--tf-hz",
        type=float,
        default=50.0,
        help="TF cache spin frequency.",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=0.1,
        help="Print interval in seconds.",
    )
    parser.add_argument(
        "--max-age",
        type=float,
        default=0.25,
        help="Maximum accepted TF cache age in seconds.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    if args.interval <= 0.0:
        raise ValueError(f"interval must be > 0, got {args.interval}")
    if args.tf_hz <= 0.0:
        raise ValueError(f"tf_hz must be > 0, got {args.tf_hz}")
    if args.max_age <= 0.0:
        raise ValueError(f"max_age must be > 0, got {args.max_age}")

    if not rclpy.ok():
        rclpy.init()

    tf_node = position_odin.TfCacheNode(
        base_frame=args.base_frame,
        update_hz=args.tf_hz,
    )
    stop_event = threading.Event()
    tf_thread = threading.Thread(
        target=position_odin.spin_tf_cache,
        args=(tf_node, stop_event),
        daemon=True,
        name="measure_weapon_odin_tf_cache",
    )
    tf_thread.start()

    try:
        while rclpy.ok():
            weapon_pose = position_odin.get_weapon_pose_in_map_synced(
                tf_cache_node=tf_node,
                max_age_sec=args.max_age,
                clock=tf_node.get_clock(),
            )
            if weapon_pose is None:
                print("waiting for valid odin TF...")
                time.sleep(args.interval)
                continue

            yaw_deg = normalize_yaw_deg(
                position_odin.radians_to_degrees(weapon_pose["yaw"])
            )
            print(
                "weapon "
                f"x={weapon_pose['x']:.4f} m, "
                f"y={weapon_pose['y']:.4f} m, "
                f"z={weapon_pose['z']:.4f} m, "
                f"yaw={yaw_deg:.2f} deg, "
                f"tf_age={weapon_pose.get('age_sec', 0.0):.3f} s"
            )
            time.sleep(args.interval)
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        tf_thread.join(timeout=1.0)
        tf_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
