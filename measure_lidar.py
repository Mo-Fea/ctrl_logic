import argparse
import math
import threading
import time

import rclpy

from lib2 import position_odin
from lib2 import tools


def normalize_yaw_deg(yaw_deg):
    while yaw_deg >= 180.0:
        yaw_deg -= 360.0
    while yaw_deg < -180.0:
        yaw_deg += 360.0
    return yaw_deg


def get_lidar_pose_in_map_raw(lidar_pose):
    """
    Convert position_odin.get_lidar_pose_in_map_synced() output to raw map pose.

    This reports the lidar frame itself in map coordinates:
    - no lidar-to-robot offset
    - no robot yaw +180deg correction
    - no blue-field logical mirroring
    """
    if lidar_pose is None:
        return None

    t_map_lidar = lidar_pose["T_map_lidar"]
    yaw_rad = math.atan2(t_map_lidar[1, 0], t_map_lidar[0, 0])
    yaw_deg = normalize_yaw_deg(math.degrees(yaw_rad))

    return {
        "x": float(t_map_lidar[0, 3]),
        "y": float(t_map_lidar[1, 3]),
        "z": float(t_map_lidar[2, 3]),
        "yaw": float(yaw_rad),
        "yaw_deg": float(yaw_deg),
        "stamp_sec": lidar_pose.get("stamp_sec"),
        "age_sec": lidar_pose.get("age_sec"),
    }


def print_lidar_pose_in_map_continuously(
    tf_node,
    interval=0.1,
    max_age_sec=0.25,
):
    """
    Continuously print lidar pose in the raw map coordinate frame.
    """
    interval = float(interval)
    max_age_sec = float(max_age_sec)
    if interval <= 0.0:
        raise ValueError(f"interval must be > 0, got {interval}")
    if max_age_sec <= 0.0:
        raise ValueError(f"max_age_sec must be > 0, got {max_age_sec}")

    while rclpy.ok():
        synced_pose = position_odin.get_lidar_pose_in_map_synced(
            tf_cache_node=tf_node,
            max_age_sec=max_age_sec,
            clock=tf_node.get_clock(),
        )
        lidar_pose = get_lidar_pose_in_map_raw(synced_pose)
        if lidar_pose is None:
            time.sleep(interval)
            continue

        print(
            f"x={lidar_pose['x']:.4f} m, "
            f"y={lidar_pose['y']:.4f} m, "
            f"z={lidar_pose['z']:.4f} m, "
            f"yaw={lidar_pose['yaw_deg']:.2f} deg"
        )
        time.sleep(interval)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Continuously print lidar pose in raw map coordinates."
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
    if args.tf_hz <= 0.0:
        raise ValueError(f"tf_hz must be > 0, got {args.tf_hz}")

    if not rclpy.ok():
        rclpy.init()

    tools.relocalization_flag = False
    tools.odometry_mode_flag = False
    tools.localization_mode_received = False
    _, flag_node, flag_thread, flag_stop_event = tools.relocalization_conformation()
    while rclpy.ok() and not tools.localization_mode_received:
        time.sleep(0.01)
    if not rclpy.ok():
        tools.destroy_ros2_thread(flag_node, flag_thread, flag_stop_event)
        return
    tf_node = position_odin.TfCacheNode(
        base_frame=args.base_frame,
        update_hz=args.tf_hz,
    )
    stop_event = threading.Event()
    tf_thread = threading.Thread(
        target=position_odin.spin_tf_cache,
        args=(tf_node, stop_event),
        daemon=True,
        name="measure_lidar_odin_tf_cache",
    )
    tf_thread.start()

    try:
        print_lidar_pose_in_map_continuously(
            tf_node=tf_node,
            interval=args.interval,
            max_age_sec=args.max_age,
        )
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        tf_thread.join(timeout=1.0)
        tf_node.destroy_node()
        tools.destroy_ros2_thread(flag_node, flag_thread, flag_stop_event)
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
