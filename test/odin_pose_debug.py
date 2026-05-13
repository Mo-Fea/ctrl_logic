#!/usr/bin/env python3

import threading
import time

import rclpy

from lib2 import module
from lib2 import position_odin


PRINT_INTERVAL_SEC = 0.5
MAX_TF_AGE_SEC = 0.25
STAIR_IDS_TO_PRINT = (-1, 1, 2, 3)


def fmt_pose(name, pose):
    if pose is None:
        return f"{name}: None"
    yaw_deg = position_odin.radians_to_degrees(float(pose["yaw"]))
    age = pose.get("age_sec")
    age_text = "None" if age is None else f"{float(age):.3f}s"
    return (
        f"{name}: x={float(pose['x']): .3f}, y={float(pose['y']): .3f}, "
        f"yaw={yaw_deg: .2f}deg, age={age_text}"
    )


def main():
    if not rclpy.ok():
        rclpy.init()

    module.configure_position_backend(1)
    tf_stop_event = threading.Event()
    tf_node = position_odin.TfCacheNode(update_hz=50.0)
    tf_thread = threading.Thread(
        target=position_odin.spin_tf_cache,
        args=(tf_node, tf_stop_event),
        daemon=True,
        name="odin_pose_debug_tf_thread",
    )
    tf_thread.start()

    print("Odin pose debug started. Press Ctrl+C to stop.")
    print(
        "External params: "
        f"LIDAR_TO_BASE=({position_odin.LIDAR_TO_BASE_X}, {position_odin.LIDAR_TO_BASE_Y}), "
        f"LIDAR_TO_WEAPON=({position_odin.LIDAR_TO_WEAPON_X}, {position_odin.LIDAR_TO_WEAPON_Y})"
    )
    for stair_id in STAIR_IDS_TO_PRINT:
        print(f"stair {stair_id}: {module.get_stair_xy(stair_id)}")

    try:
        while True:
            lidar_pose = position_odin.get_lidar_pose_in_map_synced(
                tf_cache_node=tf_node,
                max_age_sec=MAX_TF_AGE_SEC,
                clock=tf_node.get_clock(),
            )
            robot_pose = None
            weapon_pose = None
            if lidar_pose is not None:
                robot_pose = position_odin.cal_robot_position(lidar_pose["T_map_lidar"])
                weapon_pose = position_odin.cal_weapon_position(lidar_pose["T_map_lidar"])
                robot_pose["age_sec"] = lidar_pose.get("age_sec")
                weapon_pose["age_sec"] = lidar_pose.get("age_sec")

            print(fmt_pose("lidar/raw", lidar_pose))
            print(fmt_pose("robot", robot_pose))
            print(fmt_pose("weapon", weapon_pose))
            print("-" * 72)
            time.sleep(PRINT_INTERVAL_SEC)

    except KeyboardInterrupt:
        print("\nStopped by user.")

    finally:
        tf_stop_event.set()
        if tf_thread.is_alive():
            tf_thread.join(timeout=1.0)
        try:
            tf_node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
