import argparse
import threading
import time

import rclpy

from lib2 import position_backend
from lib2 import position_odin
from lib2 import tools


def normalize_yaw_deg(yaw_deg):
    while yaw_deg >= 180.0:
        yaw_deg -= 360.0
    while yaw_deg < -180.0:
        yaw_deg += 360.0
    return yaw_deg


def parse_args():
    parser = argparse.ArgumentParser(
        description="Continuously print robot pose calculated from odin TF."
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
    parser.add_argument(
        "--field",
        choices=("red", "blue", "1", "2"),
        default=None,
        help="Field half used for printed coordinates: red/1 or blue/2.",
    )
    return parser.parse_args()


def configure_field(field):
    if field in ("red", "1"):
        position_backend.set_field_type(position_backend.FIELD_TYPE_RED)
        return "red"
    position_backend.set_field_type(position_backend.FIELD_TYPE_BLUE)
    return "blue"


def prompt_field():
    while True:
        field = input("请选择当前半场（1=red/right，2=blue/left）：").strip().lower()
        if field in ("red", "r", "1"):
            return "red"
        if field in ("blue", "b", "2"):
            return "blue"
        print("输入无效，请输入 1/red 或 2/blue")


def main():
    args = parse_args()
    if args.interval <= 0.0:
        raise ValueError(f"interval must be > 0, got {args.interval}")
    if args.tf_hz <= 0.0:
        raise ValueError(f"tf_hz must be > 0, got {args.tf_hz}")
    if args.max_age <= 0.0:
        raise ValueError(f"max_age must be > 0, got {args.max_age}")

    field_name = configure_field(args.field if args.field is not None else prompt_field())

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
        name="measure_odin_tf_cache",
    )
    tf_thread.start()

    try:
        while rclpy.ok():
            lidar_pose = position_odin.get_lidar_pose_in_map_synced(
                tf_cache_node=tf_node,
                max_age_sec=args.max_age,
                clock=tf_node.get_clock(),
            )
            if lidar_pose is None:
                time.sleep(args.interval)
                continue

            robot_pose = position_odin.cal_robot_position(
                lidar_pose["T_map_lidar"]
            )
            yaw_deg = normalize_yaw_deg(
                position_odin.radians_to_degrees(robot_pose["yaw"])
            )
            print(
                f"x={robot_pose['x']:.4f} m, "
                f"y={robot_pose['y']:.4f} m, "
                f"z={robot_pose['z']:.4f} m, "
                f"yaw={yaw_deg:.2f} deg"
            )
            time.sleep(args.interval)
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
