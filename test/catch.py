#!/usr/bin/env python3

import time
import math

from lib2 import module, move, tools


LIDAR_TYPE = 2
ODOM_TOPIC = "/lio/robo/odom"

WEAPON_ID = 1
MOVE_SPEED = 600
FINAL_YAW_DEG = -90.0
HOLD_AFTER_DONE_SEC = 10.0

WAIT_READY_TIMEOUT_SEC = 8.0
WAIT_READY_STABLE_FRAMES = 5
WAIT_READY_POLL_SEC = 0.02
MAX_READY_DATA_AGE_SEC = 0.25


def values_are_finite(*values):
    return all(math.isfinite(float(value)) for value in values)


def wait_runtime_ready(position_runtime, odom_runtime):
    deadline = time.time() + WAIT_READY_TIMEOUT_SEC
    stable_frames = 0

    while time.time() < deadline:
        robot_pose = position_runtime.get_robot_pose()
        weapon_pose = position_runtime.get_weapon_pose(max_tf_age_sec=MAX_READY_DATA_AGE_SEC)
        update_time = position_runtime.get_latest_update_time()
        odometry = odom_runtime.get_odometry(max_age_sec=MAX_READY_DATA_AGE_SEC)

        data_finite = (
            robot_pose is not None
            and weapon_pose is not None
            and odometry is not None
            and values_are_finite(
                robot_pose["x"],
                robot_pose["y"],
                robot_pose["z"],
                robot_pose["yaw"],
                weapon_pose["x"],
                weapon_pose["y"],
                weapon_pose["z"],
                weapon_pose["yaw"],
                odometry["linear_x"],
                odometry["linear_y"],
                odometry["angular_z"],
            )
        )
        pose_fresh = (
            robot_pose is not None
            and weapon_pose is not None
            and update_time is not None
            and (time.time() - float(update_time)) <= MAX_READY_DATA_AGE_SEC
        )
        if pose_fresh and data_finite:
            stable_frames += 1
            if stable_frames >= WAIT_READY_STABLE_FRAMES:
                return True
        else:
            stable_frames = 0

        time.sleep(WAIT_READY_POLL_SEC)

    return False


def main():
    sender = None
    flag_node = None
    flag_thread = None
    flag_stop_event = None
    position_runtime = None
    odom_runtime = None

    try:
        print("Starting weapon catch task...")
        sender, _, flag_node, flag_thread, flag_stop_event = module.init(lidar_type=LIDAR_TYPE)
        position_runtime = module.start_position_thread(sender)
        odom_runtime = module.start_odometry_thread(topic=ODOM_TOPIC)

        print("Waiting for robot pose, weapon pose and odometry...")
        if not wait_runtime_ready(position_runtime, odom_runtime):
            print("Catch task failed: runtime data not ready.")
            return

        print(f"Fetching weapon id={WEAPON_ID}...")
        fetch_result = module.fetch_weapon(
            sender=sender,
            position_runtime=position_runtime,
            odom_runtime=odom_runtime,
            weapon_id=WEAPON_ID,
            v=MOVE_SPEED,
        )
        if fetch_result["move_result"] is None:
            print("Catch task failed: fetch_weapon move_result is None.")
            return

        print(f"Rotating to final yaw {FINAL_YAW_DEG:.2f} deg...")
        rotate_result = move.rotate_to_target_yaw_segmented(
            sender=sender,
            position_runtime=position_runtime,
            target_yaw_deg=FINAL_YAW_DEG,
        )

        print("Catch task finished.")
        print({
            "fetch_result": fetch_result,
            "rotate_result": rotate_result,
        })
        print(f"Holding current output for {HOLD_AFTER_DONE_SEC:.1f}s before shutdown...")
        time.sleep(HOLD_AFTER_DONE_SEC)

    except KeyboardInterrupt:
        print("\nStopped by user.")

    finally:
        tf_node = None
        tf_thread = None
        tf_stop_event = None
        if position_runtime is not None:
            position_threads = position_runtime.get_threads()
            tf_node = position_threads["tf_node"]
            tf_thread = position_threads["tf_thread"]
            tf_stop_event = position_threads["tf_stop_event"]
            position_threads["position_stop_event"].set()
            if (
                position_threads["position_thread"] is not None
                and position_threads["position_thread"].is_alive()
            ):
                position_threads["position_thread"].join(timeout=1.0)

        odom_node = None
        odom_thread = None
        odom_stop_event = None
        if odom_runtime is not None:
            odom_threads = odom_runtime.get_threads()
            odom_node = odom_threads["odom_node"]
            odom_thread = odom_threads["odom_thread"]
            odom_stop_event = odom_threads["odom_stop_event"]

        tools.handle_ctrl_c(
            sender=sender,
            flag_node=flag_node,
            flag_thread=flag_thread,
            flag_stop_event=flag_stop_event,
            tf_node=tf_node,
            tf_thread=tf_thread,
            tf_stop_event=tf_stop_event,
            extra_node=odom_node,
            extra_thread=odom_thread,
            extra_stop_event=odom_stop_event,
            shutdown_rclpy=True,
        )


if __name__ == "__main__":
    main()
