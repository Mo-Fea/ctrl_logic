#!/usr/bin/env python3

import math
import time

from lib2 import module, move, tools


LIDAR_TYPE = 1
ODOM_TOPIC = "/odin1/odometry_highfreq"

WEAPON_ID = 1
START_STAIR_ID = -1
START_FIXED_YAW_DEG = 0.01
MOVE_SPEED = 600
FINAL_DIRECTION = 1
HOLD_AFTER_DONE_SEC = 5.0

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
        update_time = position_runtime.get_latest_update_time()
        odometry = odom_runtime.get_odometry(max_age_sec=MAX_READY_DATA_AGE_SEC)

        data_finite = (
            robot_pose is not None
            and odometry is not None
            and values_are_finite(
                robot_pose["x"],
                robot_pose["y"],
                robot_pose["z"],
                robot_pose["yaw"],
                odometry["linear_x"],
                odometry["linear_y"],
                odometry["angular_z"],
            )
        )
        pose_fresh = (
            robot_pose is not None
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
        print("Starting region 2 full challenge test...")
        sender, _, flag_node, flag_thread, flag_stop_event = module.init(
            lidar_type=LIDAR_TYPE
        )
        position_runtime = module.start_position_thread(sender)
        odom_runtime = module.start_odometry_thread(topic=ODOM_TOPIC)

        print("Waiting for robot pose and odometry...")
        if not wait_runtime_ready(position_runtime, odom_runtime):
            print("Region 2 test failed: runtime data not ready.")
            return

        print(f"Fetching weapon {WEAPON_ID}...")
        weapon_result = module.fetch_weapon(
            sender=sender,
            position_runtime=position_runtime,
            odom_runtime=odom_runtime,
            weapon_id=WEAPON_ID,
            v=MOVE_SPEED,
        )
        print("fetch_weapon finished.")
        print(weapon_result)

        print("Resetting weapon state...")
        reset_weapon_result = move.reset_weapon_after_fetch(sender)
        print("weapon reset finished.")
        print(reset_weapon_result)

        start_x, start_y = module.get_stair_xy(START_STAIR_ID)
        print(
            f"Moving to stair {START_STAIR_ID} "
            f"({start_x:.3f}, {start_y:.3f}) "
            f"with fixed_yaw={START_FIXED_YAW_DEG:.2f} deg..."
        )
        start_move_result = module.move_to_des(
            sender=sender,
            position_runtime=position_runtime,
            odom_runtime=odom_runtime,
            x=start_x,
            y=start_y,
            target_deg=START_FIXED_YAW_DEG,
            v=MOVE_SPEED,
        )
        if start_move_result is None:
            print("Region 2 test failed: move to start stair failed.")
            return
        print("Move to start stair finished.")
        print(start_move_result)

        print("Executing CHALLENGE_ACTION_MATRIX...")
        matrix_result = module.execute_action_matrix(
            sender=sender,
            position_runtime=position_runtime,
            odom_runtime=odom_runtime,
            action_matrix=module.CHALLENGE_ACTION_MATRIX,
            final_direction=FINAL_DIRECTION,
        )
        print("Challenge action matrix finished.")
        print(matrix_result)

        print(f"Holding current output for {HOLD_AFTER_DONE_SEC:.1f}s before shutdown...")
        time.sleep(HOLD_AFTER_DONE_SEC)
        print("Region 2 full challenge test completed.")

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
