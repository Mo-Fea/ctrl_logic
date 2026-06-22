#!/usr/bin/env python3

import math
import time

from lib2 import module, tools


LIDAR_TYPE = 1
ODOM_TOPIC = "/odin1/odometry_highfreq"

MOVE_SPEED = 600
STAIR_MOVE_SPEED = 600

START_STAIR_ID = -1
FIRST_CLIMB_FROM_STAIR_ID = -1
FIRST_CLIMB_TO_STAIR_ID = 2
START_FIXED_YAW_DEG = 0.01
FINAL_DIRECTION = 1

START_DELAY_SEC = 120.0
HOLD_AFTER_DONE_SEC = 30.0

# 后续需要更换默认动作矩阵时，直接改这个全局变量。
ACTION_MATRIX = [
    [2, 5, 1, 0, 1],
    [2, 5, 1, 1, 0],
    [5, 8, 1, 1, 0],
    [8, 11, 1, 1, 0],
    [11, 12, 3, 1, 0],
    [12, 15, 1, 1, 0],
]

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


def cleanup_runtime(
    sender,
    flag_node,
    flag_thread,
    flag_stop_event,
    position_runtime,
    odom_runtime,
):
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


def main():
    sender = None
    flag_node = None
    flag_thread = None
    flag_stop_event = None
    position_runtime = None
    odom_runtime = None

    try:
        print("Starting complete red default action-matrix test...")
        sender, _, flag_node, flag_thread, flag_stop_event = module.init(
            lidar_type=LIDAR_TYPE
        )
        position_runtime = module.start_position_thread(sender)
        odom_runtime = module.start_odometry_thread(topic=ODOM_TOPIC)

        print("Waiting for robot pose and odometry...")
        if not wait_runtime_ready(position_runtime, odom_runtime):
            print("Complete test failed: runtime data not ready.")
            return

        print(f"Waiting {START_DELAY_SEC:.1f}s before starting movement...")
        time.sleep(START_DELAY_SEC)

        start_x, start_y = module.get_stair_xy(START_STAIR_ID)
        print(
            f"Moving to stair {START_STAIR_ID} "
            f"({start_x:.3f}, {start_y:.3f}) "
            f"with target_yaw={START_FIXED_YAW_DEG:.2f} deg..."
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
            print("Complete test failed: move to -1 failed.")
            return
        print("Move to -1 finished.")
        print(start_move_result)

        climb_direction = tools.stair_id_to_direction(
            FIRST_CLIMB_FROM_STAIR_ID,
            FIRST_CLIMB_TO_STAIR_ID,
        )
        climb_target_x, climb_target_y = module.get_stair_xy(FIRST_CLIMB_TO_STAIR_ID)
        print(
            f"Climbing {FIRST_CLIMB_FROM_STAIR_ID}->{FIRST_CLIMB_TO_STAIR_ID}: "
            f"direction={climb_direction}, "
            f"target=({climb_target_x:.3f}, {climb_target_y:.3f})"
        )
        climb_result = module.climb(
            sender=sender,
            position_runtime=position_runtime,
            odom_runtime=odom_runtime,
            direction1=climb_direction,
            direction2=FINAL_DIRECTION,
            x=climb_target_x,
            y=climb_target_y,
            move_speed=STAIR_MOVE_SPEED,
        )
        print("Initial climb finished.")
        print(climb_result)

        print("[Default action matrix]")
        print(ACTION_MATRIX)
        print("Executing default action matrix...")
        matrix_result = module.execute_action_matrix(
            sender=sender,
            position_runtime=position_runtime,
            odom_runtime=odom_runtime,
            action_matrix=ACTION_MATRIX,
            final_direction=FINAL_DIRECTION,
        )
        print("Default action matrix finished.")
        print(matrix_result)
        if not matrix_result.get("completed", False):
            print("Complete red test failed: action matrix did not complete.")
            return

        print(f"Waiting {HOLD_AFTER_DONE_SEC:.1f}s after action matrix...")
        time.sleep(HOLD_AFTER_DONE_SEC)
        print("Complete red default action-matrix test completed.")

    except KeyboardInterrupt:
        print("\nStopped by user.")

    finally:
        cleanup_runtime(
            sender=sender,
            flag_node=flag_node,
            flag_thread=flag_thread,
            flag_stop_event=flag_stop_event,
            position_runtime=position_runtime,
            odom_runtime=odom_runtime,
        )


if __name__ == "__main__":
    main()
