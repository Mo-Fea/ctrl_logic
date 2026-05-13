#!/usr/bin/env python3

import threading
import time

from lib2 import module, move, tools


LIDAR_TYPE = 1
INITIAL_STAIR_ID = -1
TARGET_SPEED = 600

INITIAL_FINAL_DIRECTION = 1

FIRST_CLIMB_DIRECTION_1 = 1
FIRST_CLIMB_DIRECTION_2 = 2
FIRST_CLIMB_TARGET_ID = 2

SECOND_CLIMB_DIRECTION_1 = 2
SECOND_CLIMB_DIRECTION_2 = 2
SECOND_CLIMB_TARGET_ID = 1

DESCEND_DIRECTION_1 = 3
DESCEND_DIRECTION_2 = 1
DESCEND_TARGET_ID = 2

HOLD_AFTER_DONE_SEC = 10.0

WAIT_READY_TIMEOUT_SEC = 8.0
WAIT_READY_STABLE_FRAMES = 5
WAIT_READY_POLL_SEC = 0.02
MAX_READY_DATA_AGE_SEC = 0.25


def get_stair_xy(stair_id):
    stair_id = int(stair_id)
    for row in module.get_stair_matrix():
        if int(row[0]) == stair_id:
            return float(row[4]), float(row[5])
    raise ValueError(f"unknown stair_id={stair_id}")


def wait_runtime_ready(position_runtime, odom_runtime):
    deadline = time.time() + WAIT_READY_TIMEOUT_SEC
    stable_frames = 0

    while time.time() < deadline:
        robot_pose = position_runtime.get_robot_pose()
        update_time = position_runtime.get_latest_update_time()
        odometry = odom_runtime.get_odometry(max_age_sec=MAX_READY_DATA_AGE_SEC)

        pose_fresh = (
            robot_pose is not None
            and update_time is not None
            and (time.time() - float(update_time)) <= MAX_READY_DATA_AGE_SEC
        )
        if pose_fresh and odometry is not None:
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
    debug_thread = None
    debug_stop_event = None

    try:
        print("Starting odin up/down test...")
        sender, _, flag_node, flag_thread, flag_stop_event = module.init(lidar_type=LIDAR_TYPE)
        position_runtime = module.start_position_thread(sender)
        odom_runtime = module.start_odometry_thread()

        print("Waiting for odin pose and odometry...")
        if not wait_runtime_ready(position_runtime, odom_runtime):
            print("Up/down test failed: odin pose/odometry not ready.")
            return

        initial_x, initial_y = get_stair_xy(INITIAL_STAIR_ID)

        debug_stop_event = threading.Event()
        debug_thread = threading.Thread(
            target=tools.debug_print,
            args=(sender, position_runtime, initial_x, initial_y),
            kwargs={
                "stop_event": debug_stop_event,
                "interval_sec": 0.2,
            },
            daemon=True,
            name="debug_print_thread",
        )
        debug_thread.start()

        initial_final_yaw = tools.direction_int_to_yaw_deg(INITIAL_FINAL_DIRECTION)
        print(
            f"Initial move to stair {INITIAL_STAIR_ID} ({initial_x:.2f}, {initial_y:.2f}) | "
            f"final_yaw={initial_final_yaw:.2f} deg"
        )
        initial_result = module.move_to_des(
            sender=sender,
            position_runtime=position_runtime,
            odom_runtime=odom_runtime,
            x=initial_x,
            y=initial_y,
            target_deg=initial_final_yaw,
            v=TARGET_SPEED,
        )
        if initial_result is None:
            print("Up/down test failed: initial move failed.")
            return
        print("Initial move finished.")
        print(initial_result)

        first_target_x, first_target_y = get_stair_xy(FIRST_CLIMB_TARGET_ID)
        print(
            f"First climb | directions=({FIRST_CLIMB_DIRECTION_1}, {FIRST_CLIMB_DIRECTION_2}) | "
            f"target_id={FIRST_CLIMB_TARGET_ID} target=({first_target_x:.2f}, {first_target_y:.2f})"
        )
        first_climb_result = module.climb(
            sender=sender,
            position_runtime=position_runtime,
            odom_runtime=odom_runtime,
            direction1=FIRST_CLIMB_DIRECTION_1,
            direction2=FIRST_CLIMB_DIRECTION_2,
            x=first_target_x,
            y=first_target_y,
            move_speed=TARGET_SPEED,
        )
        print("First climb finished.")
        print(first_climb_result)

        second_target_x, second_target_y = get_stair_xy(SECOND_CLIMB_TARGET_ID)
        print(
            f"Second climb | directions=({SECOND_CLIMB_DIRECTION_1}, {SECOND_CLIMB_DIRECTION_2}) | "
            f"target_id={SECOND_CLIMB_TARGET_ID} target=({second_target_x:.2f}, {second_target_y:.2f})"
        )
        second_climb_result = module.climb(
            sender=sender,
            position_runtime=position_runtime,
            odom_runtime=odom_runtime,
            direction1=SECOND_CLIMB_DIRECTION_1,
            direction2=SECOND_CLIMB_DIRECTION_2,
            x=second_target_x,
            y=second_target_y,
            move_speed=TARGET_SPEED,
        )
        print("Second climb finished.")
        print(second_climb_result)

        descend_target_x, descend_target_y = get_stair_xy(DESCEND_TARGET_ID)
        print(
            f"Descend | directions=({DESCEND_DIRECTION_1}, {DESCEND_DIRECTION_2}) | "
            f"current=({second_target_x:.2f}, {second_target_y:.2f}) | "
            f"target_id={DESCEND_TARGET_ID} target=({descend_target_x:.2f}, {descend_target_y:.2f})"
        )
        descend_result = module.descend(
            sender=sender,
            position_runtime=position_runtime,
            odom_runtime=odom_runtime,
            direction1=DESCEND_DIRECTION_1,
            direction2=DESCEND_DIRECTION_2,
            current_x=second_target_x,
            current_y=second_target_y,
            des_x=descend_target_x,
            des_y=descend_target_y,
            move_speed=TARGET_SPEED,
        )
        print("Descend finished.")
        print(descend_result)

        hold_yaw = tools.direction_int_to_yaw_deg(DESCEND_DIRECTION_2)
        print(f"Hold still for {HOLD_AFTER_DONE_SEC:.1f}s before shutdown...")
        hold_result = move.wait_with_target_yaw(
            sender=sender,
            duration_sec=HOLD_AFTER_DONE_SEC,
            target_yaw_deg=hold_yaw,
        )
        print(hold_result)
        print("Up/down test completed.")

    except KeyboardInterrupt:
        print("\nStopped by user.")

    finally:
        if debug_stop_event is not None:
            debug_stop_event.set()
        if debug_thread is not None and debug_thread.is_alive():
            debug_thread.join(timeout=1.0)

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
