#!/usr/bin/env python3

import math
import threading
import time

from lib2 import module, tools


LIDAR_TYPE = 2
MID360_ODOM_TOPIC = "/lio/robo/odom"

START_STAIR_ID = -1
KFS_TARGET_STAIR_ID = 2
TARGET_SPEED = 600
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
    debug_thread,
    debug_stop_event,
):
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
        print("Starting move-to--1 + KFS fetch task...")
        sender, _, flag_node, flag_thread, flag_stop_event = module.init(lidar_type=LIDAR_TYPE)
        position_runtime = module.start_position_thread(sender)
        odom_runtime = module.start_odometry_thread(topic=MID360_ODOM_TOPIC)

        print("Waiting for mid360 pose and odometry...")
        if not wait_runtime_ready(position_runtime, odom_runtime):
            print("Task failed: mid360 pose/odometry not ready.")
            return

        start_x, start_y = module.get_stair_xy(START_STAIR_ID)
        fetch_direction = tools.stair_id_to_direction(
            START_STAIR_ID,
            KFS_TARGET_STAIR_ID,
            stair_matrix=module.get_stair_matrix(),
        )
        if fetch_direction == 0:
            print(
                f"Task failed: stair {KFS_TARGET_STAIR_ID} is not adjacent to "
                f"stair {START_STAIR_ID}."
            )
            return

        debug_stop_event = threading.Event()
        debug_thread = threading.Thread(
            target=tools.debug_print,
            args=(sender, position_runtime, start_x, start_y),
            kwargs={
                "stop_event": debug_stop_event,
                "interval_sec": 0.2,
            },
            daemon=True,
            name="debug_print_thread",
        )
        debug_thread.start()

        target_yaw = tools.direction_int_to_yaw_deg(fetch_direction)
        print(
            f"Move to stair {START_STAIR_ID} ({start_x:.2f}, {start_y:.2f}) | "
            f"face stair {KFS_TARGET_STAIR_ID} | direction={fetch_direction} | "
            f"speed={TARGET_SPEED} | final_yaw={target_yaw:.2f} deg"
        )
        move_result = module.move_to_des(
            sender=sender,
            position_runtime=position_runtime,
            odom_runtime=odom_runtime,
            x=start_x,
            y=start_y,
            target_deg=target_yaw,
            v=TARGET_SPEED,
        )
        if move_result is None:
            print(f"Task failed: move to stair {START_STAIR_ID} returned no valid result.")
            return
        print(move_result)

        print(
            f"Fetch and store KFS | from stair {START_STAIR_ID} "
            f"toward stair {KFS_TARGET_STAIR_ID} | direction={fetch_direction}"
        )
        fetch_result = module.fetch_and_store_kfs(
            sender=sender,
            position_runtime=position_runtime,
            odom_runtime=odom_runtime,
            stair_id=START_STAIR_ID,
            direction=fetch_direction,
            move_speed=TARGET_SPEED,
        )
        if not fetch_result.get("completed", False):
            print("Task failed: fetch_and_store_kfs did not complete.")
            print(fetch_result)
            return

        print("Move-to--1 + KFS fetch task finished.")
        print(
            {
                "move_result": move_result,
                "fetch_result": fetch_result,
            }
        )
        print(f"Holding current output for {HOLD_AFTER_DONE_SEC:.1f}s before shutdown...")
        time.sleep(HOLD_AFTER_DONE_SEC)

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
            debug_thread=debug_thread,
            debug_stop_event=debug_stop_event,
        )


if __name__ == "__main__":
    main()
