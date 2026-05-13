#!/usr/bin/env python3

import threading
import time

from lib2 import module, move, tools


MID360_ODOM_TOPIC = "/lio/robo/odom"
TARGET_X = 2.0
TARGET_Y = -2.0
TARGET_FINAL_YAW_DEG = None
TARGET_SPEED = 600
HOLD_AFTER_MOVE_SEC = 10.0
WAIT_READY_TIMEOUT_SEC = 8.0
WAIT_READY_STABLE_FRAMES = 5
WAIT_READY_POLL_SEC = 0.02
MAX_READY_DATA_AGE_SEC = 0.25


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
        print("Starting level_2 test...")
        sender, _, flag_node, flag_thread, flag_stop_event = module.init(lidar_type=2)
        position_runtime = module.start_position_thread(sender)
        odom_runtime = module.start_odometry_thread(topic=MID360_ODOM_TOPIC)

        print("Waiting for mid360 pose and odometry...")
        if not wait_runtime_ready(position_runtime, odom_runtime):
            print("Move test failed: mid360 pose/odometry not ready.")
            return

        debug_stop_event = threading.Event()
        debug_thread = threading.Thread(
            target=tools.debug_print,
            args=(sender, position_runtime, TARGET_X, TARGET_Y),
            kwargs={
                "stop_event": debug_stop_event,
                "interval_sec": 0.2,
            },
            daemon=True,
            name="debug_print_thread",
        )
        debug_thread.start()

        print(
            f"Moving to target ({TARGET_X:.2f}, {TARGET_Y:.2f}) | "
            f"speed={TARGET_SPEED} | final_yaw={TARGET_FINAL_YAW_DEG}"
        )
        result = module.move_to_des(
            sender=sender,
            position_runtime=position_runtime,
            odom_runtime=odom_runtime,
            x=TARGET_X,
            y=TARGET_Y,
            target_deg=TARGET_FINAL_YAW_DEG,
            v=TARGET_SPEED,
        )

        if result is None:
            print("Move test failed: no valid position available.")
            return

        print("Move test finished.")
        print(result)
        print(f"Holding current output for {HOLD_AFTER_MOVE_SEC:.1f}s before shutdown...")
        time.sleep(HOLD_AFTER_MOVE_SEC)

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
            if position_threads["position_thread"] is not None and position_threads["position_thread"].is_alive():
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
