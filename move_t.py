#!/usr/bin/env python3

import math
import threading
import time

from lib2 import module, tools


LIDAR_TYPE = 1
ODOM_TOPIC = "/odin1/odometry_highfreq"

TARGET_X = -0.8
TARGET_Y = -4.8
TARGET_YAW_DEG = 0.01
MOVE_SPEED = 600

WAIT_READY_TIMEOUT_SEC = 8.0
WAIT_READY_STABLE_FRAMES = 5
WAIT_READY_POLL_SEC = 0.02
MAX_READY_DATA_AGE_SEC = 0.25
DEBUG_PRINT_INTERVAL_SEC = 0.2


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


def debug_print(sender, position_runtime, odom_runtime, stop_event):
    started_at = time.time()
    while not stop_event.is_set():
        state = sender.get_state()
        channels = state["channels"]
        robot_pose = position_runtime.get_robot_pose()
        yaw_deg = position_runtime.get_current_yaw_deg()
        odometry = odom_runtime.get_odometry(max_age_sec=None)
        last_send_time = state.get("last_send_time")
        send_age_sec = None if last_send_time is None else time.time() - float(last_send_time)

        if robot_pose is None:
            pose_text = "pose=None"
            distance_text = "dist=None"
        else:
            current_x = float(robot_pose["x"])
            current_y = float(robot_pose["y"])
            dx = TARGET_X - current_x
            dy = TARGET_Y - current_y
            distance = math.hypot(dx, dy)
            pose_text = f"pos=({current_x:7.3f},{current_y:7.3f})"
            distance_text = f"dist={distance:6.3f} dx={dx:7.3f} dy={dy:7.3f}"

        if odometry is None:
            odom_text = "odom=None"
        else:
            speed = math.hypot(float(odometry["linear_x"]), float(odometry["linear_y"]))
            odom_text = f"speed={speed:5.3f} wz={float(odometry['angular_z']):6.3f}"

        print(
            "[move_t] "
            f"t={time.time() - started_at:7.2f}s | "
            f"seq={state['seq']:5d} send_ok={state['last_send_ok']} "
            f"send_age={send_age_sec if send_age_sec is not None else 'None'} | "
            f"yaw={yaw_deg if yaw_deg is not None else 'None'} "
            f"yaw_i16={state['yaw_i16']:6d} des={state['des_yaw_i16']:6d} | "
            f"ch={channels} | "
            f"{pose_text} | {distance_text} | {odom_text}"
        )
        time.sleep(DEBUG_PRINT_INTERVAL_SEC)


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
        print("Starting move-to-target test...")
        sender, _, flag_node, flag_thread, flag_stop_event = module.init(lidar_type=LIDAR_TYPE)
        position_runtime = module.start_position_thread(sender)
        odom_runtime = module.start_odometry_thread(topic=ODOM_TOPIC)

        print("Waiting for pose and odometry...")
        if not wait_runtime_ready(position_runtime, odom_runtime):
            print("Move-to-target test failed: pose/odometry not ready.")
            return

        debug_stop_event = threading.Event()
        debug_thread = threading.Thread(
            target=debug_print,
            args=(sender, position_runtime, odom_runtime, debug_stop_event),
            daemon=True,
            name="move_t_debug_thread",
        )
        debug_thread.start()

        print(
            f"Moving to ({TARGET_X:.3f}, {TARGET_Y:.3f}) "
            f"with fixed_yaw={TARGET_YAW_DEG:.2f} deg..."
        )
        result = module.move_to_des(
            sender=sender,
            position_runtime=position_runtime,
            odom_runtime=odom_runtime,
            x=TARGET_X,
            y=TARGET_Y,
            target_deg=TARGET_YAW_DEG,
            v=MOVE_SPEED,
        )
        print("Move-to-target test finished.")
        print(result)

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
