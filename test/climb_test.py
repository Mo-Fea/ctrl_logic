import threading
import time
from lib2 import module, move, tools


MID360_ODOM_TOPIC = "/lio/robo/odom"

INITIAL_X = 1.89
INITIAL_Y = 1.0
INITIAL_FINAL_DIRECTION = 1

FIRST_CLIMB_DIRECTION_1 = 1
FIRST_CLIMB_DIRECTION_2 = 1
FIRST_CLIMB_TARGET_X = 2.9
FIRST_CLIMB_TARGET_Y = 1.0
FIRST_WAIT_SEC = 5.0

SECOND_CLIMB_DIRECTION_1 = 2
SECOND_CLIMB_DIRECTION_2 = 1
SECOND_CLIMB_TARGET_X = 2.9
SECOND_CLIMB_TARGET_Y = 2.2
SECOND_WAIT_SEC = 10.0

TARGET_SPEED = 600
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
        print("Starting mid360 climb test...")
        sender, _, flag_node, flag_thread, flag_stop_event = module.init(lidar_type=2)
        position_runtime = module.start_position_thread(sender)
        odom_runtime = module.start_odometry_thread(topic=MID360_ODOM_TOPIC)

        print("Waiting for mid360 pose and odometry...")
        if not wait_runtime_ready(position_runtime, odom_runtime):
            print("Climb test failed: mid360 pose/odometry not ready.")
            return

        debug_stop_event = threading.Event()
        debug_thread = threading.Thread(
            target=tools.debug_print,
            args=(sender, position_runtime, INITIAL_X, INITIAL_Y),
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
            f"Move to initial target ({INITIAL_X:.2f}, {INITIAL_Y:.2f}) | "
            f"speed={TARGET_SPEED} | final_yaw={initial_final_yaw:.2f} deg"
        )
        initial_result = module.move_to_des(
            sender=sender,
            position_runtime=position_runtime,
            odom_runtime=odom_runtime,
            x=INITIAL_X,
            y=INITIAL_Y,
            target_deg=initial_final_yaw,
            v=TARGET_SPEED,
        )
        if initial_result is None:
            print("Initial move failed: no valid position available.")
            return

        print("Initial move finished.")
        print(initial_result)

        print(
            f"First climb | directions=({FIRST_CLIMB_DIRECTION_1}, {FIRST_CLIMB_DIRECTION_2}) | "
            f"target=({FIRST_CLIMB_TARGET_X:.2f}, {FIRST_CLIMB_TARGET_Y:.2f})"
        )
        first_climb_result = module.climb(
            sender=sender,
            position_runtime=position_runtime,
            odom_runtime=odom_runtime,
            direction1=FIRST_CLIMB_DIRECTION_1,
            direction2=FIRST_CLIMB_DIRECTION_2,
            x=FIRST_CLIMB_TARGET_X,
            y=FIRST_CLIMB_TARGET_Y,
            move_speed=TARGET_SPEED,
        )
        print("First climb finished.")
        print(first_climb_result)

        first_wait_yaw = tools.direction_int_to_yaw_deg(FIRST_CLIMB_DIRECTION_2)
        print(f"Wait with target yaw {first_wait_yaw:.2f} deg for {FIRST_WAIT_SEC:.1f}s...")
        first_wait_result = move.wait_with_target_yaw(
            sender=sender,
            duration_sec=FIRST_WAIT_SEC,
            target_yaw_deg=first_wait_yaw,
        )
        print(first_wait_result)

        print(
            f"Second climb | directions=({SECOND_CLIMB_DIRECTION_1}, {SECOND_CLIMB_DIRECTION_2}) | "
            f"target=({SECOND_CLIMB_TARGET_X:.2f}, {SECOND_CLIMB_TARGET_Y:.2f})"
        )
        second_climb_result = module.climb(
            sender=sender,
            position_runtime=position_runtime,
            odom_runtime=odom_runtime,
            direction1=SECOND_CLIMB_DIRECTION_1,
            direction2=SECOND_CLIMB_DIRECTION_2,
            x=SECOND_CLIMB_TARGET_X,
            y=SECOND_CLIMB_TARGET_Y,
            move_speed=TARGET_SPEED,
        )
        print("Second climb finished.")
        print(second_climb_result)

        second_wait_yaw = tools.direction_int_to_yaw_deg(SECOND_CLIMB_DIRECTION_2)
        print(f"Wait with target yaw {second_wait_yaw:.2f} deg for {SECOND_WAIT_SEC:.1f}s...")
        second_wait_result = move.wait_with_target_yaw(
            sender=sender,
            duration_sec=SECOND_WAIT_SEC,
            target_yaw_deg=second_wait_yaw,
        )
        print(second_wait_result)
        print("Climb test task completed.")

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
