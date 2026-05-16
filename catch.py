#!/usr/bin/env python3

import time
import math
import threading

from lib2 import module, tools


LIDAR_TYPE = 1
ODOM_TOPIC = "/odin1/odometry_highfreq"

PRE_GRAB_WAYPOINTS = [
    (-0.8, -4.8),
    (-1.0, 0.0),
    (-2.4, -2.4),
]
PRE_GRAB_STAIR_ID = -1
START_STAIR_ID = -1
TARGET_STAIR_ID = 2
MOVE_SPEED = 600
FINAL_DIRECTION = 1
ADJUST_DISTANCE = 0.25
GRAB_POSE_HOLD_SEC = 4.0
KFS_SUCTION_HOLD_SEC = 3.0
TRANSITION_POSE_HOLD_SEC = 3.0
STORE_POSE_HOLD_SEC = 2.0
HOLD_AFTER_DONE_SEC = 5.0

WAIT_READY_TIMEOUT_SEC = 8.0
WAIT_READY_STABLE_FRAMES = 5
WAIT_READY_POLL_SEC = 0.02
MAX_READY_DATA_AGE_SEC = 0.25
CHANNEL_DEBUG_INTERVAL_SEC = 0.2


def values_are_finite(*values):
    return all(math.isfinite(float(value)) for value in values)


def channel_debug_print(sender, stop_event, interval_sec=CHANNEL_DEBUG_INTERVAL_SEC):
    started_at = time.time()
    while not stop_event.is_set():
        state = sender.get_state()
        channels = state["channels"]
        last_send_time = state.get("last_send_time")
        send_age_sec = None if last_send_time is None else time.time() - float(last_send_time)
        print(
            "[channels] "
            f"t={time.time() - started_at:7.2f}s | "
            f"seq={state['seq']:5d} | "
            f"send_ok={state['last_send_ok']} | "
            f"send_age={send_age_sec if send_age_sec is not None else 'None'} | "
            f"yaw_i16={state['yaw_i16']:6d} | "
            f"des_yaw_i16={state['des_yaw_i16']:6d} | "
            f"ch={channels}"
        )
        time.sleep(float(interval_sec))


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
    debug_thread = None
    debug_stop_event = None

    try:
        print("Starting KFS catch test...")
        sender, _, flag_node, flag_thread, flag_stop_event = module.init(lidar_type=LIDAR_TYPE)
        debug_stop_event = threading.Event()
        debug_thread = threading.Thread(
            target=channel_debug_print,
            args=(sender, debug_stop_event),
            daemon=True,
            name="channel_debug_thread",
        )
        debug_thread.start()
        position_runtime = module.start_position_thread(sender)
        odom_runtime = module.start_odometry_thread(topic=ODOM_TOPIC)

        print("Waiting for robot pose and odometry...")
        if not wait_runtime_ready(position_runtime, odom_runtime):
            print("Catch task failed: runtime data not ready.")
            return

        pre_grab_yaw_deg = tools.direction_int_to_yaw_deg(FINAL_DIRECTION)
        waypoint_results = []
        for index, (target_x, target_y) in enumerate(PRE_GRAB_WAYPOINTS, start=1):
            print(
                f"Pre-grab waypoint {index}/{len(PRE_GRAB_WAYPOINTS)}: "
                f"move to ({target_x:.3f}, {target_y:.3f}) "
                f"with fixed_yaw={pre_grab_yaw_deg:.2f} deg..."
            )
            waypoint_result = module.move_to_des(
                sender=sender,
                position_runtime=position_runtime,
                odom_runtime=odom_runtime,
                x=target_x,
                y=target_y,
                target_deg=pre_grab_yaw_deg,
                v=MOVE_SPEED,
            )
            if waypoint_result is None:
                print(f"Catch task failed: waypoint {index} move failed.")
                return
            print(f"Pre-grab waypoint {index} finished.")
            print(waypoint_result)
            waypoint_results.append(waypoint_result)

        pre_grab_x, pre_grab_y = module.get_stair_xy(PRE_GRAB_STAIR_ID)
        print(
            f"Moving to pre-grab stair {PRE_GRAB_STAIR_ID} "
            f"({pre_grab_x:.3f}, {pre_grab_y:.3f}) "
            f"with fixed_yaw={pre_grab_yaw_deg:.2f} deg..."
        )
        pre_grab_result = module.move_to_des(
            sender=sender,
            position_runtime=position_runtime,
            odom_runtime=odom_runtime,
            x=pre_grab_x,
            y=pre_grab_y,
            target_deg=pre_grab_yaw_deg,
            v=MOVE_SPEED,
        )
        if pre_grab_result is None:
            print("Catch task failed: pre-grab move failed.")
            return
        print("Pre-grab move finished.")
        print(pre_grab_result)

        move_direction = tools.stair_id_to_direction(START_STAIR_ID, TARGET_STAIR_ID)
        print(
            f"Execute {START_STAIR_ID}->{TARGET_STAIR_ID} grab after pre-grab move."
        )

        action_row = [
            START_STAIR_ID,
            TARGET_STAIR_ID,
            move_direction,
            0,
            1,
        ]
        height_relation = module.get_stair_height_relation(START_STAIR_ID, move_direction)
        grab_pose_id = module.kfs_pose_id_from_height_relation(height_relation)
        final_target_yaw_deg = tools.direction_int_to_yaw_deg(FINAL_DIRECTION)
        print(
            "Executing KFS grab: "
            f"action_row={action_row}, "
            f"height_relation={height_relation}, "
            f"pose_id={grab_pose_id}, "
            f"ch5=2, ch6={grab_pose_id}, ch7 1->3, "
            f"grab_pose_hold_sec={GRAB_POSE_HOLD_SEC:.1f}"
        )
        fetch_result = module.fetch_and_store_kfs(
            sender=sender,
            position_runtime=position_runtime,
            odom_runtime=odom_runtime,
            stair_id=START_STAIR_ID,
            direction=move_direction,
            final_target_yaw_deg=final_target_yaw_deg,
            adjust_distance=ADJUST_DISTANCE,
            move_speed=MOVE_SPEED,
            suction_hold_sec=KFS_SUCTION_HOLD_SEC,
            grab_pose_hold_sec=GRAB_POSE_HOLD_SEC,
            transition_pose_hold_sec=TRANSITION_POSE_HOLD_SEC,
            store_pose_hold_sec=STORE_POSE_HOLD_SEC,
        )
        action_result = {
            "action_row": action_row,
            "height_relation": height_relation,
            "grab_pose_id": grab_pose_id,
            "fetch_result": fetch_result,
        }
        if not fetch_result.get("completed"):
            print("Catch task failed: KFS fetch did not complete.")
            print(action_result)
            return

        print("Catch task finished.")
        print({
            "waypoint_results": waypoint_results,
            "pre_grab_result": pre_grab_result,
            "move_skipped": False,
            "action_row": action_row,
            "action_result": action_result,
            "fetch_result": fetch_result,
        })
        print(f"Holding current output for {HOLD_AFTER_DONE_SEC:.1f}s before shutdown...")
        time.sleep(HOLD_AFTER_DONE_SEC)

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
