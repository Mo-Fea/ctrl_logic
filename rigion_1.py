#!/usr/bin/env python3

import threading
import time

from lib2 import module, move, tools


LIDAR_TYPE = 1
ODOM_TOPIC = "/odin1/odometry_highfreq"

WEAPON_ID = 1
TARGET_SPEED = 600
MOVE_TO_ENTRY_FINAL_DIRECTION = 1
INITIAL_STAIR_ID = -1

STAIR_TRANSITIONS = [
    (-1, 2),
    (2, 1),
    (1, 4),
]

HOLD_AFTER_DONE_SEC = 10.0

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


def execute_transition(sender, position_runtime, odom_runtime, from_id, to_id):
    move_dir = tools.stair_id_to_direction(from_id, to_id)
    height_relation = module.get_stair_height_relation(from_id, move_dir)
    from_x, from_y = module.get_stair_xy(from_id)
    to_x, to_y = module.get_stair_xy(to_id)
    final_direction = move_dir

    print(
        f"Stair transition {from_id}->{to_id} | "
        f"move_dir={move_dir} | height_relation={height_relation} | "
        f"final_direction={final_direction} | target=({to_x:.3f}, {to_y:.3f})"
    )
    result = module.execute_stair_transition(
        sender=sender,
        position_runtime=position_runtime,
        odom_runtime=odom_runtime,
        from_x=from_x,
        from_y=from_y,
        to_x=to_x,
        to_y=to_y,
        height_relation=height_relation,
        task_direction=move_dir,
        final_direction=final_direction,
    )
    print(f"Stair transition {from_id}->{to_id} finished.")
    print(result)
    return result


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
        print("Starting region 1 mission test...")
        sender, _, flag_node, flag_thread, flag_stop_event = module.init(lidar_type=LIDAR_TYPE)
        position_runtime = module.start_position_thread(sender)
        odom_runtime = module.start_odometry_thread(topic=ODOM_TOPIC)

        print("Waiting for odin pose and odometry...")
        if not wait_runtime_ready(position_runtime, odom_runtime):
            print("Region 1 mission failed: odin pose/odometry not ready.")
            return

        entry_x, entry_y = module.get_stair_xy(INITIAL_STAIR_ID)
        debug_stop_event = threading.Event()
        debug_thread = threading.Thread(
            target=tools.debug_print,
            args=(sender, position_runtime, entry_x, entry_y),
            kwargs={
                "stop_event": debug_stop_event,
                "interval_sec": 0.2,
            },
            daemon=True,
            name="debug_print_thread",
        )
        debug_thread.start()

        print(f"Fetching weapon id={WEAPON_ID}...")
        weapon_result = module.fetch_weapon(
            sender=sender,
            position_runtime=position_runtime,
            odom_runtime=odom_runtime,
            weapon_id=WEAPON_ID,
            v=TARGET_SPEED,
        )
        print("Weapon fetch finished.")
        print(weapon_result)

        entry_final_yaw = tools.direction_int_to_yaw_deg(MOVE_TO_ENTRY_FINAL_DIRECTION)
        print(
            f"Moving to stair {INITIAL_STAIR_ID} ({entry_x:.3f}, {entry_y:.3f}) | "
            f"fixed_yaw={entry_final_yaw:.2f} deg"
        )
        entry_move_result = move.move_to_target(
            sender=sender,
            position_runtime=position_runtime,
            odom_runtime=odom_runtime,
            target_x=entry_x,
            target_y=entry_y,
            final_target_yaw_deg=entry_final_yaw,
            cruise_forward_cmd=TARGET_SPEED,
            reference="robot",
        )
        if entry_move_result is None:
            print("Region 1 mission failed: move to stair -1 failed.")
            return
        print("Move to stair -1 finished.")
        print(entry_move_result)

        transition_results = []
        for from_id, to_id in STAIR_TRANSITIONS:
            transition_results.append(
                execute_transition(
                    sender=sender,
                    position_runtime=position_runtime,
                    odom_runtime=odom_runtime,
                    from_id=from_id,
                    to_id=to_id,
                )
            )

        hold_yaw = tools.direction_int_to_yaw_deg(1)
        print(f"Hold still for {HOLD_AFTER_DONE_SEC:.1f}s before shutdown...")
        hold_result = move.wait_with_target_yaw(
            sender=sender,
            duration_sec=HOLD_AFTER_DONE_SEC,
            target_yaw_deg=hold_yaw,
        )
        print(hold_result)

        print("Region 1 mission completed.")
        print({
            "weapon_result": weapon_result,
            "entry_move_result": entry_move_result,
            "transition_results": transition_results,
            "hold_result": hold_result,
        })

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
