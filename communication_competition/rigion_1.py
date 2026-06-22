#!/usr/bin/env python3

import math
import queue
import threading
import time

from lib2 import module, position_backend, tools
from utils import challenge_lib


LIDAR_TYPE = 1
FIELD_TYPE = position_backend.FIELD_TYPE_RED
ODOM_TOPIC = "/odin1/odometry_highfreq"

START_STAIR_ID = -1
START_FIXED_YAW_DEG = 0.01
MOVE_SPEED = 600
FINAL_DIRECTION = 1
POST_EXIT_TARGET_1_YAW_DEG = 0.01
POST_EXIT_TARGET_1_X = 5.437
POST_EXIT_TARGET_1_Y = -4.156
POST_EXIT_TARGET_2_YAW_DEG = 0.01
POST_EXIT_TARGET_2_X = 8.378
POST_EXIT_TARGET_2_Y = -3.797
POST_EXIT_TARGET_3_YAW_DEG = 90.0
POST_EXIT_TARGET_3_X = 7.537
POST_EXIT_TARGET_3_Y = 0.897

QR_STABLE_FRAME_COUNT = 5
HOLD_AFTER_DONE_SEC = 3.0

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


def get_action_matrix_from_queue(action_matrix_queue, scanner):
    if action_matrix_queue.empty():
        if scanner.last_error is not None:
            raise RuntimeError(
                "QR scanner failed: "
                f"{repr(scanner.last_error)} | "
                f"last_qr_data={scanner.last_qr_data!r} | "
                f"last_stable_count={scanner.last_stable_count}"
            )
        raise RuntimeError(
            "QR scanner released lock but action matrix queue is empty: "
            f"last_qr_data={scanner.last_qr_data!r} | "
            f"last_stable_count={scanner.last_stable_count}"
        )
    return action_matrix_queue.get()


def main():
    sender = None
    flag_node = None
    flag_thread = None
    flag_stop_event = None
    position_runtime = None
    odom_runtime = None
    scanner = None

    action_matrix_queue = queue.Queue()
    visual_lock = threading.Lock()

    try:
        print("Starting region 1 challenge matrix test...")
        position_backend.set_field_type(FIELD_TYPE)
        print(f"Field type set to {position_backend.get_field_type()} (RED).")
        print("Starting background QR scanner...")
        scanner = challenge_lib.start_background_qr_scanner(
            result_queue=action_matrix_queue,
            stable_frame_count=QR_STABLE_FRAME_COUNT,
            show_window=True,
            stop_after_success=True,
            put_action_matrix_only=True,
            running_lock=visual_lock,
        )

        sender, _, flag_node, flag_thread, flag_stop_event = module.init(
            lidar_type=LIDAR_TYPE
        )
        position_runtime = module.start_position_thread(sender)
        odom_runtime = module.start_odometry_thread(topic=ODOM_TOPIC)

        print("Waiting for robot pose and odometry...")
        if not wait_runtime_ready(position_runtime, odom_runtime):
            print("Region 1 test failed: runtime data not ready.")
            return

        print("Waiting for visual scanner lock before region 1 movement...")
        with visual_lock:
            action_matrix = get_action_matrix_from_queue(action_matrix_queue, scanner)
            print("[QR action matrix from queue]")
            print(action_matrix)

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
                print("Region 1 test failed: move to start stair failed.")
                return
            print("Move to start stair finished.")
            print(start_move_result)

            print("Executing QR action matrix...")
            matrix_result = module.execute_action_matrix(
                sender=sender,
                position_runtime=position_runtime,
                odom_runtime=odom_runtime,
                action_matrix=action_matrix,
                final_direction=FINAL_DIRECTION,
            )
            print("QR action matrix finished.")
            print(matrix_result)

        print(
            "Moving to post-exit target 1 "
            f"({POST_EXIT_TARGET_1_X:.3f}, {POST_EXIT_TARGET_1_Y:.3f}) "
            f"with target_yaw={POST_EXIT_TARGET_1_YAW_DEG:.2f} deg..."
        )
        post_move_1 = module.move_to_des(
            sender=sender,
            position_runtime=position_runtime,
            odom_runtime=odom_runtime,
            x=POST_EXIT_TARGET_1_X,
            y=POST_EXIT_TARGET_1_Y,
            target_deg=POST_EXIT_TARGET_1_YAW_DEG,
            v=MOVE_SPEED,
        )
        print("Post-exit target 1 move finished.")
        print(post_move_1)

        print(
            "Moving to post-exit target 2 "
            f"({POST_EXIT_TARGET_2_X:.3f}, {POST_EXIT_TARGET_2_Y:.3f}) "
            f"with target_yaw={POST_EXIT_TARGET_2_YAW_DEG:.2f} deg..."
        )
        post_move_2 = module.move_to_des(
            sender=sender,
            position_runtime=position_runtime,
            odom_runtime=odom_runtime,
            x=POST_EXIT_TARGET_2_X,
            y=POST_EXIT_TARGET_2_Y,
            target_deg=POST_EXIT_TARGET_2_YAW_DEG,
            v=MOVE_SPEED,
        )
        print("Post-exit target 2 move finished.")
        print(post_move_2)

        print(
            "Moving to post-exit target 3 "
            f"({POST_EXIT_TARGET_3_X:.3f}, {POST_EXIT_TARGET_3_Y:.3f}) "
            f"with target_yaw={POST_EXIT_TARGET_3_YAW_DEG:.2f} deg..."
        )
        post_move_3 = module.move_to_des(
            sender=sender,
            position_runtime=position_runtime,
            odom_runtime=odom_runtime,
            x=POST_EXIT_TARGET_3_X,
            y=POST_EXIT_TARGET_3_Y,
            target_deg=POST_EXIT_TARGET_3_YAW_DEG,
            v=MOVE_SPEED,
        )
        print("Post-exit target 3 move finished.")
        print(post_move_3)

        print(f"Holding current output for {HOLD_AFTER_DONE_SEC:.1f}s before shutdown...")
        time.sleep(HOLD_AFTER_DONE_SEC)
        print("Region 1 challenge matrix test completed.")

    except KeyboardInterrupt:
        print("\nStopped by user.")

    finally:
        if scanner is not None:
            scanner.stop()
            scanner.join(timeout=1.0)

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
