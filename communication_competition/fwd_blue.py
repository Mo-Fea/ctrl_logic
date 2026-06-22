#!/usr/bin/env python3

import math
import time

from lib2 import module, move, position_backend, tools


LIDAR_TYPE = 1
FIELD_TYPE = position_backend.FIELD_TYPE_BLUE
ODOM_TOPIC = "/odin1/odometry_highfreq"

START_DELAY_SEC = 10.0
TARGET_YAW_DEG = 0.01
FORWARD_CMD = 600
FORWARD_DURATION_SEC = 10.0
LOOP_INTERVAL_SEC = 0.02

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


def drive_forward_for_duration(sender):
    des_yaw_i16 = move.encode_target_yaw_i16(TARGET_YAW_DEG)
    deadline = time.time() + float(FORWARD_DURATION_SEC)
    channels = None

    while time.time() < deadline:
        channels = move.set_motion_channels(
            sender,
            lateral_cmd=0,
            forward_cmd=FORWARD_CMD,
            rotation_cmd=0,
            des_yaw_i16=des_yaw_i16,
        )
        time.sleep(LOOP_INTERVAL_SEC)

    stop_channels = move.set_motion_channels(
        sender,
        lateral_cmd=0,
        forward_cmd=0,
        rotation_cmd=0,
        des_yaw_i16=des_yaw_i16,
    )
    return {
        "completed": True,
        "duration_sec": float(FORWARD_DURATION_SEC),
        "forward_cmd": int(FORWARD_CMD),
        "last_drive_channels": channels,
        "stop_channels": stop_channels,
    }


def main():
    sender = None
    flag_node = None
    flag_thread = None
    flag_stop_event = None
    position_runtime = None
    odom_runtime = None

    try:
        print("Starting blue timed forward test...")
        position_backend.set_field_type(FIELD_TYPE)
        print(f"Field type set to {position_backend.get_field_type()} (BLUE).")

        sender, _, flag_node, flag_thread, flag_stop_event = module.init(
            lidar_type=LIDAR_TYPE
        )
        position_runtime = module.start_position_thread(sender)
        odom_runtime = module.start_odometry_thread(topic=ODOM_TOPIC)

        print("Waiting for robot pose and odometry...")
        if not wait_runtime_ready(position_runtime, odom_runtime):
            print("Blue forward test failed: runtime data not ready.")
            return

        print(f"Waiting in place for {START_DELAY_SEC:.1f}s...")
        time.sleep(START_DELAY_SEC)

        print(f"Rotating in place to target_yaw={TARGET_YAW_DEG:.2f} deg...")
        rotate_result = move.rotate_to_target_yaw_segmented(
            sender=sender,
            position_runtime=position_runtime,
            odom_runtime=odom_runtime,
            target_yaw_deg=TARGET_YAW_DEG,
        )
        if rotate_result is None or rotate_result.get("timed_out"):
            print("Blue forward test failed: rotate failed.")
            print(rotate_result)
            return
        print("Rotate finished.")
        print(rotate_result)

        print(
            f"Driving forward with ch2={FORWARD_CMD} "
            f"for {FORWARD_DURATION_SEC:.1f}s..."
        )
        forward_result = drive_forward_for_duration(sender)
        print("Forward drive finished.")
        print(forward_result)

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
        print("Blue timed forward test finished.")


if __name__ == "__main__":
    main()
