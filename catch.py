#!/usr/bin/env python3

import math
import time

from lib2 import module, move, position_backend, tools


LIDAR_TYPE = 1
ODOM_TOPIC = "/odin1/odometry_highfreq"
FIELD_TYPE = position_backend.FIELD_TYPE_BLUE
WEAPON_ID = 3
MOVE_SPEED = 300
TARGET_YAW_DEG = -90.0
WEAPON_APPROACH_OFFSET_Y = 1.0
INITIAL_FORWARD_CMD = 300
INITIAL_FORWARD_DURATION_SEC = 0.5
WEAPON_MODE_SETTLE_SEC = 0.3
WEAPON_GRAB_ARM_SEC = 0.3
WEAPON_GRAB_HOLD_SEC = 1.0
WEAPON_GRIPPER_RESET_VALUE = -100
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
        weapon_pose = position_runtime.get_weapon_pose()
        update_time = position_runtime.get_latest_update_time()
        odometry = odom_runtime.get_odometry(max_age_sec=MAX_READY_DATA_AGE_SEC)

        data_finite = (
            robot_pose is not None
            and weapon_pose is not None
            and odometry is not None
            and values_are_finite(
                robot_pose["x"],
                robot_pose["y"],
                robot_pose["z"],
                robot_pose["yaw"],
                weapon_pose["x"],
                weapon_pose["y"],
                weapon_pose["z"],
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


def print_weapon_debug(position_runtime, weapon_id):
    position_lib = module.get_position_lib()
    get_weapon_targets = getattr(position_lib, "get_weapon_targets", None)
    weapon_targets = (
        get_weapon_targets()
        if get_weapon_targets is not None
        else getattr(position_lib, "WEAPON_TARGETS", {})
    )
    target = weapon_targets.get(int(weapon_id))
    weapon_pose = position_runtime.get_weapon_pose()
    robot_pose = position_runtime.get_robot_pose()

    print(f"weapon target[{weapon_id}] = {target}")
    if robot_pose is not None:
        print(
            "robot_pose: "
            f"x={float(robot_pose['x']):.3f}, "
            f"y={float(robot_pose['y']):.3f}, "
            f"z={float(robot_pose['z']):.3f}, "
            f"yaw={math.degrees(float(robot_pose['yaw'])):.2f}deg"
        )
    if weapon_pose is not None:
        print(
            "weapon_pose: "
            f"x={float(weapon_pose['x']):.3f}, "
            f"y={float(weapon_pose['y']):.3f}, "
            f"z={float(weapon_pose['z']):.3f}"
        )
    if target is not None and weapon_pose is not None:
        dx = float(target[0]) - float(weapon_pose["x"])
        dy = float(target[1]) - float(weapon_pose["y"])
        print(
            "weapon distance to target: "
            f"dx={dx:.3f}, dy={dy:.3f}, dist={math.hypot(dx, dy):.3f}m"
        )


def main():
    sender = None
    flag_node = None
    flag_thread = None
    flag_stop_event = None
    position_runtime = None
    odom_runtime = None

    try:
        print(
            "Starting blue weapon catch test, "
            f"weapon_id={WEAPON_ID}, speed={MOVE_SPEED}..."
        )
        position_backend.set_field_type(FIELD_TYPE)
        print(f"Field type set to {position_backend.get_field_type()} (BLUE).")
        sender, _, flag_node, flag_thread, flag_stop_event = module.init(
            lidar_type=LIDAR_TYPE
        )
        position_runtime = module.start_position_thread(sender)
        odom_runtime = module.start_odometry_thread(topic=ODOM_TOPIC)

        print(
            f"Driving forward for {INITIAL_FORWARD_DURATION_SEC:.1f}s "
            f"with ch2={INITIAL_FORWARD_CMD}..."
        )
        initial_forward_result = move.drive_with_channels_for_duration(
            sender=sender,
            duration_sec=INITIAL_FORWARD_DURATION_SEC,
            forward_cmd=INITIAL_FORWARD_CMD,
            brake_reverse_cmd=0,
            brake_duration_sec=0.0,
        )
        print("Initial forward drive finished.")
        print(initial_forward_result)

        print("Waiting for robot/weapon pose and odometry...")
        if not wait_runtime_ready(position_runtime, odom_runtime):
            print("fetch_weapon test failed: runtime data not ready.")
            return

        print_weapon_debug(position_runtime, WEAPON_ID)

        position_lib = module.get_position_lib()
        get_weapon_targets = getattr(position_lib, "get_weapon_targets", None)
        weapon_targets = (
            get_weapon_targets()
            if get_weapon_targets is not None
            else getattr(position_lib, "WEAPON_TARGETS", None)
        )
        if weapon_targets is None:
            raise AttributeError(
                f"{position_lib.__name__} must define get_weapon_targets() or WEAPON_TARGETS"
            )
        if WEAPON_ID not in weapon_targets:
            raise ValueError(f"weapon_id={WEAPON_ID} not in {sorted(weapon_targets)}")

        weapon_x, weapon_y = weapon_targets[WEAPON_ID]
        approach_x = float(weapon_x)
        approach_y = float(weapon_y) + float(WEAPON_APPROACH_OFFSET_Y)

        print(
            f"Moving weapon reference to approach point "
            f"({approach_x:.3f}, {approach_y:.3f}) "
            f"with target_yaw={TARGET_YAW_DEG:.2f} deg..."
        )
        approach_result = module.move_to_des(
            sender=sender,
            position_runtime=position_runtime,
            odom_runtime=odom_runtime,
            x=approach_x,
            y=approach_y,
            target_deg=TARGET_YAW_DEG,
            v=MOVE_SPEED,
            reference="weapon",
        )
        print("approach move returned:")
        print(approach_result)
        if approach_result is None:
            print("weapon approach test failed: approach move failed.")
            return

        print(
            f"Moving weapon reference to weapon target "
            f"({float(weapon_x):.3f}, {float(weapon_y):.3f}) "
            f"with target_yaw={TARGET_YAW_DEG:.2f} deg..."
        )
        target_result = module.move_to_des(
            sender=sender,
            position_runtime=position_runtime,
            odom_runtime=odom_runtime,
            x=float(weapon_x),
            y=float(weapon_y),
            target_deg=TARGET_YAW_DEG,
            v=MOVE_SPEED,
            stop_distance=0.02,
            reference="weapon",
        )
        print("target move returned:")
        print(target_result)
        print_weapon_debug(position_runtime, WEAPON_ID)
        if target_result is None:
            print("weapon approach test failed: target move failed.")
            return

        print("Entering weapon mode and grabbing weapon...")
        mode_deadline = time.time() + WEAPON_MODE_SETTLE_SEC
        while time.time() < mode_deadline:
            mode_channels = move.set_channel_values(
                sender,
                des_yaw_i16=0,
                channel_values={
                    1: 0,
                    2: 0,
                    4: WEAPON_GRIPPER_RESET_VALUE,
                    5: 3,
                    6: tools.SAFE_SWITCH_VALUE,
                    7: tools.SAFE_SWITCH_VALUE,
                },
            )
            time.sleep(0.02)

        grab_arm_channels = move.set_channel_values(
            sender,
            des_yaw_i16=0,
            channel_values={
                1: 0,
                2: 0,
                4: WEAPON_GRIPPER_RESET_VALUE,
                5: 3,
                6: tools.SAFE_SWITCH_VALUE,
                7: tools.SAFE_SWITCH_VALUE,
            },
        )
        time.sleep(WEAPON_GRAB_ARM_SEC)

        grab_fire_channels = move.set_channel_values(
            sender,
            des_yaw_i16=0,
            channel_values={
                1: 0,
                2: 0,
                4: 3,
                5: 3,
                6: tools.SAFE_SWITCH_VALUE,
                7: tools.SAFE_SWITCH_VALUE,
            },
        )
        time.sleep(WEAPON_GRAB_HOLD_SEC)
        print("weapon grab finished.")
        print({
            "mode_channels": mode_channels,
            "grab_arm_channels": grab_arm_channels,
            "grab_fire_channels": grab_fire_channels,
        })

        print(f"Holding current output for {HOLD_AFTER_DONE_SEC:.1f}s before shutdown...")
        time.sleep(HOLD_AFTER_DONE_SEC)
        print("weapon approach test completed.")

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


if __name__ == "__main__":
    main()
