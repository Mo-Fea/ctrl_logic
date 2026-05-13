#!/usr/bin/env python3

import argparse
import math
import time

from lib2 import module, move, tools


LIDAR_TYPE = 2
ODOM_TOPIC = "/lio/robo/odom"

DEFAULT_TARGET_X = 2.0
DEFAULT_TARGET_Y = -2.0
DEFAULT_FINAL_YAW_DEG = 0.01
DEFAULT_LATERAL_CMD = 200
HOLD_AFTER_MOVE_SEC = 10.0

WAIT_READY_TIMEOUT_SEC = 8.0
WAIT_READY_STABLE_FRAMES = 5
WAIT_READY_POLL_SEC = 0.02
MAX_READY_DATA_AGE_SEC = 0.25

STOP_DISTANCE = 0.02
REACHED_SPEED_MPS = 0.05
REACHED_YAW_RATE_RAD = 0.05
YAW_GATE_DEG = 10.0
NEAR_DISTANCE = 1.0
FINE_DISTANCE = 0.4
NEAR_LATERAL_CMD = 100
FINE_LATERAL_CMD = 75
LOOP_INTERVAL_SEC = 0.02


def normalize_yaw_deg(yaw_deg):
    return tools.yaw_normalization(float(yaw_deg))


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


def move_lateral_to_target(
    sender,
    position_runtime,
    odom_runtime,
    target_x,
    target_y,
    final_yaw_deg,
    lateral_cmd=DEFAULT_LATERAL_CMD,
    stop_distance=STOP_DISTANCE,
    timeout_sec=None,
):
    """
    用 ch0 横移到点。

    每轮按 target - current 计算目标方向角，然后发送 target_direction + 90deg
    作为底盘保持航向。ch0 为正时，机器人右移方向等价于该目标方向。
    """
    if not values_are_finite(target_x, target_y, final_yaw_deg, lateral_cmd):
        raise ValueError("target_x, target_y, final_yaw_deg and lateral_cmd must be finite")

    deadline = None if timeout_sec is None else (time.time() + float(timeout_sec))
    result = None

    while True:
        robot_pose = position_runtime.get_robot_pose()
        odometry = odom_runtime.get_odometry(max_age_sec=None)

        if robot_pose is not None and odometry is not None:
            current_x = float(robot_pose["x"])
            current_y = float(robot_pose["y"])
            current_yaw_deg = normalize_yaw_deg(
                module.position_lib.radians_to_degrees(float(robot_pose["yaw"]))
            )
            linear_x = float(odometry["linear_x"])
            linear_y = float(odometry["linear_y"])
            angular_z_rad = float(odometry["angular_z"])

            if not values_are_finite(
                current_x,
                current_y,
                current_yaw_deg,
                linear_x,
                linear_y,
                angular_z_rad,
            ):
                sender.set_channels_and_des_yaw_i16(
                    tools.compose_channels(lateral_cmd=0, forward_cmd=0, rotation_cmd=0),
                    0,
                )
                time.sleep(LOOP_INTERVAL_SEC)
                continue

            distance_xy, dx, dy = move.distance_to_target_xy(
                current_x=current_x,
                current_y=current_y,
                target_x=target_x,
                target_y=target_y,
            )
            target_direction_deg = module.position_lib.cal_target_yaw_deg(
                target_x=target_x,
                target_y=target_y,
                robot_x=current_x,
                robot_y=current_y,
            )
            hold_yaw_deg = normalize_yaw_deg(target_direction_deg + 90.0)
            yaw_error_deg = move.heading_error_deg(current_yaw_deg, hold_yaw_deg)

            linear_speed_mps = math.hypot(linear_x, linear_y)
            reached = (
                distance_xy <= float(stop_distance)
                and linear_speed_mps <= REACHED_SPEED_MPS
                and abs(angular_z_rad) <= REACHED_YAW_RATE_RAD
            )

            current_lateral_cmd = int(abs(lateral_cmd))
            if distance_xy <= NEAR_DISTANCE:
                current_lateral_cmd = int(NEAR_LATERAL_CMD)
            if distance_xy <= FINE_DISTANCE:
                current_lateral_cmd = int(FINE_LATERAL_CMD)
            if abs(yaw_error_deg) > YAW_GATE_DEG:
                current_lateral_cmd = 0

            channels = tools.compose_channels(
                lateral_cmd=current_lateral_cmd,
                forward_cmd=0,
                rotation_cmd=0,
            )
            sender.set_channels_and_des_yaw_i16(
                channels,
                move.encode_target_yaw_i16(hold_yaw_deg),
            )

            result = {
                "current_x": current_x,
                "current_y": current_y,
                "target_x": float(target_x),
                "target_y": float(target_y),
                "dx": float(dx),
                "dy": float(dy),
                "distance_xy": float(distance_xy),
                "current_yaw_deg": float(current_yaw_deg),
                "target_direction_deg": float(target_direction_deg),
                "hold_yaw_deg": float(hold_yaw_deg),
                "heading_error_deg": float(yaw_error_deg),
                "lateral_cmd": int(current_lateral_cmd),
                "channels": channels,
                "reached": bool(reached),
            }

            if reached:
                final_hold_yaw_deg = normalize_yaw_deg(float(final_yaw_deg) + 90.0)
                stop_channels = tools.compose_channels(lateral_cmd=0, forward_cmd=0, rotation_cmd=0)
                sender.set_channels_and_des_yaw_i16(
                    stop_channels,
                    move.encode_target_yaw_i16(final_hold_yaw_deg),
                )
                result.update({
                    "channels": stop_channels,
                    "final_input_yaw_deg": float(normalize_yaw_deg(final_yaw_deg)),
                    "final_hold_yaw_deg": float(final_hold_yaw_deg),
                })
                return result

        if deadline is not None and time.time() >= deadline:
            if result is not None:
                result["timed_out"] = True
            return result

        time.sleep(LOOP_INTERVAL_SEC)


def parse_args():
    parser = argparse.ArgumentParser(description="Move to a target using ch0 lateral control.")
    parser.add_argument("--x", type=float, default=DEFAULT_TARGET_X)
    parser.add_argument("--y", type=float, default=DEFAULT_TARGET_Y)
    parser.add_argument("--yaw", type=float, default=DEFAULT_FINAL_YAW_DEG)
    parser.add_argument("--speed", type=int, default=DEFAULT_LATERAL_CMD)
    parser.add_argument("--hold", type=float, default=HOLD_AFTER_MOVE_SEC)
    return parser.parse_args()


def main():
    args = parse_args()

    sender = None
    flag_node = None
    flag_thread = None
    flag_stop_event = None
    position_runtime = None
    odom_runtime = None

    try:
        print("Starting level_move lateral move test...")
        sender, _, flag_node, flag_thread, flag_stop_event = module.init(lidar_type=LIDAR_TYPE)
        position_runtime = module.start_position_thread(sender)
        odom_runtime = module.start_odometry_thread(topic=ODOM_TOPIC)

        print("Waiting for mid360 pose and odometry...")
        if not wait_runtime_ready(position_runtime, odom_runtime):
            print("Lateral move failed: pose/odometry not ready.")
            return

        print(
            f"Moving laterally to ({args.x:.2f}, {args.y:.2f}) | "
            f"input_yaw={args.yaw:.2f} deg | hold_final_yaw={normalize_yaw_deg(args.yaw + 90.0):.2f} deg"
        )
        result = move_lateral_to_target(
            sender=sender,
            position_runtime=position_runtime,
            odom_runtime=odom_runtime,
            target_x=args.x,
            target_y=args.y,
            final_yaw_deg=args.yaw,
            lateral_cmd=args.speed,
        )

        if result is None:
            print("Lateral move failed: no valid result.")
            return

        print("Lateral move finished.")
        print(result)
        print(f"Holding current output for {args.hold:.1f}s before shutdown...")
        time.sleep(float(args.hold))

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
