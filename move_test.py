#!/usr/bin/env python3

import math
import time

from lib2 import module, move, tools


LIDAR_TYPE = 1
ODOM_TOPIC = "/odin1/odometry_highfreq"

TARGET_POINTS = [
    (-0.8, -4.8),
    (-1.0, 0.0),
    (-2.4, -2.4),
]
FIXED_YAW_DEG = 0.01

MAX_MOVE_CMD = 600
MIN_MOVE_CMD = 0
MIN_ACTIVE_MOVE_CMD = 180
STOP_DISTANCE = 0.03
STOP_SPEED_MPS = 0.06
YAW_GATE_DEG = 2.0
LATERAL_CMD_SIGN = -1
FORWARD_CMD_SIGN = 1
LOOP_INTERVAL_SEC = 0.02
MOVE_TIMEOUT_SEC = 600.0

PID_KP = 700.0
PID_KI = 0.0
PID_KD = 80.0
PID_INTEGRAL_LIMIT = 1.0

WAIT_READY_TIMEOUT_SEC = 8.0
WAIT_READY_STABLE_FRAMES = 5
WAIT_READY_POLL_SEC = 0.02
MAX_READY_DATA_AGE_SEC = 0.25


def clamp(value, min_value, max_value):
    return max(float(min_value), min(float(max_value), float(value)))


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


def world_error_to_fixed_body(dx, dy, fixed_yaw_deg):
    """
    Convert map-frame target error into the body axes implied by fixed_yaw_deg.

    ch2 forward is positive along fixed_yaw_deg.
    ch0 lateral is positive to the left of fixed_yaw_deg.
    """
    yaw_rad = math.radians(float(fixed_yaw_deg))
    cos_yaw = math.cos(yaw_rad)
    sin_yaw = math.sin(yaw_rad)

    forward_error = cos_yaw * dx + sin_yaw * dy
    lateral_error = -sin_yaw * dx + cos_yaw * dy
    return lateral_error, forward_error


def move_to_target_vector_pid(
    sender,
    position_runtime,
    odom_runtime,
    target_x,
    target_y,
    fixed_yaw_deg,
    kp=PID_KP,
    ki=PID_KI,
    kd=PID_KD,
    max_cmd=MAX_MOVE_CMD,
    min_cmd=MIN_MOVE_CMD,
    min_active_cmd=MIN_ACTIVE_MOVE_CMD,
    stop_distance=STOP_DISTANCE,
    stop_speed_mps=STOP_SPEED_MPS,
    yaw_gate_deg=YAW_GATE_DEG,
    lateral_cmd_sign=LATERAL_CMD_SIGN,
    forward_cmd_sign=FORWARD_CMD_SIGN,
    integral_limit=PID_INTEGRAL_LIMIT,
    loop_interval_sec=LOOP_INTERVAL_SEC,
    timeout_sec=MOVE_TIMEOUT_SEC,
    print_every_n=10,
):
    """
    Fixed-heading vector movement test.

    Distance PID outputs a scalar command magnitude. The command is then split
    between ch0 and ch2 by the live target error direction in fixed-heading
    body axes.
    """
    if not math.isfinite(float(target_x)) or not math.isfinite(float(target_y)):
        raise ValueError("target_x and target_y must be finite")

    fixed_yaw_deg = move.normalize_yaw_deg(fixed_yaw_deg)
    des_yaw_i16 = move.encode_target_yaw_i16(fixed_yaw_deg)
    started_at = time.time()
    deadline = None if timeout_sec is None else started_at + float(timeout_sec)

    integral = 0.0
    previous_distance = None
    previous_time = None
    last_result = None
    loop_count = 0

    while True:
        now = time.time()
        robot_pose = position_runtime.get_robot_pose()
        odometry = odom_runtime.get_odometry(max_age_sec=None)

        if robot_pose is not None and odometry is not None:
            current_x = float(robot_pose["x"])
            current_y = float(robot_pose["y"])
            dx = float(target_x) - current_x
            dy = float(target_y) - current_y
            distance = math.hypot(dx, dy)

            linear_speed_mps = math.hypot(
                float(odometry["linear_x"]),
                float(odometry["linear_y"]),
            )
            current_yaw_deg = move.normalize_yaw_deg(
                module.get_position_lib().radians_to_degrees(float(robot_pose["yaw"]))
            )
            yaw_error_deg = move.heading_error_deg(current_yaw_deg, fixed_yaw_deg)

            dt = 0.0 if previous_time is None else max(1e-6, now - previous_time)
            derivative = 0.0
            if previous_distance is not None and dt > 0.0:
                derivative = (distance - previous_distance) / dt
            integral = clamp(integral + distance * dt, -integral_limit, integral_limit)

            scalar_cmd = (
                float(kp) * distance
                + float(ki) * integral
                + float(kd) * derivative
            )
            scalar_cmd = clamp(scalar_cmd, min_cmd, max_cmd)
            if distance <= float(stop_distance):
                scalar_cmd = 0.0
            elif scalar_cmd < float(min_active_cmd):
                scalar_cmd = float(min_active_cmd)

            lateral_error, forward_error = world_error_to_fixed_body(
                dx=dx,
                dy=dy,
                fixed_yaw_deg=fixed_yaw_deg,
            )
            body_error_norm = math.hypot(lateral_error, forward_error)
            if body_error_norm > 1e-6:
                lateral_cmd = scalar_cmd * lateral_error / body_error_norm
                forward_cmd = scalar_cmd * forward_error / body_error_norm
            else:
                lateral_cmd = 0.0
                forward_cmd = 0.0

            yaw_gate_active = abs(yaw_error_deg) > float(yaw_gate_deg)
            if yaw_gate_active:
                lateral_cmd = 0.0
                forward_cmd = 0.0

            channels = move.set_motion_channels(
                sender,
                lateral_cmd=round(clamp(lateral_cmd_sign * lateral_cmd, -max_cmd, max_cmd)),
                forward_cmd=round(clamp(forward_cmd_sign * forward_cmd, -max_cmd, max_cmd)),
                rotation_cmd=0,
                des_yaw_i16=des_yaw_i16,
            )

            last_result = {
                "current_x": current_x,
                "current_y": current_y,
                "target_x": float(target_x),
                "target_y": float(target_y),
                "dx": float(dx),
                "dy": float(dy),
                "distance": float(distance),
                "linear_speed_mps": float(linear_speed_mps),
                "current_yaw_deg": float(current_yaw_deg),
                "yaw_error_deg": float(yaw_error_deg),
                "yaw_gate_deg": float(yaw_gate_deg),
                "yaw_gate_active": bool(yaw_gate_active),
                "fixed_yaw_deg": float(fixed_yaw_deg),
                "des_yaw_i16": int(des_yaw_i16),
                "lateral_error": float(lateral_error),
                "forward_error": float(forward_error),
                "scalar_cmd": float(scalar_cmd),
                "lateral_cmd": int(channels[0]),
                "forward_cmd": int(channels[2]),
                "lateral_cmd_sign": int(lateral_cmd_sign),
                "forward_cmd_sign": int(forward_cmd_sign),
                "pid": {
                    "kp": float(kp),
                    "ki": float(ki),
                    "kd": float(kd),
                    "integral": float(integral),
                    "derivative": float(derivative),
                },
                "channels": channels,
                "elapsed_sec": float(now - started_at),
            }

            if loop_count % int(print_every_n) == 0:
                print(
                    "[vector_pid] "
                    f"dist={distance:.3f}m | "
                    f"cmd={scalar_cmd:.1f} | "
                    f"ch0={channels[0]:4d} ch2={channels[2]:4d} | "
                    f"yaw_err={yaw_error_deg:6.2f}deg | "
                    f"gate={yaw_gate_active} | "
                    f"pos=({current_x:.3f},{current_y:.3f}) | "
                    f"err=({dx:.3f},{dy:.3f}) | "
                    f"speed={linear_speed_mps:.3f}m/s"
                )

            if distance <= float(stop_distance) and linear_speed_mps <= float(stop_speed_mps):
                stop_channels = move.set_motion_channels(sender, des_yaw_i16=des_yaw_i16)
                last_result["completed"] = True
                last_result["channels"] = stop_channels
                return last_result

            previous_distance = distance
            previous_time = now
            loop_count += 1

        if deadline is not None and time.time() >= deadline:
            stop_channels = move.set_motion_channels(sender, des_yaw_i16=des_yaw_i16)
            if last_result is None:
                return None
            last_result["timed_out"] = True
            last_result["channels"] = stop_channels
            return last_result

        time.sleep(float(loop_interval_sec))


def main():
    sender = None
    flag_node = None
    flag_thread = None
    flag_stop_event = None
    position_runtime = None
    odom_runtime = None

    try:
        print("Starting vector PID move test...")
        sender, _, flag_node, flag_thread, flag_stop_event = module.init(lidar_type=LIDAR_TYPE)
        position_runtime = module.start_position_thread(sender)
        odom_runtime = module.start_odometry_thread(topic=ODOM_TOPIC)

        print("Waiting for pose and odometry...")
        if not wait_runtime_ready(position_runtime, odom_runtime):
            print("Move test failed: pose/odometry not ready.")
            return

        results = []
        for index, (target_x, target_y) in enumerate(TARGET_POINTS, start=1):
            print(
                f"Vector PID waypoint {index}/{len(TARGET_POINTS)}: "
                f"move to ({target_x:.3f}, {target_y:.3f}) "
                f"with fixed_yaw={FIXED_YAW_DEG:.2f} deg"
            )
            result = move_to_target_vector_pid(
                sender=sender,
                position_runtime=position_runtime,
                odom_runtime=odom_runtime,
                target_x=target_x,
                target_y=target_y,
                fixed_yaw_deg=FIXED_YAW_DEG,
            )
            print(f"Vector PID waypoint {index} finished.")
            print(result)
            results.append(result)
            if result is None or result.get("timed_out"):
                print("Vector PID sequence stopped before remaining waypoints.")
                return

        stop_channels = move.set_motion_channels(
            sender,
            des_yaw_i16=move.encode_target_yaw_i16(FIXED_YAW_DEG),
        )
        print("Vector PID move sequence completed.")
        print(results)

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
