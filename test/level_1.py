#!/usr/bin/env python3
import time

from lib import move, tools
from r2_logic.lib2 import position_odin


FRAME_RATE = 50.0
FRAME_INTERVAL = 1.0 / FRAME_RATE
FLAG_TOPIC = "/odin1/flag1"
BASE_FRAME = "odin1_base_link"
WAIT_TARGET_YAW_DEG = 90.0
FINAL_TARGET_YAW_DEG = 0.01
MAX_ODOM_AGE_SEC = 0.25
MAX_TF_AGE_SEC = 0.25
FINAL_YAW_TOLERANCE_DEG = 1.0

MISSION_STAGES = [
    {"target": (1.89, 1.0, 0.0), "climb": 1, "name": "stage_1_first_climb"},
    {"target": (2.9, 1.0, 0.0), "climb": 2, "name": "stage_2_second_climb"},
    {"target": (2.9, 2.2, 0.0), "climb": None, "name": "stage_3_final_target"},
]


class LevelPhase:
    WAIT_RELOCALIZATION = "wait_relocalization"
    MOVING_TO_TARGET = "moving_to_target"
    CLIMBING = "climbing"
    FINAL_ALIGN = "final_align"
    DONE = "done"


def get_current_yaw_deg(tf_node):
    map_pose = position_odin.get_lidar_pose_in_map_synced(
        tf_cache_node=tf_node,
        max_age_sec=MAX_TF_AGE_SEC,
        clock=tf_node.get_clock(),
    )
    if map_pose is None:
        return None
    return move.normalize_yaw_deg(position_odin.radians_to_degrees(map_pose["yaw"]))


def current_stage(stage_index):
    if stage_index < 0 or stage_index >= len(MISSION_STAGES):
        return None
    return MISSION_STAGES[stage_index]


def format_target(target):
    return f"({target[0]:.2f}, {target[1]:.2f}, {target[2]:.2f})"


def main():
    sock = None
    seq = 0
    get_flag = None
    flag_node = None
    flag_thread = None
    flag_stop_event = None
    tf_node = None
    tf_thread = None
    tf_stop_event = None
    odom_node = None
    odom_thread = None
    odom_stop_event = None

    phase = LevelPhase.WAIT_RELOCALIZATION
    last_print_time = 0.0
    stage_index = 0

    try:
        print("Starting level controller...")
        sock = tools.connect()
        get_flag, flag_node, flag_thread, flag_stop_event = tools.relocalization_conformation(
            topic=FLAG_TOPIC
        )

        tf_node = position_odin.TfCacheNode(base_frame=BASE_FRAME, update_hz=50.0)
        tf_stop_event = tools.threading.Event()
        tf_thread = tools.threading.Thread(
            target=position_odin.spin_tf_cache,
            args=(tf_node, tf_stop_event),
            daemon=True,
        )
        tf_thread.start()

        odom_node, odom_thread, odom_stop_event = move.start_odometry_subscription()
        controller = move.MoveController(
            odom_node=odom_node,
            tf_cache_node=tf_node,
        )

        next_send = time.time()

        while True:
            now = time.time()
            stage = current_stage(stage_index)
            channels = tools.compose_channels(lateral_cmd=0, forward_cmd=0, rotation_cmd=0)
            yaw_i16 = 0
            des_yaw_i16 = move.encode_target_yaw_i16(WAIT_TARGET_YAW_DEG)

            if phase == LevelPhase.WAIT_RELOCALIZATION:
                if get_flag():
                    phase = LevelPhase.MOVING_TO_TARGET
                    print(
                        f"Relocalization confirmed, moving to {stage['name']} target "
                        f"{format_target(stage['target'])}."
                    )

            elif phase == LevelPhase.MOVING_TO_TARGET:
                result = controller.des_move(
                    x=stage["target"][0],
                    y=stage["target"][1],
                    z=stage["target"][2],
                    max_odom_age_sec=MAX_ODOM_AGE_SEC,
                    max_tf_age_sec=MAX_TF_AGE_SEC,
                )
                if result is not None:
                    channels = result["channels"]
                    yaw_i16 = result["yaw_i16"]
                    des_yaw_i16 = result["des_yaw_i16"]

                    if (now - last_print_time) >= 0.5:
                        print(
                            f"[{stage['name']}:{result['mode'].upper()}] "
                            f"dist_xy={result['distance_xy']:.3f} m | "
                            f"target_yaw={result['target_yaw_deg']:.2f} deg | "
                            f"yaw_error={result['heading_error_deg']:.2f} deg | "
                            f"fwd={result.get('forward_cmd', 0):.0f} | "
                            f"ch2={result['channels'][2]}"
                        )
                        last_print_time = now

                    if result["mode"] == "reached":
                        if stage["climb"] is not None:
                            phase = LevelPhase.CLIMBING
                            print(
                                f"{stage['name']} target reached, starting climb({stage['climb']})."
                            )
                        else:
                            phase = LevelPhase.FINAL_ALIGN
                            print(
                                f"Final target reached, aligning final yaw to "
                                f"{FINAL_TARGET_YAW_DEG:.2f} deg."
                            )

            elif phase == LevelPhase.CLIMBING:
                sock, seq, climb_ok = move.climb(
                    stage["climb"],
                    sock=sock,
                    seq=seq,
                    tf_cache_node=tf_node,
                )
                if not climb_ok:
                    print(f"{stage['name']} climb failed, stopping mission.")
                    break

                stage_index += 1
                next_stage = current_stage(stage_index)
                if next_stage is None:
                    phase = LevelPhase.DONE
                    print("Mission complete.")
                    continue

                next_send = time.time()
                phase = LevelPhase.MOVING_TO_TARGET
                print(
                    f"{stage['name']} climb succeeded, moving to {next_stage['name']} target "
                    f"{format_target(next_stage['target'])}."
                )
                continue

            elif phase == LevelPhase.FINAL_ALIGN:
                current_yaw_deg = get_current_yaw_deg(tf_node)
                if current_yaw_deg is not None:
                    result = move.rotate_to_target_yaw(
                        target_yaw_deg=FINAL_TARGET_YAW_DEG,
                        current_yaw_deg=current_yaw_deg,
                        tolerance_deg=FINAL_YAW_TOLERANCE_DEG,
                    )
                    if result is not None:
                        channels = result["channels"]
                        yaw_i16 = result["yaw_i16"]
                        des_yaw_i16 = result["des_yaw_i16"]

                        if (now - last_print_time) >= 0.5:
                            print(
                                f"[FINAL_ALIGN] "
                                f"target_yaw={result['target_yaw_deg']:.2f} deg | "
                                f"yaw_error={result['heading_error_deg']:.2f} deg"
                            )
                            last_print_time = now

                        if result["heading_reached"]:
                            phase = LevelPhase.DONE
                            print("Final yaw aligned, mission complete.")

            elif phase == LevelPhase.DONE:
                current_yaw_deg = get_current_yaw_deg(tf_node)
                if current_yaw_deg is not None:
                    yaw_i16 = move.encode_current_yaw_i16(current_yaw_deg)
                    des_yaw_i16 = move.encode_target_yaw_i16(FINAL_TARGET_YAW_DEG)

            frame = tools.build_frame(
                seq=seq,
                channels=channels,
                yaw_i16=yaw_i16,
                des_yaw_i16=des_yaw_i16,
            )
            sock, ok = tools.send_frame(
                sock=sock,
                frame=frame,
                connect_func=tools.connect,
                tcp_ip=tools.TCP_IP,
                tcp_port=tools.TCP_PORT,
                retry_interval=tools.CONNECT_RETRY_INTERVAL,
            )
            if not ok:
                next_send = time.time()
                continue

            seq = (seq + 1) & 0xFFFF

            next_send += FRAME_INTERVAL
            sleep_time = next_send - time.time()
            if sleep_time > 0.0:
                time.sleep(sleep_time)
            else:
                next_send = time.time()

    except KeyboardInterrupt:
        print("\nStopped by user.")

    finally:
        try:
            sock, _ = tools.send_stop_frame(
                sock=sock,
                seq=seq,
                send_frame_func=tools.send_frame,
                connect_func=tools.connect,
                tcp_ip=tools.TCP_IP,
                tcp_port=tools.TCP_PORT,
                retry_interval=tools.CONNECT_RETRY_INTERVAL,
            )
        except Exception:
            pass

        tools.socket_close(sock)

        if odom_node is not None:
            move.stop_odometry_subscription(odom_node, odom_thread, odom_stop_event)

        if tf_stop_event is not None:
            tf_stop_event.set()
        if tf_thread is not None and tf_thread.is_alive():
            tf_thread.join(timeout=1.0)
        if tf_node is not None:
            try:
                tf_node.destroy_node()
            except Exception:
                pass

        if flag_node is not None:
            tools.destroy_ros2_thread(
                node=flag_node,
                spin_thread=flag_thread,
                stop_event=flag_stop_event,
                shutdown_rclpy=False,
            )

        if tools.rclpy.ok():
            try:
                tools.rclpy.shutdown()
            except Exception:
                pass


if __name__ == "__main__":
    main()
