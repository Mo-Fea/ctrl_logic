#!/usr/bin/env python3

import math
import queue
import re
import threading
import time
from dataclasses import dataclass

from lib2 import kfs, module, move, position_backend, position_resource, tools, weapon
from utils import challenge_lib, race


INVALID_INPUT_MESSAGE = "严格按照上述数字进行输入"
LIDAR_TYPE = position_backend.LIDAR_TYPE_ODIN
MOVE_TIMEOUT_SEC = 30.0
QR_STABLE_FRAME_COUNT = 2
current_stair_id = 0


@dataclass
class RuntimeContext:
    sender: object = None
    flag_node: object = None
    flag_thread: object = None
    flag_stop_event: object = None
    position_runtime: object = None
    odom_runtime: object = None
    image_source: int = 1


def wait_runtime_ready(context, timeout_sec=8.0):
    deadline = time.time() + float(timeout_sec)
    while time.time() < deadline:
        robot_pose = context.position_runtime.get_robot_pose()
        odometry = context.odom_runtime.get_odometry(max_age_sec=0.25)
        if robot_pose is not None and odometry is not None:
            return True
        time.sleep(0.02)
    return False


def cleanup_runtime(context):
    tf_node = None
    tf_thread = None
    tf_stop_event = None
    if context.position_runtime is not None:
        position_threads = context.position_runtime.get_threads()
        tf_node = position_threads["tf_node"]
        tf_thread = position_threads["tf_thread"]
        tf_stop_event = position_threads["tf_stop_event"]
        position_threads["position_stop_event"].set()
        position_thread = position_threads["position_thread"]
        if position_thread is not None and position_thread.is_alive():
            position_thread.join(timeout=1.0)

    odom_node = None
    odom_thread = None
    odom_stop_event = None
    if context.odom_runtime is not None:
        odom_threads = context.odom_runtime.get_threads()
        odom_node = odom_threads["odom_node"]
        odom_thread = odom_threads["odom_thread"]
        odom_stop_event = odom_threads["odom_stop_event"]

    tools.handle_ctrl_c(
        sender=context.sender,
        flag_node=context.flag_node,
        flag_thread=context.flag_thread,
        flag_stop_event=context.flag_stop_event,
        tf_node=tf_node,
        tf_thread=tf_thread,
        tf_stop_event=tf_stop_event,
        extra_node=odom_node,
        extra_thread=odom_thread,
        extra_stop_event=odom_stop_event,
        shutdown_rclpy=True,
    )


def read_choice(valid_choices, prompt="> "):
    valid_choices = {str(choice) for choice in valid_choices}
    while True:
        choice = input(prompt).strip()
        if choice in valid_choices:
            return choice
        print(INVALID_INPUT_MESSAGE)


def read_finite_float(prompt):
    while True:
        raw_value = input(prompt).strip()
        try:
            value = float(raw_value)
        except ValueError:
            print(INVALID_INPUT_MESSAGE)
            continue
        if not math.isfinite(value):
            print(INVALID_INPUT_MESSAGE)
            continue
        return value


def read_integer(prompt):
    while True:
        raw_value = input(prompt).strip()
        try:
            return int(raw_value)
        except ValueError:
            print(INVALID_INPUT_MESSAGE)


def convert_yaw_after_map_rotation(input_yaw_deg):
    """
    将测试员按原始地图坐标系输入的航向转换到当前地图坐标系。

    在上一次逆时针旋转 90 度的基础上，地图又顺时针旋转 180 度，
    因此当前地图航向 = 原始输入航向 + 90 度。结果归一化到 [-180, 180)，
    用户直接输入 0 时保留 0，用于关闭航向 PID；其他输入转换后若为
    0 度，则改为 0.01 度，避免意外关闭航向 PID。
    """
    input_yaw_deg = float(input_yaw_deg)
    if math.isclose(input_yaw_deg, 0.0, abs_tol=1e-9):
        return 0.0

    converted_yaw_deg = (input_yaw_deg + 90.0 + 180.0) % 360.0 - 180.0
    if math.isclose(converted_yaw_deg, 0.0, abs_tol=1e-9):
        return 0.01
    return converted_yaw_deg


def read_yaw_deg():
    pattern = re.compile(r"^[+-]?\d+\.\d{2}$")
    prompt = "请输入原始地图坐标系下的最终机器朝向（-180.00到180.00，须保留两位小数）"
    while True:
        raw_value = input(prompt).strip()
        if not pattern.fullmatch(raw_value):
            print(INVALID_INPUT_MESSAGE)
            continue
        value = float(raw_value)
        if -180.0 <= value <= 180.0:
            converted_value = convert_yaw_after_map_rotation(value)
            print(f"地图旋转后的目标角度：{converted_value:.2f}")
            return converted_value
        print(INVALID_INPUT_MESSAGE)


def read_move_speed():
    prompt = "请输入机器移动最大速度（0-600）："
    while True:
        raw_value = input(prompt).strip()
        if not raw_value.isdigit():
            print(INVALID_INPUT_MESSAGE)
            continue
        value = int(raw_value)
        if 0 <= value <= 600:
            return value
        print(INVALID_INPUT_MESSAGE)


def read_stop_distance():
    pattern = re.compile(r"^\+?\d+\.\d{2,}$")
    prompt = "请输入机器到点判断阈值(单位m，至少保留两位):"
    while True:
        raw_value = input(prompt).strip()
        if not pattern.fullmatch(raw_value):
            print(INVALID_INPUT_MESSAGE)
            continue
        value = float(raw_value)
        if math.isfinite(value) and value > 0.0:
            return value
        print(INVALID_INPUT_MESSAGE)


def read_adjust_distance():
    pattern = re.compile(r"^\+?\d+\.\d{2}$")
    prompt = "请输入微调距离(单位m，保留两位，0-0.5):"
    while True:
        raw_value = input(prompt).strip()
        if not pattern.fullmatch(raw_value):
            print(INVALID_INPUT_MESSAGE)
            continue
        value = float(raw_value)
        if 0.0 <= value <= 0.5:
            return value
        print(INVALID_INPUT_MESSAGE)


def read_rotation_target_yaw():
    prompt = "请输入目标角度（-180.00到180.00，最好保留两位小数）"
    while True:
        value = read_finite_float(prompt)
        if -180.0 <= value <= 180.0:
            return value
        print(INVALID_INPUT_MESSAGE)


def read_positive_float(prompt):
    while True:
        value = read_finite_float(prompt)
        if value > 0.0:
            return value
        print(INVALID_INPUT_MESSAGE)


def read_positive_float_with_default(prompt, default_value):
    default_value = float(default_value)
    while True:
        raw_value = input(prompt).strip()
        if raw_value == "":
            return default_value
        try:
            value = float(raw_value)
        except ValueError:
            print(INVALID_INPUT_MESSAGE)
            continue
        if math.isfinite(value) and value > 0.0:
            return value
        print(INVALID_INPUT_MESSAGE)


def read_rotation_tolerance():
    pattern = re.compile(r"^[+]?\d+\.\d{2}$")
    prompt = "请输入完成判断阈值（保留两位小数，且大于0）："
    while True:
        raw_value = input(prompt).strip()
        if not pattern.fullmatch(raw_value):
            print(INVALID_INPUT_MESSAGE)
            continue
        value = float(raw_value)
        if value > 0.0:
            return value
        print(INVALID_INPUT_MESSAGE)


def print_main_menu():
    print("---------------------------------------------------------------------------------------------")
    print("这是R2完全测试脚本，ctrl+c可退出测试")
    print("严格按照要求输入，不要乱输入，否则机器可能损坏")
    print("请输入你想要进行的测试类型：")
    print("1.简单动作测试（只需要操控单个部件，或者移动，旋转测试）")
    print("2.复杂动作测试（多个部件联动，或者区域流程，或者完整流程）")
    print("3.恢复至初态")


def select_field_type():
    print("请选择测试场地半场：")
    print("1.红场（我们场地为红场）")
    print("2.蓝场")
    field_choice = read_choice({"1", "2"})
    field_name = "红场" if field_choice == "1" else "蓝场"
    print(f"已选择（{field_name}）")
    return {
        "field_type": int(field_choice),
        "field_name": field_name,
    }


def select_qr_image_source():
    image_source = int(read_choice(
        {"1", "2"},
        prompt="选择二维码识别摄像头(1.d435i 2.odin1)：",
    ))
    source_name = "d435i" if image_source == 1 else "odin1"
    print(f"已选择二维码识别摄像头：{source_name}")
    return image_source


def print_simple_action_menu():
    print("---------------------------------------------------------------------------------------------")
    print("进入简单动作测试，注意这里所有部件动作不会自动复位")
    print("如果需要恢复则重新进入简单测试，并且根据选项自己慢慢恢复")
    print("1.移动测试")
    print("2.旋转测试")
    print("3.吸盘头旋转测试")
    print("4.吸盘吸取状态测试")
    print("5.机械臂姿态测试")
    print("6.武器头夹爪开合测试")
    print("7.武器头拉起放下测试")
    print("8.锁轮测试")
    print("0.返回最上级菜单")


def run_move_test(context):
    print("---------------------------------------------------------------------------------------------")
    print("开始移动测试")
    print(
        "注意，地图坐标系为，让机器正面面向梅林，机器面向的方向即为x轴正方向，"
        "x轴正方向逆时针旋转90度为y轴正方向，z轴数值向上"
    )
    print("x轴逆时针旋转为正，顺时针旋转为负")

    reference_choice = read_choice(
        {"1", "2"},
        prompt="请输入参考系(1为机器人参考系，2为武器头夹爪参考系)：",
    )
    target_x = read_finite_float("请输入目标x坐标：")
    target_y = read_finite_float("请输入目标y坐标：")
    target_yaw_deg = read_yaw_deg()
    move_speed = read_move_speed()
    stop_distance = read_stop_distance()

    reference_name = {
        "1": "机器人参考系",
        "2": "武器头夹爪参考系",
    }[reference_choice]
    print(
        f"以（{reference_name}），朝向（{target_yaw_deg:.2f}），"
        f"移动到（{target_x}，{target_y}）"
    )

    parameters = {
        "reference": "robot" if reference_choice == "1" else "weapon",
        "target_x": target_x,
        "target_y": target_y,
        "target_yaw_deg": target_yaw_deg,
        "move_speed": move_speed,
        "stop_distance": stop_distance,
    }
    if move_speed == 0:
        print("机器移动最大速度为0，本次不执行闭环移动")
        return {
            "parameters": parameters,
            "executed": False,
            "reason": "move_speed_is_zero",
        }

    result = module.move_to_des(
        sender=context.sender,
        position_runtime=context.position_runtime,
        odom_runtime=context.odom_runtime,
        x=target_x,
        y=target_y,
        target_deg=target_yaw_deg,
        v=move_speed,
        reference=parameters["reference"],
        stop_distance=stop_distance,
        total_timeout_sec=MOVE_TIMEOUT_SEC,
    )
    print("移动测试执行结果：")
    print(result)
    return {
        "parameters": parameters,
        "executed": True,
        "result": result,
    }


def run_rotation_test(context):
    print("---------------------------------------------------------------------------------------------")
    print("请输入旋转测试模式")
    print("1.细节旋转测试")
    print("2.简易旋转测试")
    rotation_mode = read_choice({"1", "2"})

    print("开始旋转测试")
    print(
        "注意，地图坐标系为，让机器正面面向梅林，机器面向的方向即为x轴正方向，"
        "x轴正方向逆时针旋转90度为y轴正方向，z轴数值向上"
    )
    print("x轴逆时针旋转为正，顺时针旋转为负")

    target_yaw_deg = read_rotation_target_yaw()
    if rotation_mode == "1":
        segment_step_deg = read_positive_float(
            "请输入大角度分段阈值（比如设置10,实际角度与目标角度相差大于10度时，"
            "会分成多个10度作为目标给下位机）："
        )
        tolerance_deg = read_rotation_tolerance()
        rotation_mode_name = "细节旋转测试"
    else:
        segment_step_deg = 120.0
        tolerance_deg = 1.0
        rotation_mode_name = "简易旋转测试"

    print(
        f"模式（{rotation_mode_name}），朝向（{target_yaw_deg:.2f}），"
        f"大角度分段阈值（{segment_step_deg}），"
        f"完成判断阈值（{tolerance_deg:.2f}）"
    )

    parameters = {
        "rotation_mode": int(rotation_mode),
        "rotation_mode_name": rotation_mode_name,
        "target_yaw_deg": target_yaw_deg,
        "segment_step_deg": segment_step_deg,
        "tolerance_deg": tolerance_deg,
    }
    rotation_started_at = time.monotonic()
    result = move.rotate_to_target_yaw_segmented(
        sender=context.sender,
        position_runtime=context.position_runtime,
        odom_runtime=context.odom_runtime,
        target_yaw_deg=target_yaw_deg,
        segment_step_deg=segment_step_deg,
        tolerance_deg=tolerance_deg,
    )
    rotation_duration_sec = time.monotonic() - rotation_started_at
    print("旋转测试执行结果：")
    print(result)
    print(f"当前旋转总时长：{rotation_duration_sec:.2f}秒")
    return {
        "parameters": parameters,
        "result": result,
        "rotation_duration_sec": float(rotation_duration_sec),
    }


def run_sucker_rotation_test(context):
    print("---------------------------------------------------------------------------------------------")
    print("开始吸盘头旋转测试")
    print(f"当前吸盘头角度：{kfs.current_sucker_deg}")
    print("1对应ch9>700,2对应200<ch9<500,3对应ch9<-700,4对应-500<ch9<-200")

    direction = read_choice(
        {"1", "2", "3", "4"},
        prompt="请输入吸盘头朝向：",
    )
    direction_result = {
        "1": {"angle_deg": 0, "ch9": 800},
        "2": {"angle_deg": 90, "ch9": 300},
        "3": {"angle_deg": -90, "ch9": -800},
        "4": {"angle_deg": 180, "ch9": -300},
    }[direction]

    print(
        f"吸盘头朝向（方向{direction}，{direction_result['angle_deg']}度），"
        f"对应ch9（{direction_result['ch9']}）"
    )

    result = kfs.sucker_to_direction(context.sender, int(direction))
    print("吸盘头旋转测试执行结果：")
    print(result)
    return {
        "direction": int(direction),
        "angle_deg": direction_result["angle_deg"],
        "ch9": direction_result["ch9"],
        "result": result,
    }


def run_sucker_suction_test(context):
    print("---------------------------------------------------------------------------------------------")
    print("开始吸盘吸取状态测试")

    cylinder_select = read_choice(
        {"0", "1", "2"},
        prompt=(
            "请输入控制气缸（0双气缸，1我约定为ch9>700时向前的那个气缸，"
            "2为另一个）："
        ),
    )
    suction_action = read_choice(
        {"1", "2"},
        prompt="请输入吸取（1）或释放（2）：",
    )

    cylinder_name = {
        "0": "双气缸",
        "1": "ch9>700时向前的气缸",
        "2": "另一个气缸",
    }[cylinder_select]
    action_result = {
        "1": {"name": "吸取", "ch4_edge": "1→3", "suction_on": True},
        "2": {"name": "释放", "ch4_edge": "3→1", "suction_on": False},
    }[suction_action]

    print(
        f"控制气缸（{cylinder_name}），执行（{action_result['name']}），"
        f"ch4边沿（{action_result['ch4_edge']}）"
    )

    cylinder_result = kfs.sucker_select_cylinder(
        context.sender,
        int(cylinder_select),
    )
    suction_result = module.set_kfs_suction(
        sender=context.sender,
        suction_on=action_result["suction_on"],
    )
    print("吸盘吸取状态测试执行结果：")
    print({
        "cylinder_result": cylinder_result,
        "suction_result": suction_result,
    })
    return {
        "cylinder_select": int(cylinder_select),
        "suction_on": action_result["suction_on"],
        "ch4_edge": action_result["ch4_edge"],
        "cylinder_result": cylinder_result,
        "suction_result": suction_result,
    }


def run_kfs_pose_test(context):
    print("---------------------------------------------------------------------------------------------")
    print("开始机械臂姿态测试")
    print("0原态，1高位吸取，2低位吸取，3.过渡态，4.放置态，5.侧吸态，6.第三层KFS姿态")

    pose_id = read_choice(
        {"0", "1", "2", "3", "4", "5", "6"},
        prompt="请输入目标姿态：",
    )
    pose_name = {
        "0": "原态",
        "1": "高位吸取",
        "2": "低位吸取",
        "3": "过渡态",
        "4": "放置态",
        "5": "侧吸态",
        "6": "第三层KFS姿态",
    }[pose_id]

    print(f"目标姿态（{pose_id}，{pose_name}）")

    pose_actions = {
        "0": kfs.kfs_zero_return_pose,
        "1": lambda sender: kfs.kfs_grab_pose(sender, pose_id=1),
        "2": lambda sender: kfs.kfs_grab_pose(sender, pose_id=2),
        "3": kfs.kfs_transition_pose,
        "4": kfs.place_kfs_pose,
        "5": kfs.kfs_side_pose,
        "6": kfs.place_3rd_kfs_pose,
    }
    result = pose_actions[pose_id](context.sender)
    print("机械臂姿态测试执行结果：")
    print(result)
    return {
        "pose_id": int(pose_id),
        "pose_name": pose_name,
        "result": result,
    }


def run_weapon_gripper_test(context):
    print("---------------------------------------------------------------------------------------------")
    print("开始武器头夹爪开合测试")

    gripper_action = read_choice(
        {"0", "1"},
        prompt="请输入开（1）或合（0）:",
    )
    action_result = {
        "0": {"name": "合", "ch4_edge": "3→1"},
        "1": {"name": "开", "ch4_edge": "1→3"},
    }[gripper_action]

    print(
        f"武器头夹爪（{action_result['name']}），"
        f"ch4边沿（{action_result['ch4_edge']}）"
    )

    result = (
        weapon.weapon_loose(context.sender)
        if gripper_action == "1"
        else weapon.weapon_seize(context.sender)
    )
    print("武器头夹爪开合测试执行结果：")
    print(result)
    return {
        "open": gripper_action == "1",
        "action_name": action_result["name"],
        "ch4_edge": action_result["ch4_edge"],
        "result": result,
    }


def run_weapon_lift_test(context):
    print("---------------------------------------------------------------------------------------------")
    print("开始武器头夹爪拉起放下测试")

    lift_action = read_choice(
        {"1", "2"},
        prompt="请输入拉起（1）或放下（2）：",
    )
    action_result = {
        "1": {"name": "拉起", "ch1_value": 100},
        "2": {"name": "放下", "ch1_value": -100},
    }[lift_action]

    print(
        f"武器头夹爪（{action_result['name']}），"
        f"ch1触发值（{action_result['ch1_value']}）"
    )

    result = (
        weapon.weapon_up(context.sender)
        if lift_action == "1"
        else weapon.weapon_down(context.sender)
    )
    print("武器头夹爪拉起放下测试执行结果：")
    print(result)
    return {
        "lift_up": lift_action == "1",
        "action_name": action_result["name"],
        "ch1_value": action_result["ch1_value"],
        "result": result,
    }


def run_wheel_lock_test(context):
    print("---------------------------------------------------------------------------------------------")
    print("开始锁轮测试")

    lock_action = read_choice(
        {"1", "2"},
        prompt="请输入锁轮（1）或松开（2）：",
    )
    if lock_action == "1":
        action_name = "锁轮"
        result = move.lock_wheel(context.sender)
    else:
        action_name = "松开"
        result = move.unlock_wheel(context.sender)

    print("锁轮测试执行结果：")
    print(result)
    return {
        "locked": lock_action == "1",
        "action_name": action_name,
        "result": result,
    }


def run_restore_initial_state(context):
    global current_stair_id

    print("---------------------------------------------------------------------------------------------")
    print("恢复至初态")
    confirm = read_choice(
        {"0", "1"},
        prompt="确认恢复（0返回，1确认）：",
    )
    if confirm == "0":
        print("已取消恢复，返回上一级")
        return {
            "completed": False,
            "cancelled": True,
        }

    previous_current_stair_id = current_stair_id
    previous_suck_count = int(kfs.suck_count)

    unlock_result = move.unlock_wheel(context.sender)
    weapon_loose_result = weapon.weapon_loose(context.sender)
    weapon_down_result = weapon.weapon_down(context.sender)

    kfs.suck_count = 1
    sucker_rotation_result = kfs.sucker_0deg(context.sender)
    cylinder_selection_result = kfs.sucker_select_both(context.sender)
    suction_off_result = module.set_kfs_suction(
        sender=context.sender,
        suction_on=False,
        pose_id=0,
    )

    current_stair_id = 0
    result = {
        "completed": True,
        "cancelled": False,
        "previous_current_stair_id": previous_current_stair_id,
        "current_stair_id": current_stair_id,
        "previous_suck_count": previous_suck_count,
        "suck_count": int(kfs.suck_count),
        "unlock_result": unlock_result,
        "weapon_loose_result": weapon_loose_result,
        "weapon_down_result": weapon_down_result,
        "sucker_rotation_result": sucker_rotation_result,
        "cylinder_selection_result": cylinder_selection_result,
        "suction_off_result": suction_off_result,
    }
    print("恢复至初态执行结果：")
    print(result)
    return result


def run_simple_action_menu(context):
    while True:
        print_simple_action_menu()
        choice = read_choice(range(9))
        if choice == "0":
            return
        if choice == "1":
            run_move_test(context)
            continue
        if choice == "2":
            run_rotation_test(context)
            continue
        if choice == "3":
            run_sucker_rotation_test(context)
            continue
        if choice == "4":
            run_sucker_suction_test(context)
            continue
        if choice == "5":
            run_kfs_pose_test(context)
            continue
        if choice == "6":
            run_weapon_gripper_test(context)
            continue
        if choice == "7":
            run_weapon_lift_test(context)
            continue
        if choice == "8":
            run_wheel_lock_test(context)
            continue


def print_complex_action_menu():
    print("进入复杂动作测试")
    print("1.初始区域测试")
    print("2.梅林区域测试")
    print("3.九宫格区域测试")
    print("4.完整流程测试")


def run_initial_region_test(context):
    print("---------------------------------------------------------------------------------------------")
    print("初始区域测试")
    print("逻辑为：先夹取目标武器头，然后后退并转向R1方向，锁轮等待")

    weapon_id = read_integer("请输入目标武器头编号：")
    move_speed = read_move_speed()

    fetch_result = module.fetch_weapon(
        sender=context.sender,
        position_runtime=context.position_runtime,
        odom_runtime=context.odom_runtime,
        weapon_id=weapon_id,
        v=move_speed,
    )

    lock_result = None
    if fetch_result.get("completed", False):
        lock_result = move.lock_wheel(context.sender)
        print(
            "\033[31m"
            "若要解锁轮子，释放武器头夹爪，请前往简单测试解锁"
            "\033[0m"
        )

    print("初始区域测试执行结果：")
    print({
        "fetch_result": fetch_result,
        "lock_result": lock_result,
    })
    return {
        "weapon_id": int(weapon_id),
        "move_speed": int(move_speed),
        "fetch_result": fetch_result,
        "lock_result": lock_result,
        "completed": bool(
            fetch_result.get("completed", False)
            and lock_result is not None
            and lock_result.get("completed", False)
        ),
    }


def print_meilin_region_test_menu():
    print("---------------------------------------------------------------------------------------------")
    print("梅林区域测试")
    print("此处所有动作只能逻辑连贯的执行")
    print("1.梅林动作测试准备（开始测试时，务必执行一次这个）")
    print("2.上下楼梯测试")
    print("3.方块吸取测试")
    print("4.侧吸测试")
    print("5.边缘微调测试")
    print("6.完整梅林测试")


def is_valid_stair_id(stair_id):
    try:
        stair_id = int(stair_id)
        valid_stair_ids = {
            int(stair_row[0])
            for stair_row in module.get_stair_matrix()
        }
    except (TypeError, ValueError, IndexError):
        return False
    return stair_id in valid_stair_ids


def run_meilin_test_prepare(context):
    global current_stair_id

    stair_id = -1
    target_direction = 1
    target_yaw_deg = tools.direction_int_to_yaw_deg(target_direction)
    target_x, target_y = module.get_stair_xy(stair_id)
    print(
        f"梅林动作测试准备：朝向（{target_yaw_deg:.2f}），"
        f"移动到台阶（{stair_id}）坐标（{target_x}，{target_y}）"
    )
    result = module.meilin_prepare(
        sender=context.sender,
        position_runtime=context.position_runtime,
        odom_runtime=context.odom_runtime,
        total_timeout_sec=MOVE_TIMEOUT_SEC,
        reference="robot",
    )
    stair_id = int(result["stair_id"])
    target_x = float(result["target_x"])
    target_y = float(result["target_y"])
    target_yaw_deg = float(result["target_yaw_deg"])
    print("梅林动作测试准备执行结果：")
    print(result)
    if result.get("completed", False):
        current_stair_id = stair_id
    return {
        "stair_id": stair_id,
        "target_x": float(target_x),
        "target_y": float(target_y),
        "target_yaw_deg": target_yaw_deg,
        "result": result,
    }


def run_stair_transition_test(context):
    global current_stair_id

    print("---------------------------------------------------------------------------------------------")
    print("上下楼梯测试")

    while True:
        if is_valid_stair_id(current_stair_id):
            from_pos = int(current_stair_id)
        else:
            from_pos = read_integer("请输入当前台阶编号：")
            current_stair_id = int(from_pos)
        to_pos = read_integer("请输入目标台阶编号：")
        move_dir = tools.stair_id_to_direction(
            from_pos,
            to_pos,
            exit_on_error=False,
        )
        if move_dir in (1, 2, 3, 4):
            break
        print("台阶不相邻，请输入正确的逻辑编号")

    action_row = [from_pos, to_pos, move_dir, 1, 0]
    print(f"生成上下楼梯动作行：{action_row}")
    result = module.execute_action_row(
        sender=context.sender,
        position_runtime=context.position_runtime,
        odom_runtime=context.odom_runtime,
        action_row=action_row,
        final_direction=move_dir,
    )
    print("上下楼梯测试执行结果：")
    print(result)
    if result.get("completed", False):
        current_stair_id = int(to_pos)
    return {
        "action_row": action_row,
        "result": result,
    }


def run_kfs_fetch_test(context):
    global current_stair_id

    print("---------------------------------------------------------------------------------------------")
    print("方块吸取测试")

    while True:
        if is_valid_stair_id(current_stair_id):
            from_pos = int(current_stair_id)
        else:
            from_pos = read_integer("请输入当前台阶编号：")
            current_stair_id = int(from_pos)
        to_pos = read_integer("请输入目标台阶编号：")
        move_dir = tools.stair_id_to_direction(
            from_pos,
            to_pos,
            exit_on_error=False,
        )
        if move_dir in (1, 2, 3, 4):
            break
        print("台阶不相邻，请输入正确的逻辑编号")

    action_row = [from_pos, to_pos, move_dir, 0, 1]
    print(f"生成方块吸取动作行：{action_row}")
    result = module.execute_action_row(
        sender=context.sender,
        position_runtime=context.position_runtime,
        odom_runtime=context.odom_runtime,
        action_row=action_row,
        final_direction=move_dir,
    )
    print("方块吸取测试执行结果：")
    print(result)
    return {
        "action_row": action_row,
        "result": result,
    }


def run_side_suck_test(context):
    print("---------------------------------------------------------------------------------------------")
    print("侧吸测试")

    if int(current_stair_id) != -1:
        print(
            "\033[31m"
            "非合法位置，请先移动到入口前的编号-1台阶"
            "\033[0m"
        )
        return {
            "completed": False,
            "executed": False,
            "reason": "current_stair_is_not_minus_one",
            "current_stair_id": int(current_stair_id),
        }

    to_pose = int(read_choice(
        {"1", "3"},
        prompt="请输入要侧吸的方块所在台阶编号（1或3）：",
    ))
    action_row = [-1, to_pose, 1, 0, 1]
    print(f"生成侧吸动作行：{action_row}")
    result = module.execute_action_row(
        sender=context.sender,
        position_runtime=context.position_runtime,
        odom_runtime=context.odom_runtime,
        action_row=action_row,
        final_direction=1,
    )
    print("侧吸测试执行结果：")
    print(result)
    return {
        "completed": bool(result.get("completed", False)),
        "executed": True,
        "current_stair_id": int(current_stair_id),
        "to_pose": int(to_pose),
        "action_row": action_row,
        "result": result,
    }


def run_edge_adjust_test(context):
    print("---------------------------------------------------------------------------------------------")
    print("边缘微调测试")

    if not is_valid_stair_id(current_stair_id):
        print("位置错误")
        return {
            "completed": False,
            "executed": False,
            "reason": "invalid_current_stair",
            "current_stair_id": int(current_stair_id),
        }

    direction = int(read_choice(
        {"1", "2", "3", "4"},
        prompt="请输入微调方向（1：0 2：90 3：-90 4：180）：",
    ))
    adjust_distance = read_adjust_distance()
    result = module.adjust_position(
        sender=context.sender,
        position_runtime=context.position_runtime,
        odom_runtime=context.odom_runtime,
        move_type=1,
        direction=direction,
        stair_id=current_stair_id,
        height_relation=2,
        adjust_distance=adjust_distance,
    )
    print("边缘微调测试执行结果：")
    print(result)
    return {
        "completed": bool(result.get("completed", False)),
        "executed": True,
        "current_stair_id": int(current_stair_id),
        "direction": int(direction),
        "adjust_distance": float(adjust_distance),
        "result": result,
    }


def run_complete_meilin_test(context):
    global current_stair_id

    print("---------------------------------------------------------------------------------------------")
    print("完整梅林测试")
    competition_mode = int(read_choice(
        {"1", "2"},
        prompt="请输入当前规则（1）挑战赛221（2）对抗赛331 ：",
    ))
    competition_result = race.configure_competition_mode(competition_mode)
    print("等待有效二维码输入中....")

    action_matrix_queue = queue.Queue()
    visual_lock = threading.Lock()
    scanner = None
    try:
        scanner = challenge_lib.start_background_qr_scanner(
            result_queue=action_matrix_queue,
            stable_frame_count=QR_STABLE_FRAME_COUNT,
            show_window=False,
            stop_after_success=True,
            put_action_matrix_only=True,
            running_lock=visual_lock,
            image_source=context.image_source,
        )
        time.sleep(0.5)
        with visual_lock:
            if action_matrix_queue.empty():
                scanner_error = getattr(scanner, "last_error", None)
                if scanner_error is not None:
                    print(f"二维码识别或路径规划失败：{scanner_error!r}")
                    failure_reason = "qr_scanner_error"
                else:
                    print("未获取到有效的完整动作矩阵")
                    failure_reason = "action_matrix_queue_empty"
                return {
                    "completed": False,
                    "executed": False,
                    "reason": failure_reason,
                    "competition_result": competition_result,
                }

            action_matrix = action_matrix_queue.get()
            print("完整动作矩阵：")
            print(action_matrix)
            print("二维码识别完成，先打开 weapon 夹爪并等待 5s")
            weapon_loose_result = weapon.weapon_loose(context.sender)
            print("weapon 夹爪打开结果：")
            print(weapon_loose_result)
            time.sleep(5.0)
            if not weapon_loose_result.get("completed", False):
                return {
                    "completed": False,
                    "executed": False,
                    "reason": "weapon_loose_failed",
                    "competition_result": competition_result,
                    "action_matrix": action_matrix,
                    "weapon_loose_result": weapon_loose_result,
                    "current_stair_id": int(current_stair_id),
                }

            matrix_result = module.execute_action_matrix(
                sender=context.sender,
                position_runtime=context.position_runtime,
                odom_runtime=context.odom_runtime,
                action_matrix=action_matrix,
                final_direction=1,
            )

        print("完整梅林测试执行结果：")
        print(matrix_result)
        if matrix_result.get("completed", False) and len(action_matrix) > 0:
            current_stair_id = int(action_matrix[-1][1])
        return {
            "completed": bool(matrix_result.get("completed", False)),
            "executed": True,
            "competition_result": competition_result,
            "action_matrix": action_matrix,
            "weapon_loose_result": weapon_loose_result,
            "matrix_result": matrix_result,
            "current_stair_id": int(current_stair_id),
        }
    finally:
        if scanner is not None:
            scanner.stop()
            scanner.join(timeout=1.0)


def run_meilin_region_test_menu(context):
    print_meilin_region_test_menu()
    choice = read_choice({"1", "2", "3", "4", "5", "6"})
    if choice == "1":
        return run_meilin_test_prepare(context)
    if choice == "2":
        return run_stair_transition_test(context)
    if choice == "3":
        return run_kfs_fetch_test(context)
    if choice == "4":
        return run_side_suck_test(context)
    if choice == "5":
        return run_edge_adjust_test(context)
    if choice == "6":
        return run_complete_meilin_test(context)

    print("该测试分支暂未实现")
    return None


def print_nine_grid_region_test_menu():
    print("---------------------------------------------------------------------------------------------")
    print("九宫格区域测试区域测试")
    print("1.放二层箱子测试")
    print("2.上R1测试（目前需要R2前端人为对准R1后进行）")
    print("3.R1上释放三层kfs测试")
    print("4.完整九宫格区域测试")


def run_place_second_level_box_test(context):
    print("---------------------------------------------------------------------------------------------")
    print("二层箱子测试")

    if int(kfs.suck_count) not in (2, 3):
        print("目前没有吸取箱子")
        return {
            "completed": False,
            "executed": False,
            "reason": "no_kfs_loaded",
            "suck_count": int(kfs.suck_count),
        }

    place_pose_result = kfs.place_kfs_pose(context.sender)
    if not place_pose_result.get("completed", False):
        return {
            "completed": False,
            "executed": True,
            "failed_step": "place_kfs_pose",
            "place_pose_result": place_pose_result,
        }
    time.sleep(1.0)

    release_pose_result = kfs.sucker_release_pose(context.sender)
    if not release_pose_result.get("completed", False):
        return {
            "completed": False,
            "executed": True,
            "failed_step": "sucker_release_pose",
            "place_pose_result": place_pose_result,
            "release_pose_result": release_pose_result,
        }

    raw_target_x = -0.950
    raw_target_y = -4.46
    target_yaw_deg = 180.0
    target_x, target_y = tools.deg180_correction(raw_target_x, raw_target_y)
    move_result = module.move_to_des(
        sender=context.sender,
        position_runtime=context.position_runtime,
        odom_runtime=context.odom_runtime,
        x=target_x,
        y=target_y,
        target_deg=target_yaw_deg,
        total_timeout_sec=MOVE_TIMEOUT_SEC,
        reference="robot",
    )
    if move_result is None or not move_result.get("completed", False):
        print("移动到二层箱子放置位置失败，不执行释放")
        return {
            "completed": False,
            "executed": True,
            "failed_step": "move_to_second_level_target",
            "place_pose_result": place_pose_result,
            "release_pose_result": release_pose_result,
            "move_target": {
                "raw_x": raw_target_x,
                "raw_y": raw_target_y,
                "x": target_x,
                "y": target_y,
                "yaw_deg": target_yaw_deg,
                "correction_yaw_deg": 180.0,
            },
            "move_result": move_result,
        }

    release_result = kfs.release_kfs(context.sender)
    completed = bool(release_result.get("completed", False))
    result = {
        "completed": completed,
        "executed": True,
        "failed_step": None if completed else "release_kfs",
        "place_pose_result": place_pose_result,
        "release_pose_result": release_pose_result,
        "move_target": {
            "raw_x": raw_target_x,
            "raw_y": raw_target_y,
            "x": target_x,
            "y": target_y,
            "yaw_deg": target_yaw_deg,
            "correction_yaw_deg": 180.0,
        },
        "move_result": move_result,
        "release_result": release_result,
        "suck_count": int(kfs.suck_count),
    }
    print("二层箱子测试执行结果：")
    print(result)
    return result


def run_climb_r1_test(context):
    print("---------------------------------------------------------------------------------------------")
    print("上R1测试")
    print("机器会以目标角度移动到R1climb点，然后执行上R1")
    read_choice({"1"}, prompt="请回复1开始测试")

    target_yaw_deg = 180.0
    coordinate_name = (
        "R1climb_blue" if position_backend.is_blue_field() else "R1climb_red"
    )
    current_position_lib = position_resource.get_position_lib()
    r1climb_target = move._get_corrected_battlefield_coordinate(
        current_position_lib,
        coordinate_name,
    )

    move_result = module.move_to_des(
        sender=context.sender,
        position_runtime=context.position_runtime,
        odom_runtime=context.odom_runtime,
        x=r1climb_target["x"],
        y=r1climb_target["y"],
        target_deg=target_yaw_deg,
        total_timeout_sec=MOVE_TIMEOUT_SEC,
        reference="robot",
    )
    if move_result is None or not move_result.get("completed", False):
        result = {
            "completed": False,
            "executed": True,
            "failed_step": "move_to_R1climb",
            "target_yaw_deg": float(target_yaw_deg),
            "r1climb_target": {
                "coordinate_name": coordinate_name,
                "raw_x": float(r1climb_target["raw_x"]),
                "raw_y": float(r1climb_target["raw_y"]),
                "x": float(r1climb_target["x"]),
                "y": float(r1climb_target["y"]),
            },
            "move_result": move_result,
            "climb_result": None,
        }
        print("上R1测试执行结果：")
        print(result)
        return result

    climb_result = module.climb_R1(
        sender=context.sender,
        position_runtime=context.position_runtime,
        odom_runtime=context.odom_runtime,
        target_yaw_deg=target_yaw_deg,
    )
    result = {
        "completed": bool(climb_result.get("completed", False)),
        "executed": True,
        "failed_step": None if climb_result.get("completed", False) else "climb_R1",
        "target_yaw_deg": float(target_yaw_deg),
        "r1climb_target": {
            "coordinate_name": coordinate_name,
            "raw_x": float(r1climb_target["raw_x"]),
            "raw_y": float(r1climb_target["raw_y"]),
            "x": float(r1climb_target["x"]),
            "y": float(r1climb_target["y"]),
        },
        "move_result": move_result,
        "climb_result": climb_result,
    }
    print("上R1测试执行结果：")
    print(result)
    return result


def run_release_third_level_kfs_test(context):
    print("---------------------------------------------------------------------------------------------")
    print("R1上释放三层kfs测试")

    current_suck_count = int(kfs.suck_count)
    if not 2 <= current_suck_count <= 3:
        print("目前没有吸取箱子")
        return {
            "completed": False,
            "executed": False,
            "reason": "invalid_suck_count",
            "suck_count": current_suck_count,
        }

    print("不一定在R1上进行测试")
    print("此测试效果为持续检测下降，当下降高度大于2cm时释放kfs")
    timeout_enabled = read_choice(
        {"1", "2"},
        prompt="是否启用Z下降检测超时（1启用，2关闭）：",
    ) == "1"
    timeout_sec = None
    if timeout_enabled:
        timeout_sec = read_positive_float_with_default(
            "请输入Z下降检测超时时间(秒，直接回车默认999)：",
            999.0,
        )
    read_choice({"1"}, prompt="请输入1开始测试：")

    place_pose_result = kfs.place_kfs_pose(context.sender)
    if not place_pose_result.get("completed", False):
        return {
            "completed": False,
            "executed": True,
            "failed_step": "place_kfs_pose",
            "place_pose_result": place_pose_result,
        }
    time.sleep(1.0)

    release_pose_result = kfs.sucker_release_pose(context.sender)
    if not release_pose_result.get("completed", False):
        return {
            "completed": False,
            "executed": True,
            "failed_step": "sucker_release_pose",
            "place_pose_result": place_pose_result,
            "release_pose_result": release_pose_result,
        }

    release_result = kfs.place_3rd_kfs(
        sender=context.sender,
        position_runtime=context.position_runtime,
        timeout_enabled=timeout_enabled,
        **({} if timeout_sec is None else {"timeout_sec": timeout_sec}),
    )
    result = {
        "completed": bool(release_result.get("completed", False)),
        "executed": True,
        "failed_step": release_result.get("failed_step"),
        "timeout_enabled": bool(timeout_enabled),
        "timeout_sec": timeout_sec,
        "place_pose_result": place_pose_result,
        "release_pose_result": release_pose_result,
        "release_result": release_result,
        "suck_count": int(kfs.suck_count),
    }
    print("R1上释放三层kfs测试执行结果：")
    print(result)
    return result


def run_nine_grid_region_test_menu(context):
    print_nine_grid_region_test_menu()
    choice = read_choice({"1", "2", "3", "4"})
    if choice == "1":
        return run_place_second_level_box_test(context)
    if choice == "2":
        return run_climb_r1_test(context)
    if choice == "3":
        return run_release_third_level_kfs_test(context)

    print("该测试分支暂未实现")
    return {
        "completed": False,
        "executed": False,
        "choice": int(choice),
        "reason": "not_implemented",
    }


def run_complex_action_menu(context):
    print_complex_action_menu()
    choice = read_choice({"1", "2", "3", "4"})
    if choice == "1":
        run_initial_region_test(context)
        return
    if choice == "2":
        run_meilin_region_test_menu(context)
        return
    if choice == "3":
        run_nine_grid_region_test_menu(context)
        return

    # 具体复杂动作将在对应分支中继续实现。
    print("该测试分支暂未实现")


def main():
    context = RuntimeContext()
    try:
        (
            context.sender,
            _,
            context.flag_node,
            context.flag_thread,
            context.flag_stop_event,
        ) = module.init(lidar_type=LIDAR_TYPE)
        print("初始化成功")

        field_result = select_field_type()
        position_backend.set_field_type(field_result["field_type"])
        print(
            "场地半场已写入位置后端："
            f"{position_backend.get_field_type()}（{field_result['field_name']}）"
        )
        context.image_source = select_qr_image_source()

        context.position_runtime = module.start_position_thread(context.sender)
        context.odom_runtime = module.start_odometry_thread(topic=module.ODOM_TOPIC)
        if wait_runtime_ready(context):
            print("位姿与里程计资源已就绪")
        else:
            print("警告：位姿或里程计在8秒内未就绪，移动和旋转测试可能失败")

        while True:
            print_main_menu()
            choice = read_choice({"1", "2", "3"})
            if choice == "1":
                run_simple_action_menu(context)
            elif choice == "2":
                run_complex_action_menu(context)
            elif choice == "3":
                run_restore_initial_state(context)
    finally:
        cleanup_runtime(context)


if __name__ == "__main__":
    try:
        main()
    except (KeyboardInterrupt, EOFError):
        print("\n测试已退出")
