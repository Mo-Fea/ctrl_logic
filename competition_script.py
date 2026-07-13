import time
import math

from lib2 import compete_logic
from lib2 import kfs
from lib2 import module
from lib2 import position_backend
from utils import race


final_strategy = 1
R1 = 2
R2 = 2
kfs_suck = 2
weapon_id = 4
WEAPON_Y_CORRECTION = 0.0
current_rule = 1
meilin_situration = "000000000000"
sender = None
get_flag = None
flag_node = None
flag_thread = None
flag_stop_event = None
position_runtime = None
odom_runtime = None

ERROR_INPUT_TEXT = "错误输入，请按照提示重新输入"
YELLOW = "\033[33m"
RESET = "\033[0m"
SEPARATOR_TEXT = "---------------------------------------------------------"


def print_separator():
    print(f"{YELLOW}{SEPARATOR_TEXT}{RESET}")


def print_back_option():
    print("0.返回上一级")


def read_int(prompt, valid_values=None, min_value=None, max_value=None):
    while True:
        raw = input(prompt).strip()
        try:
            value = int(raw)
        except ValueError:
            print(ERROR_INPUT_TEXT)
            continue

        if valid_values is not None and value not in valid_values:
            print(ERROR_INPUT_TEXT)
            continue
        if min_value is not None and value < min_value:
            print(ERROR_INPUT_TEXT)
            continue
        if max_value is not None and value > max_value:
            print(ERROR_INPUT_TEXT)
            continue
        return value


def read_float(prompt):
    while True:
        raw = input(prompt).strip()
        try:
            value = float(raw)
        except ValueError:
            print(ERROR_INPUT_TEXT)
            continue
        if not math.isfinite(value):
            print(ERROR_INPUT_TEXT)
            continue
        return value


def read_qr_payload(prompt):
    while True:
        value = input(prompt).strip()
        if len(value) == 12 and all(char in "0123" for char in value):
            return value
        print(ERROR_INPUT_TEXT)


def wait_start(prompt="请输入1开始执行：", allow_back=True):
    valid_values = (0, 1) if allow_back else (1,)
    value = read_int(prompt, valid_values=valid_values)
    return value == 1


def set_field_by_input():
    field_type = read_int("请输入当前红蓝半场（1.红 2.蓝）：", valid_values=(1, 2))
    if field_type == 1:
        position_backend.set_field_type(position_backend.FIELD_TYPE_RED)
    else:
        position_backend.set_field_type(position_backend.FIELD_TYPE_BLUE)
    return field_type


def set_rule_by_input():
    global current_rule
    global R1
    global R2

    current_rule = read_int("请输入当前赛制(1.挑战赛 2.对抗赛)：", valid_values=(1, 2))
    if current_rule == 1:
        R1 = 2
        R2 = 2
    else:
        R1 = 3
        R2 = 3
    return current_rule


def set_final_strategy_by_input():
    global final_strategy

    final_strategy = read_int(
        "请输入当前决定的获胜逻辑（1.大胜 2.190分）：",
        valid_values=(1, 2),
    )
    return final_strategy


def read_weapon_id():
    global weapon_id

    weapon_id = read_int("请输入需要抓取的武器头编号（1-6）：", min_value=1, max_value=6)
    return weapon_id


def print_current_config(action_name):
    print(f"准备执行：{action_name}")
    print(
        "当前配置："
        f"final_strategy={final_strategy}, "
        f"R1={R1}, "
        f"R2={R2}, "
        f"kfs_suck={kfs_suck}, "
        f"weapon_id={weapon_id}, "
        f"weapon_y_correction={WEAPON_Y_CORRECTION}, "
        f"current_rule={current_rule}, "
        f"meilin_situration={meilin_situration}"
    )


def initialize_runtime():
    global sender
    global get_flag
    global flag_node
    global flag_thread
    global flag_stop_event
    global position_runtime
    global odom_runtime

    (
        sender,
        get_flag,
        flag_node,
        flag_thread,
        flag_stop_event,
    ) = module.init(lidar_type=position_backend.LIDAR_TYPE_ODIN)
    position_runtime = module.start_position_thread(sender)
    odom_runtime = module.start_odometry_thread(topic=module.ODOM_TOPIC)
    print("初始化成功")


def get_rigion_3_final_strategy():
    if final_strategy == 1:
        return 2
    if final_strategy == 2:
        return 1
    raise ValueError(f"final_strategy must be 1 or 2, got {final_strategy}")


def print_flow_failure(region_name, result):
    print(
        f"{region_name}执行失败："
        f"failed_step={result.get('failed_step')}"
    )


def run_full_confrontation_match():
    print_current_config("完整比赛/对抗赛")
    config_result = race.configure_kfs_counts(
        r1_count=R1,
        r2_count=R2,
        required_r2_pickup_count=kfs_suck,
    )
    print(f"规划数量配置已更新：{config_result}")

    region_1_result = compete_logic.rigion_1(
        sender=sender,
        position_runtime=position_runtime,
        odom_runtime=odom_runtime,
        weapon_id=weapon_id,
        fetch_weapon_y_correction=WEAPON_Y_CORRECTION,
    )
    if not region_1_result.get("completed", False):
        print_flow_failure("区域1", region_1_result)
        return {
            "completed": False,
            "failed_region": "rigion_1",
            "region_1_result": region_1_result,
        }
    print("区域1执行完成")

    replan_result = None
    if region_1_result.get("action_matrix") is None:
        print("区域1未获取到二维码动作矩阵，使用当前 meilin_situration 重规划")
        replan_result = compete_logic.rigion_2_retry_plan(
            r1_count=R1,
            r2_count=R2,
            required_r2_pickup_count=kfs_suck,
            qr_string=meilin_situration,
        )
        if not replan_result.get("completed", False):
            print_flow_failure("梅林重规划", replan_result)
            return {
                "completed": False,
                "failed_region": "rigion_2_retry_plan",
                "region_1_result": region_1_result,
                "replan_result": replan_result,
                "region_2_result": None,
            }
        print("梅林重规划执行完成")

    region_2_result = compete_logic.rigion_2(
        sender=sender,
        position_runtime=position_runtime,
        odom_runtime=odom_runtime,
        queue_timeout_sec=1.0,
    )
    if not region_2_result.get("completed", False):
        print_flow_failure("区域2", region_2_result)
        return {
            "completed": False,
            "failed_region": "rigion_2",
            "region_1_result": region_1_result,
            "replan_result": replan_result,
            "region_2_result": region_2_result,
        }
    print("区域2执行完成")

    region_3_result = compete_logic.rigion_3(
        sender=sender,
        position_runtime=position_runtime,
        odom_runtime=odom_runtime,
        final_strategy=get_rigion_3_final_strategy(),
    )
    if not region_3_result.get("completed", False):
        print_flow_failure("区域3", region_3_result)
        return {
            "completed": False,
            "failed_region": "rigion_3",
            "region_1_result": region_1_result,
            "replan_result": replan_result,
            "region_2_result": region_2_result,
            "region_3_result": region_3_result,
        }
    print("完整对抗逻辑执行完成")
    return {
        "completed": True,
        "failed_region": None,
        "region_1_result": region_1_result,
        "replan_result": replan_result,
        "region_2_result": region_2_result,
        "region_3_result": region_3_result,
    }


def run_full_challenge_wulin_match():
    print_current_config("完整比赛/挑战赛/武林区域")
    config_result = race.configure_kfs_counts(
        r1_count=R1,
        r2_count=R2,
        required_r2_pickup_count=kfs_suck,
    )
    print(f"规划数量配置已更新：{config_result}")

    region_1_result = compete_logic.rigion_1(
        sender=sender,
        position_runtime=position_runtime,
        odom_runtime=odom_runtime,
        weapon_id=weapon_id,
        fetch_weapon_y_correction=WEAPON_Y_CORRECTION,
    )
    if not region_1_result.get("completed", False):
        print_flow_failure("区域1", region_1_result)
        return {
            "completed": False,
            "failed_region": "rigion_1",
            "region_1_result": region_1_result,
        }
    print("区域1执行完成")

    region_2_result = compete_logic.rigion_2(
        sender=sender,
        position_runtime=position_runtime,
        odom_runtime=odom_runtime,
        queue_timeout_sec=1.0,
    )
    if not region_2_result.get("completed", False):
        print_flow_failure("区域2", region_2_result)
        return {
            "completed": False,
            "failed_region": "rigion_2",
            "region_1_result": region_1_result,
            "region_2_result": region_2_result,
        }
    print("完整比赛/挑战赛/武林区域执行完成")
    return {
        "completed": True,
        "failed_region": None,
        "region_1_result": region_1_result,
        "region_2_result": region_2_result,
    }


def run_full_challenge_jiugong_match(
    wait_before_region_3=False,
    kfs_preparation_result=None,
    column=None,
    extra_needed=None,
):
    print_current_config("完整比赛/挑战赛/九宫区域")

    prepared_here = kfs_preparation_result is None
    if kfs_preparation_result is None:
        kfs_preparation_result = kfs.kfs_suck_preparation(sender=sender, count=kfs_suck)
    if not kfs_preparation_result.get("completed", False):
        print("KFS吸取准备执行失败")
        return {
            "completed": False,
            "failed_region": "kfs_suck_preparation",
            "kfs_preparation_result": kfs_preparation_result,
            "region_3_result": None,
        }
    if prepared_here:
        print("KFS吸取准备执行完成")

    if wait_before_region_3:
        print("等待10秒")
        time.sleep(10.0)

    region_3_result = compete_logic.rigion3_challenge(
        sender=sender,
        position_runtime=position_runtime,
        odom_runtime=odom_runtime,
        final_strategy=get_rigion_3_final_strategy(),
        column=column,
        extra_needed=extra_needed,
    )
    if not region_3_result.get("completed", False):
        print_flow_failure("区域3", region_3_result)
        return {
            "completed": False,
            "failed_region": "rigion_3",
            "kfs_preparation_result": kfs_preparation_result,
            "region_3_result": region_3_result,
        }

    print("完整比赛/挑战赛/九宫区域执行完成")
    return {
        "completed": True,
        "failed_region": None,
        "kfs_preparation_result": kfs_preparation_result,
        "region_3_result": region_3_result,
    }


def run_challenge_meilin_retry():
    print_current_config("区域重试/挑战赛/梅林区域")

    replan_result = compete_logic.rigion_2_retry_plan(
        r1_count=R1,
        r2_count=R2,
        required_r2_pickup_count=kfs_suck,
        qr_string=meilin_situration,
    )
    if not replan_result.get("completed", False):
        print_flow_failure("梅林重规划", replan_result)
        return {
            "completed": False,
            "failed_region": "rigion_2_retry_plan",
            "replan_result": replan_result,
            "region_2_result": None,
        }
    print("梅林重规划执行完成")

    region_2_result = compete_logic.rigion_2(
        sender=sender,
        position_runtime=position_runtime,
        odom_runtime=odom_runtime,
        queue_timeout_sec=1.0,
    )
    if not region_2_result.get("completed", False):
        print_flow_failure("区域2", region_2_result)
        return {
            "completed": False,
            "failed_region": "rigion_2",
            "replan_result": replan_result,
            "region_2_result": region_2_result,
        }

    print("区域重试/挑战赛/梅林区域执行完成")
    return {
        "completed": True,
        "failed_region": None,
        "replan_result": replan_result,
        "region_2_result": region_2_result,
    }


def run_confrontation_meilin_retry():
    print_current_config("区域重试/对抗赛/梅林区域")

    replan_result = compete_logic.rigion_2_retry_plan(
        r1_count=R1,
        r2_count=R2,
        required_r2_pickup_count=kfs_suck,
        qr_string=meilin_situration,
    )
    if not replan_result.get("completed", False):
        print_flow_failure("梅林重规划", replan_result)
        return {
            "completed": False,
            "failed_region": "rigion_2_retry_plan",
            "replan_result": replan_result,
            "region_2_result": None,
            "region_3_result": None,
        }
    print("梅林重规划执行完成")

    region_2_result = compete_logic.rigion_2(
        sender=sender,
        position_runtime=position_runtime,
        odom_runtime=odom_runtime,
        queue_timeout_sec=1.0,
    )
    if not region_2_result.get("completed", False):
        print_flow_failure("区域2", region_2_result)
        return {
            "completed": False,
            "failed_region": "rigion_2",
            "replan_result": replan_result,
            "region_2_result": region_2_result,
            "region_3_result": None,
        }
    print("区域2执行完成")

    region_3_result = compete_logic.rigion_3(
        sender=sender,
        position_runtime=position_runtime,
        odom_runtime=odom_runtime,
        final_strategy=get_rigion_3_final_strategy(),
    )
    if not region_3_result.get("completed", False):
        print_flow_failure("区域3", region_3_result)
        return {
            "completed": False,
            "failed_region": "rigion_3",
            "replan_result": replan_result,
            "region_2_result": region_2_result,
            "region_3_result": region_3_result,
        }

    print("区域重试/对抗赛/梅林区域执行完成")
    return {
        "completed": True,
        "failed_region": None,
        "replan_result": replan_result,
        "region_2_result": region_2_result,
        "region_3_result": region_3_result,
    }


def run_challenge_jiugong_retry(kfs_preparation_result=None):
    print_current_config("区域重试/挑战赛/九宫格区域")

    prepared_here = kfs_preparation_result is None
    if kfs_preparation_result is None:
        kfs_preparation_result = kfs.kfs_suck_preparation(sender=sender, count=kfs_suck)
    if not kfs_preparation_result.get("completed", False):
        print("KFS吸取准备执行失败")
        return {
            "completed": False,
            "failed_region": "kfs_suck_preparation",
            "kfs_preparation_result": kfs_preparation_result,
            "preparation_pose_result": None,
            "sucker_release_pose_result": None,
            "strategy_result": None,
        }
    if prepared_here:
        print("KFS吸取准备执行完成")

    read_int("输入1吸取完毕：", valid_values=(1,))

    if final_strategy == 1:
        preparation_pose_name = "place_kfs_pose"
        preparation_pose_result = kfs.place_kfs_pose(sender)
    else:
        preparation_pose_name = "place_3rd_kfs_pose"
        preparation_pose_result = kfs.place_3rd_kfs_pose(sender)

    if not preparation_pose_result.get("completed", False):
        print(f"{preparation_pose_name}执行失败")
        return {
            "completed": False,
            "failed_region": preparation_pose_name,
            "kfs_preparation_result": kfs_preparation_result,
            "preparation_pose_name": preparation_pose_name,
            "preparation_pose_result": preparation_pose_result,
            "sucker_release_pose_result": None,
            "strategy_result": None,
        }
    print(f"{preparation_pose_name}执行完成")

    sucker_release_pose_result = kfs.sucker_release_pose(sender)
    if not sucker_release_pose_result.get("completed", False):
        print("sucker_release_pose执行失败")
        return {
            "completed": False,
            "failed_region": "sucker_release_pose",
            "kfs_preparation_result": kfs_preparation_result,
            "preparation_pose_name": preparation_pose_name,
            "preparation_pose_result": preparation_pose_result,
            "sucker_release_pose_result": sucker_release_pose_result,
            "strategy_result": None,
        }
    print("sucker_release_pose执行完成")

    time.sleep(2.0)

    strategy_result = compete_logic.rigion_3_execute_strategy(
        sender=sender,
        position_runtime=position_runtime,
        odom_runtime=odom_runtime,
        final_strategy=get_rigion_3_final_strategy(),
        run_sucker_release_pose=False,
        run_preparation_pose=False,
    )
    if not strategy_result.get("completed", False):
        print_flow_failure("最终策略", strategy_result)
        return {
            "completed": False,
            "failed_region": "rigion_3_execute_strategy",
            "kfs_preparation_result": kfs_preparation_result,
            "preparation_pose_name": preparation_pose_name,
            "preparation_pose_result": preparation_pose_result,
            "sucker_release_pose_result": sucker_release_pose_result,
            "strategy_result": strategy_result,
        }

    print("区域重试/挑战赛/九宫格区域执行完成")
    return {
        "completed": True,
        "failed_region": None,
        "kfs_preparation_result": kfs_preparation_result,
        "preparation_pose_name": preparation_pose_name,
        "preparation_pose_result": preparation_pose_result,
        "sucker_release_pose_result": sucker_release_pose_result,
        "strategy_result": strategy_result,
    }


def challenge_wulin_flow():
    print_separator()
    print("武林区域（挑战赛）")
    print_back_option()
    value = read_int("请输入需要抓取的武器头编号（1-6）：", valid_values=(0, 1, 2, 3, 4, 5, 6))
    if value == 0:
        return

    global weapon_id
    weapon_id = value
    if not wait_start():
        return
    run_full_challenge_wulin_match()


def challenge_jiugong_flow():
    global kfs_suck

    print_separator()
    print("九宫区域（挑战赛）")
    print_back_option()
    kfs_suck = 2
    kfs_preparation_result = kfs.kfs_suck_preparation(sender=sender, count=kfs_suck)
    if not kfs_preparation_result.get("completed", False):
        print("KFS吸取准备执行失败")
        return
    print("KFS吸取准备执行完成")
    if not wait_start():
        return
    wait_choice = read_int("是否需要等待(0.否 1.是)：", valid_values=(0, 1))
    column = None
    extra_needed = None
    if get_rigion_3_final_strategy() == 2:
        column = read_int("请输入需要放置的列号（1-3）：", valid_values=(1, 2, 3))
        extra_needed = read_int("是否需要额外吸取（0.否 1.是）：", valid_values=(0, 1))
    run_full_challenge_jiugong_match(
        wait_before_region_3=(wait_choice == 1),
        kfs_preparation_result=kfs_preparation_result,
        column=column,
        extra_needed=extra_needed,
    )


def confrontation_full_flow():
    print_separator()
    print("完整对抗逻辑")
    print_back_option()
    value = read_int("请输入需要抓取的武器头编号（1-6）：", valid_values=(0, 1, 2, 3, 4, 5, 6))
    if value == 0:
        return

    global weapon_id
    weapon_id = value
    if not wait_start():
        return
    run_full_confrontation_match()


def complete_challenge_flow():
    while True:
        print_separator()
        print("请选择进行的项目（1.武林区域 2.九宫区域）：")
        print_back_option()
        choice = read_int("", valid_values=(0, 1, 2))
        if choice == 0:
            return
        if choice == 1:
            challenge_wulin_flow()
        elif choice == 2:
            challenge_jiugong_flow()


def complete_match_flow():
    print_separator()
    if current_rule == 1:
        complete_challenge_flow()
    elif current_rule == 2:
        confrontation_full_flow()
    else:
        print(ERROR_INPUT_TEXT)


def meilin_retry_flow():
    global R1
    global R2
    global kfs_suck
    global meilin_situration

    print_separator()
    print("梅林重试")
    print_back_option()
    R1 = read_int("请输入当前R1_KFS数量（0-3）：", min_value=0, max_value=3)
    R2 = read_int("请输入当前R2_KFS数量（0-3）：", min_value=0, max_value=3)
    kfs_suck = read_int("请输入还需要吸取的R2_KFS数量（0-2）：", min_value=0, max_value=2)
    meilin_situration = read_qr_payload("请输入当前梅林情况：")
    if not wait_start():
        return
    if current_rule == 1:
        run_challenge_meilin_retry()
    elif current_rule == 2:
        run_confrontation_meilin_retry()
    else:
        print(ERROR_INPUT_TEXT)


def jiugong_retry_flow():
    global kfs_suck

    print_separator()
    print("九宫格区域重试")
    print_back_option()
    kfs_suck = read_int("请输入需要吸取的KFS数量（0-2）：", min_value=0, max_value=2)
    kfs_preparation_result = kfs.kfs_suck_preparation(sender=sender, count=kfs_suck)
    if not kfs_preparation_result.get("completed", False):
        print("KFS吸取准备执行失败")
        return
    print("KFS吸取准备执行完成")
    if not wait_start():
        return
    if current_rule == 1:
        run_challenge_jiugong_retry(kfs_preparation_result=kfs_preparation_result)
    elif current_rule == 2:
        run_challenge_jiugong_retry(kfs_preparation_result=kfs_preparation_result)
    else:
        print(ERROR_INPUT_TEXT)


def area_retry_flow():
    global WEAPON_Y_CORRECTION

    WEAPON_Y_CORRECTION = read_float("weapon点y向偏差（单位m）：")
    while True:
        print_separator()
        print("区域重试")
        print("1.武馆区域重试")
        print("2.梅林区域重试")
        print("3.九宫格区域重试")
        print_back_option()
        choice = read_int("", valid_values=(0, 1, 2, 3))
        if choice == 0:
            return
        if choice == 1:
            if current_rule == 1:
                challenge_wulin_flow()
            elif current_rule == 2:
                confrontation_full_flow()
            else:
                print(ERROR_INPUT_TEXT)
        elif choice == 2:
            if current_rule in (1, 2):
                meilin_retry_flow()
            else:
                print(ERROR_INPUT_TEXT)
        elif choice == 3:
            if current_rule in (1, 2):
                jiugong_retry_flow()
            else:
                print(ERROR_INPUT_TEXT)


def reset_to_initial_state_flow():
    print_separator()
    print("恢复至初态")
    print("此处后续实现")


def main_menu():
    while True:
        print_separator()
        print("完整比赛脚本")
        print("1.完整比赛")
        print("2.区域重试")
        print("3.恢复至初态")
        choice = read_int("", valid_values=(1, 2, 3))
        if choice == 1:
            complete_match_flow()
        elif choice == 2:
            area_retry_flow()
        elif choice == 3:
            reset_to_initial_state_flow()


def main():
    initialize_runtime()
    set_field_by_input()
    set_rule_by_input()
    set_final_strategy_by_input()
    main_menu()


if __name__ == "__main__":
    main()
