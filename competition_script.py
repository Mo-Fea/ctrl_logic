from lib2 import position_backend


final_strategy = 1
R1 = 2
R2 = 2
kfs_suck = 2
weapon_id = 4
current_rule = 1
meilin_situration = "000000000000"

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
        f"current_rule={current_rule}, "
        f"meilin_situration={meilin_situration}"
    )


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
    print_current_config("完整比赛/挑战赛/武林区域")


def challenge_jiugong_flow():
    print_separator()
    print("九宫区域（挑战赛）")
    print_back_option()
    if not wait_start():
        return
    print_current_config("完整比赛/挑战赛/九宫区域")


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
    print_current_config("完整比赛/对抗赛")


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
    print_current_config("区域重试/梅林区域重试")


def jiugong_retry_flow():
    global kfs_suck

    print_separator()
    print("九宫格区域重试")
    print_back_option()
    kfs_suck = read_int("请输入需要吸取的KFS数量（0-2）：", min_value=0, max_value=2)
    if not wait_start():
        return
    print_current_config("区域重试/九宫格区域重试")


def area_retry_flow():
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
            meilin_retry_flow()
        elif choice == 3:
            jiugong_retry_flow()


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
    set_field_by_input()
    set_rule_by_input()
    set_final_strategy_by_input()
    main_menu()


if __name__ == "__main__":
    main()
