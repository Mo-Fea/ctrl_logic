import time
import math

from lib2 import compete_logic
from lib2 import kfs
from lib2 import module
from lib2 import position_backend
from lib2 import position_odin
from lib2 import tools
from utils import challenge_lib, race


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
odometry_start_config = None
localization_ready = False

ERROR_INPUT_TEXT = "错误输入，请按照提示重新输入"
YELLOW = "\033[33m"
RESET = "\033[0m"
SEPARATOR_TEXT = "---------------------------------------------------------"
QR_SCANNER_STARTUP_TIMEOUT_SEC = 30.0


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
    if value != 1:
        return False
    ensure_localization_ready()
    return True


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


def set_odometry_start_config_by_input():
    """预先保存 odom 模式的雷达逻辑地图启动点，不立即应用。"""
    global odometry_start_config

    print("请选择 odom 模式雷达启动点（仅在收到 odom 标志后生效）：")
    print("0. 手动输入已知雷达坐标")
    for point_id, (point_name, (x, y)) in sorted(
        position_odin.ODOMETRY_START_POINTS.items()
    ):
        print(f"{int(point_id)}. {point_name}: ({float(x):.4f}, {float(y):.4f})")

    point_id = read_int("请输入启动点编号：", valid_values=(0, 1, 2))
    if point_id == 0:
        x = read_float("请输入启动点雷达x坐标（m）：")
        y = read_float("请输入启动点雷达y坐标（m）：")
        name = "manual"
    else:
        name, (x, y) = position_odin.ODOMETRY_START_POINTS[point_id]

    odometry_start_config = {
        "id": int(point_id),
        "name": str(name),
        "x": float(x),
        "y": float(y),
    }
    print(
        "已保存 odom 雷达启动点："
        f"{odometry_start_config['name']} "
        f"({odometry_start_config['x']:.4f}, "
        f"{odometry_start_config['y']:.4f})"
    )
    return dict(odometry_start_config)


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
    ) = module.init(
        lidar_type=position_backend.LIDAR_TYPE_ODIN,
        wait_relocalization=False,
        prompt_odometry_start_point=False,
    )
    print("通信与定位标志监听已启动；输入1开始执行后初始化坐标资源")


def ensure_localization_ready():
    """首次开始执行时确认定位模式、应用预设 odom 起点并启动坐标资源。"""
    global flag_node
    global flag_thread
    global flag_stop_event
    global position_runtime
    global odom_runtime
    global localization_ready

    if localization_ready:
        return
    if sender is None:
        raise RuntimeError("runtime is not initialized")

    print("等待重定位/odom 模式标志...")
    while not tools.localization_mode_received:
        time.sleep(0.01)

    if tools.is_odometry_mode():
        if odometry_start_config is None:
            raise RuntimeError("odometry start config was not selected")
        position_lib = module.get_position_lib()
        set_start_pose = getattr(position_lib, "set_odometry_start_lidar_pose", None)
        if set_start_pose is None:
            raise RuntimeError("current position backend does not support odometry start pose")
        set_start_pose(
            odometry_start_config["x"],
            odometry_start_config["y"],
        )
        print(
            "已应用 odom 雷达启动点："
            f"{odometry_start_config['name']} "
            f"({odometry_start_config['x']:.4f}, "
            f"{odometry_start_config['y']:.4f})"
        )
    else:
        print("已确认重定位模式")

    if flag_node is not None or flag_thread is not None or flag_stop_event is not None:
        tools.destroy_ros2_thread(
            node=flag_node,
            spin_thread=flag_thread,
            stop_event=flag_stop_event,
            shutdown_rclpy=False,
        )
        flag_node = None
        flag_thread = None
        flag_stop_event = None

    position_runtime = module.start_position_thread(sender)
    odom_runtime = module.start_odometry_thread(topic=module.ODOM_TOPIC)
    localization_ready = True
    print("定位模式与坐标资源已就绪")


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


def start_qr_scanner_or_exit(
    action_matrix_queue=None,
    startup_timeout_sec=QR_SCANNER_STARTUP_TIMEOUT_SEC,
):
    """
    在比赛参数已生效后启动二维码识别线程。

    只有成功打开图像源、获取到首帧彩色图并完成首轮识别时才允许进入区域 1；
    任一启动步骤失败或超时均终止程序，避免无扫码能力时继续执行硬件动作。
    """
    action_matrix_queue = (
        compete_logic.ACTION_MATRIX_QUEUE
        if action_matrix_queue is None
        else action_matrix_queue
    )
    compete_logic.clear_action_matrix_queue(action_matrix_queue)
    scanner = challenge_lib.start_background_qr_scanner(
        result_queue=action_matrix_queue,
        stable_frame_count=2,
        show_window=False,
        stop_after_success=True,
        put_action_matrix_only=True,
        image_source=1,
    )

    if scanner.wait_until_ready(timeout=float(startup_timeout_sec)):
        print("二维码识别线程已启动：彩色图像与首轮识别校验通过")
        return scanner, action_matrix_queue

    scanner.stop()
    scanner.join(timeout=1.0)
    startup_error = scanner.last_error
    raise SystemExit(
        "二维码识别线程启动失败或未能获取彩色图像，程序终止："
        f"{startup_error!r}"
    )


def run_full_confrontation_match():
    print_current_config("完整比赛/对抗赛")
    config_result = race.configure_kfs_counts(
        r1_count=R1,
        r2_count=R2,
        required_r2_pickup_count=kfs_suck,
    )
    print(f"规划数量配置已更新：{config_result}")
    scanner, action_matrix_queue = start_qr_scanner_or_exit()

    region_1_result = compete_logic.rigion_1(
        sender=sender,
        position_runtime=position_runtime,
        odom_runtime=odom_runtime,
        scanner=scanner,
        weapon_id=weapon_id,
        fetch_weapon_y_correction=WEAPON_Y_CORRECTION,
        action_matrix_queue=action_matrix_queue,
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
    scanner, action_matrix_queue = start_qr_scanner_or_exit()

    region_1_result = compete_logic.rigion_1(
        sender=sender,
        position_runtime=position_runtime,
        odom_runtime=odom_runtime,
        scanner=scanner,
        weapon_id=weapon_id,
        fetch_weapon_y_correction=WEAPON_Y_CORRECTION,
        action_matrix_queue=action_matrix_queue,
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
        print("等待3秒")
        time.sleep(3.0)

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


def run_challenge_jiugong_retry(
    final_strategy,
    kfs_preparation_result=None,
    column=None,
    extra_needed=None,
    wait_before_region_3=False,
):
    """挑战赛九宫格重试：跳过入场，直接执行挑战版最终策略。"""
    final_strategy = int(final_strategy)
    if final_strategy not in (1, 2):
        raise ValueError(f"final_strategy must be 1 or 2, got {final_strategy}")
    if final_strategy == 2 and (column is None or extra_needed is None):
        raise ValueError(
            "column and extra_needed are required when final_strategy is 2"
        )

    print("准备执行：区域重试/挑战赛/九宫格区域")
    print(
        "本次策略："
        f"{'大胜逻辑' if final_strategy == 2 else '190逻辑'}"
    )

    prepared_here = kfs_preparation_result is None
    if kfs_preparation_result is None:
        kfs_preparation_result = kfs.kfs_suck_preparation(sender=sender, count=2)
    if not kfs_preparation_result.get("completed", False):
        print("KFS吸取准备执行失败")
        return {
            "completed": False,
            "failed_region": "kfs_suck_preparation",
            "kfs_preparation_result": kfs_preparation_result,
            "strategy_result": None,
        }
    if prepared_here:
        print("KFS吸取准备执行完成")

    if wait_before_region_3:
        print("等待3秒")
        time.sleep(3.0)

    strategy_result = compete_logic.rigion_3_execute_strategy_challenge(
        sender=sender,
        position_runtime=position_runtime,
        odom_runtime=odom_runtime,
        final_strategy=final_strategy,
        column=column,
        extra_needed=extra_needed,
    )
    if not strategy_result.get("completed", False):
        print_flow_failure("挑战赛最终策略", strategy_result)
        return {
            "completed": False,
            "failed_region": "rigion_3_execute_strategy_challenge",
            "kfs_preparation_result": kfs_preparation_result,
            "strategy_result": strategy_result,
        }

    print("区域重试/挑战赛/九宫格区域执行完成")
    return {
        "completed": True,
        "failed_region": None,
        "kfs_preparation_result": kfs_preparation_result,
        "strategy_result": strategy_result,
    }


def run_confrontation_jiugong_retry(kfs_preparation_result=None):
    print_current_config("区域重试/对抗赛/九宫格区域")

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

    if current_rule == 1:
        selected_logic = read_int(
            "所需执行逻辑(1.大胜逻辑 2.190逻辑 0.返回)：",
            valid_values=(0, 1, 2),
        )
        if selected_logic == 0:
            return

        # 与挑战赛九宫格完整比赛一致，固定准备双气缸并使 suck_count=3。
        kfs_suck = 2
        kfs_preparation_result = kfs.kfs_suck_preparation(
            sender=sender,
            count=kfs_suck,
        )
        if not kfs_preparation_result.get("completed", False):
            print("KFS吸取准备执行失败")
            return
        print("KFS吸取准备执行完成")
        if not wait_start():
            return

        column = None
        extra_needed = None
        # 菜单 1=大胜，对应挑战策略内部编号 2；菜单 2=190，对应编号 1。
        challenge_final_strategy = 2 if selected_logic == 1 else 1
        if challenge_final_strategy == 2:
            column = read_int(
                "请输入需要放置的列号（1-3）：",
                valid_values=(1, 2, 3),
            )
            extra_needed = read_int(
                "是否需要额外吸取（0.否 1.是）：",
                valid_values=(0, 1),
            )

        run_challenge_jiugong_retry(
            final_strategy=challenge_final_strategy,
            kfs_preparation_result=kfs_preparation_result,
            column=column,
            extra_needed=extra_needed,
        )
    elif current_rule == 2:
        kfs_suck = read_int(
            "请输入需要吸取的KFS数量（0-2）：",
            min_value=0,
            max_value=2,
        )
        kfs_preparation_result = kfs.kfs_suck_preparation(
            sender=sender,
            count=kfs_suck,
        )
        if not kfs_preparation_result.get("completed", False):
            print("KFS吸取准备执行失败")
            return
        print("KFS吸取准备执行完成")
        if not wait_start():
            return
        run_confrontation_jiugong_retry(
            kfs_preparation_result=kfs_preparation_result,
        )
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
    set_field_by_input()
    set_rule_by_input()
    set_final_strategy_by_input()
    set_odometry_start_config_by_input()
    initialize_runtime()
    main_menu()


if __name__ == "__main__":
    main()
