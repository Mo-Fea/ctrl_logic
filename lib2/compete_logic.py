import queue
import threading
import time

from lib2 import module
from lib2 import move
from lib2 import kfs
from lib2 import weapon
from utils import challenge_lib
from utils import race


ACTION_MATRIX_QUEUE = queue.Queue()


def clear_action_matrix_queue(action_matrix_queue=None):
    action_matrix_queue = (
        ACTION_MATRIX_QUEUE if action_matrix_queue is None else action_matrix_queue
    )
    while True:
        try:
            action_matrix_queue.get_nowait()
        except queue.Empty:
            break
    return action_matrix_queue


def _peek_queue(action_matrix_queue):
    with action_matrix_queue.mutex:
        if not action_matrix_queue.queue:
            return None
        return action_matrix_queue.queue[0]


def _wait_until_lock_held(lock, timeout_sec=1.0, poll_interval_sec=0.01):
    deadline = time.time() + float(timeout_sec)
    while time.time() < deadline:
        acquired = lock.acquire(blocking=False)
        if acquired:
            lock.release()
            time.sleep(float(poll_interval_sec))
            continue
        return True
    return False


def _monitor_qr_scanner_detection(scanner, stop_event, poll_interval_sec=0.05):
    last_printed_data = None
    while not stop_event.is_set():
        data = scanner.last_qr_data
        if data and data != last_printed_data:
            detect_path = getattr(scanner, "last_detect_path", None)
            stable_count = getattr(scanner, "last_stable_count", 0)
            print(
                "[QR识别线程]: 已识别到二维码 "
                f"data={data!r}, stable_count={stable_count}, path={detect_path}"
            )
            last_printed_data = data
        if scanner.done_event.is_set():
            break
        time.sleep(float(poll_interval_sec))


def rigion_2_retry_plan(
    r1_count,
    r2_count,
    required_r2_pickup_count,
    qr_string,
    result_queue=None,
):
    """
    区域 2 重规划：根据当前 KFS 数量和 12 位场地字符串生成完整动作矩阵。

    向 queue 推入 action_matrix，行为与后台 QR 线程 put_action_matrix_only=True 一致。
    """
    result_queue = result_queue if result_queue is not None else ACTION_MATRIX_QUEUE

    try:
        r1_count = int(r1_count)
        r2_count = int(r2_count)
        required_r2_pickup_count = int(required_r2_pickup_count)
    except (TypeError, ValueError) as exc:
        return {
            "completed": False,
            "failed_step": "invalid_retry_plan_input",
            "error": exc,
            "r1_count": r1_count,
            "r2_count": r2_count,
            "required_r2_pickup_count": required_r2_pickup_count,
            "qr_data": qr_string,
            "result_queue": result_queue,
        }

    qr_string = "" if qr_string is None else str(qr_string).strip()
    if not challenge_lib.is_valid_qr_payload(qr_string):
        return {
            "completed": False,
            "failed_step": "invalid_retry_plan_qr_string",
            "error": ValueError("qr_string must be 12 chars using only 0/1/2/3"),
            "r1_count": int(r1_count),
            "r2_count": int(r2_count),
            "required_r2_pickup_count": int(required_r2_pickup_count),
            "qr_data": qr_string,
            "result_queue": result_queue,
        }

    config_result = race.configure_kfs_counts(
        r1_count=r1_count,
        r2_count=r2_count,
        required_r2_pickup_count=required_r2_pickup_count,
    )

    try:
        plan_result = challenge_lib.build_plan_result_from_qr(qr_string)
    except Exception as exc:
        return {
            "completed": False,
            "failed_step": "build_retry_plan",
            "error": exc,
            "config_result": config_result,
            "qr_data": qr_string,
            "r1_count": int(r1_count),
            "r2_count": int(r2_count),
            "required_r2_pickup_count": int(required_r2_pickup_count),
            "result_queue": result_queue,
        }

    result_queue.put(plan_result.action_matrix)

    return {
        "completed": True,
        "failed_step": None,
        "config_result": config_result,
        "qr_data": qr_string,
        "r1_count": int(r1_count),
        "r2_count": int(r2_count),
        "required_r2_pickup_count": int(required_r2_pickup_count),
        "kfs": plan_result.kfs,
        "path": plan_result.path,
        "action_matrix": plan_result.action_matrix,
        "result_queue": result_queue,
    }


def rigion_3_execute_strategy(
    sender,
    position_runtime,
    odom_runtime,
    final_strategy=1,
    run_sucker_release_pose=True,
    run_preparation_pose=True,
):
    """
    区域 3 策略流程：不执行 enter_battlefield，只执行最终胜利策略。

    final_strategy=1: high_score190
    final_strategy=2: totally_win
    """
    final_strategy = int(final_strategy)
    if final_strategy not in (1, 2):
        raise ValueError(f"final_strategy must be 1 or 2, got {final_strategy}")

    if run_sucker_release_pose:
        sucker_release_pose_result = kfs.sucker_release_pose(sender)
        if not sucker_release_pose_result.get("completed", False):
            return {
                "completed": False,
                "failed_step": "sucker_release_pose",
                "final_strategy": int(final_strategy),
                "strategy_name": None,
                "sucker_release_pose_result": sucker_release_pose_result,
                "preparation_pose_name": None,
                "preparation_pose_result": None,
                "strategy_result": None,
            }
    else:
        sucker_release_pose_result = {
            "completed": True,
            "skipped": True,
        }

    if final_strategy == 1:
        preparation_pose_name = "place_3rd_kfs_pose"
        strategy_name = "high_score190"
        if run_preparation_pose:
            preparation_pose_result = kfs.place_3rd_kfs_pose(sender)
        else:
            preparation_pose_result = {
                "completed": True,
                "skipped": True,
            }
    else:
        preparation_pose_name = "place_kfs_pose"
        strategy_name = "totally_win"
        if run_preparation_pose:
            preparation_pose_result = kfs.place_kfs_pose(sender)
        else:
            preparation_pose_result = {
                "completed": True,
                "skipped": True,
            }

    if not preparation_pose_result.get("completed", False):
        return {
            "completed": False,
            "failed_step": preparation_pose_name,
            "final_strategy": int(final_strategy),
            "strategy_name": strategy_name,
            "sucker_release_pose_result": sucker_release_pose_result,
            "preparation_pose_name": preparation_pose_name,
            "preparation_pose_result": preparation_pose_result,
            "strategy_result": None,
        }

    if final_strategy == 1:
        strategy_result = module.high_score190(
            sender=sender,
            position_runtime=position_runtime,
            odom_runtime=odom_runtime,
        )
    else:
        strategy_result = module.totally_win(
            sender=sender,
            position_runtime=position_runtime,
            odom_runtime=odom_runtime,
        )

    return {
        "completed": bool(strategy_result.get("completed", False)),
        "failed_step": None if strategy_result.get("completed", False) else strategy_name,
        "final_strategy": int(final_strategy),
        "strategy_name": strategy_name,
        "sucker_release_pose_result": sucker_release_pose_result,
        "preparation_pose_name": preparation_pose_name,
        "preparation_pose_result": preparation_pose_result,
        "strategy_result": strategy_result,
    }


def rigion_2(
    sender,
    position_runtime,
    odom_runtime,
    action_matrix_queue=None,
    queue_timeout_sec=None,
    final_direction=None,
    execute_action_matrix_kwargs=None,
):
    """
    区域 2 流程：从 queue 获取动作矩阵并执行完整梅林动作矩阵。
    """
    action_matrix_queue = (
        ACTION_MATRIX_QUEUE if action_matrix_queue is None else action_matrix_queue
    )
    execute_action_matrix_kwargs = (
        {}
        if execute_action_matrix_kwargs is None
        else dict(execute_action_matrix_kwargs)
    )

    try:
        if queue_timeout_sec is None:
            action_matrix = action_matrix_queue.get()
        else:
            action_matrix = action_matrix_queue.get(timeout=float(queue_timeout_sec))
    except queue.Empty:
        return {
            "completed": False,
            "failed_step": "action_matrix_queue_empty",
            "action_matrix": None,
            "action_matrix_queue": action_matrix_queue,
            "matrix_result": None,
        }

    matrix_result = module.execute_action_matrix(
        sender=sender,
        position_runtime=position_runtime,
        odom_runtime=odom_runtime,
        action_matrix=action_matrix,
        final_direction=final_direction,
        **execute_action_matrix_kwargs,
    )

    return {
        "completed": bool(matrix_result.get("completed", False)),
        "failed_step": matrix_result.get("failure_reason"),
        "action_matrix": action_matrix,
        "action_matrix_queue": action_matrix_queue,
        "matrix_result": matrix_result,
    }


def rigion_1(
    sender,
    position_runtime,
    odom_runtime,
    weapon_id=4,
    image_source=1,
    stable_frame_count=2,
    show_window=False,
    scanner_start_timeout_sec=1.0,
    scanner_release_timeout_sec=None,
    fetch_weapon_kwargs=None,
    fetch_weapon_y_correction=0.0,
    lock_wheel_kwargs=None,
    unlock_wheel_kwargs=None,
    weapon_down_kwargs=None,
    weapon_loose_kwargs=None,
    final_wait_sec=5.0,
    action_matrix_queue=None,
):
    """
    区域 1 流程：后台识别 QR，同时抓取 weapon，等待 QR 线程释放锁后进入后续动作。

    weapon_id 取 1~6，默认 4。fetch_weapon_y_correction 用于修正
    抓取 weapon 时接近点和返回点的 y 坐标，默认 0。
    """
    weapon_id = int(weapon_id)
    if weapon_id < 1 or weapon_id > 6:
        raise ValueError(f"weapon_id must be 1..6, got {weapon_id}")

    fetch_weapon_kwargs = (
        {} if fetch_weapon_kwargs is None else dict(fetch_weapon_kwargs)
    )
    fetch_weapon_kwargs["y_correction"] = float(fetch_weapon_y_correction)
    lock_wheel_kwargs = (
        {} if lock_wheel_kwargs is None else dict(lock_wheel_kwargs)
    )
    unlock_wheel_kwargs = (
        {} if unlock_wheel_kwargs is None else dict(unlock_wheel_kwargs)
    )
    weapon_down_kwargs = (
        {} if weapon_down_kwargs is None else dict(weapon_down_kwargs)
    )
    weapon_loose_kwargs = (
        {} if weapon_loose_kwargs is None else dict(weapon_loose_kwargs)
    )

    action_matrix_queue = (
        ACTION_MATRIX_QUEUE if action_matrix_queue is None else action_matrix_queue
    )
    clear_action_matrix_queue(action_matrix_queue)
    scanner = challenge_lib.start_background_qr_scanner(
        result_queue=action_matrix_queue,
        stable_frame_count=stable_frame_count,
        show_window=show_window,
        stop_after_success=True,
        put_action_matrix_only=True,
        image_source=image_source,
    )
    scanner_lock_started = _wait_until_lock_held(
        challenge_lib.SCANNER_RUNNING_LOCK,
        timeout_sec=scanner_start_timeout_sec,
    )
    if not scanner_lock_started:
        scanner.stop()
        scanner.join(timeout=1.0)
        return {
            "completed": False,
            "failed_step": "start_qr_scanner_lock",
            "weapon_id": int(weapon_id),
            "scanner": scanner,
            "scanner_error": scanner.last_error,
            "scanner_lock_started": False,
            "action_matrix": None,
            "action_matrix_queue": action_matrix_queue,
        }

    qr_print_stop_event = threading.Event()
    qr_print_thread = threading.Thread(
        target=_monitor_qr_scanner_detection,
        args=(scanner, qr_print_stop_event),
        daemon=True,
        name="challenge_qr_detection_print",
    )
    qr_print_thread.start()

    fetch_result = None
    initial_weapon_loose_result = None
    weapon_down_result = None
    lock_result = None
    unlock_result = None
    weapon_loose_result = None
    qr_wait_result = False
    action_matrix = None
    final_wait_completed = False

    try:
        initial_weapon_loose_result = weapon.weapon_loose(
            sender,
            **weapon_loose_kwargs,
        )
        if not initial_weapon_loose_result.get("completed", False):
            return {
                "completed": False,
                "failed_step": "weapon_loose_after_scanner_lock",
                "weapon_id": int(weapon_id),
                "scanner": scanner,
                "scanner_lock_started": bool(scanner_lock_started),
                "initial_weapon_loose_result": initial_weapon_loose_result,
                "weapon_down_result": weapon_down_result,
                "fetch_result": fetch_result,
                "lock_result": lock_result,
                "qr_wait_result": qr_wait_result,
                "action_matrix": action_matrix,
                "action_matrix_queue": action_matrix_queue,
            }

        weapon_down_result = weapon.weapon_down(
            sender,
            **weapon_down_kwargs,
        )
        if not weapon_down_result.get("completed", False):
            return {
                "completed": False,
                "failed_step": "weapon_down_after_scanner_lock",
                "weapon_id": int(weapon_id),
                "scanner": scanner,
                "scanner_lock_started": bool(scanner_lock_started),
                "initial_weapon_loose_result": initial_weapon_loose_result,
                "weapon_down_result": weapon_down_result,
                "fetch_result": fetch_result,
                "lock_result": lock_result,
                "qr_wait_result": qr_wait_result,
                "action_matrix": action_matrix,
                "action_matrix_queue": action_matrix_queue,
            }

        fetch_result = module.fetch_weapon(
            sender=sender,
            position_runtime=position_runtime,
            odom_runtime=odom_runtime,
            weapon_id=weapon_id,
            **fetch_weapon_kwargs,
        )

        if not fetch_result.get("completed", False):
            return {
                "completed": False,
                "failed_step": "fetch_weapon",
                "weapon_id": int(weapon_id),
                "scanner": scanner,
                "scanner_lock_started": bool(scanner_lock_started),
                "initial_weapon_loose_result": initial_weapon_loose_result,
                "weapon_down_result": weapon_down_result,
                "fetch_result": fetch_result,
                "lock_result": lock_result,
                "qr_wait_result": qr_wait_result,
                "action_matrix": action_matrix,
                "action_matrix_queue": action_matrix_queue,
            }

        lock_result = move.lock_wheel(sender, **lock_wheel_kwargs)
        if not lock_result.get("completed", False):
            return {
                "completed": False,
                "failed_step": "lock_wheel",
                "weapon_id": int(weapon_id),
                "scanner": scanner,
                "scanner_lock_started": bool(scanner_lock_started),
                "initial_weapon_loose_result": initial_weapon_loose_result,
                "weapon_down_result": weapon_down_result,
                "fetch_result": fetch_result,
                "lock_result": lock_result,
                "qr_wait_result": qr_wait_result,
                "action_matrix": action_matrix,
                "action_matrix_queue": action_matrix_queue,
            }

        qr_wait_result = challenge_lib.wait_until_scanner_released(
            timeout=scanner_release_timeout_sec,
        )
        if not qr_wait_result:
            return {
                "completed": False,
                "failed_step": "wait_for_qr_scanner_lock",
                "weapon_id": int(weapon_id),
                "scanner": scanner,
                "scanner_lock_started": bool(scanner_lock_started),
                "initial_weapon_loose_result": initial_weapon_loose_result,
                "weapon_down_result": weapon_down_result,
                "fetch_result": fetch_result,
                "lock_result": lock_result,
                "qr_wait_result": qr_wait_result,
                "scanner_error": scanner.last_error,
                "action_matrix": action_matrix,
                "action_matrix_queue": action_matrix_queue,
            }

        action_matrix = _peek_queue(action_matrix_queue)

        unlock_result = move.unlock_wheel(sender, **unlock_wheel_kwargs)
        if not unlock_result.get("completed", False):
            return {
                "completed": False,
                "failed_step": "unlock_wheel",
                "weapon_id": int(weapon_id),
                "scanner": scanner,
                "scanner_lock_started": bool(scanner_lock_started),
                "initial_weapon_loose_result": initial_weapon_loose_result,
                "weapon_down_result": weapon_down_result,
                "fetch_result": fetch_result,
                "lock_result": lock_result,
                "qr_wait_result": qr_wait_result,
                "action_matrix": action_matrix,
                "action_matrix_queue": action_matrix_queue,
                "unlock_result": unlock_result,
            }

        weapon_loose_result = weapon.weapon_loose(
            sender,
            **weapon_loose_kwargs,
        )
        if not weapon_loose_result.get("completed", False):
            return {
                "completed": False,
                "failed_step": "weapon_loose",
                "weapon_id": int(weapon_id),
                "scanner": scanner,
                "scanner_lock_started": bool(scanner_lock_started),
                "initial_weapon_loose_result": initial_weapon_loose_result,
                "weapon_down_result": weapon_down_result,
                "fetch_result": fetch_result,
                "lock_result": lock_result,
                "qr_wait_result": qr_wait_result,
                "action_matrix": action_matrix,
                "action_matrix_queue": action_matrix_queue,
                "unlock_result": unlock_result,
                "weapon_loose_result": weapon_loose_result,
            }

        time.sleep(float(final_wait_sec))
        final_wait_completed = True

        return {
            "completed": True,
            "failed_step": None,
            "weapon_id": int(weapon_id),
            "scanner": scanner,
            "scanner_lock_started": bool(scanner_lock_started),
            "initial_weapon_loose_result": initial_weapon_loose_result,
            "weapon_down_result": weapon_down_result,
            "fetch_result": fetch_result,
            "lock_result": lock_result,
            "qr_wait_result": qr_wait_result,
            "scanner_error": scanner.last_error,
            "action_matrix": action_matrix,
            "action_matrix_queue": action_matrix_queue,
            "unlock_result": unlock_result,
            "weapon_loose_result": weapon_loose_result,
            "final_wait_sec": float(final_wait_sec),
            "final_wait_completed": bool(final_wait_completed),
        }
    finally:
        qr_print_stop_event.set()
        if not qr_wait_result:
            scanner.stop()
        scanner.join(timeout=1.0)
        qr_print_thread.join(timeout=0.5)


def rigion_3(
    sender,
    position_runtime,
    odom_runtime,
    final_strategy=1,
    enter_battlefield_kwargs=None,
):
    """
    区域 3 流程：进入九宫格后执行最终胜利策略。

    final_strategy=1: high_score190
    final_strategy=2: totally_win
    """
    final_strategy = int(final_strategy)
    if final_strategy not in (1, 2):
        raise ValueError(f"final_strategy must be 1 or 2, got {final_strategy}")

    enter_battlefield_kwargs = (
        {}
        if enter_battlefield_kwargs is None
        else dict(enter_battlefield_kwargs)
    )

    enter_battlefield_result = move.enter_battlefield(
        sender=sender,
        position_runtime=position_runtime,
        odom_runtime=odom_runtime,
        **enter_battlefield_kwargs,
    )
    if not enter_battlefield_result.get("completed", False):
        return {
            "completed": False,
            "failed_step": "enter_battlefield",
            "final_strategy": int(final_strategy),
            "enter_battlefield_result": enter_battlefield_result,
            "strategy_result": None,
        }

    strategy_flow_result = rigion_3_execute_strategy(
        sender=sender,
        position_runtime=position_runtime,
        odom_runtime=odom_runtime,
        final_strategy=final_strategy,
    )

    return {
        "completed": bool(strategy_flow_result.get("completed", False)),
        "failed_step": strategy_flow_result.get("failed_step"),
        "final_strategy": strategy_flow_result.get("final_strategy"),
        "strategy_name": strategy_flow_result.get("strategy_name"),
        "enter_battlefield_result": enter_battlefield_result,
        "sucker_release_pose_result": strategy_flow_result.get("sucker_release_pose_result"),
        "preparation_pose_name": strategy_flow_result.get("preparation_pose_name"),
        "preparation_pose_result": strategy_flow_result.get("preparation_pose_result"),
        "strategy_result": strategy_flow_result.get("strategy_result"),
    }


def rigion3_challenge(
    sender,
    position_runtime,
    odom_runtime,
    final_strategy=1,
    column=None,
    extra_needed=None,
    enter_battlefield_kwargs=None,
):
    """
    挑战赛区域 3 流程。

    保留 rigion_3() 的入场逻辑，但最终策略改为挑战赛版本：
      final_strategy=1: high_score190_challenge
      final_strategy=2: totally_win_challenge(column, extra_needed)
    """
    final_strategy = int(final_strategy)
    if final_strategy not in (1, 2):
        raise ValueError(f"final_strategy must be 1 or 2, got {final_strategy}")
    if final_strategy == 2 and (column is None or extra_needed is None):
        raise ValueError(
            "column and extra_needed are required when final_strategy is 2"
        )

    enter_battlefield_kwargs = (
        {}
        if enter_battlefield_kwargs is None
        else dict(enter_battlefield_kwargs)
    )

    enter_battlefield_result = move.enter_battlefield(
        sender=sender,
        position_runtime=position_runtime,
        odom_runtime=odom_runtime,
        **enter_battlefield_kwargs,
    )
    if not enter_battlefield_result.get("completed", False):
        return {
            "completed": False,
            "failed_step": "enter_battlefield",
            "final_strategy": int(final_strategy),
            "column": column,
            "extra_needed": extra_needed,
            "enter_battlefield_result": enter_battlefield_result,
            "strategy_name": None,
            "strategy_result": None,
        }

    if final_strategy == 1:
        strategy_name = "high_score190_challenge"
        strategy_result = module.high_score190_challenge(
            sender=sender,
            position_runtime=position_runtime,
            odom_runtime=odom_runtime,
        )
    else:
        strategy_name = "totally_win_challenge"
        strategy_result = module.totally_win_challenge(
            sender=sender,
            position_runtime=position_runtime,
            odom_runtime=odom_runtime,
            column=column,
            extra_needed=extra_needed,
        )

    completed = bool(strategy_result.get("completed", False))
    return {
        "completed": completed,
        "failed_step": (
            None
            if completed
            else strategy_result.get("failed_step", strategy_name)
        ),
        "final_strategy": int(final_strategy),
        "column": column,
        "extra_needed": extra_needed,
        "enter_battlefield_result": enter_battlefield_result,
        "strategy_name": strategy_name,
        "strategy_result": strategy_result,
    }
