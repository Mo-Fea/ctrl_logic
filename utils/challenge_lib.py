import queue
import threading
import time
from dataclasses import dataclass

import numpy as np

try:
    from . import race
    from .process import (
        create_qr_detector,
        detect_qr_data,
        get_color_frame,
        is_valid_qr_payload,
        open_image_source,
    )
except ImportError:
    import race
    from process import (
        create_qr_detector,
        detect_qr_data,
        get_color_frame,
        is_valid_qr_payload,
        open_image_source,
    )

try:
    import cv2
except ImportError:
    cv2 = None


ACTION_MATRIX_COLUMNS = [
    "from_pos",
    "to_pos",
    "move_dir",
    "height_action",
    "grab_action",
]

MOVE_DIR_NAMES = {
    0: "原地",
    1: "方向1(红+Y/蓝-Y)",
    2: "方向2(红-X/蓝+X)",
    3: "方向3(红+X/蓝-X)",
    4: "方向4(红-Y/蓝+Y)",
}

GRAB_ACTION_NAMES = {
    0: "不抓取",
    1: "抓取",
}

EXIT_ACTION_TARGETS = {}
ENTRY_FROM_POS = -1
ENTRY_TO_POS = 2

SCANNER_RUNNING_LOCK = threading.Lock()
PLANNING_CONFIG_LOCK = threading.Lock()


@dataclass
class ChallengePlanResult:
    qr_data: str
    kfs: dict
    path: list
    action_matrix: np.ndarray


def qr_to_kfs(qr_string):
    """
    将12位二维码字符串转换为 race.py 使用的 kfs 字典。

    0=None, 1=R1, 2=R2, 3=low/fake。
    """
    qr_string = "" if qr_string is None else str(qr_string).strip()
    mapping = {"1": "R1", "2": "R2", "3": "low", "0": None}
    kfs = {}

    for index in range(12):
        pos = index + 1
        char = qr_string[index] if index < len(qr_string) else "0"
        kfs[pos] = mapping.get(char, None)
    return kfs


def _direction_code(from_pos, to_pos):
    """按执行层当前红/蓝半场语义计算方向码。"""
    if from_pos == to_pos:
        return 0

    from lib2 import tools

    direction = tools.stair_id_to_direction(
        int(from_pos),
        int(to_pos),
        exit_on_error=False,
    )
    if direction == 0:
        raise ValueError(f"位置 {from_pos} 到 {to_pos} 不是相邻格，无法生成动作矩阵")
    return int(direction)


def _height_action(from_pos, to_pos):
    """返回台阶动作：1=需要上下楼梯，0=不移动/等高。"""
    if from_pos == to_pos:
        return 0

    from_height = race.pos_to_coord[int(from_pos)][2]
    to_height = race.pos_to_coord[int(to_pos)][2]
    return 1 if (to_height - from_height) != 0 else 0


def _entry_action_row():
    return [
        ENTRY_FROM_POS,
        ENTRY_TO_POS,
        _direction_code(ENTRY_FROM_POS, ENTRY_TO_POS),
        1,
        0,
    ]


def _append_exit_action(rows):
    # 13/15 已经作为出口辅助点进入 race.py 路径规划，不再由动作矩阵层额外追加。
    if not rows:
        return rows

    last_to_pos = int(rows[-1][1])
    exit_to_pos = EXIT_ACTION_TARGETS.get(last_to_pos)
    if exit_to_pos is not None:
        rows.append([
            last_to_pos,
            exit_to_pos,
            _direction_code(last_to_pos, exit_to_pos),
            1,
            0,
        ])
    return rows


def path_to_action_matrix(kfs, path):
    """
    将 race.plan_path() 返回路径转换成 n*5 动作矩阵。

    行格式:
      [from_pos, to_pos, move_dir, height_action, grab_action]
    """
    if not path:
        return np.zeros((0, 5), dtype=int)

    is_valid, message = race.validate_path(kfs, path)
    if not is_valid:
        raise ValueError(f"无法生成动作矩阵：{message}")

    rows = []
    current_pos = path[0]
    taken_set = set()

    if kfs.get(current_pos) == "R2":
        rows.append([current_pos, current_pos, 0, 0, 1])
        taken_set.add(current_pos)

    for step in path[1:]:
        if isinstance(step, int):
            if kfs.get(step) == "R2" and step not in taken_set:
                rows.append([
                    current_pos,
                    step,
                    _direction_code(current_pos, step),
                    0,
                    1,
                ])
                taken_set.add(step)

            rows.append([
                current_pos,
                step,
                _direction_code(current_pos, step),
                _height_action(current_pos, step),
                0,
            ])
            current_pos = step

        elif isinstance(step, tuple) and len(step) == 3 and step[1] == "reach":
            from_pos, _, to_pos = step
            if from_pos != current_pos:
                raise ValueError(f"夹取起点 {from_pos} 与当前位置 {current_pos} 不一致")

            rows.append([
                from_pos,
                to_pos,
                _direction_code(from_pos, to_pos),
                0,
                1,
            ])
            taken_set.add(to_pos)

        else:
            raise ValueError(f"未知路径步骤：{step}")

    rows = _append_exit_action(rows)
    if not rows:
        return np.zeros((0, 5), dtype=int)
    return np.array(rows, dtype=int)


def build_action_matrix_from_kfs(kfs):
    """
    从 KFS 字典规划挑战赛路径并生成动作矩阵。

    返回:
      action_matrix, path
    """
    layout_valid, layout_message = race.validate_kfs_layout(kfs)
    if not layout_valid:
        raise ValueError(f"无法生成动作矩阵：{layout_message}")

    path = race.plan_path(kfs)
    if not path:
        raise ValueError("当前 KFS 布局下未找到可行路径")
    return path_to_action_matrix(kfs, path), path


def build_action_matrix_from_qr(qr_string):
    """
    从12位二维码字符串直接生成动作矩阵。

    返回:
      action_matrix, path, kfs
    """
    data = "" if qr_string is None else str(qr_string).strip()
    if not is_valid_qr_payload(data):
        raise ValueError(f"二维码内容无效: {qr_string!r}")
    kfs = qr_to_kfs(data)
    action_matrix, path = build_action_matrix_from_kfs(kfs)
    return action_matrix, path, kfs


def _path_planning_cost(kfs, path):
    """返回 race.py 当前代价配置下的场内规划总代价。"""
    return race.calculate_path_cost(kfs, path)


def build_action_matrix_with_pre_entry_pickup(qr_string):
    """
    根据前 1/2/3 号格的 R2-KFS 情况生成带场外预吸取行的动作矩阵。

    - 前三位没有 2：将 -1->2 入口上楼行放在完整矩阵第一行。
    - 第二位是 2：优先清除 2 号格。
    - 否则清除 1 号或 3 号格；两者都是 2 时分别规划，选规划总代价较小者。
    - 只要前三位中 2 的数量为 1/2/3，R2 布局数和剩余抓取数都只减 1。
    - 有场外预吸取时，预吸取行是第一行，-1->2 入口上楼行是第二行。

    返回值与 build_action_matrix_from_qr() 一致：
      action_matrix, path, effective_kfs
    """
    data = "" if qr_string is None else str(qr_string).strip()
    if not is_valid_qr_payload(data):
        raise ValueError(f"二维码内容无效: {qr_string!r}")

    original_kfs = qr_to_kfs(data)
    layout_valid, layout_message = race.validate_kfs_layout(original_kfs)
    if not layout_valid:
        raise ValueError(f"无法生成动作矩阵：{layout_message}")

    pre_entry_r2_positions = [
        position
        for position, value in enumerate(data[:3], start=1)
        if value == "2"
    ]
    pre_entry_r2_count = len(pre_entry_r2_positions)
    if pre_entry_r2_count == 0:
        action_matrix, path, effective_kfs = build_action_matrix_from_qr(data)
        action_matrix = np.vstack((
            np.array([_entry_action_row()], dtype=int),
            action_matrix,
        ))
        return action_matrix, path, effective_kfs
    if pre_entry_r2_count not in (1, 2, 3):
        raise ValueError(
            "前三位中 R2-KFS 数量必须是 0/1/2/3，"
            f"当前为 {pre_entry_r2_count}"
        )

    with PLANNING_CONFIG_LOCK:
        original_r2_count = race.R2_KFS_COUNT
        original_required_count = race.REQUIRED_R2_PICKUP_COUNT
        if original_r2_count <= 0:
            raise ValueError("R2_KFS_COUNT 必须大于 0 才能执行场外预吸取")
        if original_required_count <= 0:
            raise ValueError(
                "REQUIRED_R2_PICKUP_COUNT 必须大于 0 才能执行场外预吸取"
            )

        race.R2_KFS_COUNT = original_r2_count - 1
        race.REQUIRED_R2_PICKUP_COUNT = original_required_count - 1

        try:
            candidate_positions = (
                [2]
                if 2 in pre_entry_r2_positions
                else list(pre_entry_r2_positions)
            )
            candidate_results = []
            candidate_errors = []

            for pickup_position in candidate_positions:
                modified_chars = list(data)
                modified_chars[pickup_position - 1] = "0"
                modified_data = "".join(modified_chars)
                try:
                    action_matrix, path, effective_kfs = build_action_matrix_from_qr(
                        modified_data
                    )
                except ValueError as exc:
                    candidate_errors.append((pickup_position, str(exc)))
                    continue

                planning_cost = float(_path_planning_cost(effective_kfs, path))
                candidate_results.append({
                    "pickup_position": int(pickup_position),
                    "movement_cost": planning_cost,
                    "planning_cost": planning_cost,
                    "action_matrix": action_matrix,
                    "path": path,
                    "effective_kfs": effective_kfs,
                })

            if not candidate_results:
                error_text = "; ".join(
                    f"{position}号格: {message}"
                    for position, message in candidate_errors
                )
                raise ValueError(
                    "前三位 R2-KFS 场外预吸取后均无可行路径"
                    + (f": {error_text}" if error_text else "")
                )

            # 相同规划总代价时保留 candidate_positions 的先后顺序，即1号优先于3号。
            selected = min(
                candidate_results,
                key=lambda result: result["movement_cost"],
            )
            pickup_position = selected["pickup_position"]
            # -1->1/-1->3 是执行层约定的侧吸特殊行，不是普通相邻台阶。
            pickup_direction = (
                1
                if pickup_position in (1, 3)
                else _direction_code(ENTRY_FROM_POS, pickup_position)
            )
            pre_entry_rows = np.array(
                [
                    [ENTRY_FROM_POS, pickup_position, pickup_direction, 0, 1],
                    _entry_action_row(),
                ],
                dtype=int,
            )
            action_matrix = np.vstack(
                (pre_entry_rows, selected["action_matrix"])
            )
            return action_matrix, selected["path"], selected["effective_kfs"]
        except Exception:
            # 规划失败后扫码线程可能继续重试，必须避免再次减 1。
            race.R2_KFS_COUNT = original_r2_count
            race.REQUIRED_R2_PICKUP_COUNT = original_required_count
            raise


def build_plan_result_from_qr(qr_string):
    action_matrix, path, kfs = build_action_matrix_with_pre_entry_pickup(qr_string)
    return ChallengePlanResult(
        qr_data=str(qr_string).strip(),
        kfs=kfs,
        path=path,
        action_matrix=action_matrix,
    )


def print_action_matrix(action_matrix):
    """打印 n*5 动作矩阵和简要说明。"""
    print("\n[机器人动作矩阵 n*5]")
    print(f"列定义: {ACTION_MATRIX_COLUMNS}")
    print(action_matrix)

    print("\n[动作序列说明]")
    for index, row in enumerate(np.asarray(action_matrix, dtype=int), start=1):
        from_pos, to_pos, move_dir, height_action, grab_action = row.tolist()
        stair_text = "需要上下楼梯" if height_action == 1 else "不用上下楼梯"
        print(
            f"{index:02d}. {from_pos}->{to_pos} | "
            f"方向={MOVE_DIR_NAMES.get(move_dir, move_dir)} | "
            f"{stair_text} | "
            f"{GRAB_ACTION_NAMES.get(grab_action, grab_action)}"
        )


def visualize(kfs, path=None):
    return race.visualize(kfs, path)


def format_path(path):
    return race.format_path(path)


class ChallengeQRScanner:
    """
    后台 QR 检测线程。

    连续 stable_frame_count 帧识别到同一个有效 12 位二维码后，执行 race 规划，
    将 ChallengePlanResult 推入 result_queue。queue 对象由调用方持有，线程退出
    后其中的结果仍然保留，主线程可继续 get()。

    线程运行期间会持有 running_lock。主线程可在第一区域任务结束后调用
    wait_until_scanner_released() 阻塞等待该锁释放，再继续运行。
    """
    def __init__(
        self,
        result_queue=None,
        stable_frame_count=5,
        show_window=False,
        window_name="R2 QR Scanner",
        stop_after_success=True,
        loop_interval_sec=0.01,
        open_camera_kwargs=None,
        image_source=1,
        put_action_matrix_only=False,
        running_lock=None,
    ):
        self.result_queue = result_queue if result_queue is not None else queue.Queue()
        self.stable_frame_count = int(stable_frame_count)
        self.show_window = bool(show_window)
        self.window_name = str(window_name)
        self.stop_after_success = bool(stop_after_success)
        self.loop_interval_sec = float(loop_interval_sec)
        self.open_camera_kwargs = {} if open_camera_kwargs is None else dict(open_camera_kwargs)
        self.image_source = int(image_source)
        self.put_action_matrix_only = bool(put_action_matrix_only)
        self.running_lock = running_lock if running_lock is not None else SCANNER_RUNNING_LOCK

        self.stop_event = threading.Event()
        self.done_event = threading.Event()
        self.thread = None
        self.last_result = None
        self.last_error = None
        self.last_qr_data = None
        self.last_stable_count = 0

    def start(self):
        if self.thread is not None and self.thread.is_alive():
            return self.thread
        self.stop_event.clear()
        self.done_event.clear()
        self.thread = threading.Thread(
            target=self._run,
            daemon=True,
            name="challenge_qr_scanner",
        )
        self.thread.start()
        return self.thread

    def stop(self):
        self.stop_event.set()

    def join(self, timeout=None):
        if self.thread is not None:
            self.thread.join(timeout=timeout)
        return not (self.thread is not None and self.thread.is_alive())

    def wait_for_result(self, timeout=None):
        return self.result_queue.get(timeout=timeout)

    def wait_until_released(self, timeout=None):
        return wait_until_scanner_released(
            running_lock=self.running_lock,
            timeout=timeout,
        )

    def _put_result(self, result):
        self.last_result = result
        if self.put_action_matrix_only:
            self.result_queue.put(result.action_matrix)
        else:
            self.result_queue.put(result)

    def _run(self):
        pipeline = None
        stable_data = None
        stable_count = 0
        lock_acquired = False
        try:
            self.running_lock.acquire()
            lock_acquired = True
            pipeline = open_image_source(
                image_source=self.image_source,
                **self.open_camera_kwargs,
            )
            detector = create_qr_detector()

            while not self.stop_event.is_set():
                frame = get_color_frame(pipeline)
                if frame is None:
                    time.sleep(self.loop_interval_sec)
                    continue

                data = detect_qr_data(frame, detector=detector)
                if is_valid_qr_payload(data):
                    if data == stable_data:
                        stable_count += 1
                    else:
                        stable_data = data
                        stable_count = 1
                else:
                    stable_data = None
                    stable_count = 0

                self.last_qr_data = stable_data
                self.last_stable_count = stable_count

                if self.show_window and cv2 is not None:
                    cv2.imshow(self.window_name, frame)
                    if cv2.waitKey(1) == ord("q"):
                        self.stop_event.set()
                        break

                if stable_data is not None and stable_count >= self.stable_frame_count:
                    try:
                        result = build_plan_result_from_qr(stable_data)
                    except Exception as exc:
                        self.last_error = exc
                        stable_data = None
                        stable_count = 0
                        time.sleep(self.loop_interval_sec)
                        continue

                    self._put_result(result)
                    if self.stop_after_success:
                        self.stop_event.set()
                        break
                    stable_data = None
                    stable_count = 0

                time.sleep(self.loop_interval_sec)
        except Exception as exc:
            self.last_error = exc
        finally:
            if pipeline is not None:
                try:
                    pipeline.stop()
                except Exception:
                    pass
            if self.show_window and cv2 is not None:
                try:
                    cv2.destroyWindow(self.window_name)
                except Exception:
                    pass
            if lock_acquired:
                self.running_lock.release()
            self.done_event.set()


def wait_until_scanner_released(running_lock=SCANNER_RUNNING_LOCK, timeout=None):
    """
    阻塞等待后台扫码线程释放 running_lock。

    返回 True 表示已经拿到并立即释放锁；False 表示超时。
    """
    if timeout is None:
        acquired = running_lock.acquire()
    else:
        acquired = running_lock.acquire(timeout=float(timeout))
    if acquired:
        running_lock.release()
    return bool(acquired)


def start_background_qr_scanner(result_queue=None, **kwargs):
    scanner = ChallengeQRScanner(result_queue=result_queue, **kwargs)
    scanner.start()
    return scanner
