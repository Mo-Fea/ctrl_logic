import numpy as np

try:
    from . import meilin
    from . import race
except ImportError:
    import meilin
    import race


TASK_CHALLENGE = "challenge"  # 挑战赛：KFS 数量由 race.py 全局变量配置
TASK_COMBAT = "combat"        # 对抗赛：更多KFS，使用 meilin.py

TASK_TYPE_ALIASES = {
    TASK_CHALLENGE: TASK_CHALLENGE,
    "race": TASK_CHALLENGE,
    "route1": TASK_CHALLENGE,
    "tiaozhan": TASK_CHALLENGE,
    "挑战赛": TASK_CHALLENGE,
    TASK_COMBAT: TASK_COMBAT,
    "meilin": TASK_COMBAT,
    "route": TASK_COMBAT,
    "duikang": TASK_COMBAT,
    "对抗赛": TASK_COMBAT,
}


# Robot action matrix columns: one row is one sequential action.
ACTION_MATRIX_COLUMNS = [
    "from_pos",
    "to_pos",
    "move_dir",
    "height_action",
    "grab_action",
]

MOVE_DIR_CODES = {
    (0, 0): 0,
    (1, 0): 1,
    (0, 1): 2,
    (0, -1): 3,
    (-1, 0): 4,
}

MOVE_DIR_NAMES = {
    0: "原地",
    1: "前方",
    2: "+90度/左",
    3: "-90度/右",
    4: "后方",
}

GRAB_ACTION_NAMES = {
    0: "不抓取",
    1: "抓取",
}

EXIT_ACTION_ROWS = {
    10: [10, 13, 1, 1, 0],
    12: [12, 15, 1, 1, 0],
}


def normalize_task_type(task_type=TASK_CHALLENGE):
    task_key = str(task_type).strip().lower()
    normalized = TASK_TYPE_ALIASES.get(task_key)
    if normalized is None:
        raise ValueError(
            f"未知任务类型: {task_type}. "
            f"可选: {TASK_CHALLENGE}/{TASK_COMBAT}/race/meilin/route1/route"
        )
    return normalized


def get_task_planner(task_type=TASK_CHALLENGE):
    task_type = normalize_task_type(task_type)
    if task_type == TASK_CHALLENGE:
        return race
    if task_type == TASK_COMBAT:
        return meilin
    raise ValueError(f"未知任务类型: {task_type}")


def qr_to_kfs(qr_string):
    """
    Convert a 12-character QR payload into the KFS dictionary used by race.py.

    Character mapping:
    0 -> empty
    1 -> R1
    2 -> R2
    3 -> low/fake
    Other characters are treated as empty for compatibility with the old route.py.
    """
    qr_string = "" if qr_string is None else str(qr_string).strip()
    mapping = {"1": "R1", "2": "R2", "3": "low", "0": None}
    kfs = {}

    for index in range(12):
        pos = index + 1
        char = qr_string[index] if index < len(qr_string) else "0"
        kfs[pos] = mapping.get(char, None)

    return kfs


def _direction_code(from_pos, to_pos, task_type=TASK_CHALLENGE):
    """Calculate grid movement direction code from the selected task planner."""
    if from_pos == to_pos:
        return 0

    planner = get_task_planner(task_type)
    from_a, from_b, _ = planner.pos_to_coord[int(from_pos)]
    to_a, to_b, _ = planner.pos_to_coord[int(to_pos)]
    delta = (to_a - from_a, to_b - from_b)
    if delta not in MOVE_DIR_CODES:
        raise ValueError(f"位置 {from_pos} 到 {to_pos} 不是相邻格，无法生成动作矩阵")
    return MOVE_DIR_CODES[delta]


def _height_action(from_pos, to_pos, task_type=TASK_CHALLENGE):
    """Return stair action: 1=needs stair action, 0=no height change/no movement."""
    if from_pos == to_pos:
        return 0

    planner = get_task_planner(task_type)
    from_height = planner.pos_to_coord[int(from_pos)][2]
    to_height = planner.pos_to_coord[int(to_pos)][2]
    if to_height - from_height != 0:
        return 1
    return 0


def _append_exit_action(rows):
    if not rows:
        return rows

    last_to_pos = int(rows[-1][1])
    exit_row = EXIT_ACTION_ROWS.get(last_to_pos)
    if exit_row is not None:
        rows.append(exit_row.copy())
    return rows


def path_to_action_matrix(kfs, path, task_type=TASK_CHALLENGE):
    """
    Convert planner output into an n*5 robot action matrix.

    Row format:
    [from_pos, to_pos, move_dir, height_action, grab_action]
    """
    task_type = normalize_task_type(task_type)
    planner = get_task_planner(task_type)

    if not path:
        return np.zeros((0, 5), dtype=int)

    is_valid, message = planner.validate_path(kfs, path)
    if not is_valid:
        raise ValueError(f"无法生成动作矩阵：{message}")

    rows = []
    current_pos = path[0]
    taken_set = set()

    # If the entry cell contains an R2-KFS, the planner treats it as taken.
    if kfs.get(current_pos) == "R2":
        rows.append([current_pos, current_pos, 0, 0, 1])
        taken_set.add(current_pos)

    for step in path[1:]:
        if isinstance(step, int):
            if kfs.get(step) == "R2" and step not in taken_set:
                rows.append([
                    current_pos,
                    step,
                    _direction_code(current_pos, step, task_type=task_type),
                    0,
                    1,
                ])
                taken_set.add(step)

            rows.append([
                current_pos,
                step,
                _direction_code(current_pos, step, task_type=task_type),
                _height_action(current_pos, step, task_type=task_type),
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
                _direction_code(from_pos, to_pos, task_type=task_type),
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


def build_action_matrix_from_kfs(kfs, task_type=TASK_CHALLENGE):
    """
    Plan a route from an existing KFS dictionary and return the action matrix.

    Returns:
      action_matrix, path

    Raises:
      ValueError when no valid path can be found.
    """
    task_type = normalize_task_type(task_type)
    planner = get_task_planner(task_type)
    path = planner.plan_path(kfs)
    if not path:
        raise ValueError("当前 KFS 布局下未找到可行路径")
    return path_to_action_matrix(kfs, path, task_type=task_type), path


def build_action_matrix_from_qr(qr_string, task_type=TASK_CHALLENGE):
    """
    Convert a QR payload directly into an action matrix.

    Returns:
      action_matrix, path, kfs
    """
    kfs = qr_to_kfs(qr_string)
    action_matrix, path = build_action_matrix_from_kfs(kfs, task_type=task_type)
    return action_matrix, path, kfs


def print_action_matrix(action_matrix):
    """Print the action matrix and a compact human-readable action list."""
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
