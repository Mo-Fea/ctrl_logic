import numpy as np

try:
    from . import meilin
except ImportError:
    import meilin


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
    0: "不夹取",
    1: "行进前抓目标格R2",
    2: "旁夹R2",
    3: "R1协助移除",
}


def qr_to_kfs(qr_string):
    """
    Convert a 12-character QR payload into the KFS dictionary used by meilin.py.

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


def _direction_code(from_pos, to_pos):
    """Calculate grid movement direction code from meilin.py position coordinates."""
    if from_pos == to_pos:
        return 0

    from_row, from_col, _ = meilin.pos_to_coord[int(from_pos)]
    to_row, to_col, _ = meilin.pos_to_coord[int(to_pos)]
    delta = (to_row - from_row, to_col - from_col)
    if delta not in MOVE_DIR_CODES:
        raise ValueError(f"位置 {from_pos} 到 {to_pos} 不是相邻格，无法生成动作矩阵")
    return MOVE_DIR_CODES[delta]


def _height_action(from_pos, to_pos):
    """Return stair action: 1=climb, 2=descend, 0=no height change/no movement."""
    if from_pos == to_pos:
        return 0

    from_height = meilin.pos_to_coord[int(from_pos)][2]
    to_height = meilin.pos_to_coord[int(to_pos)][2]
    diff = to_height - from_height
    if diff > 0:
        return 1
    if diff < 0:
        return 2
    return 0


def path_to_action_matrix(kfs, path):
    """
    Convert meilin.plan_path() output into an n*5 robot action matrix.

    Row format:
    [from_pos, to_pos, move_dir, height_action, grab_action]
    """
    if not path:
        return np.zeros((0, 5), dtype=int)

    is_valid, message = meilin.validate_path(kfs, path)
    if not is_valid:
        raise ValueError(f"无法生成动作矩阵：{message}")

    rows = []
    current_pos = path[0]
    taken_set = set()

    # If the entry cell contains an R2-KFS, meilin.py treats it as taken.
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

            grab_action = 3 if kfs.get(step) == "R1" else 0
            rows.append([
                current_pos,
                step,
                _direction_code(current_pos, step),
                _height_action(current_pos, step),
                grab_action,
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
                2,
            ])
            taken_set.add(to_pos)

        else:
            raise ValueError(f"未知路径步骤：{step}")

    if not rows:
        return np.zeros((0, 5), dtype=int)
    return np.array(rows, dtype=int)


def build_action_matrix_from_kfs(kfs):
    """
    Plan a route from an existing KFS dictionary and return the action matrix.

    Returns:
      action_matrix, path

    Raises:
      ValueError when no valid path can be found.
    """
    path = meilin.plan_path(kfs)
    if not path:
        raise ValueError("当前 KFS 布局下未找到可行路径")
    return path_to_action_matrix(kfs, path), path


def build_action_matrix_from_qr(qr_string):
    """
    Convert a QR payload directly into an action matrix.

    Returns:
      action_matrix, path, kfs
    """
    kfs = qr_to_kfs(qr_string)
    action_matrix, path = build_action_matrix_from_kfs(kfs)
    return action_matrix, path, kfs


def print_action_matrix(action_matrix):
    """Print the action matrix and a compact human-readable action list."""
    print("\n[机器人动作矩阵 n*5]")
    print(f"列定义: {ACTION_MATRIX_COLUMNS}")
    print(action_matrix)

    print("\n[动作序列说明]")
    for index, row in enumerate(np.asarray(action_matrix, dtype=int), start=1):
        from_pos, to_pos, move_dir, height_action, grab_action = row.tolist()
        if height_action == 1:
            stair_text = "上台阶"
        elif height_action == 2:
            stair_text = "下台阶"
        else:
            stair_text = "无台阶动作"

        print(
            f"{index:02d}. {from_pos}->{to_pos} | "
            f"方向={MOVE_DIR_NAMES.get(move_dir, move_dir)} | "
            f"{stair_text} | "
            f"{GRAB_ACTION_NAMES.get(grab_action, grab_action)}"
        )
