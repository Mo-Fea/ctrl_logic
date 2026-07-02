import heapq
import random
from collections.abc import Mapping
from itertools import count
from lib2 import position_backend


def _import_matplotlib():
    import matplotlib.pyplot as plt
    import matplotlib.patches as patches

    # 设置matplotlib支持中文显示
    plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'Arial Unicode MS']
    plt.rcParams['axes.unicode_minus'] = False
    return plt, patches

# ============================================
# 规则说明：
# 1. R1/R2/Fake 的场上数量由下方全局变量配置
# 2. 随机测试布局中，R1 放在允许区域，R2/Fake 放在树林内空位
# 3. 二维码布局只统一校验类型和数量，摆放区域规则由后续赛事逻辑处理
# 4. R2必须从2号进入场地
# 5. (x_index, y_index, height) 最后一位是高度，R2上下的高度差只能是20
# 6. R2 必须拿取的数量由 REQUIRED_R2_PICKUP_COUNT 配置
# 7. R1，R2都不能碰假的（fake）kfs
# 8. R2在梅林里上下台阶的时候要保证下一个台阶上无KFS，如果有要抓取，但是只能抓取R2能够抓取的
# 9. R1的KSF只能由R1自己抓取，如果R1的ksf挡住R2的路，则R1把R1的1kfs拿开让R2前进
# 10. R2从10号或12号位置出口侧离开，并最终移动到出口辅助点 13/15
# 11. R2如果要夹取他旁边的一个kfs但是不去那个台阶，也是可以的，这种情况用蓝线标出
# 12. 在路径尽可能短的情况下完成路径规划
# ============================================

# 位置编号 → (forward_index, left_index, height)
# 这里是与真实地图轴解耦的逻辑网格；方向码按当前红/蓝半场语义由
# get_direction_code() 统一计算，真实地图中的 yaw 映射由 lib2/tools.py 处理。
# 颜色：20-深绿色   40-正常绿    60-浅绿色
#
# 地图布局：
#          x=0      x=1      x=2      x=3
# y=2:    [1:40]   [4:60]   [7:40]   [10:20]   ← 10号出口侧
# y=1:    [2:20]   [5:40]   [8:60]   [11:40]   ← 入口在2号
# y=0:    [3:40]   [6:20]   [9:40]   [12:20]   ← 12号出口侧

# 赛事数量配置：比赛模式 1 为 2R1/2R2/1Fake，模式 2 为 3R1/3R2/1Fake。
# REQUIRED_R2_PICKUP_COUNT 独立表示 R2 完成路径前必须拿到的数量。
COMPETITION_KFS_COUNT_PROFILES = {
    1: (2, 2, 1),
    2: (3, 3, 1),
}
COMPETITION_MODE = 1
R1_KFS_COUNT, R2_KFS_COUNT, FAKE_KFS_COUNT = COMPETITION_KFS_COUNT_PROFILES[
    COMPETITION_MODE
]
REQUIRED_R2_PICKUP_COUNT = 2

# 路径规划代价配置。KFS 布局只覆盖 1-12 号格；13/15 只作为出口辅助点参与规划。
PATH_MOVE_COST = 1.0
PATH_REACH_COST = 0.0
PATH_ROTATION_COST = 1.0
PATH_R1_MOVE_EXTRA_COST = 0.5
PATH_INITIAL_YAW_DEG_RED = 90.0
PATH_INITIAL_YAW_DEG_BLUE = -90.0
PATH_COST_EPSILON = 1e-9


def configure_competition_mode(mode):
    """
    切换比赛例程对应的 KFS 数量配置。

    mode=1: 2个 R1-KFS、2个 R2-KFS、1个 Fake-KFS
    mode=2: 3个 R1-KFS、3个 R2-KFS、1个 Fake-KFS

    本函数不修改 REQUIRED_R2_PICKUP_COUNT。
    """
    if not isinstance(mode, int) or isinstance(mode, bool):
        raise ValueError(f"competition mode 必须是整数 1 或 2，当前为 {mode!r}")
    if mode not in COMPETITION_KFS_COUNT_PROFILES:
        raise ValueError(f"competition mode 必须是 1 或 2，当前为 {mode}")

    global COMPETITION_MODE
    global R1_KFS_COUNT, R2_KFS_COUNT, FAKE_KFS_COUNT

    r1_count, r2_count, fake_count = COMPETITION_KFS_COUNT_PROFILES[mode]
    COMPETITION_MODE = mode
    R1_KFS_COUNT = r1_count
    R2_KFS_COUNT = r2_count
    FAKE_KFS_COUNT = fake_count

    return {
        "competition_mode": COMPETITION_MODE,
        "r1_kfs_count": R1_KFS_COUNT,
        "r2_kfs_count": R2_KFS_COUNT,
        "fake_kfs_count": FAKE_KFS_COUNT,
        "required_r2_pickup_count": REQUIRED_R2_PICKUP_COUNT,
    }


def configure_kfs_counts(r1_count=None, r2_count=None, required_r2_pickup_count=None):
    """手动配置 R1/R2 数量和 R2 必须抓取数量；Fake 数量保持当前配置。"""
    global R1_KFS_COUNT, R2_KFS_COUNT, REQUIRED_R2_PICKUP_COUNT

    if r1_count is not None:
        R1_KFS_COUNT = int(r1_count)
    if r2_count is not None:
        R2_KFS_COUNT = int(r2_count)
    if required_r2_pickup_count is not None:
        REQUIRED_R2_PICKUP_COUNT = int(required_r2_pickup_count)

    valid, message = validate_kfs_count_config()
    if not valid:
        raise ValueError(message)

    return {
        "competition_mode": COMPETITION_MODE,
        "r1_kfs_count": R1_KFS_COUNT,
        "r2_kfs_count": R2_KFS_COUNT,
        "fake_kfs_count": FAKE_KFS_COUNT,
        "required_r2_pickup_count": REQUIRED_R2_PICKUP_COUNT,
    }

POS_TO_COORD_RED = {
    1: (0, 2, 40),
    2: (0, 1, 20),   # 入口位置
    3: (0, 0, 40),
    4: (1, 2, 60),
    5: (1, 1, 40),
    6: (1, 0, 20),
    7: (2, 2, 40),
    8: (2, 1, 60),
    9: (2, 0, 40),
    10: (3, 2, 20),  # 10号出口侧
    11: (3, 1, 40),
    12: (3, 0, 20),  # 12号出口侧
    13: (4, 2, 0),   # 10号对应出口辅助点
    15: (4, 0, 0),   # 12号对应出口辅助点
}



POS_TO_COORD_BLUE = {
    # Left field placeholders. Fill these with measured/logical coordinates if needed.
    1: (0, 0, 40),
    2: (0, 1, 20),   # 入口位置
    3: (0, 2, 40),
    4: (1, 0, 20),
    5: (1, 1, 40),
    6: (1, 2, 60),
    7: (2, 0, 40),
    8: (2, 1, 60),
    9: (2, 2, 40),
    10: (3, 0, 20),  # 10号出口侧
    11: (3, 1, 40),
    12: (3, 2, 20),  # 12号出口侧
    13: (4, 0, 0),   # 10号对应出口辅助点
    15: (4, 2, 0),   # 12号对应出口辅助点
}


def get_pos_to_coord():
    if position_backend.is_blue_field():
        return POS_TO_COORD_BLUE
    return POS_TO_COORD_RED


def get_coord_to_pos():
    return {coord: pos for pos, coord in get_pos_to_coord().items()}


class _FieldMapping(Mapping):
    def _mapping(self):
        return get_pos_to_coord()

    def __getitem__(self, key):
        return self._mapping()[key]

    def __iter__(self):
        return iter(self._mapping())

    def __len__(self):
        return len(self._mapping())

    def __contains__(self, key):
        return key in self._mapping()

    def items(self):
        return self._mapping().items()

    def keys(self):
        return self._mapping().keys()

    def values(self):
        return self._mapping().values()


class _CoordToPosMapping(Mapping):
    def _mapping(self):
        return get_coord_to_pos()

    def __getitem__(self, key):
        return self._mapping()[key]

    def __iter__(self):
        return iter(self._mapping())

    def __len__(self):
        return len(self._mapping())

    def __contains__(self, key):
        return key in self._mapping()

    def items(self):
        return self._mapping().items()

    def keys(self):
        return self._mapping().keys()

    def values(self):
        return self._mapping().values()


pos_to_coord = _FieldMapping()
coord_to_pos = _CoordToPosMapping()

GRID_X_COUNT = 5
GRID_Y_COUNT = 3
ENTRY_POS = 2  # R2从2号位置进入
EXIT_POSITIONS = [13, 15]  # R2最终移动到13或15出口辅助点
KFS_POSITIONS = list(range(1, 13))

# 内部位置（树林内，R2-KFS和假KFS放置区域）
INTERIOR_POS = [4, 5, 6, 7, 8, 9]
# 树林边靠近通道的位置（R1-KFS放置区域）
R1_ALLOWED_POS = [1, 2, 3, 4, 6, 7, 9, 10, 11, 12]

KFS_TYPES = ("R1", "R2", "low")


def configure_path_costs(
    move_cost=None,
    reach_cost=None,
    rotation_cost=None,
    r1_move_extra_cost=None,
    initial_yaw_red=None,
    initial_yaw_blue=None,
):
    """修改路径规划代价配置；参数为 None 时保持原值。"""
    global PATH_MOVE_COST
    global PATH_REACH_COST
    global PATH_ROTATION_COST
    global PATH_R1_MOVE_EXTRA_COST
    global PATH_INITIAL_YAW_DEG_RED
    global PATH_INITIAL_YAW_DEG_BLUE

    if move_cost is not None:
        PATH_MOVE_COST = float(move_cost)
    if reach_cost is not None:
        PATH_REACH_COST = float(reach_cost)
    if rotation_cost is not None:
        PATH_ROTATION_COST = float(rotation_cost)
    if r1_move_extra_cost is not None:
        PATH_R1_MOVE_EXTRA_COST = float(r1_move_extra_cost)
    if initial_yaw_red is not None:
        PATH_INITIAL_YAW_DEG_RED = float(initial_yaw_red)
    if initial_yaw_blue is not None:
        PATH_INITIAL_YAW_DEG_BLUE = float(initial_yaw_blue)

    return {
        "move_cost": float(PATH_MOVE_COST),
        "reach_cost": float(PATH_REACH_COST),
        "rotation_cost": float(PATH_ROTATION_COST),
        "r1_move_extra_cost": float(PATH_R1_MOVE_EXTRA_COST),
        "initial_yaw_red": float(PATH_INITIAL_YAW_DEG_RED),
        "initial_yaw_blue": float(PATH_INITIAL_YAW_DEG_BLUE),
    }


def get_direction_code(from_pos, to_pos):
    """按当前红/蓝半场语义计算 1-12 内相邻格方向码。"""
    if from_pos == to_pos:
        return 0

    from_x, from_y, _ = pos_to_coord[int(from_pos)]
    to_x, to_y, _ = pos_to_coord[int(to_pos)]
    delta = (to_x - from_x, to_y - from_y)

    if position_backend.is_blue_field():
        direction_codes = {
            (1, 0): 4,
            (-1, 0): 1,
            (0, 1): 3,
            (0, -1): 2,
        }
    else:
        direction_codes = {
            (1, 0): 1,
            (-1, 0): 4,
            (0, 1): 2,
            (0, -1): 3,
        }
    return direction_codes.get(delta, 0)


def _opposite_direction(direction):
    """返回四方向编号的反向方向。"""
    opposite_map = {
        1: 4,
        2: 3,
        3: 2,
        4: 1,
    }
    return opposite_map.get(int(direction), 0)


def get_move_facing_direction(from_pos, to_pos):
    """
    返回移动到 to_pos 时用于旋转代价和后续状态的朝向方向。

    普通移动/上楼：朝向实际移动方向。
    下楼移动：到达下一个较低台阶时，朝向按实际移动方向的反向计算。

    注意：这里不改变动作矩阵中的实际移动方向，只影响路径规划代价中的
    current_direction 状态。
    """
    move_direction = get_direction_code(from_pos, to_pos)
    if move_direction == 0:
        return 0

    from_height = pos_to_coord[int(from_pos)][2]
    to_height = pos_to_coord[int(to_pos)][2]
    if to_height < from_height:
        return _opposite_direction(move_direction)
    return move_direction


def _direction_to_yaw_deg(direction):
    direction = int(direction)
    if position_backend.is_blue_field():
        direction_to_yaw = {
            1: 90.0,
            2: 0.01,
            3: 180.0,
            4: -90.0,
        }
    else:
        direction_to_yaw = {
            1: 90.0,
            2: 180.0,
            3: 0.01,
            4: -90.0,
        }
    return direction_to_yaw.get(direction)


def _yaw_matches(left, right):
    return abs(float(left) - float(right)) <= 0.05


def get_initial_direction():
    initial_yaw = (
        PATH_INITIAL_YAW_DEG_BLUE
        if position_backend.is_blue_field()
        else PATH_INITIAL_YAW_DEG_RED
    )
    for direction in (1, 2, 3, 4):
        direction_yaw = _direction_to_yaw_deg(direction)
        if direction_yaw is not None and _yaw_matches(direction_yaw, initial_yaw):
            return direction
    raise ValueError(
        "初始航向角必须对应当前半场的四方向之一，"
        f"当前为 {initial_yaw}"
    )


def _rotation_step_cost(current_direction, target_direction):
    if int(current_direction) == int(target_direction):
        return 0.0
    return float(PATH_ROTATION_COST)


def _reach_step_cost(current_direction, target_direction):
    return (
        float(PATH_REACH_COST)
        + _rotation_step_cost(current_direction, target_direction)
    )


def _move_step_cost(current_direction, target_direction, next_pos, kfs):
    cost = (
        float(PATH_MOVE_COST)
        + _rotation_step_cost(current_direction, target_direction)
    )
    if kfs.get(next_pos) == "R1":
        cost += float(PATH_R1_MOVE_EXTRA_COST)
    return cost


def validate_kfs_count_config():
    """校验顶部的 KFS 数量配置。"""
    count_config = {
        "R1_KFS_COUNT": R1_KFS_COUNT,
        "R2_KFS_COUNT": R2_KFS_COUNT,
        "FAKE_KFS_COUNT": FAKE_KFS_COUNT,
        "REQUIRED_R2_PICKUP_COUNT": REQUIRED_R2_PICKUP_COUNT,
    }
    for name, value in count_config.items():
        if not isinstance(value, int) or isinstance(value, bool) or value < 0:
            return False, f"{name} 必须是非负整数，当前为 {value!r}"

    if REQUIRED_R2_PICKUP_COUNT > R2_KFS_COUNT:
        return False, (
            "REQUIRED_R2_PICKUP_COUNT 不能大于 R2_KFS_COUNT："
            f"{REQUIRED_R2_PICKUP_COUNT} > {R2_KFS_COUNT}"
        )

    total_count = R1_KFS_COUNT + R2_KFS_COUNT + FAKE_KFS_COUNT
    if total_count > len(KFS_POSITIONS):
        return False, f"KFS 总数 {total_count} 超过可放置格子数 {len(KFS_POSITIONS)}"

    return True, "KFS 数量配置合法"


def validate_kfs_layout(kfs):
    """
    校验 KFS 布局的类型和数量。

    本函数只校验数量，不限制 R1/R2/Fake 的摆放区域，
    因此可以接受前 1/2/3 号格出现 R2-KFS 的新赛事布局。
    """
    config_valid, config_message = validate_kfs_count_config()
    if not config_valid:
        return False, config_message
    if not isinstance(kfs, Mapping):
        return False, f"kfs 必须是映射类型，当前为 {type(kfs).__name__}"

    valid_positions = set(KFS_POSITIONS)
    extra_positions = sorted(position for position in kfs if position not in valid_positions)
    if extra_positions:
        return False, f"KFS 布局包含无效格子编号: {extra_positions}"

    invalid_values = {
        position: kfs.get(position)
        for position in valid_positions
        if kfs.get(position) not in (None, *KFS_TYPES)
    }
    if invalid_values:
        return False, f"KFS 布局包含无效类型: {invalid_values}"

    actual_counts = {
        "R1": sum(kfs.get(position) == "R1" for position in valid_positions),
        "R2": sum(kfs.get(position) == "R2" for position in valid_positions),
        "low": sum(kfs.get(position) == "low" for position in valid_positions),
    }
    expected_counts = {
        "R1": R1_KFS_COUNT,
        "R2": R2_KFS_COUNT,
        "low": FAKE_KFS_COUNT,
    }
    mismatches = [
        f"{kfs_type}={actual_counts[kfs_type]}(应为{expected_counts[kfs_type]})"
        for kfs_type in KFS_TYPES
        if actual_counts[kfs_type] != expected_counts[kfs_type]
    ]
    if mismatches:
        return False, "KFS 数量不符合当前配置: " + ", ".join(mismatches)

    return True, f"KFS 布局数量校验通过: {actual_counts}"


def get_grid_neighbors(pos):
    """获取某个位置的相邻位置"""
    if pos == 'start':
        return [ENTRY_POS]
    if pos == 'end':
        return []

    x, y, _ = pos_to_coord[pos]
    candidates = []
    for dx, dy in [(1, 0), (0, 1), (0, -1), (-1, 0)]:
        nx, ny = x + dx, y + dy
        for p, (px, py, ph) in pos_to_coord.items():
            if px == nx and py == ny:
                candidates.append(p)
                break
    return candidates


def can_move_to(current_pos, next_pos, kfs, taken_count, taken_set):
    """
    检查是否可以移动到下一个位置
    考虑高度差限制和KFS抓取规则
    taken_set: 已经被夹取或踩踏抓取的R2-KFS位置集合（这些位置视为空）
    """
    current_height = pos_to_coord[current_pos][2]
    next_height = pos_to_coord[next_pos][2]

    # 检查高度差是否为20
    height_diff = abs(next_height - current_height)
    if height_diff != 20:
        return False, "高度差必须为20"

    kfs_type = kfs.get(next_pos)

    # R1的KFS：R1会拿走，R2直接通过
    if kfs_type == 'R1':
        return True, "通过（R1-KFS已由R1移除）"

    # 假KFS：R2不能碰
    if kfs_type == 'low':
        return False, "假KFS不能碰"

    # R2的KFS
    if kfs_type == 'R2':
        # 如果已经被夹取过了，视为空位，可以直接通过
        if next_pos in taken_set:
            return True, "通过（R2-KFS已夹取）"
        # 检查是否已经拿满
        if taken_count >= REQUIRED_R2_PICKUP_COUNT:
            return False, f"已拿满{REQUIRED_R2_PICKUP_COUNT}个R2-KFS，无法再抓取"
        # 可以踩踏抓取
        return True, "抓取R2-KFS"

    # 空位置，可以通过
    return True, "通过"


def can_reach_to(current_pos, target_pos, kfs, taken_count, taken_set):
    """
    检查是否可以从当前位置夹取目标位置的R2-KFS（规则11：不移动到目标位置）
    条件：
    - 目标位置与当前位置网格相邻
    - 目标位置有R2-KFS且尚未被抓取
    - 抓取后不超过 REQUIRED_R2_PICKUP_COUNT 限制
    """
    # 检查是否网格相邻
    if target_pos not in get_grid_neighbors(current_pos):
        return False, "目标位置不相邻"

    # 检查目标位置是否有R2-KFS
    if kfs.get(target_pos) != 'R2':
        return False, "目标位置没有R2-KFS"

    # 检查是否已经被抓取
    if target_pos in taken_set:
        return False, "R2-KFS已被抓取"

    # 检查是否超过限制
    if taken_count >= REQUIRED_R2_PICKUP_COUNT:
        return False, f"已拿满{REQUIRED_R2_PICKUP_COUNT}个R2-KFS"

    return True, "夹取R2-KFS"


def generate_kfs():
    """
    生成KFS的随机放置
    遵循规则：
    - R1_KFS_COUNT 个 R1-KFS 放在 R1_ALLOWED_POS 中
    - R2_KFS_COUNT 个 R2-KFS 放在树林内的空置方块
    - FAKE_KFS_COUNT 个假 KFS 放在树林内剩余空位
    """
    config_valid, config_message = validate_kfs_count_config()
    if not config_valid:
        raise ValueError(config_message)

    kfs = {p: None for p in KFS_POSITIONS}

    if R1_KFS_COUNT > len(R1_ALLOWED_POS):
        raise ValueError(
            f"R1_KFS_COUNT={R1_KFS_COUNT} 超过 R1 可摆放位置数 {len(R1_ALLOWED_POS)}"
        )

    # R1 也可能占用树林内格子。先限制其占用数，避免数量配置本身可行，
    # 但随机到某个 R1 布局后没有足够空位放置 R2/Fake。
    r1_interior_allowed = [p for p in R1_ALLOWED_POS if p in INTERIOR_POS]
    r1_exterior_allowed = [p for p in R1_ALLOWED_POS if p not in INTERIOR_POS]
    required_interior_free = R2_KFS_COUNT + FAKE_KFS_COUNT
    max_r1_interior_count = min(
        R1_KFS_COUNT,
        len(r1_interior_allowed),
        len(INTERIOR_POS) - required_interior_free,
    )
    min_r1_interior_count = max(0, R1_KFS_COUNT - len(r1_exterior_allowed))
    if max_r1_interior_count < min_r1_interior_count:
        raise ValueError(
            "当前数量与随机摆放区域冲突："
            f"R1={R1_KFS_COUNT}, R2={R2_KFS_COUNT}, Fake={FAKE_KFS_COUNT}"
        )

    r1_interior_count = random.randint(
        min_r1_interior_count,
        max_r1_interior_count,
    )
    r1_positions = (
        random.sample(r1_interior_allowed, r1_interior_count)
        + random.sample(
            r1_exterior_allowed,
            R1_KFS_COUNT - r1_interior_count,
        )
    )
    for p in r1_positions:
        kfs[p] = 'R1'

    remaining = [p for p in INTERIOR_POS if kfs[p] is None]
    if R2_KFS_COUNT > len(remaining):
        raise ValueError(
            f"R2_KFS_COUNT={R2_KFS_COUNT} 超过当前树林内空位数 {len(remaining)}"
        )
    r2_positions = random.sample(remaining, R2_KFS_COUNT)
    for p in r2_positions:
        kfs[p] = 'R2'

    remaining = [p for p in INTERIOR_POS if kfs[p] is None]
    if FAKE_KFS_COUNT > len(remaining):
        raise ValueError(
            f"FAKE_KFS_COUNT={FAKE_KFS_COUNT} 超过当前树林内空位数 {len(remaining)}"
        )
    fake_positions = random.sample(remaining, FAKE_KFS_COUNT)
    for p in fake_positions:
        kfs[p] = 'low'

    return kfs, r2_positions


def plan_path_with_cost(kfs):
    """
    使用 Dijkstra 规划 R2 的最低总代价路径。

    路径格式说明：
    - 整数：R2实际移动到的位置，基础 cost = PATH_MOVE_COST。
    - 元组 (from_pos, 'reach', to_pos)：R2在from_pos夹取to_pos的KFS，
      基础 cost = PATH_REACH_COST。

    若当前朝向与下一操作方向不同，额外增加 PATH_ROTATION_COST。
    若移动目标格有 R1-KFS，额外增加 PATH_R1_MOVE_EXTRA_COST。
    """
    layout_valid, layout_message = validate_kfs_layout(kfs)
    if not layout_valid:
        raise ValueError(layout_message)

    # 初始化
    initial_taken_set = set()
    initial_taken_count = 0

    if kfs.get(ENTRY_POS) == 'R2':
        initial_taken_set.add(ENTRY_POS)
        initial_taken_count = 1

    initial_path = [ENTRY_POS]
    initial_direction = get_initial_direction()

    initial_taken_set_frozen = frozenset(initial_taken_set)
    initial_state = (
        ENTRY_POS,
        initial_taken_count,
        initial_taken_set_frozen,
        initial_direction,
    )
    best_cost = {initial_state: 0.0}
    push_counter = count()
    heap = [(
        0.0,
        next(push_counter),
        ENTRY_POS,
        initial_taken_count,
        initial_taken_set_frozen,
        initial_direction,
        initial_path,
    )]

    best_path = None
    best_path_cost = float('inf')

    while heap:
        (
            cost,
            _,
            current_pos,
            taken_count,
            taken_set_frozen,
            current_direction,
            path,
        ) = heapq.heappop(heap)
        taken_set = set(taken_set_frozen)

        state = (
            current_pos,
            taken_count,
            taken_set_frozen,
            current_direction,
        )

        # 跳过过期状态
        if cost > best_cost.get(state, float('inf')) + PATH_COST_EPSILON:
            continue

        # 提前终止
        if cost >= best_path_cost - PATH_COST_EPSILON:
            continue

        # 到达出口且已拿满
        if current_pos in EXIT_POSITIONS and taken_count == REQUIRED_R2_PICKUP_COUNT:
            if cost < best_path_cost - PATH_COST_EPSILON:
                best_path_cost = cost
                best_path = path.copy()
            continue

        neighbors = get_grid_neighbors(current_pos)

        # === 转移1: 夹取相邻的R2-KFS (cost = 0, 规则11) ===
        for target_pos in neighbors:
            can_reach, reason = can_reach_to(current_pos, target_pos, kfs,
                                            taken_count, taken_set)
            if not can_reach:
                continue

            reach_direction = get_direction_code(current_pos, target_pos)
            if reach_direction == 0:
                continue
            new_taken_count = taken_count + 1
            new_taken_set_frozen = frozenset(taken_set | {target_pos})
            new_state = (
                current_pos,
                new_taken_count,
                new_taken_set_frozen,
                reach_direction,
            )
            new_cost = cost + _reach_step_cost(
                current_direction,
                reach_direction,
            )

            if (
                new_state not in best_cost
                or new_cost < best_cost[new_state] - PATH_COST_EPSILON
            ):
                best_cost[new_state] = new_cost
                new_path = path + [(current_pos, 'reach', target_pos)]
                heapq.heappush(heap, (
                    new_cost,
                    next(push_counter),
                    current_pos,
                    new_taken_count,
                    new_taken_set_frozen,
                    reach_direction,
                    new_path,
                ))

        # === 转移2: 移动到相邻位置 (cost = 1) ===
        for next_pos in neighbors:
            can_move, reason = can_move_to(current_pos, next_pos, kfs,
                                           taken_count, taken_set)
            if not can_move:
                continue

            new_taken_count = taken_count
            new_taken_set = taken_set.copy()

            if kfs.get(next_pos) == 'R2' and next_pos not in taken_set:
                new_taken_count = taken_count + 1
                new_taken_set = taken_set | {next_pos}

            move_direction = get_direction_code(current_pos, next_pos)
            if move_direction == 0:
                continue
            move_facing_direction = get_move_facing_direction(current_pos, next_pos)
            if move_facing_direction == 0:
                continue
            new_taken_set_frozen = frozenset(new_taken_set)
            new_state = (
                next_pos,
                new_taken_count,
                new_taken_set_frozen,
                move_facing_direction,
            )
            new_cost = cost + _move_step_cost(
                current_direction,
                move_facing_direction,
                next_pos,
                kfs,
            )

            if (
                new_state not in best_cost
                or new_cost < best_cost[new_state] - PATH_COST_EPSILON
            ):
                best_cost[new_state] = new_cost
                new_path = path + [next_pos]
                heapq.heappush(heap, (
                    new_cost,
                    next(push_counter),
                    next_pos,
                    new_taken_count,
                    new_taken_set_frozen,
                    move_facing_direction,
                    new_path,
                ))

    return best_path, best_path_cost


def plan_path(kfs):
    path, _ = plan_path_with_cost(kfs)
    return path


def calculate_path_cost(kfs, path):
    """按当前代价配置复算一条合法路径的总代价。"""
    is_valid, message = validate_path(kfs, path)
    if not is_valid:
        raise ValueError(message)

    current_pos = path[0]
    current_direction = get_initial_direction()
    taken_set = set()
    taken_count = 0
    total_cost = 0.0

    if kfs.get(current_pos) == 'R2':
        taken_set.add(current_pos)
        taken_count = 1

    for step in path[1:]:
        if isinstance(step, int):
            move_direction = get_direction_code(current_pos, step)
            if move_direction == 0:
                raise ValueError(f"{current_pos}->{step} 不是相邻移动")
            move_facing_direction = get_move_facing_direction(current_pos, step)
            if move_facing_direction == 0:
                raise ValueError(f"{current_pos}->{step} 无法计算移动朝向")
            total_cost += _move_step_cost(
                current_direction,
                move_facing_direction,
                step,
                kfs,
            )
            if kfs.get(step) == 'R2' and step not in taken_set:
                taken_set.add(step)
                taken_count += 1
            current_pos = step
            current_direction = move_facing_direction
        elif isinstance(step, tuple) and len(step) == 3 and step[1] == 'reach':
            from_pos, _, to_pos = step
            if from_pos != current_pos:
                raise ValueError(
                    f"夹取起点 {from_pos} 与当前位置 {current_pos} 不一致"
                )
            reach_direction = get_direction_code(from_pos, to_pos)
            if reach_direction == 0:
                raise ValueError(f"{from_pos}->{to_pos} 不是相邻夹取")
            total_cost += _reach_step_cost(current_direction, reach_direction)
            taken_set.add(to_pos)
            taken_count += 1
            current_direction = reach_direction
        else:
            raise ValueError(f"未知路径步骤：{step}")

    return float(total_cost)


def get_move_positions(path):
    """从路径中提取实际移动位置序列（跳过夹取动作）"""
    return [step for step in path if isinstance(step, int)]


def get_reach_actions(path):
    """从路径中提取夹取动作列表 [(from_pos, 'reach', to_pos), ...]"""
    return [step for step in path
            if isinstance(step, tuple) and len(step) == 3 and step[1] == 'reach']


def get_taken_positions(kfs, path):
    """获取路径中所有被抓取的R2-KFS位置集合"""
    taken = set()
    for step in path:
        if isinstance(step, int) and kfs.get(step) == 'R2':
            taken.add(step)
        elif isinstance(step, tuple) and step[1] == 'reach':
            taken.add(step[2])
    return taken


def format_path(path):
    """将路径格式化为可读字符串"""
    parts = []
    for step in path:
        if isinstance(step, int):
            parts.append(str(step))
        elif isinstance(step, tuple) and step[1] == 'reach':
            parts.append(f"[夹取{step[2]}]")
    return ' → '.join(parts)


def get_action_sequence(kfs, path):
    """
    将路径转换为详细的行动序列
    返回每一步的动作描述列表（包含夹取动作）
    """
    if not path:
        return []

    actions = []
    taken_count = 0

    for i, step in enumerate(path):
        if isinstance(step, int):
            pos = step
            _, _, h = pos_to_coord[pos]
            height_label = {20: "深绿(20)", 40: "正常绿(40)", 60: "浅绿(60)"}.get(h, str(h))

            if i == 0:
                kfs_type = kfs.get(pos)
                if kfs_type == 'R2':
                    taken_count += 1
                    actions.append(
                        f"【入口】进入位置{pos} ({height_label}) → 发现R2-KFS，抓取！"
                        f"（已抓取 {taken_count}/{REQUIRED_R2_PICKUP_COUNT}）"
                    )
                else:
                    actions.append(f"【入口】进入位置{pos} ({height_label})")
            else:
                # 找到上一个移动位置
                prev_pos = None
                for j in range(i - 1, -1, -1):
                    if isinstance(path[j], int):
                        prev_pos = path[j]
                        break

                prev_h = pos_to_coord[prev_pos][2]
                direction = "上" if h > prev_h else "下"

                kfs_type = kfs.get(pos)
                if kfs_type == 'R2':
                    taken_count += 1
                    actions.append(
                        f"移动: {prev_pos}→{pos} ({direction}台阶, {height_label})"
                        f" → 踩踏抓取R2-KFS！（已抓取 {taken_count}/{REQUIRED_R2_PICKUP_COUNT}）"
                    )
                elif kfs_type == 'R1':
                    actions.append(
                        f"移动: {prev_pos}→{pos} ({direction}台阶, {height_label})"
                        f" → R1-KFS在此，R1已移除，直接通过"
                    )
                elif kfs_type == 'low':
                    actions.append(
                        f"移动: {prev_pos}→{pos} ({direction}台阶, {height_label})"
                        f" → ⚠假KFS，不可通过！（不应出现）"
                    )
                else:
                    actions.append(
                        f"移动: {prev_pos}→{pos} ({direction}台阶, {height_label})"
                        f" → 通过"
                    )

            # 检查是否到达出口（最后一步是移动且在出口位置）
            move_positions = get_move_positions(path)
            if pos in EXIT_POSITIONS and pos == move_positions[-1]:
                actions.append(
                    f"【出口】从位置{pos}离开场地（共抓取 {taken_count}/{REQUIRED_R2_PICKUP_COUNT} 个R2-KFS）"
                )

        elif isinstance(step, tuple) and step[1] == 'reach':
            from_pos, _, to_pos = step
            taken_count += 1
            actions.append(
                f"夹取: 在位置{from_pos}夹取位置{to_pos}的R2-KFS！"
                f"（不移动到目标位置，已抓取 {taken_count}/{REQUIRED_R2_PICKUP_COUNT}）"
            )

    return actions


def visualize(kfs, path=None):
    """可视化KFS放置和R2路径（白色=移动，蓝色虚线=夹取）"""
    try:
        plt, patches = _import_matplotlib()
    except (ImportError, AttributeError) as exc:
        filename = save_visualization_svg(kfs, path)
        print(f"[警告]: matplotlib 不可用，已改为生成 SVG: {filename}")
        print(f"[原因]: {exc}")
        return filename

    fig, ax = plt.subplots(figsize=(10, 7))
    ax.set_xlim(-1.3, 5.3)
    ax.set_ylim(-0.8, 2.8)
    ax.set_aspect('equal')

    # 画格子
    for pos, (x, y, height) in sorted(pos_to_coord.items()):
        if height == 20:
            face_color = '#006400'
        elif height == 40:
            face_color = '#228B22'
        elif height == 60:
            face_color = '#90EE90'
        else:
            face_color = 'white'

        rect = patches.Rectangle((x - 0.5, y - 0.5), 1, 1,
                                 linewidth=1.5, edgecolor='black',
                                 facecolor=face_color)
        ax.add_patch(rect)

        # 高度标签
        ax.text(x, y - 0.35, f"h={height}", ha='center', va='center',
                fontsize=7, color='white', alpha=0.7)

        # 位置编号 + 入口/出口标记
        label = f"{pos}"
        if pos == ENTRY_POS:
            label = f"{pos}\n(入口)"
        elif pos in EXIT_POSITIONS:
            label = f"{pos}\n(出口)"

        ax.text(x, y + 0.1, label, ha='center', va='center',
                fontsize=10, color='white', fontweight='bold')

    # 画入口箭头
    ax.annotate('', xy=(-0.55, 1), xytext=(-1.05, 1),
                arrowprops=dict(arrowstyle='->', color='red', lw=2.5))
    ax.text(-1.08, 1.15, 'R2 进入', ha='right', va='bottom',
            fontsize=11, color='red', fontweight='bold')

    # 画出口箭头
    for exit_pos in EXIT_POSITIONS:
        ex, ey, _ = pos_to_coord[exit_pos]
        ax.annotate('', xy=(ex + 0.95, ey), xytext=(ex + 0.5, ey),
                    arrowprops=dict(arrowstyle='->', color='blue', lw=2))
        ax.text(ex + 1.02, ey + 0.08, '出口', ha='left', va='bottom',
                fontsize=10, color='blue', fontweight='bold')

    # KFS 圆圈
    color_map = {'R1': '#a8d8ff', 'R2': '#ff6666', 'low': '#ffff66'}
    label_map = {'R1': 'R1', 'R2': 'R2', 'low': '假'}
    edge_color_map = {'R1': '#4488cc', 'R2': '#cc3333', 'low': '#cccc33'}

    for pos, typ in kfs.items():
        if typ is None:
            continue
        x, y, _ = pos_to_coord[pos]
        circle = patches.Circle((x, y), 0.3, color=color_map[typ],
                                alpha=0.85, zorder=5,
                                edgecolor=edge_color_map[typ], linewidth=2)
        ax.add_patch(circle)
        ax.text(x, y, label_map[typ], ha='center', va='center',
                fontsize=11, fontweight='bold', color='black', zorder=6)

    # 路径
    if path:
        move_positions = get_move_positions(path)
        reach_actions = get_reach_actions(path)

        # 画移动路径（白色实线）
        if move_positions:
            points = []
            for step in move_positions:
                x, y, _ = pos_to_coord[step]
                points.append((x, y))

            xs, ys = zip(*points)
            ax.plot(xs, ys, color='white', linewidth=4.5, alpha=0.9, zorder=10)
            ax.scatter(xs, ys, color='white', s=200, edgecolor='white', zorder=11)

        # 画夹取路径（蓝色虚线 + 菱形标记，规则11）
        for from_pos, _, to_pos in reach_actions:
            fx, fy, _ = pos_to_coord[from_pos]
            tx, ty, _ = pos_to_coord[to_pos]
            ax.plot([fx, tx], [fy, ty],
                    color='#4488FF', linewidth=3, linestyle='--', alpha=0.9, zorder=9)
            ax.scatter([tx], [ty], color='#4488FF', s=120, marker='D',
                       edgecolor='white', linewidth=1.5, zorder=11)

        # 顺序标签 - 只在移动位置上标注步数
        move_idx = 0
        total_moves = len(move_positions)

        for step in path:
            if isinstance(step, int):
                pos_num = step
                x, y, _ = pos_to_coord[pos_num]

                if move_idx == 0:
                    txt = "START"
                    offset_y, va = -0.25, 'top'
                elif move_idx == total_moves - 1:
                    txt = "END"
                    offset_y, va = 0.25, 'bottom'
                else:
                    txt = str(move_idx)
                    offset_y, va = 0.25, 'bottom'

                ax.text(x, y + offset_y, txt,
                        ha='center', va=va,
                        fontsize=9, fontweight='bold',
                        bbox=dict(facecolor='white', edgecolor='darkgreen',
                                  boxstyle='round,pad=0.3', alpha=0.9),
                        zorder=12)
                move_idx += 1

            elif isinstance(step, tuple) and step[1] == 'reach':
                _, _, to_pos = step
                tx, ty, _ = pos_to_coord[to_pos]
                ax.text(tx + 0.35, ty - 0.15, "夹取",
                        ha='left', va='top',
                        fontsize=8, fontweight='bold', color='#4488FF',
                        bbox=dict(facecolor='white', edgecolor='#4488FF',
                                  boxstyle='round,pad=0.2', alpha=0.9), zorder=12)

    # 图例
    legend_elements = [
        patches.Patch(facecolor='#006400', edgecolor='black', label='深绿 h=20'),
        patches.Patch(facecolor='#228B22', edgecolor='black', label='正常绿 h=40'),
        patches.Patch(facecolor='#90EE90', edgecolor='black', label='浅绿 h=60'),
        patches.Patch(facecolor='#a8d8ff', edgecolor='#4488cc', label='R1-KFS'),
        patches.Patch(facecolor='#ff6666', edgecolor='#cc3333', label='R2-KFS'),
        patches.Patch(facecolor='#ffff66', edgecolor='#cccc33', label='假KFS'),
        plt.Line2D([0], [0], color='white', linewidth=3, label='移动路径'),
        plt.Line2D([0], [0], color='#4488FF', linewidth=2, linestyle='--', label='夹取路径'),
    ]
    ax.legend(handles=legend_elements, loc='upper left', fontsize=8,
              framealpha=0.9)

    reach_count = len(get_reach_actions(path)) if path else 0
    ax.set_title("崇武探幽 - R2最短路径规划\n"
                 f"（从位置2进入，抓取{REQUIRED_R2_PICKUP_COUNT}个R2-KFS，从13/15出口离开）\n"
                 f"白色线=移动  蓝色虚线=夹取(共{reach_count}次)",
                 fontsize=13, pad=15)
    ax.set_xticks([0, 1, 2, 3, 4])
    ax.set_xticklabels(['x0', 'x1', 'x2', 'x3', 'x4'], fontsize=11)
    ax.set_yticks([0, 1, 2])
    ax.set_yticklabels(['-90deg', 'center', '+90deg'], fontsize=11)
    ax.grid(True, linestyle=':', alpha=0.3)

    plt.tight_layout()
    plt.show()


def save_visualization_svg(kfs, path=None, filename="race_visualization.svg"):
    """不依赖 matplotlib 的 SVG 路线图输出。"""
    width = 760
    height = 560
    margin_left = 110
    margin_top = 90
    cell = 105

    def sx(x):
        return margin_left + float(x) * cell

    def sy(y):
        return margin_top + (GRID_Y_COUNT - 1 - float(y)) * cell

    def esc(value):
        return (
            str(value)
            .replace("&", "&amp;")
            .replace("<", "&lt;")
            .replace(">", "&gt;")
            .replace('"', "&quot;")
        )

    height_color = {
        20: "#006400",
        40: "#228B22",
        60: "#90EE90",
        0: "#d8d8d8",
    }
    kfs_color = {"R1": "#a8d8ff", "R2": "#ff6666", "low": "#ffff66"}
    kfs_edge = {"R1": "#4488cc", "R2": "#cc3333", "low": "#cccc33"}
    kfs_label = {"R1": "R1", "R2": "R2", "low": "假"}

    lines = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#f3f4f6"/>',
        '<text x="30" y="34" font-size="22" font-family="sans-serif" font-weight="700" fill="#111827">R2 路径规划图</text>',
        '<text x="30" y="58" font-size="13" font-family="sans-serif" fill="#374151">白线=移动路径，蓝色虚线=夹取路径</text>',
    ]

    for pos, (x, y, h) in sorted(pos_to_coord.items()):
        px = sx(x)
        py = sy(y)
        fill = height_color.get(h, "#ffffff")
        lines.append(
            f'<rect x="{px - cell / 2:.1f}" y="{py - cell / 2:.1f}" width="{cell}" height="{cell}" '
            f'fill="{fill}" stroke="#111827" stroke-width="2"/>'
        )
        name = str(pos)
        if pos == ENTRY_POS:
            name = f"{pos} 入口"
        elif pos in EXIT_POSITIONS:
            name = f"{pos} 出口"
        text_fill = "#111827" if h == 60 or h == 0 else "#ffffff"
        lines.append(
            f'<text x="{px:.1f}" y="{py - 7:.1f}" font-size="18" font-family="sans-serif" '
            f'font-weight="700" text-anchor="middle" fill="{text_fill}">{esc(name)}</text>'
        )
        lines.append(
            f'<text x="{px:.1f}" y="{py + 20:.1f}" font-size="13" font-family="sans-serif" '
            f'text-anchor="middle" fill="{text_fill}">h={esc(h)}</text>'
        )

    # 入口和出口箭头。
    entry_x, entry_y, _ = pos_to_coord[ENTRY_POS]
    epx = sx(entry_x)
    epy = sy(entry_y)
    lines.append('<defs><marker id="arrow-red" markerWidth="10" markerHeight="10" refX="8" refY="3" orient="auto" markerUnits="strokeWidth"><path d="M0,0 L0,6 L9,3 z" fill="#dc2626"/></marker><marker id="arrow-blue" markerWidth="10" markerHeight="10" refX="8" refY="3" orient="auto" markerUnits="strokeWidth"><path d="M0,0 L0,6 L9,3 z" fill="#2563eb"/></marker></defs>')
    lines.append(
        f'<line x1="{epx - 95:.1f}" y1="{epy:.1f}" x2="{epx - 58:.1f}" y2="{epy:.1f}" '
        'stroke="#dc2626" stroke-width="4" marker-end="url(#arrow-red)"/>'
    )
    for exit_pos in EXIT_POSITIONS:
        ex, ey, _ = pos_to_coord[exit_pos]
        px = sx(ex)
        py = sy(ey)
        lines.append(
            f'<line x1="{px + 58:.1f}" y1="{py:.1f}" x2="{px + 95:.1f}" y2="{py:.1f}" '
            'stroke="#2563eb" stroke-width="4" marker-end="url(#arrow-blue)"/>'
        )

    if path:
        move_positions = get_move_positions(path)
        reach_actions = get_reach_actions(path)
        if len(move_positions) >= 2:
            points = []
            for step in move_positions:
                x, y, _ = pos_to_coord[step]
                points.append(f"{sx(x):.1f},{sy(y):.1f}")
            lines.append(
                f'<polyline points="{" ".join(points)}" fill="none" stroke="#ffffff" '
                'stroke-width="9" stroke-linecap="round" stroke-linejoin="round" opacity="0.95"/>'
            )
        for from_pos, _, to_pos in reach_actions:
            fx, fy, _ = pos_to_coord[from_pos]
            tx, ty, _ = pos_to_coord[to_pos]
            lines.append(
                f'<line x1="{sx(fx):.1f}" y1="{sy(fy):.1f}" x2="{sx(tx):.1f}" y2="{sy(ty):.1f}" '
                'stroke="#4488ff" stroke-width="5" stroke-dasharray="10 8" stroke-linecap="round"/>'
            )
            lines.append(
                f'<rect x="{sx(tx) - 9:.1f}" y="{sy(ty) - 9:.1f}" width="18" height="18" '
                'fill="#4488ff" stroke="#ffffff" stroke-width="2" transform="rotate(45 '
                f'{sx(tx):.1f} {sy(ty):.1f})"/>'
            )
        for index, step in enumerate(move_positions):
            x, y, _ = pos_to_coord[step]
            label = "S" if index == 0 else ("E" if index == len(move_positions) - 1 else str(index))
            lines.append(
                f'<circle cx="{sx(x):.1f}" cy="{sy(y):.1f}" r="17" fill="#ffffff" stroke="#14532d" stroke-width="2"/>'
            )
            lines.append(
                f'<text x="{sx(x):.1f}" y="{sy(y) + 5:.1f}" font-size="14" font-family="sans-serif" '
                f'font-weight="700" text-anchor="middle" fill="#14532d">{esc(label)}</text>'
            )

    for pos, typ in sorted(kfs.items()):
        if typ is None or pos not in pos_to_coord:
            continue
        x, y, _ = pos_to_coord[pos]
        px = sx(x)
        py = sy(y)
        lines.append(
            f'<circle cx="{px:.1f}" cy="{py:.1f}" r="25" fill="{kfs_color.get(typ, "#ffffff")}" '
            f'stroke="{kfs_edge.get(typ, "#111827")}" stroke-width="3" opacity="0.9"/>'
        )
        lines.append(
            f'<text x="{px:.1f}" y="{py + 5:.1f}" font-size="16" font-family="sans-serif" '
            f'font-weight="700" text-anchor="middle" fill="#111827">{esc(kfs_label.get(typ, typ))}</text>'
        )

    legend_y = height - 92
    legend_items = [
        ("#006400", "h=20"),
        ("#228B22", "h=40"),
        ("#90EE90", "h=60"),
        ("#d8d8d8", "h=0/不可走"),
        ("#ff6666", "R2"),
        ("#a8d8ff", "R1"),
        ("#ffff66", "假"),
    ]
    for index, (color, label) in enumerate(legend_items):
        x = 30 + index * 100
        lines.append(f'<rect x="{x}" y="{legend_y}" width="18" height="18" fill="{color}" stroke="#111827"/>')
        lines.append(
            f'<text x="{x + 24}" y="{legend_y + 14}" font-size="13" font-family="sans-serif" fill="#111827">{esc(label)}</text>'
        )

    lines.append("</svg>")
    with open(filename, "w", encoding="utf-8") as fp:
        fp.write("\n".join(lines))
    return filename


def count_steps(path):
    """计算路径的上下台阶次数（仅计算移动步数，不含夹取动作）"""
    move_positions = get_move_positions(path)
    if not move_positions or len(move_positions) < 2:
        return {'up': 0, 'down': 0, 'total': 0}

    up_count = 0
    down_count = 0

    for i in range(len(move_positions) - 1):
        current_height = pos_to_coord[move_positions[i]][2]
        next_height = pos_to_coord[move_positions[i + 1]][2]

        if next_height > current_height:
            up_count += 1
        elif next_height < current_height:
            down_count += 1

    return {'up': up_count, 'down': down_count, 'total': up_count + down_count}


def validate_path(kfs, path):
    """验证路径是否满足所有规则（包括夹取动作）"""
    layout_valid, layout_message = validate_kfs_layout(kfs)
    if not layout_valid:
        return False, layout_message

    if not path:
        return False, "路径为空"

    # 第一步必须是位置（整数）
    if not isinstance(path[0], int):
        return False, "路径第一步必须是位置编号"

    if path[0] != ENTRY_POS:
        return False, f"起点必须是位置{ENTRY_POS}"

    current_pos = path[0]
    taken_set = set()
    taken_count = 0

    # 入口位置如果有R2-KFS，自动抓取
    if kfs.get(current_pos) == 'R2':
        taken_set.add(current_pos)
        taken_count = 1

    for i in range(1, len(path)):
        step = path[i]

        if isinstance(step, int):
            # 移动动作
            can_move, reason = can_move_to(current_pos, step, kfs,
                                           taken_count, taken_set)
            if not can_move:
                return False, f"第{i}步移动失败: {reason}"

            if kfs.get(step) == 'R2' and step not in taken_set:
                taken_set.add(step)
                taken_count += 1

            current_pos = step

        elif isinstance(step, tuple) and len(step) == 3 and step[1] == 'reach':
            from_pos, _, to_pos = step
            if from_pos != current_pos:
                return False, (f"第{i}步夹取的起始位置{from_pos}"
                               f"与当前位置{current_pos}不符")

            can_reach, reason = can_reach_to(current_pos, to_pos, kfs,
                                             taken_count, taken_set)
            if not can_reach:
                return False, f"第{i}步夹取失败: {reason}"

            taken_set.add(to_pos)
            taken_count += 1

        else:
            return False, f"第{i}步格式无效: {step}"

    # 检查终点
    if current_pos not in EXIT_POSITIONS:
        return False, f"终点必须是位置{EXIT_POSITIONS}之一，当前在{current_pos}"

    # 检查抓取数量
    if taken_count != REQUIRED_R2_PICKUP_COUNT:
        return False, (f"必须抓取{REQUIRED_R2_PICKUP_COUNT}个R2-KFS，"
                       f"实际抓取了{taken_count}个")

    return True, "路径验证通过 ✓"


def test():
    """连续测试10000次，统计路径规划结果"""
    print("=" * 60)
    print("崇武探幽 - R2路径规划系统 批量测试（含夹取策略）")
    print("=" * 60)

    num_tests = 10000
    success_count = 0
    failure_count = 0

    pickup_distribution = {}
    path_lengths = []
    reach_distribution = {}
    exit_distribution = {10: 0, 12: 0}
    steps_up = []
    steps_down = []
    steps_total = []

    # 记录失败案例
    failure_examples = []

    print(f"\n开始测试 {num_tests} 次...")
    print("-" * 60)

    for i in range(num_tests):
        kfs, r2_pos = generate_kfs()
        path = plan_path(kfs)

        if path:
            success_count += 1

            taken_positions = get_taken_positions(kfs, path)
            pickup_count = len(taken_positions)
            pickup_distribution[pickup_count] = pickup_distribution.get(pickup_count, 0) + 1

            move_count = len(get_move_positions(path))
            path_lengths.append(move_count)

            reach_count = len(get_reach_actions(path))
            reach_distribution[reach_count] = reach_distribution.get(reach_count, 0) + 1

            move_positions = get_move_positions(path)
            exit_pos = move_positions[-1]
            if exit_pos in exit_distribution:
                exit_distribution[exit_pos] += 1

            steps = count_steps(path)
            steps_up.append(steps['up'])
            steps_down.append(steps['down'])
            steps_total.append(steps['total'])
        else:
            failure_count += 1
            if len(failure_examples) < 5:
                failure_examples.append({
                    'kfs': {p: t for p, t in kfs.items() if t is not None},
                    'r2_pos': r2_pos
                })

        if (i + 1) % 2000 == 0:
            print(f"已测试 {i + 1}/{num_tests} 次...")

    # 计算统计结果
    success_rate = (success_count / num_tests) * 100
    avg_path_length = sum(path_lengths) / len(path_lengths) if path_lengths else 0
    min_path_length = min(path_lengths) if path_lengths else 0
    max_path_length = max(path_lengths) if path_lengths else 0

    avg_steps_up = sum(steps_up) / len(steps_up) if steps_up else 0
    avg_steps_down = sum(steps_down) / len(steps_down) if steps_down else 0
    avg_steps_total = sum(steps_total) / len(steps_total) if steps_total else 0
    min_steps_total = min(steps_total) if steps_total else 0
    max_steps_total = max(steps_total) if steps_total else 0

    # 显示统计结果
    print("\n" + "=" * 60)
    print("【测试结果统计】")
    print("=" * 60)
    print(f"总测试次数: {num_tests}")
    print(f"成功次数: {success_count} ({success_rate:.2f}%)")
    print(f"失败次数: {failure_count} ({100 - success_rate:.2f}%)")

    if failure_examples:
        print("\n【失败案例示例】")
        for idx, ex in enumerate(failure_examples):
            print(f"  案例{idx + 1}: KFS放置={ex['kfs']}, R2-KFS位置={ex['r2_pos']}")

    print("\n【R2-KFS抓取数量分布】")
    for count in sorted(pickup_distribution.keys()):
        freq = pickup_distribution[count]
        percentage = (freq / success_count) * 100 if success_count > 0 else 0
        print(f"  抓取 {count} 个: {freq} 次 ({percentage:.2f}%)")

    print("\n【夹取次数分布（规则11）】")
    for count in sorted(reach_distribution.keys()):
        freq = reach_distribution[count]
        percentage = (freq / success_count) * 100 if success_count > 0 else 0
        label = "无夹取" if count == 0 else f"{count}次夹取"
        print(f"  {label}: {freq} 次 ({percentage:.2f}%)")

    print("\n【移动步数统计（不含夹取，规则12最短路径）】")
    print(f"  平均移动步数: {avg_path_length:.2f} 步")
    print(f"  最少移动步数: {min_path_length} 步")
    print(f"  最多移动步数: {max_path_length} 步")

    print("\n【上下台阶次数统计】")
    print(f"  平均上台阶次数: {avg_steps_up:.2f} 次")
    print(f"  平均下台阶次数: {avg_steps_down:.2f} 次")
    print(f"  平均总台阶次数: {avg_steps_total:.2f} 次")
    print(f"  最少总台阶次数: {min_steps_total} 次")
    print(f"  最多总台阶次数: {max_steps_total} 次")

    print("\n【出口位置分布】")
    for exit_pos in sorted(exit_distribution.keys()):
        count = exit_distribution[exit_pos]
        percentage = (count / success_count) * 100 if success_count > 0 else 0
        print(f"  位置 {exit_pos}: {count} 次 ({percentage:.2f}%)")

    print("\n" + "=" * 60)
    return {
        'total_tests': num_tests,
        'success_count': success_count,
        'failure_count': failure_count,
        'success_rate': success_rate,
    }


def run_single_visualization():
    """单次随机可视化：生成KFS放置，规划路径并显示可视化"""
    print("=" * 60)
    print("崇武探幽 - R2路径规划系统 单次可视化（含夹取策略）")
    print("=" * 60)

    # 生成KFS放置
    kfs, r2_pos = generate_kfs()

    print("\n【KFS放置情况】")
    print("-" * 40)
    for pos in range(1, 13):
        typ = kfs[pos]
        if typ is not None:
            x, y, h = pos_to_coord[pos]
            type_label = {'R1': 'R1-KFS（R1抓取）', 'R2': 'R2-KFS（R2抓取）',
                          'low': '假KFS（不可触碰）'}.get(typ, typ)
            print(f"  位置 {pos:2d} (x={x},y={y},h={h}): {type_label}")

    # 规划路径
    path = plan_path(kfs)

    if path:
        move_positions = get_move_positions(path)
        reach_actions = get_reach_actions(path)
        taken_positions = get_taken_positions(kfs, path)

        print(f"\n【规划路径】")
        print(f"  路径: {format_path(path)}")
        print(f"  移动步数: {len(move_positions)} 步")
        print(f"  夹取次数: {len(reach_actions)} 次")
        print(f"  抓取的R2-KFS位置: {sorted(taken_positions)}")
        print(f"  抓取数量: {len(taken_positions)} 个")

        # 区分夹取和踩踏抓取
        reach_taken = {step[2] for step in reach_actions}
        step_taken = taken_positions - reach_taken
        if reach_taken:
            print(f"    其中夹取: {sorted(reach_taken)}")
        if step_taken:
            print(f"    其中踩踏: {sorted(step_taken)}")

        # 上下台阶
        steps = count_steps(path)
        print(f"  上台阶: {steps['up']} 次, 下台阶: {steps['down']} 次, "
              f"总计: {steps['total']} 次")

        # 验证路径
        is_valid, message = validate_path(kfs, path)
        print(f"  路径验证: {message}")

        # 详细行动序列
        print(f"\n【R2详细行动序列】")
        print("-" * 50)
        actions = get_action_sequence(kfs, path)
        for action in actions:
            print(f"  {action}")

        # 可视化
        print("\n正在生成可视化图表...")
        visualize(kfs, path)
    else:
        print("\n⚠ 未找到满足条件的路径！")
        print("可能原因：假KFS阻挡了所有可行路径")


def run_all_routes_analysis():
    """
    分析所有可能的KFS放置组合，找出R2的可行路线模式
    """
    print("=" * 60)
    print("崇武探幽 - 全路线模式分析（含夹取策略）")
    print("=" * 60)

    total = 0
    success = 0
    num_samples = 50000
    print(f"\n随机采样 {num_samples} 种KFS放置组合...")

    path_set = set()
    no_path_configs = []
    reach_usage_count = 0

    for i in range(num_samples):
        kfs, r2_pos = generate_kfs()
        path = plan_path(kfs)

        total += 1
        if path:
            success += 1
            # 用移动位置作为路径key（去重）
            path_key = tuple(get_move_positions(path))
            path_set.add(path_key)
            if get_reach_actions(path):
                reach_usage_count += 1
        else:
            if len(no_path_configs) < 3:
                no_path_configs.append({p: t for p, t in kfs.items() if t is not None})

    print(f"\n【分析结果】")
    print(f"  总采样数: {total}")
    print(f"  成功找到路径: {success} ({success / total * 100:.2f}%)")
    print(f"  使用夹取策略的路径: {reach_usage_count} ({reach_usage_count / success * 100:.2f}%)" if success else "")
    print(f"  去重后不同移动路径数: {len(path_set)}")

    if no_path_configs:
        print(f"\n【无法找到路径的案例】")
        for idx, cfg in enumerate(no_path_configs):
            print(f"  案例{idx + 1}: {cfg}")

    # 统计路径长度分布（移动步数）
    path_lens = [len(p) for p in path_set]
    if path_lens:
        print(f"\n【去重后移动步数分布】")
        from collections import Counter
        len_dist = Counter(path_lens)
        for length in sorted(len_dist.keys()):
            print(f"  {length}步路径: {len_dist[length]} 种")

        # 显示最短路径
        min_len = min(path_lens)
        shortest_paths = [p for p in path_set if len(p) == min_len]
        print(f"\n【最短路径（{min_len}步移动）】")
        for idx, p in enumerate(shortest_paths[:5]):
            print(f"  路径{idx + 1}: {' → '.join(map(str, p))}")
        if len(shortest_paths) > 5:
            print(f"  ... 共 {len(shortest_paths)} 种")

    print("\n" + "=" * 60)


def qr_to_kfs(qr_string):
    """
    将 12 位手动输入字符转换为 KFS 布局。

    编码：
      0=None
      1=R1
      2=R2
      3=low/Fake
    """
    data = "" if qr_string is None else str(qr_string).strip()
    if len(data) != 12 or any(char not in "0123" for char in data):
        raise ValueError("二维码字符必须是 12 位，且只能包含 0/1/2/3")

    mapping = {
        "0": None,
        "1": "R1",
        "2": "R2",
        "3": "low",
    }
    return {index + 1: mapping[char] for index, char in enumerate(data)}


def print_current_kfs_count_config():
    print(
        "当前数量配置："
        f"R1={R1_KFS_COUNT}, "
        f"R2={R2_KFS_COUNT}, "
        f"Fake={FAKE_KFS_COUNT}, "
        f"需要抓取R2={REQUIRED_R2_PICKUP_COUNT}"
    )


def prompt_int(prompt, default=None):
    while True:
        text = input(prompt).strip()
        if text == "" and default is not None:
            return int(default)
        try:
            value = int(text)
        except ValueError:
            print("请输入整数")
            continue
        if value < 0:
            print("请输入非负整数")
            continue
        return value


def prompt_kfs_count_config():
    """程序开始前询问数量配置。空输入则保留当前默认值。"""
    print("请配置KFS数量（直接回车保留默认值）")
    while True:
        r1_count = prompt_int(
            f"请输入R1数量（默认{R1_KFS_COUNT}）：",
            default=R1_KFS_COUNT,
        )
        r2_count = prompt_int(
            f"请输入R2数量（默认{R2_KFS_COUNT}）：",
            default=R2_KFS_COUNT,
        )
        required_r2_count = prompt_int(
            f"请输入需要抓取的R2数量（默认{REQUIRED_R2_PICKUP_COUNT}）：",
            default=REQUIRED_R2_PICKUP_COUNT,
        )
        try:
            config = configure_kfs_counts(
                r1_count=r1_count,
                r2_count=r2_count,
                required_r2_pickup_count=required_r2_count,
            )
        except ValueError as exc:
            print(f"数量配置错误：{exc}")
            print("请重新输入")
            continue
        print_current_kfs_count_config()
        return config


def print_kfs_layout(kfs):
    print("\n【KFS放置】")
    for pos in KFS_POSITIONS:
        typ = kfs[pos]
        if typ is not None:
            x, y, h = pos_to_coord[pos]
            type_label = {
                "R1": "R1-KFS（R1抓取）",
                "R2": "R2-KFS（R2抓取）",
                "low": "假KFS（不可触碰）",
            }.get(typ, typ)
            print(f"  位置 {pos:2d} (x={x},y={y},h={h}): {type_label}")


def print_plan_result(kfs, path):
    if not path:
        print("\n⚠ 未找到满足条件的路径！")
        print("可能原因：假KFS阻挡了所有可行路径")
        return

    move_positions = get_move_positions(path)
    reach_actions = get_reach_actions(path)
    taken_positions = get_taken_positions(kfs, path)

    print(f"\n【规划路径】")
    print(f"  路径: {format_path(path)}")
    print(f"  移动步数: {len(move_positions)} 步")
    print(f"  夹取次数: {len(reach_actions)} 次")
    print(f"  抓取的R2-KFS位置: {sorted(taken_positions)}")
    print(f"  抓取数量: {len(taken_positions)} 个")

    reach_taken = {step[2] for step in reach_actions}
    step_taken = taken_positions - reach_taken
    if reach_taken:
        print(f"    其中夹取: {sorted(reach_taken)}")
    if step_taken:
        print(f"    其中踩踏: {sorted(step_taken)}")

    steps = count_steps(path)
    print(f"  上台阶: {steps['up']} 次, 下台阶: {steps['down']} 次, "
          f"总计: {steps['total']} 次")

    is_valid, message = validate_path(kfs, path)
    print(f"  路径验证: {message}")

    print(f"\n【R2详细行动序列】")
    print("-" * 50)
    actions = get_action_sequence(kfs, path)
    for action in actions:
        print(f"  {action}")


def plan_path_with_pre_entry_processing(qr_string):
    """
    按 challenge_lib 的入口前三位逻辑预处理后再调用路径规划。

    - 前三位没有 2：直接用原始布局规划。
    - 第二位是 2：优先预吸取 2 号位。
    - 否则在 1/3 号位中选择一个预吸取；两者都有时选规划代价更低者。
    - 发生预吸取时，R2 总数和需要抓取 R2 数量都减 1，再对修改后的布局规划。
    """
    data = "" if qr_string is None else str(qr_string).strip()
    original_kfs = qr_to_kfs(data)
    layout_valid, layout_message = validate_kfs_layout(original_kfs)
    if not layout_valid:
        raise ValueError(layout_message)

    pre_entry_r2_positions = [
        position
        for position, value in enumerate(data[:3], start=1)
        if value == "2"
    ]
    if not pre_entry_r2_positions:
        return {
            "pre_entry_pickup": False,
            "pickup_position": None,
            "original_kfs": original_kfs,
            "effective_kfs": original_kfs,
            "path": plan_path(original_kfs),
            "planning_cost": None,
        }

    global R2_KFS_COUNT
    global REQUIRED_R2_PICKUP_COUNT

    original_r2_count = R2_KFS_COUNT
    original_required_count = REQUIRED_R2_PICKUP_COUNT
    if original_r2_count <= 0:
        raise ValueError("R2_KFS_COUNT 必须大于 0 才能执行入口预吸取")
    if original_required_count <= 0:
        raise ValueError("REQUIRED_R2_PICKUP_COUNT 必须大于 0 才能执行入口预吸取")

    R2_KFS_COUNT = original_r2_count - 1
    REQUIRED_R2_PICKUP_COUNT = original_required_count - 1

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
                effective_kfs = qr_to_kfs(modified_data)
                layout_valid, layout_message = validate_kfs_layout(effective_kfs)
                if not layout_valid:
                    raise ValueError(layout_message)
                path = plan_path(effective_kfs)
                if not path:
                    raise ValueError("当前 KFS 布局下未找到可行路径")
                planning_cost = float(calculate_path_cost(effective_kfs, path))
            except ValueError as exc:
                candidate_errors.append((pickup_position, str(exc)))
                continue

            candidate_results.append({
                "pre_entry_pickup": True,
                "pickup_position": int(pickup_position),
                "original_kfs": original_kfs,
                "effective_kfs": effective_kfs,
                "path": path,
                "planning_cost": planning_cost,
            })

        if not candidate_results:
            error_text = "; ".join(
                f"{position}号格: {message}"
                for position, message in candidate_errors
            )
            raise ValueError(
                "前三位 R2-KFS 入口预吸取后均无可行路径"
                + (f": {error_text}" if error_text else "")
            )

        return min(
            candidate_results,
            key=lambda result: result["planning_cost"],
        )
    except Exception:
        R2_KFS_COUNT = original_r2_count
        REQUIRED_R2_PICKUP_COUNT = original_required_count
        raise


def run_manual_qr_planning():
    """模式4：手动输入 12 位字符并执行路径规划。"""
    print("=" * 60)
    print("崇武探幽 - 手动12位字符路径规划")
    print("=" * 60)
    print_current_kfs_count_config()
    print("字符规则：0=空，1=R1，2=R2，3=Fake/low")

    while True:
        qr_string = input("请输入符合当前数量配置的12位字符：").strip()
        try:
            plan_result = plan_path_with_pre_entry_processing(qr_string)
        except ValueError as exc:
            print(f"输入或规划错误：{exc}")
            continue
        break

    original_kfs = plan_result["original_kfs"]
    kfs = plan_result["effective_kfs"]
    path = plan_result["path"]

    print_kfs_layout(original_kfs)
    if plan_result["pre_entry_pickup"]:
        print(
            "\n【入口前三位预处理】"
            f"\n  预吸取位置: {plan_result['pickup_position']}"
            f"\n  预处理后数量配置: R2={R2_KFS_COUNT}, "
            f"需要抓取R2={REQUIRED_R2_PICKUP_COUNT}"
        )
        print("\n【预处理后KFS放置】")
        print_kfs_layout(kfs)
    else:
        print("\n【入口前三位预处理】无")

    print_plan_result(kfs, path)
    if path:
        print("\n正在生成可视化图表...")
        visualize(kfs, path)


# ============================================
# 主程序入口
# ============================================
if __name__ == "__main__":
    print("崇武探幽 - R2行动路线规划系统")
    print("=" * 40)
    prompt_kfs_count_config()
    print("=" * 40)
    print("1. 批量测试（10000次随机KFS放置）")
    print("2. 单次可视化（随机生成+路径+图表）")
    print("3. 全路线模式分析")
    print("4. 手动输入12位字符路径规划")
    print("=" * 40)

    mode = input("选择运行模式（1/2/3/4）: ").strip()

    if mode == '1':
        test()
    elif mode == '2':
        run_single_visualization()
    elif mode == '3':
        run_all_routes_analysis()
    elif mode == '4':
        run_manual_qr_planning()
    else:
        print("无效选择，默认运行单次可视化")
        run_single_visualization()
