import random
from collections import deque


def _import_matplotlib():
    import matplotlib.pyplot as plt
    import matplotlib.patches as patches

    # 设置matplotlib支持中文显示
    plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'Arial Unicode MS']
    plt.rcParams['axes.unicode_minus'] = False
    return plt, patches

# ============================================
# 规则说明：
# 1. 2个R1的武术秘籍(R1KFS)由对方参赛队员放置在树林边靠近通道的任何方块上
# 2. 2个R2的武术秘籍(R2KFS)由对方参赛队员放置在树林内的任何空置方块上
# 3. 1个假KFS由对方参赛队员放置在树林内任何空置的方块上，但不得放在靠近入口
# 4. R2必须从2号进入场地
# 5. (x_index, y_index, height) 最后一位是高度，R2上下的高度差只能是20
# 6. R2拿一个R2-KFS，R1全拿
# 7. R1，R2都不能碰假的（fake）kfs
# 8. R2在梅林里上下台阶的时候要保证下一个台阶上无KFS，如果有要抓取，但是只能抓取R2能够抓取的
# 9. R1的KSF只能由R1自己抓取，如果R1的ksf挡住R2的路，则R1把R1的1kfs拿开让R2前进
# 10. R2从10号或12号位置出口侧离开；lib2 中真正的出口辅助点是 13/15，这里只规划 1-12
# 11. R2如果要夹取他旁边的一个kfs但是不去那个台阶，也是可以的，这种情况用蓝线标出
# 12. 在路径尽可能短的情况下完成路径规划
# ============================================

# 位置编号 → (x_index, y_index, height)
# 与 lib2/module.py 的梅林矩阵保持一致：
#   x+ 方向为前方，方向码 1
#   y+ 方向为 +90deg/左，方向码 2
#   y- 方向为 -90deg/右，方向码 3
#   x- 方向为后方，方向码 4
# 颜色：20-深绿色   40-正常绿    60-浅绿色
#
# 地图布局：
#          x=0      x=1      x=2      x=3
# y=2:    [1:40]   [4:20]   [7:40]   [10:20]   ← 10号出口侧
# y=1:    [2:20]   [5:40]   [8:60]   [11:40]   ← 入口在2号
# y=0:    [3:40]   [6:60]   [9:40]   [12:20]   ← 12号出口侧

max_get_R2 = 1  # R2只需要拿1个R2-KFS

pos_to_coord = {
    1: (0, 2, 40),
    2: (0, 1, 20),   # 入口位置
    3: (0, 0, 40),
    4: (1, 2, 20),
    5: (1, 1, 40),
    6: (1, 0, 60),
    7: (2, 2, 40),
    8: (2, 1, 60),
    9: (2, 0, 40),
    10: (3, 2, 20),  # 10号出口侧
    11: (3, 1, 40),
    12: (3, 0, 20)   # 12号出口侧
}

coord_to_pos = {v: k for k, v in pos_to_coord.items()}

GRID_X_COUNT = 4
GRID_Y_COUNT = 3
ENTRY_POS = 2  # R2从2号位置进入
EXIT_POSITIONS = [10, 12]  # R2从10号或12号位置出口侧离开

# 内部位置（树林内，R2-KFS和假KFS放置区域）
INTERIOR_POS = [4, 5, 6, 7, 8, 9]
# 树林边靠近通道的位置（R1-KFS放置区域）
R1_ALLOWED_POS = [1, 2, 3, 4, 6, 7, 9, 10, 11, 12]


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
        if 0 <= nx < GRID_X_COUNT and 0 <= ny < GRID_Y_COUNT:
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
        if taken_count >= max_get_R2:
            return False, f"已拿满{max_get_R2}个R2-KFS，无法再抓取"
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
    - 抓取后不超过max_get_R2限制
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
    if taken_count >= max_get_R2:
        return False, f"已拿满{max_get_R2}个R2-KFS"

    return True, "夹取R2-KFS"


def generate_kfs():
    """
    生成KFS的随机放置
    遵循规则：
    - 2个R1的KFS放在树林边靠近通道的位置 {1,2,3,4,6,7,9,10,11,12} 中随机选2个
    - 2个R2的KFS放在树林内的空置方块
    - 1个假KFS放在树林内但不在入口附近（不在1,2,3）
    """
    kfs = {p: None for p in range(1, 13)}

    # 2个R1的KFS放在树林边靠近通道的位置 {1,2,3,4,6,7,9,10,11,12} 中随机选2个
    r1_positions = random.sample(R1_ALLOWED_POS, 2)
    for p in r1_positions:
        kfs[p] = 'R1'

    # 2个R2的KFS放在树林内的空置方块
    remaining = [p for p in INTERIOR_POS if kfs[p] is None]
    r2_positions = random.sample(remaining, 2)
    for p in r2_positions:
        kfs[p] = 'R2'

    # 1个假KFS放在树林内但不在入口附近（不在1,2,3）
    remaining = [p for p in INTERIOR_POS if kfs[p] is None]
    if remaining:
        low_pos = random.choice(remaining)
        kfs[low_pos] = 'low'

    return kfs, r2_positions


def plan_path(kfs):
    """
    使用0-1 BFS算法规划R2的最短路径（规则12）

    路径格式说明：
    - 整数：R2实际移动到的位置，cost = 1
    - 元组 (from_pos, 'reach', to_pos)：R2在from_pos夹取to_pos的KFS，cost = 0（规则11）

    夹取动作不增加路径步数，移动动作增加1步。
    0-1 BFS保证找到移动步数最少的路径。
    """
    # 初始化
    initial_taken_set = set()
    initial_taken_count = 0

    if kfs.get(ENTRY_POS) == 'R2':
        initial_taken_set.add(ENTRY_POS)
        initial_taken_count = 1

    initial_path = [ENTRY_POS]

    # 0-1 BFS
    initial_state = (ENTRY_POS, initial_taken_count, frozenset(initial_taken_set))
    best_cost = {initial_state: 0}
    dq = deque([(ENTRY_POS, initial_taken_count, frozenset(initial_taken_set),
                 initial_path, 0)])

    best_path = None
    best_path_cost = float('inf')

    while dq:
        current_pos, taken_count, taken_set_frozen, path, cost = dq.popleft()
        taken_set = set(taken_set_frozen)

        state = (current_pos, taken_count, taken_set_frozen)

        # 跳过过期状态
        if cost > best_cost.get(state, float('inf')):
            continue

        # 提前终止
        if cost >= best_path_cost:
            continue

        # 到达出口且已拿满
        if current_pos in EXIT_POSITIONS and taken_count == max_get_R2:
            if cost < best_path_cost:
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

            new_taken_count = taken_count + 1
            new_taken_set_frozen = frozenset(taken_set | {target_pos})
            new_state = (current_pos, new_taken_count, new_taken_set_frozen)
            new_cost = cost

            if new_state not in best_cost or new_cost < best_cost[new_state]:
                best_cost[new_state] = new_cost
                new_path = path + [(current_pos, 'reach', target_pos)]
                dq.appendleft((current_pos, new_taken_count, new_taken_set_frozen,
                               new_path, new_cost))

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

            new_taken_set_frozen = frozenset(new_taken_set)
            new_state = (next_pos, new_taken_count, new_taken_set_frozen)
            new_cost = cost + 1

            if new_state not in best_cost or new_cost < best_cost[new_state]:
                best_cost[new_state] = new_cost
                new_path = path + [next_pos]
                dq.append((next_pos, new_taken_count, new_taken_set_frozen,
                           new_path, new_cost))

    return best_path


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
                        f"（已抓取 {taken_count}/{max_get_R2}）"
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
                        f" → 踩踏抓取R2-KFS！（已抓取 {taken_count}/{max_get_R2}）"
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
                    f"【出口】从位置{pos}离开场地（共抓取 {taken_count}/{max_get_R2} 个R2-KFS）"
                )

        elif isinstance(step, tuple) and step[1] == 'reach':
            from_pos, _, to_pos = step
            taken_count += 1
            actions.append(
                f"夹取: 在位置{from_pos}夹取位置{to_pos}的R2-KFS！"
                f"（不移动到目标位置，已抓取 {taken_count}/{max_get_R2}）"
            )

    return actions


def visualize(kfs, path=None):
    """可视化KFS放置和R2路径（白色=移动，蓝色虚线=夹取）"""
    plt, patches = _import_matplotlib()
    fig, ax = plt.subplots(figsize=(10, 7))
    ax.set_xlim(-1.3, 4.3)
    ax.set_ylim(-0.8, 2.8)
    ax.set_aspect('equal')

    # 画格子
    for x in range(GRID_X_COUNT):
        for y in range(GRID_Y_COUNT):
            pos = None
            height = None
            for p, (px, py, ph) in pos_to_coord.items():
                if px == x and py == y:
                    pos = p
                    height = ph
                    break

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
                 f"（从位置2进入，抓取{max_get_R2}个R2-KFS，从10/12出口侧离开）\n"
                 f"白色线=移动  蓝色虚线=夹取(共{reach_count}次)",
                 fontsize=13, pad=15)
    ax.set_xticks([0, 1, 2, 3])
    ax.set_xticklabels(['x0', 'x1', 'x2', 'x3'], fontsize=11)
    ax.set_yticks([0, 1, 2])
    ax.set_yticklabels(['-90deg', 'center', '+90deg'], fontsize=11)
    ax.grid(True, linestyle=':', alpha=0.3)

    plt.tight_layout()
    plt.show()


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
    if taken_count != max_get_R2:
        return False, (f"必须抓取{max_get_R2}个R2-KFS，"
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


# ============================================
# 主程序入口
# ============================================
if __name__ == "__main__":
    print("崇武探幽 - R2行动路线规划系统")
    print("=" * 40)
    print("1. 批量测试（10000次随机KFS放置）")
    print("2. 单次可视化（随机生成+路径+图表）")
    print("3. 全路线模式分析")
    print("=" * 40)

    mode = input("选择运行模式（1/2/3）: ").strip()

    if mode == '1':
        test()
    elif mode == '2':
        run_single_visualization()
    elif mode == '3':
        run_all_routes_analysis()
    else:
        print("无效选择，默认运行单次可视化")
        run_single_visualization()
