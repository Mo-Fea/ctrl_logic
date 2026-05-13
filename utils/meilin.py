import random
from collections import deque

def _import_matplotlib():
    import matplotlib.pyplot as plt
    import matplotlib.patches as patches

    # 设置matplotlib支持中文显示
    plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'Arial Unicode MS']
    plt.rcParams['axes.unicode_minus'] = False  # 解决负号显示问题
    return plt, patches

# ============================================
# 规则说明：
# 1. 3个R1的武术秘籍(R1KFS)由对方参赛队员放置在树林边靠近通道的任何方块上 {1,2,3,4,6,7,9,10,11,12}
# 2. 4个R2的武术秘籍(R2KFS)由对方参赛队员放置在树林内的任何空置方块上
# 3. 1个假KFS由对方参赛队员放置在树林内任何空置的方块上，但不得放在靠近入口-> 除了{1,2,3}
# 4. R2必须从2号进入场地
# 5. (row, col, height) 最后一位是高度，R2上下的高度差只能是20
# 6. R2拿两个R2-KFS，R1全拿
# 7. R1，R2都不能碰假的（fake）kfs
# 8. R2在梅林里上下台阶的时候要保证下一个台阶上无KFS，如果有要抓取，但是只能抓取R2能够抓取的
# 9. R1的KSF只能由R1自己抓取，如果R1的ksf挡住R2的路，则R1把R1的1kfs拿开让R2前进
# 10. R2从10号或12号位置出口（靠近通道的方块）
# 11. R2如果要夹取他旁边的一个kfs但是不去那个台阶，也是可以的，这种情况用蓝线标出
# 12. 在路径尽可能短的情况下完成路径规划
# ============================================

# 位置编号 → (row, col, height)  
# row:0=最上(1,2,3) → row:3=最下(10,11,12)
# col:0=Left, col:1=Center, col:2=Right
# 颜色：20-深绿色   40-正常绿    60-浅绿色



max_get_R2 = 2  # R2最多只能拿2个R2-KFS


pos_to_coord = {
    1: (0, 2, 40),  
    2: (0, 1, 20),   # 中上 - 入口位置
    3: (0, 0, 40),   
    4: (1, 2, 20),
    5: (1, 1, 40),
    6: (1, 0, 60),
    7: (2, 2, 40),
    8: (2, 1, 60),
    9: (2, 0, 40),
    10: (3, 2, 20),  # 右下（出）
    11: (3, 1, 40),  # 中下
    12: (3, 0, 20)   # 左下（出）
}

coord_to_pos = {v: k for k, v in pos_to_coord.items()}

GRID_ROWS = 4
GRID_COLS = 3
ENTRY_POS = 2  # R2从2号位置进入
EXIT_POSITIONS = [10, 12]  # R2从10号或12号位置出口

def get_grid_neighbors(pos):
    """获取某个位置的网格相邻位置（不考虑高度差）"""
    if pos == 'start':
        return [ENTRY_POS]
    if pos == 'end':
        return []
    
    r, c, _ = pos_to_coord[pos]
    candidates = []
    for dr, dc in [(-1,0), (1,0), (0,-1), (0,1)]:
        nr, nc = r + dr, c + dc
        if 0 <= nr < GRID_ROWS and 0 <= nc < GRID_COLS:
            for p, (pr, pc, ph) in pos_to_coord.items():
                if pr == nr and pc == nc:
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
    
    # 检查下一个位置的KFS类型
    kfs_type = kfs.get(next_pos)
    
    # 如果下一个位置有R1的KFS，R2可以直接通过（R1会拿走）
    if kfs_type == 'R1':
        return True, "通过（R1-KFS已移除）"
    
    # 如果下一个位置有假KFS，R2不能通过
    if kfs_type == 'low':
        return False, "假KFS不能碰"
    
    # 如果下一个位置有R2的KFS
    if kfs_type == 'R2':
        # 如果已经被夹取过了，视为空位，可以直接通过
        if next_pos in taken_set:
            return True, "通过（R2-KFS已夹取）"
        # 检查是否已经拿了max_get_R2个R2-KFS
        if taken_count >= max_get_R2:
            return False, f"已拿满{max_get_R2}个R2-KFS"
        # 必须抓取（踩踏抓取）
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
    - 3个R1的KFS放在树林边靠近通道的位置 {1,2,3,4,6,7,9,10,11,12} 中任意3个
    - 4个R2的KFS放在树林内的空置方块
    - 1个假KFS放在树林内但不在入口附近（不在1,2,3）
    """
    kfs = {p: None for p in range(1, 13)}
    
    # R1的KFS放在树林边靠近通道的位置 {1,2,3,4,6,7,9,10,11,12} 中随机选3个
    r1_allowed = [1, 2, 3, 4, 6, 7, 9, 10, 11, 12]
    r1_pos = random.sample(r1_allowed, 3)
    for p in r1_pos:
        kfs[p] = 'R1'
    
    # R2的KFS放在树林内的空置方块（随机选择4个）
    remaining = [p for p in range(1, 13) if kfs[p] is None]
    r2_pos = random.sample(remaining, 4)
    for p in r2_pos:
        kfs[p] = 'R2'
    
    # 假KFS放在树林内但不在入口附近（不在1,2,3）
    remaining = [p for p in range(1, 13) if kfs[p] is None]
    low_possible = [p for p in remaining if p not in [1, 2, 3]]
    if low_possible:
        low_pos = random.choice(low_possible)
        kfs[low_pos] = 'low'
    
    return kfs, r2_pos

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
    
    # 入口位置如果有R2-KFS，自动抓取
    if kfs.get(ENTRY_POS) == 'R2':
        initial_taken_set.add(ENTRY_POS)
        initial_taken_count = 1
    
    initial_path = [ENTRY_POS]
    
    # 0-1 BFS: cost 0 的转移加到 deque 左端，cost 1 的转移加到右端
    # 状态: (位置, 已抓取数量, frozenset(已抓取集合))
    initial_state = (ENTRY_POS, initial_taken_count, frozenset(initial_taken_set))
    best_cost = {initial_state: 0}  # state -> 已知最小cost
    dq = deque([(ENTRY_POS, initial_taken_count, frozenset(initial_taken_set),
                 initial_path, 0)])
    
    best_path = None
    best_path_cost = float('inf')
    
    while dq:
        current_pos, taken_count, taken_set_frozen, path, cost = dq.popleft()
        taken_set = set(taken_set_frozen)
        
        state = (current_pos, taken_count, taken_set_frozen)
        
        # 跳过已经找到更优路径的过期状态
        if cost > best_cost.get(state, float('inf')):
            continue
        
        # 提前终止
        if cost >= best_path_cost:
            continue
        
        # 检查是否到达出口条件
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
            new_cost = cost  # 夹取不增加路径步数
            
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
            
            # 如果目标位置有R2-KFS且未被夹取，则踩踏抓取
            if kfs.get(next_pos) == 'R2' and next_pos not in taken_set:
                new_taken_count = taken_count + 1
                new_taken_set = taken_set | {next_pos}
            
            new_taken_set_frozen = frozenset(new_taken_set)
            new_state = (next_pos, new_taken_count, new_taken_set_frozen)
            new_cost = cost + 1  # 移动增加路径步数
            
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

def visualize(kfs, path=None):
    """可视化KFS放置和R2路径（白色=移动，蓝色虚线=夹取）"""
    plt, patches = _import_matplotlib()
    fig, ax = plt.subplots(figsize=(7, 9))
    ax.set_xlim(-0.6, 2.6)
    ax.set_ylim(-0.6, 3.6)
    ax.set_aspect('equal')
    
    # 画格子 + 边框
    for r in range(GRID_ROWS):
        for c in range(GRID_COLS):
            pos = None
            height = None
            for p, (pr, pc, ph) in pos_to_coord.items():
                if pr == r and pc == c:
                    pos = p
                    height = ph
                    break
            
            if height == 20:
                face_color = '#006400'  # 深绿色
            elif height == 40:
                face_color = '#228B22'  # 正常绿
            elif height == 60:
                face_color = '#90EE90'  # 浅绿色
            else:
                face_color = 'white'
            
            rect = patches.Rectangle((c-0.5, 3.5-r-0.5), 1, 1,
                                    linewidth=1.2, edgecolor='black', facecolor=face_color)
            ax.add_patch(rect)
            
            label = f"{pos}"
            if pos == ENTRY_POS:
                label = f"{pos}\n(入口)"
            elif pos in EXIT_POSITIONS:
                label = f"{pos}\n(出口)"
            
            ax.text(c, 3.5-r, label, ha='center', va='center',
                   fontsize=9, color='gray', fontweight='bold')
    
    # KFS 圆圈
    color_map = {'R1': '#a8d8ff', 'R2': '#ff9999', 'low': '#ffff99'}
    label_map = {'R1': 'R1', 'R2': 'R2', 'low': 'Fake'}
    
    for pos, typ in kfs.items():
        if typ is None:
            continue
        r, c, _ = pos_to_coord[pos]
        circle = patches.Circle((c, 3.5-r), 0.32, color=color_map[typ], alpha=0.75, zorder=5)
        ax.add_patch(circle)
        ax.text(c, 3.5-r, label_map[typ], ha='center', va='center',
               fontsize=10, fontweight='bold', color='black', zorder=6)
    
    # 路径
    if path:
        move_positions = get_move_positions(path)
        reach_actions = get_reach_actions(path)
        
        # 画移动路径（白色实线）
        if move_positions:
            points = []
            for step in move_positions:
                r, c, _ = pos_to_coord[step]
                points.append((c, 3.5 - r))
            
            xs, ys = zip(*points)
            ax.plot(xs, ys, color='white', linewidth=4, alpha=0.9, zorder=10)
            ax.scatter(xs, ys, color='white', s=180, edgecolor='white', zorder=11)
        
        # 画夹取路径（蓝色虚线 + 菱形标记，规则11）
        for from_pos, _, to_pos in reach_actions:
            fr, fc, _ = pos_to_coord[from_pos]
            tr, tc, _ = pos_to_coord[to_pos]
            ax.plot([fc, tc], [3.5-fr, 3.5-tr],
                   color='#4488FF', linewidth=3, linestyle='--', alpha=0.9, zorder=9)
            # 在夹取目标位置画菱形标记
            ax.scatter([tc], [3.5-tr], color='#4488FF', s=120, marker='D',
                      edgecolor='white', linewidth=1.5, zorder=11)
        
        # 顺序标签 - 只在移动位置上标注步数
        move_idx = 0
        total_moves = len(move_positions)
        
        for step in path:
            if isinstance(step, int):
                pos_num = step
                r, c, _ = pos_to_coord[pos_num]
                x, y = c, 3.5 - r
                
                if move_idx == 0:
                    txt = "START"
                    offset_y, va = -0.22, 'top'
                elif move_idx == total_moves - 1:
                    txt = "END"
                    offset_y, va = 0.22, 'bottom'
                else:
                    txt = str(move_idx)
                    offset_y, va = 0.22, 'bottom'
                
                ax.text(x, y + offset_y, txt,
                       ha='center', va=va,
                       fontsize=10, fontweight='bold',
                       bbox=dict(facecolor='white', edgecolor='darkgreen',
                                boxstyle='round,pad=0.4', alpha=0.9), zorder=12)
                move_idx += 1
            
            elif isinstance(step, tuple) and step[1] == 'reach':
                _, _, to_pos = step
                tr, tc, _ = pos_to_coord[to_pos]
                # 在夹取目标旁标注 "夹取"
                ax.text(tc + 0.35, 3.5 - tr - 0.15, "夹取",
                       ha='left', va='top',
                       fontsize=8, fontweight='bold', color='#4488FF',
                       bbox=dict(facecolor='white', edgecolor='#4488FF',
                                boxstyle='round,pad=0.2', alpha=0.9), zorder=12)
    
    # 装饰
    reach_count = len(get_reach_actions(path)) if path else 0
    ax.set_title(f"R2 最短路径规划\n"
                 f"(从位置2进入，抓取{max_get_R2}个R2-KFS，从10/12出口)\n"
                 f"白色线=移动  蓝色虚线=夹取(共{reach_count}次)",
                 fontsize=12, pad=20)
    ax.set_xticks([0, 1, 2])
    ax.set_xticklabels(['Left', 'Center', 'Right'], fontsize=11)
    ax.set_yticks([])
    ax.invert_yaxis()  # 让 row 0 在顶部
    ax.grid(True, linestyle=':', alpha=0.4)
    
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
        current_pos = move_positions[i]
        next_pos = move_positions[i + 1]
        
        current_height = pos_to_coord[current_pos][2]
        next_height = pos_to_coord[next_pos][2]
        
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
            
            # 如果目标位置有R2-KFS且未被夹取，则踩踏抓取
            if kfs.get(step) == 'R2' and step not in taken_set:
                taken_set.add(step)
                taken_count += 1
            
            current_pos = step
        
        elif isinstance(step, tuple) and len(step) == 3 and step[1] == 'reach':
            from_pos, _, to_pos = step
            # 检查夹取起始位置是否为当前位置
            if from_pos != current_pos:
                return False, (f"第{i}步夹取的起始位置{from_pos}"
                               f"与当前位置{current_pos}不符")
            
            can_reach, reason = can_reach_to(current_pos, to_pos, kfs,
                                             taken_count, taken_set)
            if not can_reach:
                return False, f"第{i}步夹取失败: {reason}"
            
            taken_set.add(to_pos)
            taken_count += 1
            # 夹取不改变当前位置
        
        else:
            return False, f"第{i}步格式无效: {step}"
    
    # 检查终点
    if current_pos not in EXIT_POSITIONS:
        return False, f"终点必须是位置{EXIT_POSITIONS}之一，当前在{current_pos}"
    
    # 检查抓取数量
    if taken_count != max_get_R2:
        return False, (f"必须抓取{max_get_R2}个R2-KFS，"
                       f"实际抓取了{taken_count}个")
    
    return True, "路径验证通过"

def format_path(path):
    """将路径格式化为可读字符串"""
    parts = []
    for step in path:
        if isinstance(step, int):
            parts.append(str(step))
        elif isinstance(step, tuple) and step[1] == 'reach':
            parts.append(f"[夹取{step[2]}]")
    return ' → '.join(parts)


def test():
    """连续测试10000次，统计路径规划结果"""
    print("=" * 60)
    print("R2 路径规划系统 - 批量测试（含夹取策略）")
    print("=" * 60)
    
    num_tests = 10000
    success_count = 0
    failure_count = 0
    
    # 统计抓取数量分布
    pickup_distribution = {}
    
    # 统计移动步数分布（不含夹取）
    path_lengths = []
    
    # 统计夹取次数分布
    reach_distribution = {}
    
    # 统计出口位置分布
    exit_distribution = {10: 0, 12: 0}
    
    # 统计上下台阶次数
    steps_up = []
    steps_down = []
    steps_total = []
    
    print(f"\n开始测试 {num_tests} 次...")
    print("-" * 60)
    
    for i in range(num_tests):
        kfs, r2_pos = generate_kfs()
        path = plan_path(kfs)
        
        if path:
            success_count += 1
            
            # 统计抓取的R2-KFS位置
            taken_positions = get_taken_positions(kfs, path)
            pickup_count = len(taken_positions)
            pickup_distribution[pickup_count] = pickup_distribution.get(pickup_count, 0) + 1
            
            # 统计移动步数（不含夹取）
            move_count = len(get_move_positions(path))
            path_lengths.append(move_count)
            
            # 统计夹取次数
            reach_count = len(get_reach_actions(path))
            reach_distribution[reach_count] = reach_distribution.get(reach_count, 0) + 1
            
            # 统计出口位置
            move_positions = get_move_positions(path)
            exit_pos = move_positions[-1]
            if exit_pos in exit_distribution:
                exit_distribution[exit_pos] += 1
            
            # 统计上下台阶次数
            steps = count_steps(path)
            steps_up.append(steps['up'])
            steps_down.append(steps['down'])
            steps_total.append(steps['total'])
        else:
            failure_count += 1
        
        if (i + 1) % 100 == 0:
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
        'pickup_distribution': pickup_distribution,
        'reach_distribution': reach_distribution,
        'avg_path_length': avg_path_length,
        'min_path_length': min_path_length,
        'max_path_length': max_path_length,
        'exit_distribution': exit_distribution,
        'avg_steps_up': avg_steps_up,
        'avg_steps_down': avg_steps_down,
        'avg_steps_total': avg_steps_total,
        'min_steps_total': min_steps_total,
        'max_steps_total': max_steps_total
    }

def run_single_visualization():
    """单次随机可视化：生成KFS放置，规划路径并显示可视化"""
    print("=" * 60)
    print("R2 路径规划系统 - 单次随机可视化（含夹取策略）")
    print("=" * 60)
    
    kfs, r2_pos = generate_kfs()
    
    print("\nKFS放置情况：")
    for pos, typ in sorted(kfs.items()):
        if typ is not None:
            print(f"  位置 {pos}: {typ}-KFS")
    
    path = plan_path(kfs)
    
    if path:
        move_positions = get_move_positions(path)
        reach_actions = get_reach_actions(path)
        taken_positions = get_taken_positions(kfs, path)
        
        print(f"\n规划路径: {format_path(path)}")
        print(f"移动步数: {len(move_positions)} 步")
        print(f"夹取次数: {len(reach_actions)} 次")
        print(f"抓取的R2-KFS位置: {sorted(taken_positions)}")
        print(f"抓取数量: {len(taken_positions)} 个")
        
        # 区分夹取和踩踏抓取
        reach_taken = {step[2] for step in reach_actions}
        step_taken = taken_positions - reach_taken
        if reach_taken:
            print(f"  其中夹取: {sorted(reach_taken)}")
        if step_taken:
            print(f"  其中踩踏: {sorted(step_taken)}")
        
        steps = count_steps(path)
        print(f"上台阶次数: {steps['up']} 次")
        print(f"下台阶次数: {steps['down']} 次")
        print(f"总台阶次数: {steps['total']} 次")
        
        is_valid, message = validate_path(kfs, path)
        print(f"路径验证: {message}")
        
        print("\n正在生成可视化图表...")
        visualize(kfs, path)
    else:
        print("\n未找到满足条件的路径")

# 运行示例
if __name__ == "__main__":
    mode = int(input("选择运行模式：1为批量测试，2为单次可视化"))
    
    if mode == 1:
        test()
    elif mode == 2:
        run_single_visualization()
    else:
        print("无效选择，默认批量测试")
        test()
