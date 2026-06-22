#!/usr/bin/env python3

from lib2 import module, tools


SAMPLE_ACTION_MATRIX = [
    [-1,2,1,1,0],
    [2,5,1,0,1],
    [2,5,1,1,0],
    [5,8,1,0,1],
    [5,8,1,1,0],
    [8,11,1,1,0],
    [11,10,2,1,0]
]


MOVE_DIR_TEXT = {
    0: "原地",
    1: "前方/x+",
    2: "+90度/左/y+",
    3: "-90度/右/y-",
    4: "后方/x-",
}


HEIGHT_RELATION_TEXT = {
    0: "无相邻高低关系",
    1: "目标方向台阶更高 -> 上楼/高位KFS",
    2: "目标方向台阶更低 -> 下楼/低位KFS",
}


def _print_step(step_no, text):
    print(f"    {step_no}. {text}")


def print_kfs_fetch_plan(
    from_pos,
    to_pos,
    move_dir,
    height_relation,
    final_direction,
    from_x,
    from_y,
    next_from_pose=0,
    next_to_pose=0,
    next_height_action=0,
):
    pose_id = module.kfs_pose_id_from_height_relation(height_relation)
    pose_name = "高位抓取姿态" if pose_id == 1 else "低位抓取姿态"
    move_yaw = tools.direction_int_to_yaw_deg(move_dir)

    print("  branch=directional/kfs_fetch")
    print("  execute=module.fetch_and_store_kfs(...)")
    _print_step(
        1,
        "读取高低关系: "
        f"get_stair_height_relation(stair_id={from_pos}, direction={move_dir}) -> {height_relation}",
    )
    _print_step(
        2,
        "吸取前微调: "
        f"adjust_position(move_type=1, stair_id={from_pos}, direction={move_dir}, "
        f"height_relation={height_relation})",
    )
    if height_relation == 1:
        print("       高位微调: 旋转到吸取方向 -> ch2=200 前进2s")
    elif height_relation == 2:
        print("       低位微调: 按台阶坐标和 adjust_distance 计算目标点后 move_to_des")
    _print_step(
        3,
        f"机械臂到抓取姿态: kfs.kfs_grab_pose(pose_id={pose_id}, {pose_name})",
    )
    _print_step(
        4,
        "吸盘吸取: set_kfs_suction(suction_on=True)，通道 ch5=2, ch4:1->3",
    )
    _print_step(
        5,
        "保持吸取: wait_with_kfs_suction(duration_sec=1.0)，持续 ch4=3",
    )
    _print_step(
        6,
        "异步后续机械臂: pose_id=3 -> 旋转双头吸盘 -> pose_id=0 -> 复位KFS通道",
    )
    if next_height_action == 1:
        next_direction = tools.stair_id_to_direction(
            next_from_pose,
            next_to_pose,
            exit_on_error=False,
        )
        if next_direction == 0:
            print("       error=下一行 height_action=1，但 next_from_pose 与 next_to_pose 不相邻")
            return
        next_height_relation = module.get_stair_height_relation(next_from_pose, next_direction)
        print(
            "       next="
            f"from={next_from_pose}, to={next_to_pose}, "
            f"height_action={next_height_action}, "
            f"direction={next_direction}, height_relation={next_height_relation}"
        )
        if next_height_relation == 1 and to_pos == next_to_pose:
            _print_step(
                7,
                "下一行是上楼且当前 to_pos 与下一行 to_pos 相同: 不回当前台阶中心，直接返回",
            )
            return

    _print_step(
        7,
        "正向回当前台阶中心: "
        f"move_to_des(x={from_x:.3f}, y={from_y:.3f}, target_deg={move_yaw:.2f})",
    )


def print_climb_plan(move_dir, final_direction, to_x, to_y):
    task_yaw = tools.direction_int_to_yaw_deg(move_dir)
    final_yaw = tools.direction_int_to_yaw_deg(final_direction)

    print("  branch=directional/stair_transition")
    print("  execute=module.climb(...)")
    _print_step(
        1,
        f"方向转换: direction1={move_dir}->{task_yaw:.2f}deg, "
        f"direction2={final_direction}->{final_yaw:.2f}deg",
    )
    _print_step(
        2,
        "上楼前准备: "
        f"adjust_position(move_type=1, direction={move_dir}, stair_id=-1, height_relation=1)",
    )
    print("       高位微调: 旋转到 direction1 -> ch2=200 前进2s")
    _print_step(
        3,
        "触发半自动上楼: move.climb()，通道 ch5=1, ch7:1->3->1",
    )
    _print_step(
        4,
        "等待上楼完成: 水平位移超过 min_distance 且 current_z - 0.15 > start_z",
    )
    _print_step(
        5,
        f"移动到目标台阶中心: move_to_des(x={to_x:.3f}, y={to_y:.3f}, target_deg={final_yaw:.2f})",
    )


def print_descend_plan(move_dir, final_direction, from_x, from_y, to_x, to_y):
    task_yaw = tools.direction_int_to_yaw_deg(move_dir)
    final_yaw = tools.direction_int_to_yaw_deg(final_direction)
    opposite_direction = {1: 4, 2: 3, 3: 2, 4: 1}[move_dir]
    align_yaw = tools.direction_int_to_yaw_deg(opposite_direction)

    print("  branch=directional/stair_transition")
    print("  execute=module.descend(...)")
    _print_step(
        1,
        f"方向转换: direction1={move_dir}->{task_yaw:.2f}deg, "
        f"direction2={final_direction}->{final_yaw:.2f}deg",
    )
    _print_step(
        2,
        f"下楼前反向对正: opposite_direction={opposite_direction}, "
        f"rotate_to_target_yaw_segmented(target_yaw_deg={align_yaw:.2f})",
    )
    _print_step(
        3,
        "触发半自动下楼: move.descend()，通道 ch5=1, ch6:1->3->1",
    )
    _print_step(
        4,
        "等待下楼完成: start_z - 0.15 > current_z",
    )
    _print_step(
        5,
        "倒退到目标台阶中心: "
        f"move_backward_to_des(from=({from_x:.3f}, {from_y:.3f}), "
        f"x={to_x:.3f}, y={to_y:.3f}, target_deg={final_yaw:.2f})",
    )


def _row_to_ints(action_row):
    row_values = list(action_row)
    if len(row_values) != module.ACTION_MATRIX_ROW_SIZE:
        raise ValueError(
            f"action_row size={len(row_values)}, "
            f"必须等于 {module.ACTION_MATRIX_ROW_SIZE}: {action_row}"
        )
    return [int(value) for value in row_values]


def explain_action_row(
    action_row,
    final_direction=1,
    row_index=None,
    next_from_pose=0,
    next_to_pose=0,
    next_height_action=0,
):
    from_pos, to_pos, move_dir, height_action, grab_action = _row_to_ints(action_row)
    final_direction = int(final_direction)
    next_from_pose = int(next_from_pose)
    next_to_pose = int(next_to_pose)
    next_height_action = int(next_height_action)
    if final_direction not in (1, 2, 3, 4):
        raise ValueError(f"final_direction 必须是 1/2/3/4, got {final_direction}")
    if move_dir not in (0, 1, 2, 3, 4):
        raise ValueError(f"move_dir 必须是 0/1/2/3/4, got {move_dir}")
    if next_height_action not in (0, 1):
        raise ValueError(f"next_height_action 必须是 0/1, got {next_height_action}")

    inferred_direction = tools.stair_id_to_direction(
        from_pos,
        to_pos,
        exit_on_error=False,
    )
    if from_pos != to_pos and inferred_direction == 0:
        raise ValueError(f"{from_pos} 与 {to_pos} 不相邻")
    if move_dir != 0 and inferred_direction != 0 and move_dir != inferred_direction:
        raise ValueError(
            f"move_dir={move_dir} 与坐标推导方向 inferred_direction={inferred_direction} 不一致"
        )

    height_relation = (
        0
        if inferred_direction == 0
        else module.get_stair_height_relation(from_pos, inferred_direction)
    )

    prefix = f"row {row_index}: " if row_index is not None else ""
    print("=" * 72)
    print(f"{prefix}action_row={[from_pos, to_pos, move_dir, height_action, grab_action]}")
    print(f"  from_pos={from_pos}, to_pos={to_pos}")
    print(f"  move_dir={move_dir} ({MOVE_DIR_TEXT[move_dir]})")
    print(f"  inferred_direction={inferred_direction} ({MOVE_DIR_TEXT.get(inferred_direction, '无')})")
    print(f"  height_action={height_action}, grab_action={grab_action}")
    print(f"  height_relation={height_relation} ({HEIGHT_RELATION_TEXT.get(height_relation, '未知')})")
    print(
        "  final_direction="
        f"{final_direction} ({MOVE_DIR_TEXT[final_direction]}), "
        f"final_yaw={tools.direction_int_to_yaw_deg(final_direction):.2f}deg"
    )

    if move_dir != 0:
        from_x, from_y = module.get_stair_xy(from_pos)
        to_x, to_y = module.get_stair_xy(to_pos)
        print(f"  from_xy=({from_x:.3f}, {from_y:.3f})")
        print(f"  to_xy=({to_x:.3f}, {to_y:.3f})")

        if grab_action == 1:
            print_kfs_fetch_plan(
                from_pos=from_pos,
                to_pos=to_pos,
                move_dir=move_dir,
                height_relation=height_relation,
                final_direction=final_direction,
                from_x=from_x,
                from_y=from_y,
                next_from_pose=next_from_pose,
                next_to_pose=next_to_pose,
                next_height_action=next_height_action,
            )
            return

        if height_action != 0:
            if height_relation == 1:
                print_climb_plan(
                    move_dir=move_dir,
                    final_direction=final_direction,
                    to_x=to_x,
                    to_y=to_y,
                )
            elif height_relation == 2:
                print_descend_plan(
                    move_dir=move_dir,
                    final_direction=final_direction,
                    from_x=from_x,
                    from_y=from_y,
                    to_x=to_x,
                    to_y=to_y,
                )
            else:
                print("  error=height_action 非0，但 height_relation 不是 1/2")
            return

        print("  error=当前地图不应出现等高普通移动")
        return

    print("  branch=stationary")
    if grab_action == 1:
        print("  action=原地抓取分支当前执行层尚未接入")
    else:
        print("  action=原地占位分支当前执行层尚未接入")


def explain_action_matrix(action_matrix, final_direction=1):
    action_rows = list(action_matrix)
    row_count = len(action_rows)
    for index, action_row in enumerate(action_rows):
        row_kwargs = {}
        if index + 1 < row_count:
            next_from_pose, next_to_pose, _, next_height_action, _ = _row_to_ints(
                action_rows[index + 1]
            )
            row_kwargs = {
                "next_from_pose": next_from_pose,
                "next_to_pose": next_to_pose,
                "next_height_action": next_height_action,
            }
        explain_action_row(
            action_row,
            final_direction=final_direction,
            row_index=index,
            **row_kwargs,
        )


def main():
    print("Action matrix explanation test")
    explain_action_matrix(SAMPLE_ACTION_MATRIX, final_direction=1)


if __name__ == "__main__":
    main()
