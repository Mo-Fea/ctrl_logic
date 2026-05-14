import os

# OpenCV 的 Qt 后端有时会优先找 wayland 插件；在部分环境里强制走 xcb 更稳。
os.environ.setdefault("QT_QPA_PLATFORM", "xcb")
os.environ.setdefault("QT_QPA_FONTDIR", "/usr/share/fonts/truetype/dejavu")
os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")

import cv2
import numpy as np
import race

try:
    from .process import (
        create_qr_detector,
        detect_qr_data,
        get_color_frame,
        is_valid_qr_payload,
        open_d435i,
    )
except ImportError:
    from process import (
        create_qr_detector,
        detect_qr_data,
        get_color_frame,
        is_valid_qr_payload,
        open_d435i,
    )


# 机器人动作矩阵列定义（n * 5，每一行表示一个顺序动作）
ACTION_MATRIX_COLUMNS = [
    "from_pos",       # 动作起点位置编号
    "to_pos",         # 动作目标位置编号；原地动作时与 from_pos 相同
    "move_dir",       # 0=原地/无移动, 1=前方, 2=+90度/左, 3=-90度/右, 4=后方
    "height_action",  # 0=不用上下楼梯, 1=需要上下楼梯
    "grab_action",    # 0=不抓取, 1=抓取
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


def qr_to_kfs(qr_string):
    """
    根据规则将12位字符串转换为 kfs 字典
    规则：1=R1, 2=R2, 3=low, 其他=None
    """
    mapping = {'1': 'R1', '2': 'R2', '3': 'low', '0': None}
    kfs = {}

    # 确保只处理前12位，防止异常数据
    for i in range(12):
        pos = i + 1
        # 获取对应字符，如果字符串长度不足则默认为'0'
        char = qr_string[i] if i < len(qr_string) else '0'
        # 映射到字典
        kfs[pos] = mapping.get(char, None)
    return kfs


def _direction_code(from_pos, to_pos):
    """根据 race.py 的 lib2 坐标语义，计算网格移动方向编码。"""
    if from_pos == to_pos:
        return 0

    from_x, from_y, _ = race.pos_to_coord[from_pos]
    to_x, to_y, _ = race.pos_to_coord[to_pos]
    delta = (to_x - from_x, to_y - from_y)
    if delta not in MOVE_DIR_CODES:
        raise ValueError(f"位置 {from_pos} 到 {to_pos} 不是相邻格，无法生成动作矩阵")
    return MOVE_DIR_CODES[delta]


def _height_action(from_pos, to_pos):
    """返回台阶动作：1=需要上下楼梯，0=不移动/等高。"""
    if from_pos == to_pos:
        return 0

    from_height = race.pos_to_coord[from_pos][2]
    to_height = race.pos_to_coord[to_pos][2]
    diff = to_height - from_height
    if diff != 0:
        return 1
    return 0


def path_to_action_matrix(kfs, path):
    """
    将 race.py 中 plan_path() 的路径转换成 n*5 机器人动作矩阵。

    race.py 的路径输入/输出约定：
    - 整数：R2实际移动到的位置
    - (from_pos, 'reach', to_pos)：R2在from_pos旁夹to_pos的R2-KFS，不改变当前位置

    矩阵列含义见 ACTION_MATRIX_COLUMNS，每一行表示一个顺序动作。
    如果下一格有R2-KFS且机器人也要走到那格，会拆成两行动作：
    1) 当前格先夹取目标格R2
    2) R2清空后再移动到目标格
    """
    if not path:
        return np.zeros((0, 5), dtype=int)

    is_valid, message = race.validate_path(kfs, path)
    if not is_valid:
        raise ValueError(f"无法生成动作矩阵：{message}")

    columns = []
    current_pos = path[0]
    taken_set = set()

    # 入口位置如果本身就是R2-KFS，race.py会视为自动抓取。
    if kfs.get(current_pos) == "R2":
        columns.append([current_pos, current_pos, 0, 0, 1])
        taken_set.add(current_pos)

    for step in path[1:]:
        if isinstance(step, int):
            if kfs.get(step) == "R2" and step not in taken_set:
                # 不能先踩上有R2-KFS的格子；必须先在当前相邻格夹走，再移动过去。
                columns.append([
                    current_pos,
                    step,
                    _direction_code(current_pos, step),
                    0,
                    1,
                ])
                taken_set.add(step)

            grab_action = 1 if kfs.get(step) == "R1" else 0

            columns.append([
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

            columns.append([
                from_pos,
                to_pos,
                _direction_code(from_pos, to_pos),
                0,
                1,
            ])
            taken_set.add(to_pos)

        else:
            raise ValueError(f"未知路径步骤：{step}")

    if not columns:
        return np.zeros((0, 5), dtype=int)
    return np.array(columns, dtype=int)


def print_action_matrix(action_matrix):
    """按 race.py 的路径结果打印 n*5 动作矩阵和简要说明。"""
    print("\n[机器人动作矩阵 n*5]")
    print(f"列定义: {ACTION_MATRIX_COLUMNS}")
    print(action_matrix)

    print("\n[动作序列说明]")
    for index, row in enumerate(action_matrix, start=1):
        from_pos, to_pos, move_dir, height_action, grab_action = row.tolist()
        if height_action == 1:
            stair_text = "需要上下楼梯"
        else:
            stair_text = "不用上下楼梯"

        print(
            f"{index:02d}. {from_pos}->{to_pos} | "
            f"方向={MOVE_DIR_NAMES.get(move_dir, move_dir)} | "
            f"{stair_text} | "
            f"{GRAB_ACTION_NAMES.get(grab_action, grab_action)}"
        )


def main():
    # 初始化 D435i
    try:
        pipeline = open_d435i()
    except Exception as exc:
        print(f"[错误]: 无法打开 D435i 彩色流: {exc}")
        print("请确认相机已连接，且没有被其他 RealSense 程序占用。")
        return

    detector = create_qr_detector()
    last_processed_data = None

    print("====================================")
    print("机器人R2路径规划系统已就绪（race.py）")
    print("请将二维码对准摄像头，按 'q' 退出")
    print("====================================")

    while True:
        frame = get_color_frame(pipeline)
        if frame is None:
            print("无法获取 D435i 彩色画面")
            break

        # 尝试识别二维码
        data = detect_qr_data(frame, detector=detector)

        # 仅在获取到 12 位有效数据时进行规划
        if is_valid_qr_payload(data):
            if data == last_processed_data:
                cv2.imshow("R2 QR Scanner", frame)
                if cv2.waitKey(1) == ord('q'):
                    break
                continue

            last_processed_data = data
            print(f"\n[检测到二维码]: {data}")

            # 1. 解析数据
            kfs = qr_to_kfs(data)

            # 2. 执行路径规划
            path = race.plan_path(kfs)

            if path:
                print("[成功]: 路径规划已完成，正在生成可视化图表...")
                print(f"[规划路径]: {race.format_path(path)}")
                action_matrix = path_to_action_matrix(kfs, path)
                print_action_matrix(action_matrix)
                race.visualize(kfs, path)
            else:
                print("[警告]: 当前二维码布局下未找到可行路径")

        # 显示实时画面
        cv2.imshow("R2 QR Scanner", frame)

        # 按 'q' 键退出
        if cv2.waitKey(1) == ord('q'):
            break

    pipeline.stop()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
