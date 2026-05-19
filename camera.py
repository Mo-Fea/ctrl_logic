#!/usr/bin/env python3

import queue
import threading
import time

from utils import challenge_lib
import read_matrix


def main():
    """
    QR 后台识别测试。

    流程：
    1. 启动 challenge_lib 后台识别线程。
    2. 主线程等待 1s，模拟先执行其他任务。
    3. 主线程进入 with scanner_lock；如果识别线程还在运行，会阻塞到其释放锁。
    4. 从 queue 中读取动作矩阵并打印。
    5. 按 read_matrix.py 的逻辑解释动作矩阵。
    """
    action_matrix_queue = queue.Queue()
    scanner_lock = threading.Lock()

    scanner = challenge_lib.start_background_qr_scanner(
        result_queue=action_matrix_queue,
        stable_frame_count=5,
        show_window=True,
        stop_after_success=True,
        put_action_matrix_only=True,
        running_lock=scanner_lock,
    )

    time.sleep(1.0)

    print("主线程等待扫码线程释放锁...")
    with scanner_lock:
        print("主线程已拿到锁，扫码线程已结束或已释放锁。")

        if action_matrix_queue.empty():
            if scanner.last_error is not None:
                print(f"扫码线程错误: {scanner.last_error}")
            else:
                print("动作矩阵 queue 为空，未获取到稳定二维码规划结果。")
            return

        action_matrix = action_matrix_queue.get()
        print("\n[queue 动作矩阵]")
        print(action_matrix)

        print("\n[read_matrix 解释]")
        read_matrix.explain_action_matrix(action_matrix, final_direction=1)

    scanner.join(timeout=1.0)


if __name__ == "__main__":
    main()
