try:
    from . import challenge_lib
except ImportError:
    import challenge_lib


def main():
    print("====================================")
    print("机器人R2路径规划系统已就绪（challenge/race）")
    print("请将二维码对准摄像头，连续5帧稳定识别后自动规划；按 'q' 退出")
    print("====================================")

    scanner = challenge_lib.start_background_qr_scanner(
        stable_frame_count=5,
        show_window=True,
        stop_after_success=True,
    )
    scanner.join()

    if scanner.last_error is not None and scanner.result_queue.empty():
        print(f"[错误]: {scanner.last_error}")
        return

    if scanner.result_queue.empty():
        print("[警告]: 未获取到有效二维码规划结果")
        return

    result = scanner.result_queue.get()
    print(f"\n[检测到稳定二维码]: {result.qr_data}")
    print("[成功]: 路径规划已完成，正在生成可视化图表...")
    print(f"[规划路径]: {challenge_lib.format_path(result.path)}")
    challenge_lib.print_action_matrix(result.action_matrix)
    challenge_lib.visualize(result.kfs, result.path)


if __name__ == "__main__":
    main()
