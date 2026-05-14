import os

# OpenCV 的 Qt 后端有时会优先找 wayland 插件；在部分环境里强制走 xcb 更稳。
os.environ.setdefault("QT_QPA_PLATFORM", "xcb")
os.environ.setdefault("QT_QPA_FONTDIR", "/usr/share/fonts/truetype/dejavu")
os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")

import cv2
import numpy as np

try:
    import pyrealsense2 as rs
except ImportError:
    rs = None


DEFAULT_COLOR_WIDTH = 640
DEFAULT_COLOR_HEIGHT = 480
DEFAULT_COLOR_FPS = 30
DEFAULT_WARMUP_FRAMES = 15
DEFAULT_QR_LENGTH = 12
DEFAULT_QR_ALLOWED_CHARS = frozenset("0123")


def open_d435i(
    width=DEFAULT_COLOR_WIDTH,
    height=DEFAULT_COLOR_HEIGHT,
    fps=DEFAULT_COLOR_FPS,
    warmup_frames=DEFAULT_WARMUP_FRAMES,
):
    """
    打开 D435i 彩色流并预热若干帧。

    返回 pyrealsense2 pipeline。调用方负责在结束时执行 pipeline.stop()。
    """
    if rs is None:
        raise RuntimeError("未安装 pyrealsense2，无法打开 D435i 彩色流")

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, int(width), int(height), rs.format.bgr8, int(fps))
    profile = pipeline.start(config)

    device = profile.get_device()
    device_name = device.get_info(rs.camera_info.name)
    serial = device.get_info(rs.camera_info.serial_number)
    print(f"[信息]: 已连接 RealSense 设备: {device_name} (S/N: {serial})")

    for _ in range(int(warmup_frames)):
        pipeline.wait_for_frames()

    return pipeline


def get_color_frame(pipeline):
    """从 D435i pipeline 读取一帧 BGR 彩色图。"""
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    if not color_frame:
        return None
    return np.asanyarray(color_frame.get_data())


def create_qr_detector():
    """创建 OpenCV 二维码检测器。"""
    return cv2.QRCodeDetector()


def detect_qr_data(frame, detector=None):
    """从单帧图像中读取二维码字符串；未识别到时返回空字符串。"""
    if detector is None:
        detector = create_qr_detector()
    data, _, _ = detector.detectAndDecode(frame)
    return "" if data is None else str(data).strip()


def is_valid_qr_payload(
    data,
    length=DEFAULT_QR_LENGTH,
    allowed_chars=DEFAULT_QR_ALLOWED_CHARS,
):
    """校验二维码内容是否为指定长度且只包含允许字符。"""
    data = "" if data is None else str(data).strip()
    if len(data) != int(length):
        return False
    return all(char in allowed_chars for char in data)
