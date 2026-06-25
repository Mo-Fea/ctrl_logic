import os
import threading

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

try:
    import rclpy
    from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )
    from sensor_msgs.msg import Image
except ImportError:
    rclpy = None
    ExternalShutdownException = RuntimeError
    SingleThreadedExecutor = None
    Node = object
    DurabilityPolicy = None
    HistoryPolicy = None
    QoSProfile = None
    ReliabilityPolicy = None
    Image = None


DEFAULT_COLOR_WIDTH = 640
DEFAULT_COLOR_HEIGHT = 480
DEFAULT_COLOR_FPS = 30
DEFAULT_WARMUP_FRAMES = 15
DEFAULT_QR_LENGTH = 12
DEFAULT_QR_ALLOWED_CHARS = frozenset("0123")
IMAGE_SOURCE_D435I = 1
IMAGE_SOURCE_ODIN = 2
DEFAULT_IMAGE_SOURCE = IMAGE_SOURCE_D435I
DEFAULT_ODIN_IMAGE_TOPIC = "/odin1/image/undistorted"
DEFAULT_ODIN_FRAME_TIMEOUT_SEC = 1.0
DEFAULT_ODIN_INITIAL_FRAME_TIMEOUT_SEC = 5.0


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


class OdinImageSource:
    """订阅 Odin ROS2 图像话题并提供与 D435i 相同的逐帧读取接口。"""

    def __init__(
        self,
        topic=DEFAULT_ODIN_IMAGE_TOPIC,
        frame_timeout_sec=DEFAULT_ODIN_FRAME_TIMEOUT_SEC,
        initial_frame_timeout_sec=DEFAULT_ODIN_INITIAL_FRAME_TIMEOUT_SEC,
    ):
        if rclpy is None or Image is None:
            raise RuntimeError("未安装 ROS2 Python 或 sensor_msgs，无法订阅 Odin 图像")

        if not rclpy.ok():
            rclpy.init()

        self.topic = str(topic)
        self.frame_timeout_sec = float(frame_timeout_sec)
        self.initial_frame_timeout_sec = float(initial_frame_timeout_sec)
        self._condition = threading.Condition()
        self._latest_frame = None
        self._frame_sequence = 0
        self._last_read_sequence = 0
        self._last_error = None
        self._has_received_frame = False
        self._stop_event = threading.Event()

        node_name = f"odin_qr_image_source_{id(self):x}"
        self.node = Node(node_name)
        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.subscription = self.node.create_subscription(
            Image,
            self.topic,
            self._image_callback,
            image_qos,
        )
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.spin_thread = threading.Thread(
            target=self._spin,
            daemon=True,
            name=node_name,
        )
        self.spin_thread.start()
        print(f"[信息]: 已订阅 Odin 图像话题: {self.topic}")

    @staticmethod
    def _decode_image(msg):
        height = int(msg.height)
        width = int(msg.width)
        step = int(msg.step)
        encoding = str(msg.encoding).lower()
        if height <= 0 or width <= 0 or step <= 0:
            return None

        raw = np.frombuffer(msg.data, dtype=np.uint8)
        required_size = height * step
        if raw.size < required_size:
            raise ValueError(
                f"Odin 图像数据长度不足: actual={raw.size}, required={required_size}"
            )
        rows = raw[:required_size].reshape(height, step)

        if encoding in ("bgr8", "8uc3"):
            return rows[:, : width * 3].reshape(height, width, 3).copy()
        if encoding == "rgb8":
            rgb = rows[:, : width * 3].reshape(height, width, 3)
            return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        if encoding in ("bgra8", "8uc4"):
            bgra = rows[:, : width * 4].reshape(height, width, 4)
            return cv2.cvtColor(bgra, cv2.COLOR_BGRA2BGR)
        if encoding == "rgba8":
            rgba = rows[:, : width * 4].reshape(height, width, 4)
            return cv2.cvtColor(rgba, cv2.COLOR_RGBA2BGR)
        if encoding in ("mono8", "8uc1"):
            gray = rows[:, :width].reshape(height, width)
            return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        if encoding in ("mono16", "16uc1"):
            byte_order = ">" if bool(msg.is_bigendian) else "<"
            gray16 = np.frombuffer(
                bytes(rows[:, : width * 2].reshape(-1)),
                dtype=f"{byte_order}u2",
            ).reshape(height, width)
            gray8 = cv2.convertScaleAbs(gray16, alpha=255.0 / 65535.0)
            return cv2.cvtColor(gray8, cv2.COLOR_GRAY2BGR)
        raise ValueError(f"暂不支持 Odin 图像编码: {msg.encoding!r}")

    def _image_callback(self, msg):
        try:
            frame = self._decode_image(msg)
        except Exception as exc:
            with self._condition:
                self._last_error = exc
                self._condition.notify_all()
            return
        if frame is None:
            return
        with self._condition:
            self._latest_frame = frame
            self._frame_sequence += 1
            self._has_received_frame = True
            self._condition.notify_all()

    def _spin(self):
        try:
            while rclpy.ok() and not self._stop_event.is_set():
                try:
                    self.executor.spin_once(timeout_sec=0.1)
                except ExternalShutdownException:
                    break
        finally:
            try:
                self.executor.remove_node(self.node)
            except Exception:
                pass

    def get_color_frame(self):
        with self._condition:
            timeout_sec = (
                self.frame_timeout_sec
                if self._has_received_frame
                else self.initial_frame_timeout_sec
            )
            received = self._condition.wait_for(
                lambda: (
                    self._frame_sequence > self._last_read_sequence
                    or self._last_error is not None
                    or self._stop_event.is_set()
                ),
                timeout=timeout_sec,
            )
            if self._stop_event.is_set():
                return None
            if not received:
                if not self._has_received_frame:
                    raise RuntimeError(
                        f"等待 Odin 图像首帧超时（{timeout_sec:.1f}s）：{self.topic}"
                    )
                return None
            if self._last_error is not None:
                error = self._last_error
                self._last_error = None
                raise RuntimeError(f"读取 Odin 图像失败: {error}") from error
            self._last_read_sequence = self._frame_sequence
            return self._latest_frame.copy()

    def stop(self):
        self._stop_event.set()
        with self._condition:
            self._condition.notify_all()
        if self.spin_thread.is_alive():
            self.spin_thread.join(timeout=1.0)
        try:
            self.executor.shutdown()
        except Exception:
            pass
        try:
            self.node.destroy_node()
        except Exception:
            pass


def open_odin_image(
    topic=DEFAULT_ODIN_IMAGE_TOPIC,
    frame_timeout_sec=DEFAULT_ODIN_FRAME_TIMEOUT_SEC,
    initial_frame_timeout_sec=DEFAULT_ODIN_INITIAL_FRAME_TIMEOUT_SEC,
):
    return OdinImageSource(
        topic=topic,
        frame_timeout_sec=frame_timeout_sec,
        initial_frame_timeout_sec=initial_frame_timeout_sec,
    )


def open_image_source(image_source=DEFAULT_IMAGE_SOURCE, **kwargs):
    """
    打开二维码图像来源。

    image_source=1: D435i 彩色流（默认）。
    image_source=2: 订阅 /odin1/image/undistorted。
    """
    image_source = int(image_source)
    if image_source == IMAGE_SOURCE_D435I:
        return open_d435i(**kwargs)
    if image_source == IMAGE_SOURCE_ODIN:
        return open_odin_image(**kwargs)
    raise ValueError(f"image_source 必须是 1(D435i) 或 2(Odin)，当前为 {image_source}")


def get_color_frame(image_source):
    """从 D435i 或 Odin 图像源读取一帧 BGR 彩色图。"""
    source_reader = getattr(image_source, "get_color_frame", None)
    if source_reader is not None:
        return source_reader()

    frames = image_source.wait_for_frames()
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
