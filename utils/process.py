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
DEFAULT_WARMUP_TIMEOUT_MS = 5000
DEFAULT_WARMUP_RETRY_COUNT = 3
DEFAULT_REALSENSE_STREAM_CANDIDATES = (
    (640, 480, 30),
    (848, 480, 30),
    (1280, 720, 30),
)
DEFAULT_QR_LENGTH = 12
DEFAULT_QR_ALLOWED_CHARS = frozenset("0123")
IMAGE_SOURCE_D435I = 1
IMAGE_SOURCE_ODIN = 2
DEFAULT_IMAGE_SOURCE = IMAGE_SOURCE_D435I
DEFAULT_ODIN_IMAGE_TOPIC = "/odin1/image/undistorted"
DEFAULT_ODIN_FRAME_TIMEOUT_SEC = 1.0
DEFAULT_ODIN_INITIAL_FRAME_TIMEOUT_SEC = 5.0


def _safe_stop_pipeline(pipeline):
    if pipeline is None:
        return
    try:
        pipeline.stop()
    except Exception:
        pass


def _get_realsense_device_list():
    if rs is None:
        return []
    devices = []
    try:
        context = rs.context()
        for device in context.query_devices():
            device_info = {}
            for info_key, info_name in (
                (rs.camera_info.name, "name"),
                (rs.camera_info.serial_number, "serial"),
                (rs.camera_info.firmware_version, "firmware"),
                (rs.camera_info.usb_type_descriptor, "usb"),
            ):
                try:
                    device_info[info_name] = device.get_info(info_key)
                except Exception:
                    device_info[info_name] = "unknown"
            devices.append(device_info)
    except Exception as exc:
        devices.append({"error": repr(exc)})
    return devices


def _format_realsense_devices(devices):
    if not devices:
        return "未发现 RealSense 设备"
    lines = []
    for index, device in enumerate(devices, start=1):
        if "error" in device:
            lines.append(f"{index}. 查询设备失败: {device['error']}")
            continue
        lines.append(
            f"{index}. {device.get('name', 'unknown')} "
            f"S/N={device.get('serial', 'unknown')} "
            f"FW={device.get('firmware', 'unknown')} "
            f"USB={device.get('usb', 'unknown')}"
        )
    return "\n".join(lines)


def _make_realsense_stream_candidates(width, height, fps, enable_fallback=True):
    candidates = [(int(width), int(height), int(fps))]
    if enable_fallback:
        for candidate in DEFAULT_REALSENSE_STREAM_CANDIDATES:
            if candidate not in candidates:
                candidates.append(candidate)
    return candidates


def _warmup_realsense_pipeline(
    pipeline,
    warmup_frames=DEFAULT_WARMUP_FRAMES,
    timeout_ms=DEFAULT_WARMUP_TIMEOUT_MS,
    retry_count=DEFAULT_WARMUP_RETRY_COUNT,
):
    warmup_frames = max(0, int(warmup_frames))
    timeout_ms = int(timeout_ms)
    retry_count = max(0, int(retry_count))

    for frame_index in range(warmup_frames):
        last_error = None
        for attempt_index in range(retry_count + 1):
            try:
                frames = pipeline.wait_for_frames(timeout_ms)
                color_frame = frames.get_color_frame()
                if color_frame:
                    break
                last_error = RuntimeError("frameset 中没有 color frame")
            except RuntimeError as exc:
                last_error = exc
            if attempt_index < retry_count:
                print(
                    "[警告]: RealSense 预热取帧失败，"
                    f"frame={frame_index + 1}/{warmup_frames}, "
                    f"retry={attempt_index + 1}/{retry_count}, "
                    f"error={last_error!r}"
                )
        else:
            raise RuntimeError(
                "RealSense 彩色流预热失败："
                f"frame={frame_index + 1}/{warmup_frames}, "
                f"timeout_ms={timeout_ms}, "
                f"retry_count={retry_count}, "
                f"last_error={last_error!r}"
            )


def open_d435i(
    width=DEFAULT_COLOR_WIDTH,
    height=DEFAULT_COLOR_HEIGHT,
    fps=DEFAULT_COLOR_FPS,
    warmup_frames=DEFAULT_WARMUP_FRAMES,
    warmup_timeout_ms=DEFAULT_WARMUP_TIMEOUT_MS,
    warmup_retry_count=DEFAULT_WARMUP_RETRY_COUNT,
    enable_fallback=True,
):
    """
    打开 RealSense D435i/D455 彩色流并预热若干帧。

    返回 pyrealsense2 pipeline。调用方负责在结束时执行 pipeline.stop()。
    """
    if rs is None:
        raise RuntimeError("未安装 pyrealsense2，无法打开 RealSense 彩色流")

    devices = _get_realsense_device_list()
    print("[信息]: RealSense 设备列表:")
    print(_format_realsense_devices(devices))
    if not devices:
        raise RuntimeError(
            "未发现 RealSense 设备。请检查 USB 连接、供电、权限和设备占用。"
        )

    candidates = _make_realsense_stream_candidates(
        width=width,
        height=height,
        fps=fps,
        enable_fallback=enable_fallback,
    )
    errors = []

    for candidate_width, candidate_height, candidate_fps in candidates:
        pipeline = None
        try:
            print(
                "[信息]: 尝试打开 RealSense 彩色流: "
                f"{candidate_width}x{candidate_height}@{candidate_fps}"
            )
            pipeline = rs.pipeline()
            config = rs.config()
            config.enable_stream(
                rs.stream.color,
                int(candidate_width),
                int(candidate_height),
                rs.format.bgr8,
                int(candidate_fps),
            )
            profile = pipeline.start(config)
            device = profile.get_device()
            device_name = device.get_info(rs.camera_info.name)
            serial = device.get_info(rs.camera_info.serial_number)
            print(
                "[信息]: 已连接 RealSense 设备: "
                f"{device_name} (S/N: {serial}), "
                f"stream={candidate_width}x{candidate_height}@{candidate_fps}"
            )

            _warmup_realsense_pipeline(
                pipeline=pipeline,
                warmup_frames=warmup_frames,
                timeout_ms=warmup_timeout_ms,
                retry_count=warmup_retry_count,
            )
            print("[信息]: RealSense 彩色流预热完成")
            return pipeline
        except Exception as exc:
            errors.append(
                f"{candidate_width}x{candidate_height}@{candidate_fps}: {exc!r}"
            )
            print(
                "[警告]: RealSense 彩色流打开或预热失败: "
                f"{candidate_width}x{candidate_height}@{candidate_fps}, "
                f"error={exc!r}"
            )
            _safe_stop_pipeline(pipeline)

    raise RuntimeError(
        "无法打开可用的 RealSense 彩色流。"
        "请确认没有 realsense-viewer/其他脚本/ROS realsense 节点占用相机，"
        "SSH 用户有 /dev/video* 权限，并检查 USB3 线缆和端口。\n"
        "设备列表:\n"
        f"{_format_realsense_devices(devices)}\n"
        "尝试结果:\n"
        + "\n".join(errors)
    )


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

    image_source=1: RealSense D435i/D455 彩色流（默认）。
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
