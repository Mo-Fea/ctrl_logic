import socket
import time
import threading
import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from std_msgs.msg import Bool
import math
import struct

TCP_IP = "192.168.1.199"
TCP_PORT = 5000
CONNECT_RETRY_INTERVAL = 0.5


SOF1 = 0xA5
SOF2 = 0x5A
TYPE = 0x01
CHANNEL_COUNT = 10

# 新payload长度:
# seq(2) + ch[10]*2(20) + yaw_i16(2) + des_yaw_i16(2) = 26 bytes = 0x1A
LEN = 0x1A



#-----------------------------------------------------------------------------------------------------
#下位机连接方法
def connect(tcp_ip=TCP_IP, tcp_port=TCP_PORT, retry_interval=CONNECT_RETRY_INTERVAL):
    """
    连接下位机（失败自动重试）
    返回: 已连接的 socket
    """
    while True:
        sock = None
        try:
            print(f"Connecting to {tcp_ip}:{tcp_port} ...")
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            sock.settimeout(2.0)
            sock.connect((tcp_ip, tcp_port))
            sock.settimeout(None)
            print("✅ Connected to chassis controller.\n")
            return sock
        except Exception as e:
            print(f"❌ Connect failed: {e}; retry in {retry_interval:.1f}s")
            if sock is not None:
                try:
                    sock.close()
                except Exception:
                    pass
            time.sleep(retry_interval)

#-----------------------------------------------------------------------------------------------------
#重定位接收方法
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class _RelocalizationNode(Node):
    def __init__(self, topic='/odin1/flag1'):
        super().__init__('relocalization_conformation_node')
        self._flag = False
        self._lock = threading.Lock()

        self._sub = self.create_subscription(
            Bool, topic, self._callback, 10
        )

    def _callback(self, msg: Bool):
        with self._lock:
            prev = self._flag
            self._flag = bool(msg.data)

        if (not prev) and self._flag:
            self.get_logger().info('🎯 Relocalization confirmed (/odin1/flag1=True)')

    def get_flag(self) -> bool:
        with self._lock:
            return self._flag


def _spin_relocalization(node: _RelocalizationNode, stop_event: threading.Event):
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        while rclpy.ok() and (not stop_event.is_set()):
            try:
                executor.spin_once(timeout_sec=0.1)
            except ExternalShutdownException:
                break
    finally:
        executor.remove_node(node)
        executor.shutdown()


def relocalization_conformation(topic='/odin1/flag1'):
    """
    启动重定位确认监听（非阻塞）
    返回:
      get_flag: 可调用函数，返回当前flag(bool)
      node: ROS节点对象
      spin_thread: 后台spin线程
      stop_event: 停止事件（供销毁函数使用）
    """
    if not rclpy.ok():
        rclpy.init()

    node = _RelocalizationNode(topic=topic)
    stop_event = threading.Event()
    spin_thread = threading.Thread(
        target=_spin_relocalization,
        args=(node, stop_event),
        daemon=True
    )
    spin_thread.start()

    return node.get_flag, node, spin_thread, stop_event

#-----------------------------------------------------------------------------------------------------
#线程销毁方法
def destroy_ros2_thread(node, spin_thread, stop_event, shutdown_rclpy=False):
    """
    停止并销毁重定位监听线程/节点
    """
    # 1) 通知线程退出
    stop_event.set()

    # 2) 等线程收尾
    if spin_thread is not None and spin_thread.is_alive():
        spin_thread.join(timeout=1.0)

    # 3) 销毁节点
    if node is not None:
        try:
            node.destroy_node()
        except Exception:
            pass

    # 4) 是否关闭rclpy（仅当你确认没有别的ROS节点在用时再True）
    if shutdown_rclpy and rclpy.ok():
        try:
            rclpy.shutdown()
        except Exception:
            pass
#-----------------------------------------------------------------------------------------------------
#状态打印类
class StatusPrinter:
    def __init__(self, print_every_n=10):
        self.print_every_n = print_every_n
        self._last_phase = None

    def print_header(self):
        print("\n" + "=" * 90)
        print("🤖 INTEGRATED CHASSIS CONTROLLER - RELOCALIZATION + NAVIGATION")
        print("=" * 90)
        print("Phase 1: Original Position Rotation (until relocalization)")
        print("Phase 2: Align direction to point (within ±5°)")
        print("Phase 3: Move forward (distance-based speed control)")
        print("Phase 4: Final alignment at origin (yaw = 0°)")
        print("=" * 90 + "\n")

    def phase(self, title: str):
        if title == self._last_phase:
            return
        self._last_phase = title
        print(f"\n{'=' * 90}")
        print(f"📍 {title}")
        print(f"{'=' * 90}\n")

    def success(self, msg: str):
        print("\n" + "🎉" * 45)
        print(f"✅ {msg}")
        print("🎉" * 45 + "\n")

    def tick(self, seq, elapsed, state_name, x, y, fwd, rot, yaw_deg=None, yaw_i16=None):
        if seq % self.print_every_n != 0:
            return
        dist = math.hypot(x, y)
        line = (
            f"[{elapsed:7.2f}s] {state_name:20s} | "
            f"Pos: ({x:7.4f}, {y:7.4f}) | "
            f"Dist: {dist:7.4f}m | "
        )
        if yaw_deg is not None and yaw_i16 is not None:
            line += f"Yaw: {yaw_deg:7.2f}° ({yaw_i16:6d}) | "
        line += f"Cmd: fwd={int(fwd):4d} rot={int(rot):4d}"
        print(line)

#-----------------------------------------------------------------------------------------------------
def crc16_ccitt(data, poly=0x1021, init=0xFFFF):
    crc = init
    for byte in data:
        crc ^= (byte << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ poly) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc

#将航向角度转换为i16格式（度*100，并限制在-327.68°~327.67°范围内）
def _clamp_i16(v):
    return max(-32768, min(32767, int(v)))

#通道数据构造方法
def compose_channels(lateral_cmd=0, forward_cmd=0, rotation_cmd=0, channel_count=CHANNEL_COUNT):
    channels = [0] * channel_count
    channels[0] = int(lateral_cmd)
    channels[2] = int(forward_cmd)
    channels[3] = int(rotation_cmd)
    return channels

#帧构造方法
def build_frame(seq, channels, yaw_i16=0, des_yaw_i16=0):
    """
    帧格式:
    SOF1 SOF2 LEN TYPE payload crc16

    payload:
      <H  seq>
      10*<h channels>
      <h  yaw_i16>       # 当前航向角(度*100)
      <h  des_yaw_i16>   # 目标航向角(度*100)
    """
    if len(channels) != CHANNEL_COUNT:
        raise ValueError(f"channels length must be {CHANNEL_COUNT}")

    yaw_i16 = _clamp_i16(yaw_i16)
    des_yaw_i16 = _clamp_i16(des_yaw_i16)

    payload = struct.pack("<H", seq & 0xFFFF)
    for ch in channels:
        payload += struct.pack("<h", _clamp_i16(ch))
    payload += struct.pack("<h", yaw_i16)
    payload += struct.pack("<h", des_yaw_i16)

    crc_data = bytes([LEN, TYPE]) + payload
    crc = crc16_ccitt(crc_data)

    frame = bytes([SOF1, SOF2, LEN, TYPE]) + payload + struct.pack("<H", crc)
    return frame

#构造全零停止帧方法
def build_stop_frame(seq, channel_count=CHANNEL_COUNT):
    """
    构造停止帧：
    - 全通道置0
    - 当前航向 yaw_i16 = 0
    - 目标航向 des_yaw_i16 = 0
    """
    channels = [0] * channel_count
    return build_frame(
        seq=seq,
        channels=channels,
        yaw_i16=0,
        des_yaw_i16=0
    )

#发送帧方法
def send_frame(sock, frame, connect_func, tcp_ip, tcp_port, retry_interval=0.5):
    """
    发送一帧；若发送失败则自动重连并返回新socket

    参数:
      sock: 当前socket
      frame: 要发送的bytes帧
      connect_func: 连接函数（例如你封装的 connect）
      tcp_ip, tcp_port, retry_interval: 传给 connect_func 的参数

    返回:
      (sock, ok)
      - sock: 可继续使用的socket（可能是重连后的）
      - ok: 是否发送成功
    """
    try:
        sock.sendall(frame)
        return sock, True
    except Exception as e:
        print(f"❌ Send failed: {e}; reconnecting...")

        try:
            sock.close()
        except Exception:
            pass

        try:
            new_sock = connect_func(
                tcp_ip=tcp_ip,
                tcp_port=tcp_port,
                retry_interval=retry_interval
            )
            # 重连成功后，尝试补发当前帧
            new_sock.sendall(frame)
            return new_sock, True
        except Exception as e2:
            print(f"❌ Re-send after reconnect failed: {e2}")
            return sock, False

#发送停止帧方法
def send_stop_frame(
    sock,
    seq,
    send_frame_func=None,
    connect_func=connect,
    tcp_ip=TCP_IP,
    tcp_port=TCP_PORT,
    retry_interval=CONNECT_RETRY_INTERVAL,
):
    """
    构造并发送停止帧（带重连）
    返回: (sock, ok)
    """
    if send_frame_func is None:
        send_frame_func = send_frame

    frame = build_stop_frame(seq)
    return send_frame_func(
        sock=sock,
        frame=frame,
        connect_func=connect_func,
        tcp_ip=tcp_ip,
        tcp_port=tcp_port,
        retry_interval=retry_interval,
    )


class SendThread:
    """
    后台持续发帧线程。

    说明:
    - 默认 50Hz 发送
    - 外部通过 set_* 方法修改当前输出状态
    - 线程内部复用 build_frame() / send_frame()
    """
    def __init__(
        self,
        sock,
        seq=0,
        hz=50.0,
        channel_count=CHANNEL_COUNT,
        connect_func=connect,
        tcp_ip=TCP_IP,
        tcp_port=TCP_PORT,
        retry_interval=CONNECT_RETRY_INTERVAL,
    ):
        self.sock = sock
        self.seq = int(seq) & 0xFFFF
        self.hz = float(hz)
        if self.hz <= 0.0:
            raise ValueError(f"hz must be > 0, got {hz}")
        self.period = 1.0 / self.hz
        self.channel_count = int(channel_count)
        self.connect_func = connect_func
        self.tcp_ip = tcp_ip
        self.tcp_port = tcp_port
        self.retry_interval = retry_interval

        self._lock = threading.Lock()
        self._channels = [0] * self.channel_count
        self._yaw_i16 = 0
        self._des_yaw_i16 = 0
        self._running = False
        self._thread = None

    def set_channels(self, channels, yaw_i16=None, des_yaw_i16=None):
        if len(channels) != self.channel_count:
            raise ValueError(f"channels length must be {self.channel_count}")
        with self._lock:
            self._channels = [int(v) for v in channels]
            if yaw_i16 is not None:
                self._yaw_i16 = int(yaw_i16)
            if des_yaw_i16 is not None:
                self._des_yaw_i16 = int(des_yaw_i16)

    def set_channel(self, index, value):
        index = int(index)
        if index < 0 or index >= self.channel_count:
            raise IndexError(f"channel index out of range: {index}")
        with self._lock:
            self._channels[index] = int(value)

    def set_yaw(self, yaw_i16=None, des_yaw_i16=None):
        with self._lock:
            if yaw_i16 is not None:
                self._yaw_i16 = int(yaw_i16)
            if des_yaw_i16 is not None:
                self._des_yaw_i16 = int(des_yaw_i16)

    def set_all_zero(self, des_yaw_i16=0, yaw_i16=0):
        with self._lock:
            self._channels = [0] * self.channel_count
            self._yaw_i16 = int(yaw_i16)
            self._des_yaw_i16 = int(des_yaw_i16)

    def get_state(self):
        with self._lock:
            return {
                "channels": list(self._channels),
                "yaw_i16": int(self._yaw_i16),
                "des_yaw_i16": int(self._des_yaw_i16),
                "seq": int(self.seq),
            }

    def start(self):
        with self._lock:
            if self._running:
                return self._thread
            self._running = True

        self._thread = threading.Thread(
            target=self._run,
            daemon=True,
            name="send_frame_thread",
        )
        self._thread.start()
        return self._thread

    def stop(self, send_stop=False):
        with self._lock:
            self._running = False

        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=1.0)

        if send_stop:
            try:
                self.sock, _ = send_stop_frame(
                    sock=self.sock,
                    seq=self.seq,
                    send_frame_func=send_frame,
                    connect_func=self.connect_func,
                    tcp_ip=self.tcp_ip,
                    tcp_port=self.tcp_port,
                    retry_interval=self.retry_interval,
                )
            except Exception:
                pass

    def _snapshot(self):
        with self._lock:
            return (
                list(self._channels),
                int(self._yaw_i16),
                int(self._des_yaw_i16),
                bool(self._running),
            )

    def _run(self):
        next_send = time.time()
        while True:
            channels, yaw_i16, des_yaw_i16, running = self._snapshot()
            if not running:
                break

            frame = build_frame(
                seq=self.seq,
                channels=channels,
                yaw_i16=yaw_i16,
                des_yaw_i16=des_yaw_i16,
            )
            self.sock, ok = send_frame(
                sock=self.sock,
                frame=frame,
                connect_func=self.connect_func,
                tcp_ip=self.tcp_ip,
                tcp_port=self.tcp_port,
                retry_interval=self.retry_interval,
            )
            if ok:
                self.seq = (self.seq + 1) & 0xFFFF
                next_send += self.period
            else:
                next_send = time.time()

            sleep_time = next_send - time.time()
            if sleep_time > 0.0:
                time.sleep(sleep_time)
            else:
                next_send = time.time()


def start_send_thread(
    sock,
    seq=0,
    hz=50.0,
    channel_count=CHANNEL_COUNT,
    connect_func=connect,
    tcp_ip=TCP_IP,
    tcp_port=TCP_PORT,
    retry_interval=CONNECT_RETRY_INTERVAL,
):
    sender = SendThread(
        sock=sock,
        seq=seq,
        hz=hz,
        channel_count=channel_count,
        connect_func=connect_func,
        tcp_ip=tcp_ip,
        tcp_port=tcp_port,
        retry_interval=retry_interval,
    )
    sender.start()
    return sender

#-----------------------------------------------------------------------------------------------------
#安全关闭socket方法（全程兜底）
def socket_close(sock):
    """
    安全关闭 socket
    - sock 为 None 时直接返回
    - 关闭前尝试 shutdown，避免连接残留
    - 全程兜底不抛异常
    """
    if sock is None:
        return

    try:
        sock.shutdown(2)  # 等价于 socket.SHUT_RDWR
    except Exception:
        pass

    try:
        sock.close()
    except Exception:
        pass

#-----------------------------------------------------------------------------------------------------
#统一的 Ctrl+C 处理函数，执行清理流程
def handle_ctrl_c(
    sock=None,
    seq=0,
    build_stop_frame_func=None,
    send_frame_func=None,
    socket_close_func=None,
    connect_func=None,
    tcp_ip=TCP_IP,
    tcp_port=TCP_PORT,
    retry_interval=CONNECT_RETRY_INTERVAL,
    node=None,
    spin_thread=None,
    stop_event=None,
    shutdown_rclpy=False,
):
    """
    Ctrl+C 终止时的统一清理函数（适配 send_frame 的重连签名）

    功能:
    1) 尝试发送一帧全零停止命令（可重连）
    2) 关闭socket
    3) 停止ROS2线程并销毁节点
    4) 可选 shutdown rclpy

    返回:
      (sock, True)
      - sock: 清理结束时的socket引用（通常已关闭）
      - True: 清理流程已执行
    """
    print("\n🛑 Ctrl+C received, shutting down gracefully...")

    # 1) 发送停止帧（可选）
    try:
        if (
            sock is not None
            and build_stop_frame_func is not None
            and send_frame_func is not None
            and connect_func is not None
        ):
            stop_frame = build_stop_frame_func(seq)
            sock, ok = send_frame_func(
                sock=sock,
                frame=stop_frame,
                connect_func=connect_func,
                tcp_ip=tcp_ip,
                tcp_port=tcp_port,
                retry_interval=retry_interval,
            )
            if not ok:
                print("⚠️ Stop frame send failed.")
    except Exception as e:
        print(f"⚠️ Failed to send stop frame: {e}")

    # 2) 关闭socket
    try:
        if socket_close_func is not None:
            socket_close_func(sock)
        elif sock is not None:
            try:
                sock.shutdown(2)
            except Exception:
                pass
            try:
                sock.close()
            except Exception:
                pass
    except Exception as e:
        print(f"⚠️ Failed to close socket: {e}")

    # 3) 停ROS线程/节点
    try:
        if stop_event is not None:
            stop_event.set()
        if spin_thread is not None and spin_thread.is_alive():
            spin_thread.join(timeout=1.0)
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
    except Exception as e:
        print(f"⚠️ Failed to stop ROS thread/node: {e}")

    # 4) 可选 shutdown rclpy
    if shutdown_rclpy:
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception as e:
            print(f"⚠️ Failed to shutdown rclpy: {e}")

    print("✅ Graceful shutdown complete.")
    return sock, True
#-----------------------------------------------------------------------------------------------------
#角度归一化方法
def yaw_normalization(yaw_deg):
    """
    将角度归一化到 [-180, 180) 区间
    """
    while yaw_deg >= 180.0:
        yaw_deg -= 360.0
    while yaw_deg < -180.0:
        yaw_deg += 360.0
    return yaw_deg

#角度转换方法
def yaw_deg_to_i16(yaw_deg):
    """
    yaw角(度) -> 归一化到[-180,180) -> int16(度*100)
    """
    yaw_deg = yaw_normalization(yaw_deg)
    return _clamp_i16(round(yaw_deg * 100.0))
#-----------------------------------------------------------------------------------------------------
