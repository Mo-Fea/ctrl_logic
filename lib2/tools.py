import socket
import time
import math
import struct
import threading
import sys
import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool


TCP_IP = "192.168.2.199"
TCP_PORT = 5000
CONNECT_RETRY_INTERVAL = 0.5
SOF1 = 0xA5
SOF2 = 0x5A
TYPE = 0x01
CHANNEL_COUNT = 10
LEN = 0x1C
SAFE_SWITCH_VALUE = 1
CYLINDER_SELECT_BOTH = 0
CYLINDER_SELECT_PF2 = 1
CYLINDER_SELECT_PF3 = 2
relocalization_flag = False
AUTO_TRIGGER_LOCK = threading.Lock()


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


class _RelocalizationNode(Node):
    def __init__(self, stop_event: threading.Event, topic='/odin1/flag1'):
        super().__init__('relocalization_conformation_node')
        self._stop_event = stop_event
        self._flag = False
        self._lock = threading.Lock()

        self._sub = self.create_subscription(
            Bool, topic, self._callback, 10
        )

    def _callback(self, msg: Bool):
        global relocalization_flag

        with self._lock:
            prev = self._flag
            self._flag = bool(msg.data)

        if (not prev) and self._flag:
            relocalization_flag = True
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

    stop_event = threading.Event()
    node = _RelocalizationNode(stop_event=stop_event, topic=topic)
    spin_thread = threading.Thread(
        target=_spin_relocalization,
        args=(node, stop_event),
        daemon=True
    )
    spin_thread.start()

    return node.get_flag, node, spin_thread, stop_event


def destroy_ros2_thread(node, spin_thread, stop_event, shutdown_rclpy=False):
    """
    停止并销毁一个 ROS 线程/节点。
    """
    if stop_event is not None:
        stop_event.set()

    if spin_thread is not None and spin_thread.is_alive():
        spin_thread.join(timeout=1.0)

    if node is not None:
        try:
            node.destroy_node()
        except Exception:
            pass

    if shutdown_rclpy and rclpy.ok():
        try:
            rclpy.shutdown()
        except Exception:
            pass


def compose_channels(lateral_cmd=0, forward_cmd=0, rotation_cmd=0, channel_count=CHANNEL_COUNT):
    channels = [0] * channel_count
    for index in range(4, channel_count):
        channels[index] = SAFE_SWITCH_VALUE
    channels[0] = int(lateral_cmd)
    channels[2] = int(forward_cmd)
    channels[3] = int(rotation_cmd)
    return channels


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


def _clamp_i16(v):
    return max(-32768, min(32767, int(v)))


def validate_cylinder_select(cylinder_select):
    cylinder_select = int(cylinder_select)
    if cylinder_select not in (
        CYLINDER_SELECT_BOTH,
        CYLINDER_SELECT_PF2,
        CYLINDER_SELECT_PF3,
    ):
        raise ValueError(
            "cylinder_select must be 0(PF2/PF3), 1(PF2) or 2(PF3), "
            f"got {cylinder_select}"
        )
    return cylinder_select


def build_frame(seq, channels, yaw_i16=0, des_yaw_i16=0, cylinder_select=CYLINDER_SELECT_BOTH):
    """
    帧格式:
    SOF1 SOF2 LEN TYPE payload crc16

    payload:
      <H  seq>
      10*<h channels>
      <h  yaw_i16>
      <h  des_yaw_i16>
      <h  cylinder_select>
    """
    if len(channels) != CHANNEL_COUNT:
        raise ValueError(f"channels length must be {CHANNEL_COUNT}")

    yaw_i16 = _clamp_i16(yaw_i16)
    des_yaw_i16 = _clamp_i16(des_yaw_i16)
    cylinder_select = validate_cylinder_select(cylinder_select)

    payload = struct.pack("<H", seq & 0xFFFF)
    for ch in channels:
        payload += struct.pack("<h", _clamp_i16(ch))
    payload += struct.pack("<h", yaw_i16)
    payload += struct.pack("<h", des_yaw_i16)
    payload += struct.pack("<h", cylinder_select)

    crc_data = bytes([LEN, TYPE]) + payload
    crc = crc16_ccitt(crc_data)

    frame = bytes([SOF1, SOF2, LEN, TYPE]) + payload + struct.pack("<H", crc)
    return frame


def build_stop_frame(seq, channel_count=CHANNEL_COUNT):
    channels = compose_channels(channel_count=channel_count)
    return build_frame(
        seq=seq,
        channels=channels,
        yaw_i16=0,
        des_yaw_i16=0,
        cylinder_select=CYLINDER_SELECT_BOTH,
    )


def send_frame(sock, frame, connect_func, tcp_ip, tcp_port, retry_interval=0.5):
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
                retry_interval=retry_interval,
            )
            new_sock.sendall(frame)
            return new_sock, True
        except Exception as e2:
            print(f"❌ Re-send after reconnect failed: {e2}")
            return sock, False


def send_stop_frame(
    sock,
    seq,
    send_frame_func=None,
    connect_func=connect,
    tcp_ip=TCP_IP,
    tcp_port=TCP_PORT,
    retry_interval=CONNECT_RETRY_INTERVAL,
):
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


def socket_close(sock):
    if sock is None:
        return

    try:
        sock.shutdown(2)
    except Exception:
        pass

    try:
        sock.close()
    except Exception:
        pass


def yaw_normalization(yaw_deg):
    while yaw_deg >= 180.0:
        yaw_deg -= 360.0
    while yaw_deg < -180.0:
        yaw_deg += 360.0
    return yaw_deg


def yaw_deg_to_i16(yaw_deg):
    yaw_deg = yaw_normalization(yaw_deg)
    return _clamp_i16(round(yaw_deg * 100.0))


RED_LIDAR_CORRECTION_BY_YAW = {
    0.01: (0.000, 0.000),
    90.00: (-0.050, -0.067),
    180.00: (0.025, -0.151),
    -90.00: (0.104, -0.081),
}

BLUE_LIDAR_CORRECTION_BY_YAW = {
    0.01: (0.000, 0.000),
    90.00: (-0.050, -0.067),
    180.00: (0.025, -0.151),
    -90.00: (0.104, -0.081),
}


def _get_lidar_correction_table():
    from lib2 import position_backend

    if position_backend.is_blue_field():
        return BLUE_LIDAR_CORRECTION_BY_YAW
    return RED_LIDAR_CORRECTION_BY_YAW


def _apply_lidar_correction(x, y, yaw_deg):
    correction_table = _get_lidar_correction_table()
    yaw_deg = round(float(yaw_deg), 2)
    if yaw_deg not in correction_table:
        print(f"{_apply_lidar_correction.__name__}输入错误: yaw_deg={yaw_deg}")
        sys.exit(1)

    dx, dy = correction_table[yaw_deg]
    return float(x) + float(dx), float(y) + float(dy)


def deg0_correction(x, y):
    return _apply_lidar_correction(x, y, 0.01)


def deg90_correction(x, y):
    return _apply_lidar_correction(x, y, 90.00)


def neg90_correction(x, y):
    return _apply_lidar_correction(x, y, -90.00)


def deg180_correction(x, y):
    return _apply_lidar_correction(x, y, 180.00)


def deg_correction(target_deg, x, y):
    target_deg = round(yaw_normalization(float(target_deg)), 2)
    if abs(target_deg) <= 0.02:
        return deg0_correction(x, y)
    if target_deg == 90.00:
        return deg90_correction(x, y)
    if target_deg == -90.00:
        return neg90_correction(x, y)
    if target_deg in (180.00, -180.00):
        return deg180_correction(x, y)

    print(f"{deg_correction.__name__}输入错误: target_deg={target_deg}")
    sys.exit(1)


def direction_int_to_yaw_deg(direction):
    direction = int(direction)
    if direction not in (1, 2, 3, 4):
        print(f"{direction_int_to_yaw_deg.__name__}输入错误")
        sys.exit(1)

    # 方向编号表达任务/场地语义；红蓝半场的 map x 轴物理方向相反，
    # 因此 x 轴相关的 2/3 方向需要按半场交换 yaw。
    from lib2 import position_backend

    if position_backend.is_blue_field():
        direction_to_yaw = {
            1: 90.00,
            2: 0.01,
            3: 180.00,
            4: -90.00,
        }
    else:
        direction_to_yaw = {
            1: 90.00,
            2: 180.00,
            3: 0.01,
            4: -90.00,
        }

    return round(float(direction_to_yaw[direction]), 2)


def map_direction_int_to_yaw_deg(direction):
    """
    按当前 map 坐标轴直接换算方向 yaw，不做红蓝半场物理方向修正。
    """
    direction = int(direction)
    if direction not in (1, 2, 3, 4):
        print(f"{map_direction_int_to_yaw_deg.__name__}输入错误")
        sys.exit(1)

    direction_to_yaw = {
        1: 90.00,
        2: 180.00,
        3: 0.01,
        4: -90.00,
    }
    return round(float(direction_to_yaw[direction]), 2)


def stair_id_to_direction(from_id, to_id, stair_matrix=None, exit_on_error=True):
    """
    根据梅林台阶编号判断 to_id 在 from_id 的哪个方向。

    返回值沿用动作矩阵方向编码:
      0: 不相邻
      1/2/3/4: 当前红蓝场语义下的四方向编号。

    当前实测方向语义:
      红场:
        dx > 0 -> direction 3  # 0.01deg, x+
        dx < 0 -> direction 2  # 180deg, x-
        dy > 0 -> direction 1  # 90deg, y+
        dy < 0 -> direction 4  # -90deg, y-
      蓝场:
        dx > 0 -> direction 2  # 0.01deg, x+
        dx < 0 -> direction 3  # 180deg, x-
        dy > 0 -> direction 4  # -90deg, y+
        dy < 0 -> direction 1  # 90deg, y-

    默认读取 position_resource.get_stair_matrix()，并使用矩阵中的真实 x/y 坐标判断，
    不依赖编号是否连续，因此 3 和 4 这种跨行编号不会被误判为左右相邻。
    """
    from_id = int(from_id)
    to_id = int(to_id)

    if stair_matrix is None:
        from lib2 import position_resource

        stair_matrix = position_resource.get_stair_matrix()

    positions = {}
    for row in stair_matrix:
        positions[int(row[0])] = (float(row[4]), float(row[5]))

    from_xy = positions.get(from_id)
    to_xy = positions.get(to_id)
    direction = 0

    if from_xy is not None and to_xy is not None:
        dx = to_xy[0] - from_xy[0]
        dy = to_xy[1] - from_xy[1]
        abs_dx = abs(dx)
        abs_dy = abs(dy)
        side_length = 1.2
        major_axis_min = side_length * 0.75
        major_axis_max = side_length * 1.25
        minor_axis_max = side_length * 0.2

        from lib2 import position_backend

        if major_axis_min <= abs_dx <= major_axis_max and abs_dy <= minor_axis_max:
            if position_backend.is_blue_field():
                direction = 2 if dx > 0.0 else 3
            else:
                direction = 3 if dx > 0.0 else 2
        elif major_axis_min <= abs_dy <= major_axis_max and abs_dx <= minor_axis_max:
            if position_backend.is_blue_field():
                direction = 4 if dy > 0.0 else 1
            else:
                direction = 1 if dy > 0.0 else 4

    if direction == 0 and exit_on_error:
        print(f"{stair_id_to_direction.__name__}输入错误: {from_id} 与 {to_id} 不相邻")
        sys.exit(1)

    return direction


def stair_id_to_matrix_index(stair_id, stair_matrix=None, exit_on_error=True):
    """
    根据台阶编号返回其在当前台阶矩阵中的 0-based 行号。

    例如当前矩阵中:
      stair_id=-1 -> 0
      stair_id=1  -> 1
      stair_id=15 -> 14
    """
    stair_id = int(stair_id)
    if stair_matrix is None:
        from lib2 import position_resource

        stair_matrix = position_resource.get_stair_matrix()

    for index, row in enumerate(stair_matrix):
        if int(row[0]) == stair_id:
            return int(index)

    if exit_on_error:
        print(f"{stair_id_to_matrix_index.__name__}输入错误: 未找到台阶编号 {stair_id}")
        sys.exit(1)
    return None


def debug_print(
    sender,
    position_runtime,
    target_x,
    target_y,
    stop_event=None,
    interval_sec=0.2,
):
    """
    持续打印调试信息：
    - 距离目标点距离
    - 当前输出目标角 des_yaw_i16 对应角度
    - 当前 yaw 到目标角的偏差
    - ch2 前进通道值

    des_yaw_i16 == 0 是协议特殊值，表示停止旋转；此时角度偏差不适用。
    """
    while stop_event is None or not stop_event.is_set():
        state = sender.get_state()
        channels = state["channels"]
        ch2 = int(channels[2])
        des_yaw_i16 = int(state["des_yaw_i16"])

        position = position_runtime.get_current_position()
        current_yaw_deg = position_runtime.get_current_yaw_deg()

        if position is None or current_yaw_deg is None:
            print(
                "[debug] waiting pose | "
                f"target_yaw_i16={des_yaw_i16} | ch2={ch2}"
            )
        else:
            distance_xy = math.hypot(
                float(target_x) - float(position["x"]),
                float(target_y) - float(position["y"]),
            )

            if des_yaw_i16 == 0:
                target_yaw_text = "STOP(0)"
                yaw_error_text = "N/A"
            else:
                target_yaw_deg = yaw_normalization(float(des_yaw_i16) / 100.0)
                yaw_error_deg = yaw_normalization(target_yaw_deg - float(current_yaw_deg))
                target_yaw_text = f"{target_yaw_deg:7.2f}deg"
                yaw_error_text = f"{yaw_error_deg:7.2f}deg"

            print(
                "[debug] "
                f"dist={distance_xy:7.3f}m | "
                f"target_yaw={target_yaw_text} | "
                f"yaw_error={yaw_error_text} | "
                f"ch2={ch2:4d}"
            )

        time.sleep(float(interval_sec))


class frame_thread:
    """
    持续发 frame 的后台线程类。

    说明:
    - 维护 ch0 ~ ch9、当前航向 yaw_i16、目标航向 des_yaw_i16、气泵选择 cylinder_select
    - start() 后按固定频率持续发送
    - 外部通过 set_* 方法修改输出状态
    - 默认 des_yaw_i16=0，关闭航向 PID；重定位成功后也不自动修改目标航向
    """
    def __init__(
        self,
        sock,
        seq=0,
        hz=70.0,
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

        self.connect_func = connect_func
        self.tcp_ip = tcp_ip
        self.tcp_port = tcp_port
        self.retry_interval = retry_interval

        self.ch0 = 0
        self.ch1 = 0
        self.ch2 = 0
        self.ch3 = 0
        self.ch4 = SAFE_SWITCH_VALUE
        self.ch5 = SAFE_SWITCH_VALUE
        self.ch6 = SAFE_SWITCH_VALUE
        self.ch7 = SAFE_SWITCH_VALUE
        self.ch8 = SAFE_SWITCH_VALUE
        self.ch9 = SAFE_SWITCH_VALUE

        self.yaw_i16 = 0
        self.des_yaw_i16 = 0
        self.cylinder_select = CYLINDER_SELECT_BOTH

        self._lock = threading.Lock()
        self._running = False
        self._thread = None
        self._first_send_event = threading.Event()
        self._last_send_ok = False
        self._last_send_time = None

    def _channels_snapshot(self):
        return [
            int(self.ch0),
            int(self.ch1),
            int(self.ch2),
            int(self.ch3),
            int(self.ch4),
            int(self.ch5),
            int(self.ch6),
            int(self.ch7),
            int(self.ch8),
            int(self.ch9),
        ]

    def _reset_channels_locked(self):
        self.ch0 = 0
        self.ch1 = 0
        self.ch2 = 0
        self.ch3 = 0
        self.ch4 = SAFE_SWITCH_VALUE
        self.ch5 = SAFE_SWITCH_VALUE
        self.ch6 = SAFE_SWITCH_VALUE
        self.ch7 = SAFE_SWITCH_VALUE
        self.ch8 = SAFE_SWITCH_VALUE
        self.ch9 = SAFE_SWITCH_VALUE

    def _set_channel_locked(self, index, value):
        index = int(index)
        if index < 0 or index >= CHANNEL_COUNT:
            raise ValueError(f"channel index must be 0..{CHANNEL_COUNT - 1}, got {index}")
        setattr(self, f"ch{index}", int(value))

    def set_channel(self, index, value):
        """
        设置单个通道值。

        frame_thread 内部维护 ch0~ch9，发送线程会在 _snapshot() 中把这些
        内部变量统一构建成完整通道列表。
        """
        index = int(index)
        if index < 0 or index >= CHANNEL_COUNT:
            raise ValueError(f"channel index must be 0..{CHANNEL_COUNT - 1}, got {index}")
        with self._lock:
            self._set_channel_locked(index, value)

    def set_channel_values(
        self,
        channel_values,
        yaw_i16=None,
        des_yaw_i16=None,
        cylinder_select=None,
        reset_channels=False,
    ):
        """
        原子设置一个或多个内部通道变量。

        channel_values 只描述需要修改的通道，例如 {0: lateral, 2: forward}。
        reset_channels=True 时先恢复 ch0~ch3=0、ch4~ch9=SAFE_SWITCH_VALUE。
        cylinder_select 用于 V3 协议的抽气泵选择：0=PF2/PF3，1=PF2，2=PF3。
        """
        with self._lock:
            if reset_channels:
                self._reset_channels_locked()
            for index, value in dict(channel_values).items():
                self._set_channel_locked(index, value)
            if yaw_i16 is not None:
                self.yaw_i16 = int(yaw_i16)
            if des_yaw_i16 is not None:
                self.des_yaw_i16 = int(des_yaw_i16)
            if cylinder_select is not None:
                self.cylinder_select = validate_cylinder_select(cylinder_select)
            return self._channels_snapshot()

    def set_ch0(self, value):
        self.set_channel(0, value)

    def set_ch1(self, value):
        self.set_channel(1, value)

    def set_ch2(self, value):
        self.set_channel(2, value)

    def set_ch3(self, value):
        self.set_channel(3, value)

    def set_ch4(self, value):
        self.set_channel(4, value)

    def set_ch5(self, value):
        self.set_channel(5, value)

    def set_ch6(self, value):
        self.set_channel(6, value)

    def set_ch7(self, value):
        self.set_channel(7, value)

    def set_ch8(self, value):
        self.set_channel(8, value)

    def set_ch9(self, value):
        self.set_channel(9, value)

    def set_current_yaw_i16(self, yaw_i16):
        with self._lock:
            self.yaw_i16 = int(yaw_i16)

    def set_des_yaw_i16(self, des_yaw_i16):
        with self._lock:
            self.des_yaw_i16 = int(des_yaw_i16)

    def set_cylinder_select(self, cylinder_select):
        with self._lock:
            self.cylinder_select = validate_cylinder_select(cylinder_select)

    def set_cylinder_select_both(self):
        self.set_cylinder_select(CYLINDER_SELECT_BOTH)

    def set_cylinder_select_pf2(self):
        self.set_cylinder_select(CYLINDER_SELECT_PF2)

    def set_cylinder_select_pf3(self):
        self.set_cylinder_select(CYLINDER_SELECT_PF3)

    def set_safe_stop(self, yaw_i16=0, des_yaw_i16=0, cylinder_select=CYLINDER_SELECT_BOTH):
        with self._lock:
            self._reset_channels_locked()
            self.yaw_i16 = int(yaw_i16)
            self.des_yaw_i16 = int(des_yaw_i16)
            self.cylinder_select = validate_cylinder_select(cylinder_select)

    def get_state(self):
        with self._lock:
            return {
                "channels": self._channels_snapshot(),
                "yaw_i16": int(self.yaw_i16),
                "des_yaw_i16": int(self.des_yaw_i16),
                "cylinder_select": int(self.cylinder_select),
                "seq": int(self.seq),
                "last_send_ok": bool(self._last_send_ok),
                "last_send_time": self._last_send_time,
            }

    def start(self):
        with self._lock:
            if self._running:
                return self._thread
            self._running = True
            self._last_send_ok = False
            self._last_send_time = None
            self._first_send_event.clear()

        self._thread = threading.Thread(
            target=self._run,
            daemon=True,
            name="frame_thread",
        )
        self._thread.start()
        return self._thread

    def wait_until_first_send(self, timeout_sec=2.0):
        """
        等待后台线程至少成功 sendall 一帧。

        这只能证明 TCP sendall 成功，不能证明下位机业务层已经执行。
        """
        return self._first_send_event.wait(timeout=float(timeout_sec))

    def stop(self, send_stop=False):
        with self._lock:
            self._running = False

        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=1.0)

        if send_stop:
            try:
                with self._lock:
                    sock = self.sock
                    seq = self.seq
                self.sock, _ = send_stop_frame(
                    sock=sock,
                    seq=seq,
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
                self.sock,
                int(self.seq),
                self._channels_snapshot(),
                int(self.yaw_i16),
                int(self.des_yaw_i16),
                int(self.cylinder_select),
                bool(self._running),
            )

    def _run(self):
        next_send = time.time()
        while True:
            sock, seq, channels, yaw_i16, des_yaw_i16, cylinder_select, running = self._snapshot()
            if not running:
                break

            frame = build_frame(
                seq=seq,
                channels=channels,
                yaw_i16=yaw_i16,
                des_yaw_i16=des_yaw_i16,
                cylinder_select=cylinder_select,
            )
            next_sock, ok = send_frame(
                sock=sock,
                frame=frame,
                connect_func=self.connect_func,
                tcp_ip=self.tcp_ip,
                tcp_port=self.tcp_port,
                retry_interval=self.retry_interval,
            )
            with self._lock:
                self.sock = next_sock
                if ok:
                    self.seq = (seq + 1) & 0xFFFF
                    self._last_send_ok = True
                    self._last_send_time = time.time()
                    self._first_send_event.set()
                else:
                    self.seq = seq
                    self._last_send_ok = False

            if ok:
                next_send += self.period
            else:
                next_send = time.time()

            sleep_time = next_send - time.time()
            if sleep_time > 0.0:
                time.sleep(sleep_time)
            else:
                next_send = time.time()


def handle_ctrl_c(
    sender=None,
    flag_node=None,
    flag_thread=None,
    flag_stop_event=None,
    tf_node=None,
    tf_thread=None,
    tf_stop_event=None,
    extra_node=None,
    extra_thread=None,
    extra_stop_event=None,
    shutdown_rclpy=False,
):
    """
    Ctrl+C 时的统一清理函数。

    当前优先支持:
    - frame_thread 发送线程
    - 重定位监听线程/节点
    - TF 或其他额外 ROS 线程/节点
    """
    print("\n🛑 Ctrl+C received, shutting down gracefully...")

    try:
        if sender is not None:
            sender.stop(send_stop=True)
            socket_close(sender.sock)
    except Exception as e:
        print(f"⚠️ Failed to stop sender: {e}")

    try:
        destroy_ros2_thread(
            node=flag_node,
            spin_thread=flag_thread,
            stop_event=flag_stop_event,
            shutdown_rclpy=False,
        )
    except Exception as e:
        print(f"⚠️ Failed to stop relocalization listener: {e}")

    try:
        destroy_ros2_thread(
            node=tf_node,
            spin_thread=tf_thread,
            stop_event=tf_stop_event,
            shutdown_rclpy=False,
        )
    except Exception as e:
        print(f"⚠️ Failed to stop tf thread/node: {e}")

    try:
        destroy_ros2_thread(
            node=extra_node,
            spin_thread=extra_thread,
            stop_event=extra_stop_event,
            shutdown_rclpy=False,
        )
    except Exception as e:
        print(f"⚠️ Failed to stop extra ROS thread/node: {e}")

    if shutdown_rclpy and rclpy.ok():
        try:
            rclpy.shutdown()
        except Exception as e:
            print(f"⚠️ Failed to shutdown rclpy: {e}")

    return True
