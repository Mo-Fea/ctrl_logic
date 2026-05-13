# R2_H 上位机控制使用指南

本指南基于当前固件源码（`main.c` / `freertos.c` / `auto_control.c` / `rc.c` / `W5500_USER.c`）整理，目标是让上位机通过网口稳定触发各类动作。

## 1. 网络与连接参数

- 主控网口芯片：W5500（TCP Server 模式）
- 主控 IP：`192.168.2.199`
- 子网掩码：`255.255.255.0`
- 网关：`192.168.1.1`
- 监听端口：`5000`（`0x1388`）
- 建议上位机与主控直连或同网段交换机连接；上位机 IP 配到 `192.168.2.x`（x != 199）

> 固件上电后默认开启 `RC_NetControl_Enable(1U)`，网口控制已启用。

## 2. 上位机控制帧协议

### 2.1 帧结构

- Byte0: `SOF1 = 0xA5`
- Byte1: `SOF2 = 0x5A`
- Byte2: `payload_len`
- Byte3: `frame_type = 0x01`
- Byte4..: payload
- 最后2字节：`CRC16-CCITT`（小端）

### 2.2 payload 三种长度（固件都兼容）

1. Legacy (`payload_len=22`)
- `seq(uint16)` + `ch[10]`（10个 int16）

2. V1 (`payload_len=24`)
- `seq(uint16)` + `ch[10]` + `yaw_raw_cdeg(int16)`

3. V2 (`payload_len=26`)【推荐】
- `seq(uint16)` + `ch[10]` + `yaw_raw_cdeg(int16)` + `target_yaw_cdeg(int16)`

> 单位 `cdeg` = `0.01°`，例如 `9000 = 90.00°`。

### 2.3 CRC 规则

- 多项式：`0x1021`
- 初值：`0xFFFF`
- 按字节高位先行
- 参与 CRC 的数据范围：从 `payload_len`（Byte2）开始，到 payload 结束（即 `payload_len + frame_type + payload`）

## 3. 通道语义（ch0~ch9）

固件接收后会做通道规范化，建议上位机直接按下表发送。

- `ch0/ch1/ch2/ch3`：摇杆量，范围建议 `-992 ~ 992`（小于20绝对值会被死区归零）
- `ch4`：二段开关，建议只发 `1` 或 `3`
- `ch5`：三段模式开关，发 `1/2/3`
- `ch6`：子功能/姿态选择；KFS 姿态选择当前使用 `1/2/3/4`
- `ch7`：二段触发开关，发 `1/3`
- `ch8/ch9`：二段扩展开关，发 `1/3`

## 4. 动作控制对照（重点）

先给一个“安全中位”参考：
- `ch = [0,0,0,0, 1,1,1,1,1,1]`

### 4.1 底盘平移 + 航向锁定（网控主流程）

- 底盘平移：
  - `ch2` -> 前后速度
  - `ch0` -> 左右平移
- 航向锁定使用 `yaw_raw_cdeg + target_yaw_cdeg`（V2帧）
- 若网口帧超时 >150ms，航向 PID 保护退出
- 建议发包频率：`20~50Hz`（必须持续发，不要断流）

### 4.2 模式切换（ch5）

- `ch5=1`：升降模式（MODE_LIFT_CONTROL）
- `ch5=2`：方块模式（MODE_CUBE）
- `ch5=3`：武器模式（MODE_WEAPON）

### 4.3 气缸控制（ch4 边沿触发）

在当前模式下，`ch4` 从 `1->3` 视为上升沿（置位），`3->1` 视为下降沿（复位）：

- `ch5=1` 时控制 `PF1`
- `ch5=2` 时控制 `PF3`
- `ch5=3` 时控制 `PF0`

### 4.4 升降模式动作（ch5=1）

1. ID5 手动升降
- `ch1` 按比例映射到 ID5 转速（约 `-5000~5000`）

2. 自动上楼
- 触发条件：`ch7` 做一次 `1->3` 上升沿
- 执行逻辑：
  - `PF1` 拉高
  - 等待 `2000ms`
  - ID5 以 `-2500` 运转
  - 当 `tinyf6_distance` 先达到 `>=250mm` 后，再降到 `<=85mm`，动作结束（ID5停止）

3. 自动下楼
- 触发条件：`ch6` 首次进入 `3`（保持3不会重复触发，需先离开3再可再次触发）
- 执行逻辑：
  - 等待 `tinyf6_distance < 85mm`
  - 底盘 ID1~ID4 以 `-500` 后退
  - 当 `tinyf6_distance > 250mm`：`PF3` 拉高并等待 `2000ms`
  - ID5 以 `+2500` 运转
  - 当 `tinyf3_distance > 250mm`：停止底盘+ID5，`PF3`复位，动作结束

### 4.5 方块/KFS 模式动作（ch5=2）

KFS 方块吸取拆成两块：

1. 吸盘机械臂姿态控制：`ch6` 选择姿态，`ch7` 上升沿触发。
2. 吸盘吸取/释放：`ch4` 边沿控制，见 4.3。

姿态选择：

- `ch6=1`：高位方块抓取姿态
- `ch6=2`：低位方块抓取姿态
- `ch6=3`：过渡态
- `ch6=4`：存储方块姿态

姿态触发：

- 触发条件：`ch5=2`，`ch6` 先选定姿态，`ch7` 做一次 `1->3` 上升沿
- 触发锁存：`ch7=3` 只触发一次；需要再次触发时，先把 `ch7` 发回 `1`，再发 `3`
- 回归 0 态：发送 `ch7=0`
- 一个完整吸取流程：先执行 `ch6=1` 或 `ch6=2` 抓取姿态；完成后执行 `ch6=3` 过渡态；过渡完成后执行 `ch6=4` 存储方块姿态。
- 到位判定：
  - ID6 误差 <= `120 ecd`
  - 脉塔误差 <= `100 cdeg`
- 超时：`8000ms`

### 4.6 武器模式动作（ch5=3）

- `ch1>0`：ID7 目标置为 `6000`
- `ch1<0`：ID7 目标置为 `0`

## 5. 上位机发包 Python 示例（TCP）

```python
import socket
import struct
import time

ROBOT_IP = "192.168.2.199"
ROBOT_PORT = 5000

SOF1 = 0xA5
SOF2 = 0x5A
FRAME_TYPE = 0x01


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= (b << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def build_frame_v2(seq: int, ch, yaw_raw_cdeg: int, target_yaw_cdeg: int) -> bytes:
    # ch: 10个int16
    assert len(ch) == 10
    payload = struct.pack('<H', seq)
    payload += struct.pack('<10h', *ch)
    payload += struct.pack('<h', yaw_raw_cdeg)
    payload += struct.pack('<h', target_yaw_cdeg)

    payload_len = len(payload)  # 26
    head_and_payload = bytes([payload_len, FRAME_TYPE]) + payload
    crc = crc16_ccitt(head_and_payload)

    frame = bytes([SOF1, SOF2]) + head_and_payload + struct.pack('<H', crc)
    return frame


def main():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((ROBOT_IP, ROBOT_PORT))

    seq = 0
    # 安全中位
    ch = [0, 0, 0, 0, 1, 1, 1, 1, 1, 1]

    try:
        while True:
            # 示例：升降模式 + 轻微前进
            ch[5] = 1      # ch5=1 升降模式
            ch[2] = 180    # ch2前后

            frame = build_frame_v2(seq, ch, yaw_raw_cdeg=0, target_yaw_cdeg=0)
            s.sendall(frame)
            seq = (seq + 1) & 0xFFFF
            time.sleep(0.02)  # 50Hz
    finally:
        s.close()


if __name__ == "__main__":
    main()
```

## 6. 动作触发模板（给上位机状态机）

### 自动上楼触发脉冲

1. 先稳定发送：`ch5=1, ch7=1`
2. 持续 100~200ms 后改为：`ch7=3`（保持 100~200ms）
3. 再回到：`ch7=1`

### 自动下楼触发脉冲

1. 先稳定发送：`ch5=1, ch6=1`
2. 改为：`ch6=3`（保持 100~200ms）
3. 再回到：`ch6=1`（否则不会再次触发）

### 方块模式同步任务触发

1. 先进入方块模式和子模式：`ch5=2, ch6=2, ch7=1`
2. 改为：`ch7=3`（保持 100~200ms）
3. 再回到：`ch7=1`

## 7. 联调注意事项

- 必须持续周期发包；断流会触发超时保护（RC/SAFE回退和航向PID退出）。
- 强烈建议固定用 V2 帧，否则 `target_yaw_cdeg` 会被置 0。
- 开关通道按 `1/2/3` 发，不要发 0/1000/2000 这种 PWM 值。
- 若动作未触发，优先检查是否做了“边沿脉冲”（1->3 或 3->1），而不是一直保持同一档位。

---
如需，我可以在下一步再给你一份“可直接运行的上位机最小程序（含GUI按钮）”，把“上楼/下楼/同步任务/模式切换”做成一键操作。
