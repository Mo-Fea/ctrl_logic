# ws_r2 当前工程进度

本目录是 R2_H 上位机控制工程。当前主线使用 `lib2/`，旧 `lib/` 基本只作参考。`R2_H-main/` 是下位机 STM32/W5500 工程，已阅读并把当前通信协议总结在本文中，后续窗口优先看本文即可，不需要重新通读单片机工程。

## 当前主线文件

- `lib2/tools.py`：TCP 连接、V2 控制帧打包、后台持续发送 `frame_thread`、重定位监听、统一清理。
- `lib2/position_backend.py`：位姿后端选择，`1=odin`，`2=mid360`；当前坐标配置跟随后端切换。
- `lib2/position_odin.py`：odin 后端，基于 `/tf`；维护 odin 对应的梅林入口、入口前点、weapon 目标点和外参。
- `lib2/position_mid360.py`：mid360 后端，基于 `/lio/robo/odom` 和 `/lio/odom`；维护 mid360 对应的梅林入口、入口前点、weapon 目标点和外参。
- `lib2/position_resource.py`：位置/里程计资源层；维护当前 `position_lib`、台阶矩阵/坐标查询、`PositionRuntime`、`OdomRuntime`、位置线程和 odom 线程启动。
- `lib2/module.py`：初始化和组合动作封装；保留 `module.start_position_thread()` / `module.start_odometry_thread()` 等兼容包装，但真实资源实现已在 `position_resource.py`。
- `lib2/move.py`：阻塞旋转、移动到点、倒退到点、上下台阶、KFS 吸盘机械臂姿态等底层动作。
- `utils/meilin.py`：梅林 12 格路径规划，0-1 BFS，返回路径。
- `utils/route.py`：D435i 扫二维码、调用规划、打印动作矩阵、可视化，有 OpenCV/matplotlib GUI。
- `utils/utils.py`：无 GUI 的纯工具，已实现 `build_action_matrix_from_qr()` / `build_action_matrix_from_kfs()`，用于生成 `n*5` 动作矩阵。
- `utils/route1.py` / `utils/race.py`：新版比赛规则路径规划入口，使用 D435i 扫二维码并生成动作矩阵；方向和高度编码与 `utils/route.py` 旧梅林版本不同，后续执行层需要明确使用哪套矩阵定义。

主要入口：

- `level_2.py`：mid360 移动到点测试，目标 `(2.0, -2.0)`，到点后不最终旋转。
- `catch.py`：抓取 weapon 流程，当前代码里 `WEAPON_ID = 1`。
- `level_move.py`：用 `ch0` 横移到点的测试入口。
- `rotation.py`：单独测试目标航向发送。

## 编程约定

- 优先复用已有封装：`module.init()`、`module.start_position_thread()`、`module.start_odometry_thread()`、`move.move_to_target()`、`move.move_backward_to_target()`、`move.rotate_to_target_yaw_segmented()`、`move.control_kfs_pose()`、`module.adjust_position()`、`module.fetch_and_store_kfs()`、`tools.compose_channels()`。
- 不要绕过 `frame_thread` 直接 `sock.sendall()`；动作只通过 `sender.set_*()` 更新发送总线。
- 不要重复写 TCP 协议、位姿计算、旋转闭环；新增动作尽量放在 `module.py` 组合已有底层动作，底层定时通道时序放在 `move.py`。
- `tools.compose_channels()` 默认开关通道是安全值 `1`，动作需要模式/边沿时显式覆盖 `ch4/ch5/ch6/ch7...`。
- KFS 吸取后再执行机械臂姿态时，必须保持 `ch4=3`，否则会产生 `3->1` 下降沿导致释放；当前通过 `move.control_kfs_pose(..., suction_ch4=3)` 保持。
- 修改通道逻辑时优先对照本文“最新通信协议”和 `R2_H-main/Core/Src/freertos.c` 的当前逻辑。
- 当前工作目录未必保留 `.git` 元数据；无论是否在 git 仓库中，工作中都要谨慎对待已有文件改动，不要回退无关内容。

## 位姿后端与坐标配置

当前项目已经把原来一套全局坐标拆成“按雷达后端配置”：

```text
position_backend.LIDAR_TYPE_ODIN   = 1
position_backend.LIDAR_TYPE_MID360 = 2
```

启动主流程时应通过 `module.init(lidar_type=...)` 选择后端。`module.init()` 内部会调用 `module.configure_position_backend(lidar_type)`，同时刷新：

```text
position_resource.position_lib
move.position_lib
position_resource.STAIR_HEIGHT_RELATION_MATRIX 兼容快照
```

不要只调用 `position_backend.set_lidar_type()` 后就直接跑动作；`position_resource.py` 和 `move.py` 都缓存了 `position_lib`，必须用 `module.configure_position_backend()` 或 `module.init()` 统一刷新。`module.py` 里有 `__getattr__` 兼容层，旧代码读取 `module.position_lib`、`module.PositionRuntime`、`module.OdomRuntime` 时会转到 `position_resource`，但新代码应优先直接理解资源状态在 `position_resource.py`。

当前坐标源：

```text
lib2/position_odin.py:
  ENTRANCE_X = 0.586
  ENTRANCE_Y = -1.776
  PRE_ENTRANCE_X = -0.500
  PRE_ENTRANCE_Y = -1.776
  WEAPON_TARGETS = odin 坐标系下的 1..6 号 weapon 点
  WEAPON_RETREAT_STOP_Y = 4.00

lib2/position_mid360.py:
  ENTRANCE_X = 2.92
  ENTRANCE_Y = 0.92
  PRE_ENTRANCE_X = 1.80
  PRE_ENTRANCE_Y = 0.957
  WEAPON_TARGETS = mid360 坐标系下的 1..6 号 weapon 点
  WEAPON_RETREAT_STOP_Y = 4.00
```

`lib2/position_resource.py` 顶部保留的 `ENTRANCE_X/Y`、`PRE_ENTRANCE_X/Y`、`STAIR_SIDE_LENGTH` 只是后端缺字段时的 fallback；主流程不应把它们当成当前真实坐标。读取坐标时优先用：

```python
module.get_entrance_x()
module.get_entrance_y()
module.get_pre_entrance_x()
module.get_pre_entrance_y()
module.get_stair_side_length()
module.get_stair_matrix()
```

这些 `module.*` 坐标函数当前只是薄包装，真实实现都在 `position_resource.py`。业务层继续用 `module.get_stair_xy()` 等兼容入口即可；资源层或工具层可以直接用 `position_resource.get_stair_xy()`。

`WEAPON_TARGETS` 和 `WEAPON_RETREAT_STOP_Y` 也从当前 `position_lib` 读取，因此切换雷达后 weapon 抓取目标点会一起切换。

## 最新通信协议

下位机 W5500 当前工作在 TCP Server：

```text
IP:   192.168.2.199
Port: 5000
Mode: TCP Server
```

上位机需要持续发送控制帧，当前推荐 70Hz 左右。下位机存在 NET 超时保护：

```text
RC_NET_TIMEOUT_MS = 240ms
NET_CTRL_TIMEOUT_MS = 150ms  # 航向 PID 使用
```

推荐使用 V2 帧，完整 32 字节：

```text
[0]     SOF1 = 0xA5
[1]     SOF2 = 0x5A
[2]     payload_len = 0x1A
[3]     frame_type = 0x01
[4:5]   seq, uint16 little-endian
[6:25]  ch[0] ~ ch[9], int16 little-endian，共 10 个通道
[26:27] yaw_now_cdeg, int16 little-endian
[28:29] target_yaw_cdeg, int16 little-endian
[30:31] crc16_ccitt, uint16 little-endian
```

CRC 规则：

```text
CRC16-CCITT
poly = 0x1021
init = 0xFFFF
范围 = payload_len + frame_type + payload，即 bytes[2:30]
```

下位机也兼容旧帧，但上位机主线应继续使用 V2：`seq + ch[10] + yaw_raw_cdeg + target_yaw_cdeg`。

### 通道映射

下位机对网口通道做了二次映射：

- `ch0~ch3`：clamp 到 `-992~992`，再做 20 死区。
- `ch4`：二段开关，只保留 `1/3`，非 `3` 会映射成 `1`。
- `ch5`：三段模式，`1/2/3`。
- `ch6`：模式子功能；KFS 姿态选择当前使用 `1/2/3/4`。
- `ch7/ch8/ch9`：二段开关通常只发 `1/3`；KFS 回归 0 态需要 `ch7=0`，这是当前新协议的例外。

安全默认通道：

```python
[0, 0, 0, 0, 1, 1, 1, 1, 1, 1]
```

重要通道：

- `ch0`：左右平移 `vy`。
- `ch1`：模式相关；升降模式控制 ID5，武器模式控制 ID7 目标位。
- `ch2`：前后移动 `vx`，正数前进，负数后退。
- `ch3`：当前主流程一般保持 0，底盘旋转主要依赖 yaw 字段闭环。
- `ch4`：气缸边沿触发，`1 -> 3` 置位，`3 -> 1` 复位。
- `ch5`：模式选择，`1=升降`，`2=方块/KFS`，`3=武器`。
- `ch6`：模式子功能；`ch5=2` 方块/KFS 模式下用于选择吸盘机械臂姿态。
- `ch7`：自动动作触发。
- `ch8/ch9`：当前未看到核心业务使用。

### ch4 气缸协议

`ch4` 是边沿触发，不是持续电平语义：

```text
1 -> 3: 置位
3 -> 1: 复位
```

不同 `ch5` 模式映射不同气缸：

```text
ch5 = 1 升降模式: PF1
ch5 = 2 方块/KFS模式: PF3
ch5 = 3 武器模式: PF0
```

因此方块/KFS 夹紧用：

```text
ch5=2, ch4: 1 -> 3
```

释放用：

```text
ch5=2, ch4: 3 -> 1
```

### KFS 方块/吸盘机械臂协议

KFS 方块吸取拆成两块动作：

1. 吸盘机械臂姿态：先进入 KFS 模式，再用 `ch6` 选择姿态。
2. 吸盘吸取/释放：仍使用 `ch4` 边沿控制。

姿态选择：

```text
ch5 = 2  # 方块/KFS 模式
ch6 = 1  # 高位方块抓取姿态
ch6 = 2  # 低位方块抓取姿态
ch6 = 3  # 过渡态
ch6 = 4  # 存储方块姿态
```

姿态触发条件：

```text
ch5 = 2  # 方块/KFS 模式
ch6 = 1/2/3/4  # 姿态选择
ch7 = 3  # 触发
```

触发逻辑是锁存触发：

```text
当 ch5=2 且 ch7 从非 3 进入 3 时，按当前 ch6 选择的姿态启动一次自动姿态任务
如果要再次触发，必须先把 ch7 发回 1，再发 3
回归 0 态使用 ch7=0
```

当前 README 旧记录中已知的高位抓取姿态参数：

```text
ID6 目标 total_ecd: -674021
脉塔目标: 8000 cdeg = 80.00 deg
脉塔速度: 60
达妙 CAN ID: 0x01
达妙目标位置: 0.0
达妙速度: 5.0
任务超时: 8000ms
```

推荐上位机触发序列：

```text
1. 进入方块模式并选择抓取姿态:
   ch = [0, 0, 0, 0, 1, 2, pose_id, 1, 1, 1]
   pose_id = 1(高位) 或 2(低位)

2. 触发所选抓取姿态:
   ch5=2, ch6=pose_id, ch7=3
   保持 0.2s~0.5s

3. 释放触发锁存，等待抓取姿态完成:
   ch5=2, ch6=pose_id, ch7=1

4. 切到过渡态并触发，等待过渡完成:
   ch5=2, ch6=3, ch7: 1 -> 3 -> 1

5. 切到存储方块姿态并触发，等待存储完成:
   ch5=2, ch6=4, ch7: 1 -> 3 -> 1

6. 如需回归 0 态:
   ch5=2, ch7=0
```

上位机当前封装默认时序：

```text
move.control_kfs_pose(pose_id=1/2/3/4):
  1. ch5=2, ch6=pose_id, ch7=1, ch0/ch2/ch3=0, target_yaw=0
  2. 等待 0.5s
  3. ch7=3 触发姿态
  4. 保持 2.0s
  5. ch6=1, ch7=1 回常态

move.control_kfs_pose(pose_id=0):
  1. ch5=2, ch7=0
  2. 阻塞等待 2.0s
```

注意：`move.control_kfs_pose()` 新增了 `suction_ch4` 参数。默认 `ch4=1`；吸取后执行 `pose_id=3/4` 时必须传 `suction_ch4=3`，避免姿态帧释放吸盘。

如果还需要方块气缸夹紧：

```text
ch5=2, ch4=1 保持
ch5=2, ch4=3 触发夹紧 PF3
```

释放方块：

```text
ch5=2, ch4=1
```

当前没有应用层 ACK，也没有上位机可读状态回传；执行自动姿态后只能按时间等待或现场观察。后续如需要严谨闭环，应增加下位机状态回传。

### 航向控制字段

`target_yaw_cdeg` 单位是 `0.01°`：

```text
90 deg  -> 9000
-90 deg -> -9000
```

特殊规则：

```text
target_yaw_cdeg = 0 表示关闭航向 PID
```

如果真实目标是 `0°`，上位机应使用 `move.encode_target_yaw_i16(0.0)`，它会编码成 `1`，避免下位机当作关闭 PID。

## 位姿后端

`module.init(lidar_type=...)` 负责选择后端：

```python
module.init(lidar_type=1)  # odin
module.init(lidar_type=2)  # mid360
```

mid360 当前逻辑：

- `/lio/robo/odom`：机器人中心在 `world` 下的位姿，移动主流程直接使用它作为 robot pose。
- `/lio/odom`：IMU/lidar 系统位姿，用于计算 lidar pose，再通过外参计算 weapon pose。
- `PositionRuntime.get_weapon_pose()` 是实时调用后端计算，不是后台缓存。

后续注意：`position_odin.py` 和 `position_mid360.py` 应尽量保持同名接口/语义，避免修改 `module.py`、`move.py` 调用层。

## 初始化流程

`module.init()` 当前是阻塞式初始化：

1. 设置位姿后端。
2. TCP 连接下位机。
3. 启动 `/odin1/flag1` 重定位监听。
4. 默认等待重定位成功。
5. 默认销毁重定位监听。
6. 创建并启动 `frame_thread`。
7. 等待首帧 TCP `sendall()` 成功后返回。

注意：首帧发送成功只能证明 TCP 层发送成功，不能证明下位机业务层已经执行。

## 移动逻辑

`module.move_to_des(...)` 是主组合移动：

- 先按当前参考点算初始目标方向。
- 分段旋转到初始目标方向。
- 调用 `move.move_to_target(...)` 阻塞移动到目标点。
- `reference="robot"`：机器人中心到点。
- `reference="weapon"`：weapon/夹爪到点，底盘 yaw 仍用 robot yaw。
- `target_deg=None`：到点后发送 `des_yaw_i16=0`，停止旋转控制，不执行最终旋转。
- `target_deg` 是角度值：到点后再分段旋转到该最终角。

`move.move_to_target(...)` 是连续闭环移动，不是分段式位移。每一轮都会按当前参考点重新计算目标航向角，然后根据距离换速度档：

```text
远距离: cruise_forward_cmd
近点区: near_forward_cmd
临点区: fine_forward_cmd
```

默认近点/临点参数在 `move.py`：

```text
near_target_distance = 0.5m
near_forward_cmd = 150
fine_target_distance = 0.4m
fine_forward_cmd = 75
stop_distance = 0.02m
```

如果要提高 weapon/KFS 夹取精度，不建议改全局默认值；应在对应组合动作调用 `move_to_target()` 时传专用保守参数，例如更早进入近点区、降低 `fine_forward_cmd`、收紧 `move_gate_deg` 和到点速度阈值。

## 台阶矩阵与编号工具

`position_resource.get_stair_matrix()` 会按当前 position 后端的入口坐标动态生成梅林台阶矩阵；`module.get_stair_matrix()` 是兼容包装。也就是说 odin 和 mid360 会得到不同的 `x/y`，但台阶编号和高低关系结构一致。矩阵当前多了梅林入口前点，第一列不是连续 `1..14`：

```text
[-1, 1, 0, 0, PRE_ENTRANCE_X, PRE_ENTRANCE_Y]  # 入口前点
[1..13, ...]                   # 正常梅林格
[15, 0, 0, 2, ..., ...]        # 原右下出口侧编号现为 15
```

矩阵行格式：

```text
[台阶编号, 前方关系, +90deg方向关系, -90deg方向关系, x, y]
```

关系值：

```text
0 = 该方向不相邻/无衔接
1 = 该方向台阶比当前台阶高
2 = 该方向台阶比当前台阶低
```

坐标换算逻辑：

```text
stair_id=-1: 当前后端的 PRE_ENTRANCE_X/Y
stair_id=1..13/15: 当前后端的 ENTRANCE_X/Y + STAIR_SIDE_LENGTH 网格偏移
```

新增工具函数：

- `tools.stair_id_to_matrix_index(stair_id, stair_matrix=None, exit_on_error=True)`：按第一列编号遍历查矩阵 0-based 行号；未传 `stair_matrix` 时默认读取 `position_resource.get_stair_matrix()`，例如 `-1 -> 0`、`1 -> 1`、`15 -> 14`。
- `tools.stair_id_to_direction(from_id, to_id, stair_matrix=None, exit_on_error=True)`：按矩阵真实 `x/y` 判断 `to_id` 在 `from_id` 的方向，返回 `1/2/3/4`，不相邻返回 `0` 或退出。
- `position_resource.get_stair_matrix_row(stair_id)` / `position_resource.get_stair_xy(stair_id)`：按编号查矩阵行和中心坐标；`module` 中保留同名兼容包装。
- `position_resource.get_stair_height_relation(stair_id, direction)`：按编号和方向取高低关系；`direction=1/2/3` 直接读矩阵列，`direction=4` 通过反查后方相邻格推导；`module` 中保留同名兼容包装。

不要再写 `STAIR_HEIGHT_RELATION_MATRIX[stair_id - 1]`，也不要在业务代码里直接依赖 fallback `ENTRANCE_X/Y`。新增 `-1` 和 `15` 后，下标会错；切换雷达后，静态全局坐标也可能不是当前后端坐标。主流程应调用 `module.get_stair_matrix()` / `module.get_stair_xy()` 或 `position_resource` 对应函数读取当前后端的实时矩阵。

`module.adjust_position(...)` 当前接口已经从输入 `current_x/current_y` 改为输入 `stair_id`：

```python
module.adjust_position(
    sender,
    position_runtime,
    odom_runtime,
    move_type=1,      # 1=正向 move_to_des, 2=反向 move_backward_to_des
    direction=1,      # 1=x+, 2=y+, 3=y-, 4=x-
    stair_id=2,
    height_relation=2, # 1=微调方向台阶较高，2=微调方向台阶较低
    adjust_distance=...
)
```

函数内部会按 `stair_id` 查矩阵坐标。`height_relation=2` 时执行原来的坐标微调逻辑，按方向和 `adjust_distance` 计算微调目标；`height_relation=1` 时先按 `direction` 旋转到目标航向，再用 `ch2=200` 前进 `2s`，完成微调。`height_relation` 不是 `1/2` 会报错并终止程序。

## KFS 吸取并存储流程

当前已在 `module.fetch_and_store_kfs(sender, position_runtime, odom_runtime, stair_id, direction, ...)` 封装 KFS 方块吸取并存储。

输入：

```text
stair_id   当前所在台阶编号
direction  吸取动作方向，1=前方, 2=+90deg/左, 3=-90deg/右, 4=后方
```

当前流程：

1. 调用 `module.get_stair_height_relation(stair_id, direction)` 读取目标方向高低关系。
2. 调用 `module.adjust_position(..., move_type=1, stair_id=stair_id, direction=direction, height_relation=height_relation)` 微调到吸取位置。
3. 高低关系映射抓取姿态：
   ```text
   height_relation=1 -> pose_id=1 高位方块抓取姿态
   height_relation=2 -> pose_id=2 低位方块抓取姿态
   height_relation=0 -> 报错退出
   ```
4. 调用 `move.control_kfs_pose(..., pose_id=1/2, suction_ch4=1)` 调整到抓取姿态。
5. 调用 `module.set_kfs_suction(..., suction_on=True)`，发送 `ch4:1->3` 吸取。
6. 调用 `module.wait_with_kfs_suction(..., duration_sec=3.0)`，保持吸取 3s。
7. 调用 `move.control_kfs_pose(..., pose_id=3, suction_ch4=3)` 执行过渡态，并保持吸取。
8. 调用 `move.control_kfs_pose(..., pose_id=4, suction_ch4=3)` 执行存储姿态，并保持吸取。
9. 调用 `module.set_kfs_suction(..., suction_on=False)`，发送 `ch4:3->1` 释放。
10. 调用 `move.control_kfs_pose(..., pose_id=0, suction_ch4=1)` 回 0 态。
11. 调用 `module.move_backward_to_des(..., x/y=当前 stair_id 中心坐标, target_deg=0.0)` 倒退回当前台阶中心。

相关底层函数：

- `move.control_kfs_pose(sender, pose_id, ..., suction_ch4=...)`
- `module.set_kfs_suction(sender, suction_on=True/False, ...)`
- `module.wait_with_kfs_suction(sender, duration_sec, ...)`

当前 `fetch_and_store_kfs()` 已做过语法检查和 monkey-patch dry run，但还没有现场实机验证。

## Weapon 抓取流程

`module.fetch_weapon(sender, position_runtime, odom_runtime, weapon_id, ...)` 已封装 weapon 抓取。

目标点从当前 position 后端读取，坐标已经分雷达维护：

- `position_odin.WEAPON_TARGETS`
- `position_mid360.WEAPON_TARGETS`

当前 odin 默认值：

- 1: `(-2.638, 1.40)`
- 2: `(-2.441, 1.40)`
- 3: `(-2.243, 1.40)`
- 4: `(-2.036, 1.40)`
- 5: `(-1.834, 1.40)`
- 6: `(-1.641, 1.40)`

当前 mid360 默认值：

- 1: `(-0.32, 4.12)`
- 2: `(-0.12, 4.12)`
- 3: `(0.074, 4.12)`
- 4: `(0.275, 4.12)`
- 5: `(0.483, 4.12)`
- 6: `(0.681, 4.12)`

当前流程：

1. 以 `reference="weapon"` 移动 weapon/夹爪到目标点。
2. 进入武器模式并夹紧抬起：
   ```text
   ch5=3, ch4=1, ch1=0     保持 0.3s
   ch5=3, ch4=1, ch1=0     保持 0.3s
   ch5=3, ch4=3, ch1=0     保持 1.0s
   ch5=3, ch4=3, ch1=100   保持 1.0s
   ```
3. 阻塞旋转到 `90°`。
4. 保持 `ch5=3,ch4=3,ch1=100`，用 `ch2=-100` 后退。
5. 当前已改为当 `weapon_pose["y"] < 当前后端 WEAPON_RETREAT_STOP_Y` 时停止后退，不再用机器人中心 `robot_pose["y"]`。
6. 再阻塞旋转到 `-90°`。
7. 完成后仍保持 `ch5=3,ch4=3,ch1=100`，方便后续用 `ch4: 3 -> 1` 放开。

注意：`catch.py` 当前在 `fetch_weapon()` 后还会再调用一次 `move.rotate_to_target_yaw_segmented(..., target_yaw_deg=-90.0)`，而 `fetch_weapon()` 内部已经包含最终旋转到 `-90°`，这可能是重复逻辑，后续可按现场效果删除。

## 动作矩阵与 KFS 路线

当前主线使用 `utils/race.py` 负责新版比赛规则路径规划，`utils/utils.py` 已更新为无 GUI 生成入口：

```python
from utils.utils import build_action_matrix_from_qr

action_matrix, path, kfs = build_action_matrix_from_qr("000020020200")
```

动作矩阵形状是 `n*5`，每行：

```text
[from_pos, to_pos, move_dir, height_action, grab_action]
```

列含义：

- `from_pos`：动作起点格编号。
- `to_pos`：动作目标格编号。
- `move_dir`：`0=原地`，`1=前方`，`2=+90度/左`，`3=-90度/右`，`4=后方`。
- `height_action`：`0=不用上下楼梯`，`1=需要上下楼梯`。上/下由 `module.get_stair_height_relation(from_pos, move_dir)` 在执行层按真实台阶矩阵判断。
- `grab_action`：`0=不抓取`，`1=抓取`。当前 `utils/utils.py`、`utils/route1.py` 与 `utils/route.py` 都只输出 `0/1`。

当前已在 `lib2/module.py` 中继续封装动作矩阵解释层。单行执行入口：

```python
module.execute_action_row(
    sender,
    position_runtime,
    odom_runtime,
    action_row,
    final_direction=1,
)
```

当前 `execute_action_row()` 已实现：

- 接收一行 `action_row`，格式仍为 `[from_pos, to_pos, move_dir, height_action, grab_action]`。
- 新增 `final_direction` 输入，默认 `1`，必须是 `1/2/3/4`，表示本行任务完成后的最终朝向。
- 检查行长度必须为 `5`，否则打印错误并 `sys.exit(1)`。
- 行长度正确后，先读取 `from_pos/to_pos`，通过 `tools.stair_id_to_direction(from_pos, to_pos)` 推导相邻方向，再调用 `module.get_stair_height_relation(from_pos, inferred_direction)` 获取高低关系并存入结果；`from_pos == to_pos` 时高低关系记为 `0`。
- 检查 `move_dir` 必须与 `from_pos/to_pos` 的坐标推导方向一致；不一致时直接报错退出。
- 检查 `move_dir` 必须为 `0/1/2/3/4`，否则打印错误并 `sys.exit(1)`。
- 当前地图相邻台阶不存在等高普通平移，因此新增防御检查：如果 `move_dir != 0 and height_action == 0 and grab_action == 0`，视为动作矩阵错误并 `sys.exit(1)`。
- `move_dir != 0` 时才获取 `from_x/from_y/to_x/to_y`，来源是 `module.get_stair_xy(from_pos/to_pos)`；原地分支暂不预取坐标。
- `grab_action == 1` 时调用 `module.fetch_and_store_kfs(...)`，输入 `stair_id=from_pos`、`direction=move_dir`、`final_target_yaw_deg=tools.direction_int_to_yaw_deg(final_direction)`；执行完立即返回。
- `height_action != 0` 时调用 `module.execute_stair_transition(...)`，把 `from_x/from_y/to_x/to_y/height_relation/move_dir/final_direction` 传入；执行完立即返回。
- `move_dir == 0` 时仍进入原地占位分支，尚未接入真实动作。

新增 `module.execute_stair_transition(...)` 用于统一执行上下楼：

```python
module.execute_stair_transition(
    sender,
    position_runtime,
    odom_runtime,
    from_x,
    from_y,
    to_x,
    to_y,
    height_relation,
    task_direction,
    final_direction,
)
```

逻辑：

- `height_relation` 必须是 `1/2`，否则报错并 `sys.exit(1)`。
- `height_relation == 1`：目标格比当前格高，调用 `module.climb(..., direction1=task_direction, direction2=final_direction, x=to_x, y=to_y)`。
- `height_relation == 2`：目标格比当前格低，调用 `module.descend(..., direction1=task_direction, direction2=final_direction, current_x=from_x, current_y=from_y, des_x=to_x, des_y=to_y)`。

`module.fetch_and_store_kfs(...)` 也已新增 `final_target_yaw_deg=0.0` 输入。吸取并存储完成后，最后倒退回当前 `stair_id` 中心点时，会把 `final_target_yaw_deg` 传给 `move_backward_to_des(..., target_deg=final_target_yaw_deg)`，因此执行矩阵行时可以通过 `final_direction` 控制夹取后的最终朝向。

当前真实动作已接入 `grab_action == 1` 和 `height_action != 0` 两类方向动作。按当前真实场地和规划规则，不应接入等高普通移动分支；如果后续地图允许相邻等高台阶，再重新讨论并补普通移动封装。

后续再封装整矩阵循环执行函数，例如：

```python
def execute_action_matrix(sender, position_runtime, odom_runtime, action_matrix, ...):
    ...
```

推荐分层：

```text
utils/process.py           只负责 D435i/二维码通用处理
utils/utils.py             只负责二维码/KFS -> action_matrix
主任务脚本                 启动规划线程或直接生成 action_matrix，再调用 module 执行
lib2/position_resource.py  只负责 position/odom 资源线程、当前后端资源、台阶矩阵/坐标查询
lib2/module.py             只负责解释 action_matrix 并调用已有组合动作；资源入口保留兼容包装
lib2/move.py               只负责底层阻塞动作
```

一次性生成动作矩阵时，若要线程传递结果，推荐 `queue.Queue(maxsize=1)`，不要裸全局变量。生成线程 `put(action_matrix)`，主任务 `get()` 阻塞等待后执行。

### 动作矩阵执行层当前工作进程

当前窗口的主线是继续封装 `lib2/module.py` 的动作矩阵执行层，并保持 `utils/race.py` 的规划规则稳定。当前状态：

- `utils/race.py`：挑战赛路径规划，维护 2 个 R1、2 个 R2、fake KFS 等约束；方向语义与 `lib2/module.py` 的梅林矩阵一致。
- `utils/meilin.py`：对抗赛路径规划，保留更多 KFS 的旧规则语义；后续会与 `combat_lib.py` 一起整理成对抗赛库。
- `utils/route1.py`：挑战赛 D435i/GUI 入口，基于 `race.py`；后续会逐步封装为库，不再作为最终主运行入口。
- `utils/route.py`：对抗赛 D435i/GUI 入口，基于 `meilin.py`；后续会逐步封装为库，不再作为最终主运行入口。
- `utils/utils.py`：无 GUI 工具入口，当前通过 `task_type` 区分 `challenge/race/route1/挑战赛` 和 `combat/meilin/route/对抗赛`，负责 `qr/kfs/path -> action_matrix`；主任务优先调用这里，不要把执行动作写进 utils。
- `lib2/tools.py`：方向码、台阶编号到矩阵下标、台阶编号到相邻方向等通用工具。
- `lib2/position_resource.py`：资源层，负责当前 `position_lib`、后端相关台阶坐标/矩阵、`PositionRuntime`、`OdomRuntime`、位置线程和 odom 线程。`tools.py` 默认读取这里的台阶矩阵，避免 `lib2` 内部为了资源反向依赖 `module.py`。
- `lib2/module.py`：动作解释和组合动作封装层。`execute_action_row()` 应只解释矩阵并调用已有组合动作；不要在这里重写 TCP 协议、底层通道细节或资源线程实现。
- `lib2/move.py`：底层阻塞动作和通道时序，例如旋转、移动、上下楼底层触发、KFS 姿态触发。

### 资源层拆分状态

当前已把 position/odom 资源相关代码从 `lib2/module.py` 拆到 `lib2/position_resource.py`：

```text
position_resource.py:
  position_lib
  configure_position_backend()
  get_stair_matrix() / get_stair_xy() / get_stair_height_relation()
  PositionRuntime / start_position_thread()
  OdomRuntime / start_odometry_thread()
```

`module.py` 保留兼容入口：

```python
module.start_position_thread(...)
module.start_odometry_thread(...)
module.get_stair_matrix()
module.get_stair_xy(stair_id)
module.get_stair_height_relation(stair_id, direction)
```

`module.configure_position_backend(lidar_type)` 当前会先刷新 `position_resource.position_lib`，再刷新 `move.position_lib`。不要直接调用 `position_backend.set_lidar_type()` 后启动位置线程或动作；这样会绕过资源同步。

`module.py` 还提供 `__getattr__` 兼容层，旧代码读取 `module.position_lib`、`module.STAIR_HEIGHT_RELATION_MATRIX`、`module.PositionRuntime`、`module.OdomRuntime` 时会转到 `position_resource`。新代码不应在 `module.py` 里新增第二份资源全局变量。

当前方向动作分支已开始复用：

```python
module.fetch_and_store_kfs(
    sender,
    position_runtime,
    odom_runtime,
    stair_id,
    direction,
    final_target_yaw_deg=0.0,
)
module.execute_stair_transition(...)
module.climb(sender, position_runtime, odom_runtime, direction1, direction2, x, y)
module.descend(sender, position_runtime, odom_runtime, direction1, direction2, current_x, current_y, des_x, des_y)
```

统一输入思路：

```text
from_pos/to_pos     来自动作矩阵
from_x/from_y       move_dir != 0 分支内按需读取
to_x/to_y           move_dir != 0 分支内按需读取
move_dir            当前任务方向，仍传 int，不要提前转角度
final_dir           最终方向，也传 int；上下楼函数内部会转角度
height_action       1=需要上下楼，0=不上下楼；上/下由 height_relation 判断
grab_action         1=抓取，0=不抓取；当前矩阵生成层只输出 0/1
```

注意：`module.climb()`、`module.descend()` 内部已经完成方向 int 到角度转换；`module.fetch_and_store_kfs()` 的夹取方向仍传 int，但最终朝向现在传 `final_target_yaw_deg`，当前 `execute_action_row()` 会由 `final_direction` 统一转换。只有直接调用 `move_to_des()` 时才需要传 `target_deg`，但当前规划保证不出现等高普通移动，暂不把普通移动作为主分支。

当前验证状态：

- 已运行 `python3 -m py_compile lib2/module.py`。
- 已运行 `python3 -m py_compile lib2/position_resource.py lib2/module.py lib2/tools.py lib2/move.py`。
- 已运行 `python3 -m py_compile utils/utils.py`。
- 已做资源层 smoke test：`module.get_stair_xy(2)` 与 `position_resource.get_stair_xy(2)` 一致，且 `module.position_lib is position_resource.position_lib` 为 `True`。
- 已用 `build_action_matrix_from_qr("000020020200")` 做过一次无 GUI 生成检查，确认 `height_action` 和 `grab_action` 都只输出 `0/1`。
- `execute_action_row()` 的真实硬件动作分支尚未实机验证。

### 协作和实现偏好

- 用户希望尽量复用已有封装，尤其是 `module.py` 组合动作、`move.py` 底层动作、`tools.py` 方向/矩阵工具；不要重复实现已有逻辑。
- 发现规划语义、动作矩阵、真实场地假设之间可能矛盾时，要明确指出并质问确认，不要为了继续写代码而默认绕过。
- 对硬件动作相关逻辑要保守，宁可加防御检查并退出，也不要静默跳过可能危险的动作。
- 讨论方向时，外部接口优先使用方向 int：`1=前方/x+`、`2=+90/左/y+`、`3=-90/右/y-`、`4=后方/x-`；角度转换应尽量留在已有封装内部。
- README 是下一窗口的交接入口；当前进度、关键假设、未完成分支和实机风险要及时写进 README。

### README 更新要求

当用户说“更新 README”或需要交接下一个窗口时，更新内容应至少包含：

- 当前已经完成的修改：文件路径、函数名、行为变化。
- 当前未完成的工作：下一步应从哪个函数/分支继续。
- 关键项目结构和文件逻辑关系：规划层、工具层、组合动作层、底层动作层分别在哪里。
- 关键方法职责和输入输出：尤其是动作矩阵字段、方向编码、上下楼/KFS 的输入。
- 当前用户确认过的假设：例如真实场地不存在相邻等高普通移动。
- 需要保留的协作偏好：尽量复用、发现逻辑矛盾要质问、硬件动作加防御检查。
- 已知风险和验证状态：哪些只做了语法/干跑，哪些还未实机验证。

## 已知注意事项

- `utils/route.py` 运行会打开 OpenCV 摄像头窗口，并在规划成功后调用 `meilin.visualize()` 弹 matplotlib 图；主任务只要生成矩阵应使用 `utils/utils.py`，避免 GUI。
- `utils/meilin.py` 已把 matplotlib 改为 `visualize()` 内按需导入，避免纯规划时被本机 numpy/matplotlib ABI 问题影响。
- `level_2.py` 到点后不旋转是通过 `TARGET_FINAL_YAW_DEG = None` 实现。
- 旧 `level_1.py` 和 `lib/` 仍可能依赖旧 odin 逻辑，不是当前主流程。
- 如果 pose 或 odometry 出现 `NaN/inf`，`move.py` 已加有限值保护，会停车跳过该轮，避免把非法 yaw 发给下位机。
- 当前 TCP 连接和首帧发送都不能证明下位机业务层已经执行，仍需现场观察或后续加 ACK。
- 移动到点能较准，但最终旋转可能带来位置漂移；现在 `level_2.py` 默认到点后不最终旋转。
- KFS 新协议的关键点是 `ch5=2,ch6=1/2,ch7:1->3` 触发高/低位抓取，完成后依次触发 `ch6=3` 过渡态、`ch6=4` 存储姿态；`ch7=0` 回归 0 态；`ch5=2,ch4:1->3` 吸取，`ch5=2,ch4:3->1` 释放。
