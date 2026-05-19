注意：主要阅读lib2内文件以及utils内文件，其他文件，除非用户指定文件，即便在本文件内有提及或讲解，也不需要阅读或纳入思考。


# ws_r2 当前工程进度

本目录是 R2_H 上位机控制工程。当前主线使用 `lib2/`，旧 `lib/` 基本只作参考。当前通信协议已总结在本文中，后续窗口优先看本文即可。

## 当前主线文件

- `lib2/tools.py`：TCP 连接、V2 控制帧打包、后台持续发送 `frame_thread`、重定位监听、统一清理；提供 `AUTO_TRIGGER_LOCK`，用于保护 KFS/上下楼等模式触发上升沿。
- `lib2/position_backend.py`：位姿后端选择，`1=odin`，`2=mid360`；当前坐标配置跟随后端切换。
- `lib2/position_odin.py`：odin 后端，基于 `/tf`；维护 odin 对应的梅林入口、入口前点、weapon 目标点和外参。
- `lib2/position_mid360.py`：mid360 后端，基于 `/lio/robo/odom` 和 `/lio/odom`；维护 mid360 对应的梅林入口、入口前点、weapon 目标点和外参。
- `lib2/position_resource.py`：位置/里程计资源层；维护当前 `position_lib`、台阶矩阵/坐标查询、`PositionRuntime`、`OdomRuntime`、位置线程和 odom 线程启动。
- `lib2/module.py`：初始化和组合动作封装；保留 `module.start_position_thread()` / `module.start_odometry_thread()` 等兼容包装，但真实资源实现已在 `position_resource.py`。
- `lib2/move.py`：阻塞旋转、移动到点、倒退到点、上下台阶、KFS 吸盘机械臂姿态等底层动作。
- `utils/meilin.py`：梅林 12 格路径规划，0-1 BFS，返回路径。
- `utils/route.py`：D435i 扫二维码、调用规划、打印动作矩阵、可视化，有 OpenCV/matplotlib GUI。
- `utils/utils.py`：无 GUI 的纯工具，已实现 `build_action_matrix_from_qr()` / `build_action_matrix_from_kfs()`，用于生成 `n*5` 动作矩阵。
- `utils/race.py`：新版挑战赛路径规划核心，维护地图、规则、`plan_path()`、`validate_path()`、可视化等。
- `utils/challenge_lib.py`：挑战赛对外库入口，封装二维码解析、`race.py` 规划、动作矩阵生成，以及后台 QR 扫描线程 `ChallengeQRScanner`。
- `utils/route1.py`：挑战赛扫码调试脚本；当前只调用 `challenge_lib.py`，使用 D435i 扫二维码并生成动作矩阵。方向和高度编码与 `utils/route.py` 旧梅林版本不同，后续执行层需要明确使用哪套矩阵定义。

主要入口：

- `level_2.py`：mid360 移动到点测试，目标 `(2.0, -2.0)`，到点后不最终旋转。
- `catch.py`：当前是 odin `fetch_weapon(weapon_id=1)` 调试脚本；会等待 robot/weapon pose 和 odom 后执行完整 weapon 抓取流程，测试速度 `MOVE_SPEED=300`。
- `move_test.py`：实验性 vector PID 到点测试，固定目标航向，用距离 PID 算速度标量，再按目标向量实时分配 `ch0/ch2`；当前按 waypoint 顺序跑 `(-0.8, -4.8) -> (-1.0, 0.0) -> (-2.4, -2.4)`。
- `move_t.py`：主线 `module.move_to_des()` 单点移动测试，目标 `(-0.8, -4.8)`，持续打印通道、yaw、当前位置、目标距离和 odom。
- `rotation_move.py`：带输入保持点的旋转位置保持测试，保持点 `(-0.8, -4.8)`，目标 yaw `180deg`，持续打印通道、yaw、位置误差和 odom。
- `ori_rot.py`：原地循环旋转测试，循环 `0.01 -> 90 -> 180 -> -90 -> 0.01`，使用当前坐标作为保持点并持续打印通道状态。
- `fwd.py`：直连控制器简单前进测试，按新 `frame_thread + move.set_motion_channels()` 接口持续发送 `ch2`。
- `up_down_test.py`：odin 上下台阶测试；当前最后一段已从下楼梯替换为 `1 -> 2` 的 KFS 抓取。
- `suck.py`：直连控制器 KFS 高位吸取/存储调试脚本，不等待定位，用于单独观察 KFS 通道时序。
- `rigion_1.py`：区域 1 简化矩阵测试脚本；初始化后移动到 `-1`，再直接执行 `module.CHALLENGE_ACTION_MATRIX`，不执行 weapon 抓取。
- `rigion_2.py`：区域 1 完整矩阵测试脚本；初始化后先 `fetch_weapon(weapon_id=1)`，执行 `move.reset_weapon_after_fetch()`，再移动到 `-1` 并执行 `module.CHALLENGE_ACTION_MATRIX`。
- `descend.py`：上下楼/KFS 小闭环测试脚本；移动到 `-1`，执行 `-1 -> 2` 上楼，随后执行 `2 -> 5` KFS 抓取，再执行 `2 -> -1` 下楼。
- `level_move.py`：用 `ch0` 横移到点的测试入口。
- `rotation.py`：单独测试目标航向发送。

## 编程约定

- 优先复用已有封装：`module.init()`、`module.start_position_thread()`、`module.start_odometry_thread()`、`move.move_to_target()`、`move.move_backward_to_target()`、`move.rotate_to_target_yaw_segmented()`、`move.control_kfs_pose()`、`module.adjust_position()`、`module.fetch_and_store_kfs()`、`tools.compose_channels()`。
- 不要绕过 `frame_thread` 直接 `sock.sendall()`；动作只通过 `sender.set_*()` 更新发送总线。
- 不要重复写 TCP 协议、位姿计算、旋转闭环；新增动作尽量放在 `module.py` 组合已有底层动作，底层定时通道时序放在 `move.py`。
- 除非明确要求，不要修改 `test/` 下的测试用例文件或旧测试脚本；这些文件可能保留旧接口/旧协议用于对照，主线修改优先落在 `lib2/`、当前调试入口和 README。
- 当前通道构建逻辑已改为 `frame_thread` 内部维护 `ch0~ch9/yaw_i16/des_yaw_i16`，后台发送线程每帧读取内部变量并构建完整 frame；主线动作不再外部构造完整 `channels` 后整体覆盖。
- 主线应优先使用 `sender.set_channel_values(...)` 或 `move.set_channel_values(...)` 只修改本动作负责的通道；移动/旋转类可用 `move.set_motion_channels(...)` 修改 `ch0/ch2/ch3/des_yaw_i16`。不要恢复旧的 `set_channels_and_des_yaw_i16(...)` 整帧覆盖方式。
- 现在没有自动安全复位；动作结束后哪些通道需要复位必须由动作边界显式管理。当前任务就是通道构建逻辑修改后的动作逻辑勘误，重点检查各组合动作完成后的最终通道状态。
- KFS 吸取后再执行机械臂姿态时，必须保持 `ch4=3`，否则会产生 `3->1` 下降沿导致释放；当前通过 `move.control_kfs_pose(..., suction_ch4=3)` 保持。
- KFS 姿态、KFS 吸盘释放、KFS 回 0、上楼和下楼的“模式设置 + 触发边沿”必须使用 `tools.AUTO_TRIGGER_LOCK` 保护。锁只覆盖 arm/fire/reset 触发临界区；触发后等待自动任务完成的阶段不持锁。
- 修改通道逻辑时优先对照本文“最新通信协议”。
- 当前工作目录未必保留 `.git` 元数据；无论是否在 git 仓库中，工作中都要谨慎对待已有文件改动，不要回退无关内容。
- 要注意用户提出需求的执行逻辑，如果有问题请务必指出，待用户确认无误或者需要修正之后再作修改。
- 注意：主要阅读lib2内文件以及utils内文件，其他文件，除非用户指定该文件，即便在本文件内有提及或讲解，也不需要阅读或纳入思考。

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

控制器当前工作在 TCP Server：

```text
IP:   192.168.2.199
Port: 5000
Mode: TCP Server
```

上位机需要持续发送控制帧，当前推荐 70Hz 左右。控制器存在网络控制超时保护：

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

控制器也兼容旧帧，但上位机主线应继续使用 V2：`seq + ch[10] + yaw_raw_cdeg + target_yaw_cdeg`。

### 通道映射

控制器对网口通道做了二次映射：

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

### 上位机通道状态管理

当前上位机发送线程 `tools.frame_thread` 内部维护：

```text
ch0..ch9
yaw_i16
des_yaw_i16
```

后台 `_run()` 每一帧调用 `_snapshot()`，从内部变量生成 `channels`，再由 `build_frame()` 打包发送。动作层不应再构造完整 `channels` 列表整体覆盖发送状态。

当前推荐接口：

```python
sender.set_channel_values({4: 3, 5: 2})
move.set_channel_values(sender, channel_values={4: 3, 5: 2})
move.set_motion_channels(sender, lateral_cmd=0, forward_cmd=300, rotation_cmd=0, des_yaw_i16=...)
```

语义：

- `set_channel_values(...)`：只修改传入的通道，不复位其他通道；可选同步修改 `yaw_i16/des_yaw_i16`。
- `set_motion_channels(...)`：只修改 `ch0/ch2/ch3/des_yaw_i16`；不会复位其他通道，也不再叠加写非移动通道。
- `sender.set_safe_stop(...)`：会恢复安全默认通道，但主线不会自动调用；只有在明确需要全局复位时再用。

因此，当前每个动作必须自己管理结束状态。例如 KFS、weapon、升降模式完成后，如果后续不继续使用对应模式，需要另写复位动作。

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
   ch5=2, ch6=0, ch7: 1 -> 3 -> 1
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

当前没有应用层 ACK，也没有上位机可读状态回传；执行自动姿态后只能按时间等待或现场观察。后续如需要严谨闭环，应增加控制器状态回传。

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

如果真实目标是 `0°`，上位机应使用 `move.encode_target_yaw_i16(0.0)`，它会编码成 `1`，避免控制器当作关闭 PID。

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

`position_resource.extract_odometry_params()` 读取 odom 速度时会保留原始 twist：

```text
raw_linear_x/raw_linear_y/raw_linear_z
raw_angular_x/raw_angular_y/raw_angular_z
```

同时按 odom pose 的 yaw 做一次平面旋转，`linear_x/linear_y` 和 `angular_x/angular_y` 返回变换后的速度，`velocity_transform = "pose_yaw_child_to_parent"`。`angular_z` 保持原始 `twist.angular.z`，因为平面 yaw 旋转不改变 z 轴角速度。

`position_resource.predict_position_xy()` / `predict_pose_xy()` 提供资源层位置预测：使用地图坐标系下的 `linear_x/linear_y` 和位姿年龄计算 `pre_x/pre_y`，默认 `prediction_dt = clamp(position_age_sec + 1/70, 0, 0.10)`。`move.move_to_target()` 当前到点判定仍使用真实 `current_x/current_y`，但 ch0/ch2 分配使用 `pre_x/pre_y`，用于补偿位姿与控制发送延迟。

## 初始化流程

`module.init()` 当前是阻塞式初始化：

1. 设置位姿后端。
2. TCP 连接控制器。
3. 启动 `/odin1/flag1` 重定位监听。
4. 默认等待重定位成功。
5. 默认销毁重定位监听。
6. 创建并启动 `frame_thread`。
7. 等待首帧 TCP `sendall()` 成功后返回。

注意：首帧发送成功只能证明 TCP 层发送成功，不能证明控制器业务层已经执行。

## 移动逻辑

当前 `lib2/move.py` / `lib2/module.py` 主线移动已经切到“当前 yaw 位置保持式移动”。移动和旋转共用 `move._calculate_position_hold_motion()`：持续下发目标 yaw，同时用当前位置到目标点/保持点的误差计算 `ch0/ch2`，不再使用固定航向 vector PID 的 yaw gate。

`module.move_to_des(...)` 是主组合移动：

- 等待当前 `reference` 参考点位姿可用。
- 调用 `move.move_to_target(...)` 阻塞移动到目标点。
- `reference="robot"`：机器人中心到点。
- `reference="weapon"`：weapon/夹爪到点，底盘 yaw 仍用 robot yaw。
- `target_deg` 现在是移动过程持续下发的目标航向角，不再是“到点后才旋转”的角度。
- `target_deg=None`：进入函数时取当前 yaw 作为移动过程目标航向；到点后发送 `des_yaw_i16=0`，停止航向控制。
- `target_deg` 是角度值：移动过程持续输出该目标航向；到点后用 `wait_until_direction_reached()` 等待当前 yaw 稳定在该角度附近。

`move.move_to_target(...)` 是位置保持式阻塞移动，不是分段式位移。每一轮逻辑：

```text
1. 读取 robot pose、reference pose 和 odom。
2. 用 reference pose 到目标点的平面距离按比例输出 scalar_cmd，默认复用 `DEFAULT_ROTATE_POSITION_KP`，也可通过 `position_hold_kp` 覆盖。
3. 将 map/world 下的 dx/dy 按当前 robot yaw 转成车体 forward/lateral 误差。
4. 按误差向量比例分配到 ch0/ch2。
5. 不再因为 yaw_error 超过 move_gate_deg 而停 ch0/ch2；底盘 yaw PID 只由 des_yaw_i16 影响。
6. 到点后停车；final_target_yaw_deg=None 时 des_yaw_i16=0，否则保持目标航向。
```

旋转逻辑 `move.rotate_to_target_yaw_segmented(...)` 也调用同一个 `_calculate_position_hold_motion()`。区别是旋转没有业务目标点输入时，会在进入函数时读取当前 robot 坐标作为保持点；传入 `des_x/des_y` 时则把输入点作为保持点。

位置保持/移动向量分解现在会先预测 yaw：

```text
predicted_yaw_deg = current_yaw_deg + degrees(angular_z_rad * prediction_dt)
prediction_dt = clamp(yaw_age_sec + 1/70, 0, 0.10)
```

`angular_z_rad` 来自 odom 的 `twist.angular.z`，是有符号角速度；正负方向需要现场通过日志确认是否与 `current_yaw_deg` 增减一致。`move_to_target()` 会读取 `odom_runtime` 使用完整预测；`rotate_to_target_yaw_segmented(..., odom_runtime=...)` 传入 odom 后也会使用角速度，否则退化为只补偿 yaw 数据年龄和 1/70s 发送周期。当前 `DEFAULT_ODOM_MAX_AGE_SEC = 0.25`，超过该年龄的 odom 不再参与 yaw 预测，本轮按 `angular_z_rad=0` 分配 ch0/ch2，避免旧角速度导致轨迹变弯。

当前默认关键参数在 `move.py`：

```text
DEFAULT_ROTATE_POSITION_KP = 900.0
DEFAULT_ROTATE_POSITION_MAX_CMD = 600  # 旋转保持默认；移动默认用 cruise_forward_cmd/v 作为上限，也可传 position_hold_max_cmd
DEFAULT_LATERAL_CMD_SIGN = -1
DEFAULT_FORWARD_CMD_SIGN = 1
DEFAULT_ODOM_MAX_AGE_SEC = 0.25
DEFAULT_MOVE_YAW_SOFT_LIMIT_1_DEG = 10.0
DEFAULT_MOVE_YAW_SOFT_LIMIT_1_MAX_CMD = 400
DEFAULT_MOVE_YAW_SOFT_LIMIT_2_DEG = 25.0
DEFAULT_MOVE_YAW_SOFT_LIMIT_2_MAX_CMD = 250
DEFAULT_STOP_DISTANCE = 0.02m
```

移动过程中会根据当前 yaw 与目标 yaw 的偏差动态限制平移输出上限：偏差 `>10deg` 时 `ch0/ch2` 向量最大输出不超过 `400`，偏差 `>25deg` 时不超过 `250`。这是软限幅，不会停止平移，只降低大角度转向时平移对底盘输出能力的占用。

`move.move_backward_to_target(...)` 现在不再维护独立倒退闭环，而是把移动过程目标航向设置为输入最终角 `+180deg` 后复用 `move_to_target()`；到点后再把目标航向字段切回原始最终角。

`move.move_to_target(...)` 现在只写移动相关通道 `ch0/ch2/ch3/des_yaw_i16`，不会复位 `ch1/ch4/ch5` 等模式/夹持通道。抓取 weapon 后，只要没有其他动作显式改这些通道，普通移动会自然保持当前夹持状态。带 weapon 上下楼前仍必须重新确认夹持通道是否需要贯穿保持，因为 `module.climb()` / `module.descend()` 会切换升降模式。

### 旋转逻辑

`move.rotate_to_target_yaw_segmented(...)` 现在是“目标航向 + 位置保持”的阻塞旋转：

```text
1. 如果 des_x/des_y 都为 0，进入函数时读取一次当前机器人坐标作为保持点。
2. 如果 des_x/des_y 非 0，使用输入坐标作为保持点。
3. 每轮持续发送目标 yaw，同时用当前位置到保持点的误差计算 ch0/ch2 做位置保持。
4. 不再因为 yaw 误差大而停 ch0/ch2；底盘 yaw PID 只由 des_yaw_i16 影响。
5. 退出条件：`des_x/des_y` 都为 `0` 时只要求 yaw 到目标角阈值内并稳定；显式传入 `des_x/des_y` 时，才额外要求当前位置距离保持点不超过 `position_tolerance`。
```

当前默认关键参数：

```text
DEFAULT_SEGMENTED_YAW_STEP_DEG = None
DEFAULT_SEGMENTED_YAW_STEP_DEG_CURRENT_POSITION = 45.0   # des_x/des_y 都为 0
DEFAULT_SEGMENTED_YAW_STEP_DEG_INPUT_POSITION = 360.0    # 使用输入保持点
DEFAULT_ROTATE_POSITION_TOLERANCE = 0.05m
DEFAULT_ROTATE_POSITION_MAX_CMD = 600
DEFAULT_ROTATE_POSITION_KP = 900.0
DEFAULT_DIRECTION_STABLE_SEC = 1.0
```

测试脚本里可局部覆盖这些参数，例如 `rotation_move.py` 目前对本次调用传 `stable_sec=0.0`，用于取消目标附近额外等待。注意协议里 `target_yaw_deg=0.0` 仍表示 `des_yaw_i16=0` 关闭航向 PID；如果要转到 0 度，应继续用 `0.01deg`。

### 旧 vector PID 到点来源

```text
1. 固定车体目标航向 FIXED_YAW_DEG，当前为 0.01deg。
2. 从当前 position 后端读取机器人中心 map/world 坐标。
3. 用目标点距离做 PID，输出速度标量 scalar_cmd。
4. 将 map/world 下的目标误差向量转换到固定航向对应的车体前进/横移轴。
5. 按实时向量比例分配到 ch0 横移和 ch2 前进。
6. 若航向误差超过 YAW_GATE_DEG，停车等待控制器 yaw PID 转回阈值内。
7. 到点后停车并进入下一个 waypoint。
```

这一节现在只作为旧测试脚本 `move_test.py` 的来源记录；主线 `move.move_to_target(...)` 已改为当前 yaw 位置保持式移动。`move_test.py` 保留为测试脚本，当前测试参数：

```text
TARGET_POINTS = [(-0.8, -4.8), (-1.0, 0.0), (-2.4, -2.4)]
FIXED_YAW_DEG = 0.01
YAW_GATE_DEG = 2.0
MIN_ACTIVE_MOVE_CMD = 180
LATERAL_CMD_SIGN = -1
FORWARD_CMD_SIGN = 1
MOVE_TIMEOUT_SEC = 600.0
```

实测过程中发现 `ch0` 符号与最初假设相反，已通过 `LATERAL_CMD_SIGN=-1` 翻转；如果后续在 weapon reference 下发现距离持续变大，应优先检查 `FORWARD_CMD_SIGN`、当前 yaw 下的坐标变换、外参、以及调试距离是否使用了正确参考点。

重构时必须重点检查受影响入口：

```text
module.move_to_des()
module.move_backward_to_des()
module.adjust_position()
module.fetch_and_store_kfs()
module.fetch_weapon()
module.climb()
module.descend()
module.execute_action_row()
```

已知风险：

```text
1. 新主线已取消 yaw gate 停车逻辑；如果 yaw 误差大时平移方向异常，应优先看当前 yaw、目标 yaw 和车体坐标变换。
2. 位置保持式比例输出没有最小通道值；如果临近目标推不动车，应优先调 `DEFAULT_ROTATE_POSITION_KP` 或本次 `cruise_forward_cmd` 上限。
3. ch0/ch2 符号必须沿用实测值：LATERAL_CMD_SIGN=-1，FORWARD_CMD_SIGN=1。
4. 旧 overshoot 回退逻辑与当前位置保持式移动不兼容，当前主线未继续使用旧过点回退。
5. weapon reference 会直接受新逻辑影响，必须单独实机验证。
6. 上下楼触发前后仍需要明确方向和姿态，不应把台阶触发定时动作一起重写。
```

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
    adjust_distance=...  # 默认 PRE_DESCEND_ADJUST_DISTANCE = 0.3m
)
```

函数内部会按 `stair_id` 查矩阵坐标。`height_relation=2` 时执行原来的坐标微调逻辑，按方向和 `adjust_distance` 计算微调目标；`height_relation=1` 时先按 `direction` 旋转到目标航向，再用 `ch2=200` 前进 `2s`，完成微调。`height_relation` 不是 `1/2` 会报错并终止程序。当前默认微调距离 `PRE_DESCEND_ADJUST_DISTANCE = 0.3m`。

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
6. 调用 `module.wait_with_kfs_suction(..., duration_sec=2.0)`，默认保持吸取 2.0s。
7. 主线程先收口为 `ch4=3,ch5=1,ch6=1,ch7=1`，随后启动 `module.start_kfs_post_suction_thread(...)` 异步执行 `pose_id=3 -> pose_id=4 -> 释放吸盘 -> 回0态 -> 复位KFS通道`，`fetch_and_store_kfs()` 立即返回。

注意：`fetch_and_store_kfs()` 当前不再自动倒退回当前台阶中心；如果后续任务需要回中心，应由外层流程显式调用底盘移动动作。

相关底层函数：

- `move.control_kfs_pose(sender, pose_id, ..., suction_ch4=...)`
- `module.set_kfs_suction(sender, suction_on=True/False, ...)`
- `module.wait_with_kfs_suction(sender, duration_sec, ...)`
- `module.start_kfs_post_suction_thread(sender, ...)`：当前由 `fetch_and_store_kfs()` 在吸取保持完成后启动；只管理 KFS 机械臂/吸盘后续，不执行回中心。

KFS 后续的主流程边界：

```text
1. 主流程完成抓取姿态、吸盘吸取和吸取保持。
2. 主流程启动 KFS 后续线程后返回，后续线程异步完成机械臂过渡/存储/释放/回0。
3. 主流程随后可继续下一步，尤其可与上下楼自动动作并发；但所有模式触发上升沿必须通过 `AUTO_TRIGGER_LOCK` 互斥。
4. 主流程按任务需要决定是否显式执行 move_backward_to_des(...) 或 move_to_des(...) 回当前台阶中心。
```

不要把回中心放进 KFS 后续线程；回中心会写 `ch0/ch2/des_yaw_i16`，和下一任务并发时会抢底盘移动通道。当前理解是：上楼梯和 KFS 姿态都是边沿触发后由控制器自动执行，触发后的持续通道值不会改变已经启动的自动任务；关键是安排好各自动任务的触发边沿时序，避免在同一时刻互相覆盖模式/触发通道。

当前所有“回退到中心点”相关逻辑位置：

```text
module.move_backward_to_des(...)
  通用倒退到指定坐标封装，底层调用 move.move_backward_to_target(...)。

module.descend(...)
  下楼触发完成后，调用 move_backward_to_des(...) 倒退到目标 to_pos 的中心坐标。
```

`fetch_and_store_kfs()` 当前还支持测试用等待参数：

```python
module.fetch_and_store_kfs(
    ...,
    suction_hold_sec=...,
    grab_pose_hold_sec=...,
    transition_pose_hold_sec=...,
    store_pose_hold_sec=...,
)
```

未传时继续使用默认等待；当前高/低位抓取在机械臂和吸盘时序上是对称的：区别只在 `pose_id=1/2`；但完整流程不完全对称，因为 `adjust_position()` 中 `height_relation=1` 走“转向后定时 ch2=200 前进 2s”，`height_relation=2` 走坐标微调。

当前 `fetch_and_store_kfs()` 已做过语法检查和部分现场调试，但完整自动抓取并存储流程仍需继续实机验证。

当前结束状态需要特别注意：`fetch_and_store_kfs()` 返回时后续线程通常仍在运行，主线程刚收口后通常保留：

```text
ch0=0
ch2=0
ch3=0
ch4=3
ch5=1
ch6=1
ch7=1
des_yaw_i16=微调/移动阶段最后保留的目标航向
```

KFS 后续线程完成后会执行吸盘释放、`pose_id=0` 回 0 态，并最终复位到 `ch4=1,ch5=1,ch6=1,ch7=1`。

当前 KFS 后续线程时序：

```text
pose3 过渡:
  lock
  ch4=3,ch5=2,ch6=3,ch7=1  0.1s
  ch4=3,ch5=2,ch6=3,ch7=3  0.3s
  ch4=3,ch5=1,ch6=1,ch7=1
  unlock
  sleep 3.0s

pose4 存储:
  lock
  ch4=3,ch5=2,ch6=4,ch7=1  0.1s
  ch4=3,ch5=2,ch6=4,ch7=3  0.4s
  ch4=3,ch5=1,ch6=1,ch7=1
  unlock
  sleep 1.5s

释放吸盘:
  lock
  ch4=3,ch5=2  0.1s
  ch4=1,ch5=2  0.5s
  ch4=1,ch5=1,ch6=1,ch7=1
  unlock
  sleep 2.0s

回 0 态:
  lock
  ch4=1,ch5=2,ch6=0,ch7=1  0.1s
  ch4=1,ch5=2,ch6=0,ch7=3  0.5s
  ch4=1,ch5=1,ch6=1,ch7=1
  unlock
```

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

当前 odin weapon 外参：

```text
LIDAR_TO_WEAPON_X = 0.107
LIDAR_TO_WEAPON_Y = -0.446
```

该值按 yaw=90deg 时尺量实际 weapon 位置相对目标偏差 `(-0.175, -0.18)` 做过一次修正：`X -= 0.18`、`Y += 0.175`。

当前 mid360 默认值：

- 1: `(-0.32, 4.12)`
- 2: `(-0.12, 4.12)`
- 3: `(0.074, 4.12)`
- 4: `(0.275, 4.12)`
- 5: `(0.483, 4.12)`
- 6: `(0.681, 4.12)`

当前流程默认移动速度 `v=300`：

1. 读取 `WEAPON_TARGETS[weapon_id] = (des_x, des_y)`。
2. 先以 `reference="weapon"`、固定航向默认 `90deg` 移动到前置点 `(des_x, des_y - 2.0)`，前置距离由 `weapon_approach_offset_y=2.0` 控制。
3. 再以 `reference="weapon"`、固定航向默认 `90deg` 移动到真实 weapon 目标点 `(des_x, des_y)`。
4. 进入武器模式并夹紧抬起：
   ```text
   ch5=3, ch4=1, ch1=0     保持 0.3s
   ch5=3, ch4=1, ch1=0     保持 0.3s
   ch5=3, ch4=3, ch1=0     保持 1.0s
   ch5=3, ch4=3, ch1=100   保持 1.0s
   ```
5. 保持 `ch5=3,ch4=3,ch1=100`，以 `90deg` 固定航向、`reference="robot"` 正向移动到中间点 `(-2.4, -1.2)`。
6. 保持 `ch5=3,ch4=3,ch1=100`，以 `-90deg` 固定航向、`reference="robot"` 正向移动到最终点 `(-2.4, -2.4)`。
7. 到最终点后保持夹持等待 `5s`。
8. 当前不在 `fetch_weapon()` 内释放 weapon，最后继续保持夹持：
   ```text
   ch5=3, ch4=3, ch1=100, ch2=0
   ```

注意：`fetch_weapon()` 第一段移动用 `reference="weapon"`，但多数旧 debug 打印只看机器人中心到某个目标点的距离。调试 weapon 抓取时必须打印 `position_runtime.get_weapon_pose()` 到对应 `WEAPON_TARGETS[weapon_id]` 的距离，否则会把机器人中心到 `-1` 或其他点的距离误认为 weapon 到点误差。

当前最终状态：

```text
ch1=100
ch2=0
ch4=3
ch5=3
```

也就是保持 weapon 模式、保持抬升、保持夹紧。后续释放/放下/退出 weapon 模式由单独复位方法处理。

`move.reset_weapon_after_fetch(sender)` 已封装 weapon 松开、放下并退出动作：

```text
1. ch4=1,ch1=0，同时松开夹爪并放下。
2. 等待 1s。
3. ch5=1，退出 weapon 模式并切回默认模式。
```

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
    next_from_pose=0,
    next_to_pose=0,
    next_height_action=0,
)
```

当前 `execute_action_row()` 已实现：

- 接收一行 `action_row`，格式仍为 `[from_pos, to_pos, move_dir, height_action, grab_action]`。
- 新增 `final_direction` 输入，默认 `1`，必须是 `1/2/3/4`，表示本行任务完成后的最终朝向。
- 新增下一行上下文输入：`next_from_pose=0`、`next_to_pose=0`、`next_height_action=0`。单独调用时默认都为 `0`；矩阵循环执行时由 `execute_action_matrix()` 自动传入下一行的 `from_pos/to_pos/height_action`。
- 检查行长度必须为 `5`，否则打印错误并 `sys.exit(1)`。
- 行长度正确后，先读取 `from_pos/to_pos`，通过 `tools.stair_id_to_direction(from_pos, to_pos)` 推导相邻方向，再调用 `module.get_stair_height_relation(from_pos, inferred_direction)` 获取高低关系并存入结果；`from_pos == to_pos` 时高低关系记为 `0`。
- 检查 `move_dir` 必须与 `from_pos/to_pos` 的坐标推导方向一致；不一致时直接报错退出。
- 检查 `move_dir` 必须为 `0/1/2/3/4`，否则打印错误并 `sys.exit(1)`。
- 当前地图相邻台阶不存在等高普通平移，因此新增防御检查：如果 `move_dir != 0 and height_action == 0 and grab_action == 0`，视为动作矩阵错误并 `sys.exit(1)`。
- `move_dir != 0` 时才获取 `from_x/from_y/to_x/to_y`，来源是 `module.get_stair_xy(from_pos/to_pos)`；原地分支暂不预取坐标。
- `grab_action == 1` 时先调用 `module.fetch_and_store_kfs(...)`，输入 `stair_id=from_pos`、`direction=move_dir`、`final_target_yaw_deg=tools.direction_int_to_yaw_deg(final_direction)`；吸取完成后再按下一行上下文决定是否正向回当前台阶中心。
- KFS 分支回中心判断：如果 `next_height_action == 1`，会用 `next_from_pose/next_to_pose` 推导下一行方向并读取下一行高低关系；当下一行高低关系为 `1` 且本行 `to_pos == next_to_pose` 时，不执行回中心，直接返回。其他情况调用 `move_to_des(x=from_x, y=from_y, target_deg=tools.direction_int_to_yaw_deg(move_dir))` 正向移动回当前台阶中心。
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
- `module.climb()` 上楼前准备动作已改为复用 `module.adjust_position(..., stair_id=-1, height_relation=1)`；高位微调不依赖真实台阶坐标，只用于按 `direction1` 对正并定时前探，不再使用单独的固定 `ch2` 前进 3s 逻辑。
- 台阶相关到点移动统一使用 `module.STAIR_MOVE_MAX_CMD = 200`：包括上楼后正向到目标格中心、下楼后倒退到目标格中心，以及 KFS 后需要正向回当前格中心的动作。
- 底层上楼 `move.climb()` 与下楼 `move.descend()` 的 fire 保持时间默认都是 `0.3s`；arm 默认 `0.1s`。上下楼触发段使用 `tools.AUTO_TRIGGER_LOCK`，复位到 idle 后释放锁，等待高度/位移完成条件时不继续持锁或反复写 idle 通道。
- 下楼触发时会显式写 `ch7=1`，避免 KFS/上楼遗留触发通道；上楼按当前要求不额外写 `ch6`。

`module.fetch_and_store_kfs(...)` 当前保留 `final_target_yaw_deg=0.0` 输入用于接口兼容，本身不自动倒退回当前 `stair_id` 中心点；KFS 后是否正向回当前中心点由 `execute_action_row()` 根据下一行上下文决定。

当前 KFS 后续是异步式：`fetch_and_store_kfs()` 只等待吸取保持完成并启动 `start_kfs_post_suction_thread()` 后返回；过渡、存储、释放、回 0 态和通道复位由后续线程执行。

当前真实动作已接入 `grab_action == 1` 和 `height_action != 0` 两类方向动作。按当前真实场地和规划规则，不应接入等高普通移动分支；如果后续地图允许相邻等高台阶，再重新讨论并补普通移动封装。

当前已封装整矩阵循环执行函数：

```python
module.execute_action_matrix(
    sender,
    position_runtime,
    odom_runtime,
    action_matrix,
    final_direction=1,
    stop_on_unimplemented=True,
)
```

逻辑：

- 接收 `n*5` 动作矩阵，也兼容单行 `5` 列输入。
- 顺序调用 `module.execute_action_row(...)`，每行统一传入当前 `final_direction`，并自动传入下一行的 `from_pos/to_pos/height_action`。
- 返回 `row_count/final_direction/results`，其中 `results` 是每行执行结果列表。
- 默认 `stop_on_unimplemented=True`；遇到原地分支或其他 `implemented=False` 的未接入真实动作行时，立即打印错误并 `sys.exit(1)`。这是硬件动作保守边界，避免矩阵里有尚未确认的动作时被静默跳过。

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
- `module.execute_action_matrix()`：已接入矩阵级顺序执行，只负责逐行调用 `execute_action_row()`；默认遇到未实现分支会报错退出。

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
next_from/to/height 下一行上下文；KFS 分支用于判断抓取后是否正向回当前台阶中心
height_action       1=需要上下楼，0=不上下楼；上/下由 height_relation 判断
grab_action         1=抓取，0=不抓取；当前矩阵生成层只输出 0/1
```

注意：`module.climb()`、`module.descend()` 内部已经完成方向 int 到角度转换；`module.fetch_and_store_kfs()` 的夹取方向仍传 int。KFS 抓取后的正向回中心由 `execute_action_row()` 负责，固定航向角使用本行 `move_dir` 转出的角度。只有直接调用 `move_to_des()` 时才需要传 `target_deg`，但当前规划保证不出现等高普通移动，暂不把普通移动作为主分支。

当前验证状态：

- 已运行 `python3 -m py_compile lib2/module.py lib2/move.py read_matrix.py`，确认 KFS 主流程、原地旋转 45 度分段、解释脚本均语法通过。
- 已运行 `python3 read_matrix.py`，确认动作矩阵解释输出与当前 KFS 流程一致；注意 README 中的解释脚本输出可能仍需随异步后续文案继续同步。
- 已运行 `python3 -m py_compile descend.py`，确认 `-1 -> 2` 上楼、`2 -> 5` KFS、`2 -> -1` 下楼测试脚本语法通过。
- 已运行 `python3 -m py_compile rigion_1.py`，确认简化矩阵测试脚本语法通过。
- 已运行 `python3 -m py_compile rigion_2.py`，确认完整 weapon + 矩阵测试脚本语法通过。
- 已运行 `python3 -m py_compile lib2/position_resource.py lib2/module.py lib2/tools.py lib2/move.py`。
- 已运行 `python3 -m py_compile utils/utils.py`。
- 已运行 `python3 -m py_compile catch.py`，当前 `catch.py` 是 odin `fetch_weapon(weapon_id=1)` 完整流程调试脚本，速度 `300`。
- 已运行 `python3 -m py_compile up_down_test.py`，最后一段已替换为 `1 -> 2` KFS 抓取。
- 已运行 `python3 -m py_compile move_test.py`，当前旧 vector PID waypoint 测试可语法通过；主移动链路已改为当前 yaw 位置保持式移动。
- 已运行 `python3 -m py_compile move_t.py rotation_move.py ori_rot.py fwd.py`，当前移动/旋转/前进调试脚本均可语法通过。
- 已运行 `python3 -m py_compile lib2/position_odin.py lib2/module.py lib2/move.py lib2/tools.py`，确认 odin weapon 外参、KFS 异步后续、上下楼锁和触发时序语法通过。
- 已运行 `python3 -m py_compile ori_rot.py lib2/move.py`，确认循环旋转测试和动态旋转分段默认可语法通过。
- 已做资源层 smoke test：`module.get_stair_xy(2)` 与 `position_resource.get_stair_xy(2)` 一致，且 `module.position_lib is position_resource.position_lib` 为 `True`。
- 已用 `build_action_matrix_from_qr("000020020200")` 做过一次无 GUI 生成检查，确认 `height_action` 和 `grab_action` 都只输出 `0/1`。
- `execute_action_row()` 的真实硬件动作分支尚未实机验证。
- `move_test.py` 的 vector PID 已通过现场日志暴露并修正了 `ch0` 横移符号问题；主线切到当前 yaw 位置保持式移动后仍需要继续实机验证 `reference="weapon"` 外参和 `ch0/ch2` 符号组合。

### 协作和实现偏好

- `lib2` 移动到点逻辑已经切到当前 yaw 位置保持式移动；后续窗口不要再把 `move_test.py` 当成主线实现来源，而应优先检查 `lib2/move.py` / `lib2/module.py` 主线行为。
- 当前优先任务是通道构建逻辑修改后的动作勘误：确认每个动作只写自己负责的通道，并逐个确认动作完成后的最终通道状态和复位边界。
- 用户希望尽量复用已有封装，尤其是 `module.py` 组合动作、`move.py` 底层动作、`tools.py` 方向/矩阵工具；不要重复实现已有逻辑。
- 发现规划语义、动作矩阵、真实场地假设之间可能矛盾时，要明确指出并质问确认，不要为了继续写代码而默认绕过。
- 对硬件动作相关逻辑要保守，宁可加防御检查并退出，也不要静默跳过可能危险的动作。
- 讨论方向时，外部接口优先使用方向 int：`1=前方/x+`、`2=+90/左/y+`、`3=-90/右/y-`、`4=后方/x-`；角度转换应尽量留在已有封装内部。
- README 是下一窗口的交接入口；当前进度、关键假设、未完成分支和实机风险要及时写进 README。

### 本轮修改错误记录

本轮围绕 vector PID、KFS、weapon 和 `rigion_1.py` 做了多次现场调试，以下错误需要后续明确避免：

- 最初把 `module.move_to_des()` / `move_backward_to_des()` 的输入误判为需要改签名；实际只需要改变内部语义，把原有 `target_deg` 当作移动过程固定 yaw，外层调用签名可以保持不变。
- 在 `fetch_weapon()` 调试时，用 `tools.debug_print(sender, position_runtime, entry_x, entry_y)` 打印的是机器人中心到 `-1` 台阶的距离，不是 weapon/夹爪到 weapon 目标点的距离。这个日志不能判断 `fetch_weapon(reference="weapon")` 是否接近目标。
- 让 `fetch_weapon()` 第一段使用固定 `90deg` 后，现场出现距离变大时，不能只看 robot debug；必须打印 `position_runtime.get_weapon_pose()` 到 `WEAPON_TARGETS[weapon_id]` 的 `dx/dy/distance`，再判断是 `ch0/ch2` 符号、固定航向、外参还是目标点错误。
- 曾把“夹取后先到第二个点，再到 `-2.4,-2.4`”误实现为“先到 `x=-1.2,y=-2.4`”或“先到 `x=-1.2,y保持当前`”，并且一度改成单段到最终点。当前要求已修正为两段正向移动：`90deg -> (-2.4,-1.2)`，`-90deg -> (-2.4,-2.4)`。
- 抓取后普通移动现在不会覆盖 `ch1/ch4/ch5`，因为 `move.move_to_target()` 只写 `ch0/ch2/ch3/des_yaw_i16`。但上下楼组合动作仍会切换 `ch5=1`，带 weapon 上下楼前必须单独设计夹持保持策略。
- 通道构建逻辑从“外部构造完整 channels 并整体覆盖”改为“只设置内部通道变量，后台线程统一构帧”后，不能再依赖隐式安全默认值。每个动作完成后都会保留最后写入的模式/触发通道，必须显式确认是否需要复位。
- `fetch_weapon()` 最新流程在 `(-2.4,-2.4)` 等待 `5s` 后不再释放 weapon，最后保持 `ch1=100,ch4=3,ch5=3`；后续释放/放下/退出 weapon 模式由单独复位方法处理。
- `fetch_weapon()` 第一段已改为两段 weapon-reference 靠近：先到 `(des_x, des_y - 2.0)`，再到 `(des_x, des_y)`；默认速度已改为 `v=300`。`catch.py` 当前只测试 `weapon_id=1` 的完整 `fetch_weapon()` 流程。
- `fetch_weapon()` 到 weapon 点后会先持续进入武器模式 `ch5=3,ch1=0,ch4=1,ch6=1,ch7=1`，再执行夹紧和 `ch1=100` 抬升；如果现场仍触发升降动作，应优先检查控制器实际模式映射或自动动作状态机。
- `fetch_and_store_kfs()` 最新流程已改为异步后续：吸取保持默认 `2.0s`，主线程启动 `start_kfs_post_suction_thread()` 后返回；后续线程负责 pose3、pose4、KFS 模式下释放吸盘、回 0 态和最终复位。
- KFS 释放吸盘必须在 `ch5=2` 模式下打 `ch4:3->1` 下降沿；曾只写 `ch4` 导致中间上楼后 `ch5=1`，吸盘没有释放。当前释放阶段已加锁并先写 `ch5=2`。
- `descend()` 下楼前会把 `direction1` 取反方向对正，例如 `2 -> -1` 推导 `direction1=4`，下楼前对正到 `0.01deg`。这符合“车头朝上一级、倒退下楼”的假设；但下楼后的 `move_backward_to_des(... target_deg=final_direction)` 当前会在移动过程中使用 `final_direction + 180deg`，不是严格保持下楼前 yaw 直接倒退。若现场要求保持当前 yaw 退回目标中心，需要新增下楼专用回中心封装。
- `suck.py` 直连测试中，若现场显示 `ch4` 只保持一次或一小段时间，必须区分“原始网口帧 ch4”与“控制端业务层内部映射/边沿状态”。上位机侧可通过打印实际发送帧确认，但不要把控制端内部恢复为 `1` 误判为上位机没有发 `3`。
- `DEFAULT_MOVE_GATE_DEG=2.0` 对固定 `90deg` 移动偏紧，现场日志中多次出现 `yaw_error` 稳定在 `2.x~4.xdeg` 导致 `ch0/ch2=0`。如果主要验证平移方向和 PID，先临时放宽门控比继续调 PID 更有效。

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
- 如果 pose 或 odometry 出现 `NaN/inf`，`move.py` 已加有限值保护，会停车跳过该轮，避免把非法 yaw 发给控制器。
- 当前 TCP 连接和首帧发送都不能证明控制器业务层已经执行，仍需现场观察或后续加 ACK。
- 移动到点能较准；旋转逻辑已加入位置保持，但是否能抑制麦克纳姆轮机械误差仍需实机继续验证。`level_2.py` 默认到点后不最终旋转。
- KFS 新协议的关键点是 `ch5=2,ch6=1/2,ch7:1->3` 触发高/低位抓取，完成后依次触发 `ch6=3` 过渡态、`ch6=4` 存储姿态；`ch7=0` 回归 0 态；`ch5=2,ch4:1->3` 吸取，`ch5=2,ch4:3->1` 释放。
- `catch.py` 当前不是 KFS 调试脚本，而是 `fetch_weapon(weapon_id=1)` 调试脚本；会打印 robot/weapon pose、weapon 到目标距离，并执行完整 weapon 抓取流程。
- `1 -> 2` 当前矩阵推导为 `direction=3`、`height_relation=2`、`pose_id=2`，是低位抓取；`-1 -> 2` 当前为 `direction=1`、`height_relation=1`、`pose_id=1`，是高位抓取。
- `move.py` 主移动逻辑已取消 yaw gate 停车；`move_test.py` 仍保留旧 yaw gate 测试逻辑，调试旧脚本时仍需注意门控过紧会让 `ch0/ch2` 长时间为 0。
- `move.py` 主移动逻辑已取消旧 PID 标量和 `MIN_ACTIVE_MOVE_CMD`；当前位置保持式移动临近目标推不动车时，优先调 `DEFAULT_ROTATE_POSITION_KP` 或本次速度上限。
- `rotate_to_target_yaw_segmented()` 未传保持点时用当前位置做过程保持且默认 `45deg` 分段，但退出只看 yaw 到位稳定；传入 `des_x/des_y` 时用输入点保持、默认 `360deg` 分段，退出还要求位置到保持点容差内。`target_yaw_deg=0.0` 仍是关闭航向 PID，不是转到 0 度。
