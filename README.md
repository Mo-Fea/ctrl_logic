注意：当前主线优先阅读 `lib2/` 和 `utils/`。旧 `lib/`、`test/` 和临时脚本只在用户明确指定时再深入。

# ws_r2 当前工程交接

本仓库是 R2_H 上位机控制工程。主线已经从“单脚本直连控制”演进为：规划层生成动作矩阵，执行层解释矩阵，再由底层动作模块持续更新控制帧。

核心链路：

```text
二维码/KFS布局
  -> utils/race.py 规划挑战赛路径
  -> utils/challenge_lib.py / utils/utils.py 生成 n*5 动作矩阵
  -> lib2/module.py 解释动作矩阵和组合动作
  -> lib2/compete_logic.py 串联比赛区域流程
  -> lib2/move.py / lib2/kfs.py / lib2/weapon.py 执行底层移动、上下楼、KFS、weapon 通道时序
  -> lib2/tools.py 的 frame_thread 持续发送 V3 控制帧
```

## 当前状态

- 当前主线使用 `lib2/` 和 `utils/`；旧 `lib/` 基本只作参考。
- 通信协议为 V3 控制帧，`tools.frame_thread` 内部维护 `ch0~ch9/yaw_i16/des_yaw_i16/cylinder_select`，动作层只局部更新自己负责的通道。
- 后端选择拆成两层：`LIDAR_TYPE` 选择位姿来源，`FIELD_TYPE` 选择红/蓝半场坐标和地图方向。
- 雷达后端：`1=odin`，`2=mid360`。主流程必须通过 `module.init(lidar_type=...)` 或 `module.configure_position_backend(...)` 刷新 `position_resource` 和 `move` 的缓存。
- `module.init()` 建立 TCP 后会保持 `des_yaw_i16=0` 关闭航向 PID，不自动原地旋转；同时设置 `ch9=800`，依次执行 `weapon.weapon_up()` 和 `weapon.weapon_loose()`。完成后将 `ch1=0`、`ch5/ch6/ch7=1`，保持夹爪打开状态 `ch4=3` 和吸盘头角度 `ch9=800`。纯通信/位姿调试可传 `initialize_machine_pose=False` 跳过机械动作。
- 场地半场：`1=red/right`，`2=blue/left`。蓝场入口、weapon、赛后三点里还有占位/镜像值，换场地前要重新核对实测坐标。
- `lib2/` 执行层和 `utils/challenge_lib.py` 生成端已按当前红/蓝半场方向语义处理方向码；`position_resource` 和 `utils/race.py` 均维护红/蓝半场台阶高度。
- `utils/race.py` 的比赛数量模式已集中化：`configure_competition_mode(1)` 使用 `2R1/2R2/1Fake`，模式 `2` 使用 `3R1/3R2/1Fake`；`REQUIRED_R2_PICKUP_COUNT` 独立表示 R2 必须抓取的数量，默认为 `2`。
- QR/KFS 规划前会按当前比赛模式严格校验 R1/R2/Fake 数量；只校验类型和数量，尚不校验摆放区域。
- 完整挑战赛矩阵当前仍会自动包含固定入口/出口行：入口为 `-1->2`，最后到 `10` 追加 `10->13`，最后到 `12` 追加 `12->15`；`move_dir` 由 `tools.stair_id_to_direction()` 按当前红/蓝半场动态生成。
- 如 QR 前三位含 R2-KFS，`challenge_lib.build_action_matrix_with_pre_entry_pickup()` 会先生成场外吸取行，再插入入口上楼行。2号优先；1/3号同时存在时分别规划并选总代价较小者。
- `module.move_to_des()` / `module.move_backward_to_des()` 默认 `v=500`；主流程显式传入 `v=600` 或 `v=300` 的调用不受默认值影响。
- 旋转到目标角当前只看角度稳定：旋转期间 `ch0/ch2/ch3=0`，不做位置保持。默认角度容忍值为 `3deg`，稳定时间为 `1s`。
- KFS 姿态和后续线程已拆到 `lib2/kfs.py`；`module.fetch_and_store_kfs()` 保留旧名，当前实际负责前探、双头吸盘切换、抓取、吸取和异步回初态，不再执行 pose4。
- `module.side_suck()` 已接入 `[-1,1,1,0,1]` / `[-1,3,1,0,1]` 特殊动作行，完成侧吸微调、气缸/角度选择、pose5、吸取、横移、异步收尾和回到 `-1`。侧吸准备点移动默认速度已改为 `move_speed=350`。
- `kfs.kfs_suck_preparation(sender,count)` 已加入，按 `count=0/1/2` 预先选择气缸并打开吸气；`count=1` 选择 PF2 并 `suck_count += 1`，`count=2` 选择双气缸并 `suck_count += 2`，`count=0` 不执行动作。
- 四轮锁角已封装为 `move.lock_wheel()` / `move.unlock_wheel()`。现有协议未说明锁存行为，切换到 KFS/weapon 模式后不保证继续锁轮。
- `module.fetch_weapon()` 按红/蓝半场自动选择接近方向、抓取方向和释放前旋转方向；函数返回时保持夹取/抬起状态，不负责释放。可用 `move_to_approach_point=0` 跳过先到 weapon 前方约 `1m` 的接近点。
- `tools.py` 已加入雷达左上角打点的角度坐标修正：`deg0_correction()`、`deg90_correction()`、`neg90_correction()`、`deg180_correction()` 和分派入口 `deg_correction(target_deg,x,y)`；红/蓝半场各维护一张修正表，当前蓝场表暂与红场一致。
- `lib2/compete_logic.py` 当前封装区域流程：`rigion_1()` 抓 weapon 并等待 QR 规划锁释放，`rigion_2()` 消费动作矩阵队列并执行完整矩阵，`rigion_2_retry_plan(r1_count,r2_count,required_r2_pickup_count,qr_string,...)` 用参数重规划并推入队列，`rigion_3()` 进入九宫格后执行 `high_score190` 或 `totally_win`。

## 文件结构

- `lib2/tools.py`：TCP 连接、V3 帧打包、后台持续发送、重定位监听、清理函数、方向工具和 `AUTO_TRIGGER_LOCK`。
- `lib2/position_backend.py`：雷达后端和红/蓝半场选择。
- `lib2/position_odin.py`：odin 后端，基于 `/tf` 同步计算 robot / weapon 位姿，维护红/蓝入口和 weapon 点。
- `lib2/position_mid360.py`：mid360 后端，基于 `/lio/robo/odom`、`/lio/odom` 和外参计算位姿。
- `lib2/position_resource.py`：位姿/里程计资源层，动态生成台阶矩阵，提供 `PositionRuntime`、`OdomRuntime` 和坐标查询。
- `lib2/move.py`：底层移动、倒退、旋转、四轮锁角/解锁、上下楼触发、定时通道输出、weapon 释放。旧 `control_kfs_pose()` 已删除，KFS 姿态统一使用 `lib2/kfs.py`。
- `lib2/kfs.py`：KFS 吸盘旋转、气缸选择、单一姿态、双头吸盘抓取、放置/释放原语和吸取后异步线程封装；pose5 侧吸触发默认保持 `1.0s`。
- `lib2/module.py`：组合动作和执行层入口，包括初始化、到点、普通/侧吸 KFS、weapon 抓取、上下楼/R1 上坡组合、动作矩阵解释。
- `lib2/compete_logic.py`：比赛区域级编排入口，当前包括区域1抓 weapon/等待 QR、区域2执行动作矩阵/参数化重规划、区域3进入九宫格并选择最终策略。
- `utils/race.py`：挑战赛规划核心，维护红/蓝 `pos_to_coord`、比赛数量模式、布局校验、Dijkstra 代价路径规划和可视化。
- `utils/challenge_lib.py`：挑战赛对外入口，封装二维码解析、场外预吸取候选规划、入口/出口动作行、动作矩阵生成和后台 QR 扫描。
- `utils/utils.py`：无 GUI 工具入口，可从 QR 或 KFS 字典生成动作矩阵；也保留对抗赛/旧梅林入口。
- `utils/process.py`：二维码图像源、检测和 payload 校验。`image_source=1` 使用 RealSense D435i/D455 彩色流；打开时会打印设备名、序列号、固件和 USB 模式，默认先试 `640x480@30`，再 fallback 到 `848x480@30`、`1280x720@30`，预热阶段每帧 `5000ms` 超时并最多重试 `3` 次。D455 若枚举为 `USB=2.1`，`640x480@30` 可能首帧超时；应优先换到真正 USB3 链路。`image_source=2` 以 `RELIABLE` QoS 订阅 `/odin1/image/undistorted`，并将常见 ROS `Image` 编码转换为 BGR。Odin 首帧超过 `5s` 未到达时会报错并释放扫码锁，避免主流程永久等待；驱动配置必须启用 `sendrgbundistort: 1`。
- `utils/route.py` / `utils/meilin.py`：旧梅林/对抗赛相关逻辑，非当前挑战赛主执行链路。
- `d455.py`：独立 RealSense D455/D435i 二维码检测脚本，用后台线程模拟主流程扫码形态，默认 `640x480@30`、预热 `15` 帧、稳定 `5` 帧后输出结果，可用 `--no-window` 在 SSH 下关闭窗口。

## 主流程脚本

- 当前工作区没有 `communication_competition/` 目录；新的比赛区域流程优先看 `lib2/compete_logic.py`。
- 当前已在 `lib2/compete_logic.py` 内逐步封装比赛主线，根脚本 `competition_script.py` 仍需后续接入这些区域函数。
- `ultimate_test_script.py`：下位机完整交互调试入口。顶层菜单已接入 `3.恢复至初态`；简单菜单 1~8 均已接入真实动作；复杂菜单已接入初始区域、梅林准备、上下楼、普通 KFS、侧吸、边缘微调、完整梅林和九宫格前三项测试；`九宫格完整测试` 和顶层 `完整流程测试` 仍未实现。
- `ultimate_test_script.py` 的完整梅林测试不再检查 `current_stair_id=-1`；扫码拿到完整动作矩阵后，会先执行 `weapon.weapon_loose()` 打开 weapon 夹爪并等待 `5s`，再调用 `module.execute_action_matrix(...)`。
- `catch.py` 仍保留旧 weapon `ch4=-100` 调试时序，不符合当前 `ch4=1/3` 边沿协议，未替换前不应连接实机运行。`move_t.py`、`ori_rot.py`、`descend.py` 等其他调试入口使用前也应先核对当前内容。

## 完全测试脚本

`ultimate_test_script.py` 启动后先调用 `module.init(lidar_type=1)`。初始化期间关闭航向 PID，不自动旋转；设置 `ch9=800`，依次执行 weapon 夹爪拉起和打开，然后进行红/蓝半场选择并启动 position/odom 资源。

简单动作菜单：

```text
1. 移动测试：参考系、x/y、最终航向、速度、到点阈值
2. 旋转测试：目标角、分段阈值、完成阈值
3. 吸盘头旋转测试：方向码 1/2/3/4
4. 吸盘吸取状态测试：气缸 0/1/2，吸取/释放
5. 机械臂姿态测试：pose0~6
6. weapon 夹爪开合测试：开启使用 ch4:1->3，闭合使用 ch4:3->1
7. weapon 夹爪拉起/放下测试
8. 锁轮测试：调用 move.lock_wheel() / move.unlock_wheel()
```

顶层菜单：

- `1.简单动作测试`
- `2.复杂动作测试`
- `3.恢复至初态`：进入后询问“确认恢复（0返回，1确认）”。确认后依次执行 `move.unlock_wheel()`、`weapon.weapon_loose()`、`weapon.weapon_down()`，再将 `kfs.suck_count=1`、吸盘转到 `0deg`、选择双气缸 `cylinderSelect=0`、调用 `module.set_kfs_suction(suction_on=False, pose_id=0)` 关闭吸气，并将脚本内 `current_stair_id=0`。

复杂动作菜单：

- `1.初始区域测试`：输入 weapon 编号和 `0~600` 移动速度，调用 `module.fetch_weapon()`；成功后调用 `move.lock_wheel()` 锁轮等待，并以红色终端文字提示用户前往简单测试解锁和释放 weapon 夹爪。
- `2.梅林区域测试`：子菜单为 `1.梅林准备`、`2.上下楼`、`3.方块吸取`、`4.侧吸`、`5.边缘微调`、`6.完整梅林`，均已接入真实动作或完整矩阵执行入口。
- `3.九宫格区域测试`：已接入 `1.放二层箱子`、`2.上R1`、`3.R1上释放三层KFS`；`4.完整九宫格区域测试` 仍未实现。
- `4.完整流程测试`：目前未实现。

脚本维护全局 `current_stair_id`，初始值为 `0`：

- 梅林准备移动到 `-1` 台阶后将其设为 `-1`。
- 上下楼测试中，当记录值存在于当前台阶矩阵时只询问目标台阶；否则先询问当前台阶并记录。动作返回后更新为目标台阶。
- 方块吸取测试使用同样的询问逻辑，生成 `[from_pos,to_pos,move_dir,0,1]`；机器仍留在 `from_pos`，因此吸取后不把记录值改为目标台阶。
- 侧吸测试只允许 `current_stair_id=-1`；输入目标 `1/3` 后生成 `[-1,to_pose,1,0,1]` 并通过 `execute_action_row()` 执行。侧吸流程最终回到 `-1`，因此记录值不变。
- 边缘微调测试要求 `current_stair_id` 是有效台阶，输入方向 `1/2/3/4` 和 `0.00~0.50m` 微调距离后调用 `module.adjust_position(move_type=1,height_relation=2)`。
- 完整梅林测试不再拦截 `current_stair_id != -1`；进入后先配置比赛模式 `1/2`，再启动后台 QR 扫描。识别并规划出完整动作矩阵后，先打开 weapon 夹爪并等待 `5s`，随后执行返回的完整动作矩阵；执行成功后将记录值更新为矩阵最后一行的目标台阶。
- 九宫格放置/释放流程依赖 `kfs.suck_count` 判断当前是否持有 KFS；二层箱子测试使用 pose4、释放前吸盘角度、固定坐标移动和 `kfs.release_kfs()`；三层释放测试使用 pose4、释放前吸盘角度和 `kfs.place_3rd_kfs()` 的 Z 下降检测。
- 上下楼和方块吸取都会验证台阶相邻性；不相邻时打印“台阶不相邻，请输入正确的逻辑编号”并重新输入。

## 编程约定

- 优先复用已有封装：`module.init()`、`module.start_position_thread()`、`module.start_odometry_thread()`、`module.move_to_des()`、`module.execute_action_matrix()`、`move.move_to_target()`、`move.rotate_to_target_yaw_segmented()`、`kfs.kfs_grab_pose()`、`module.fetch_and_store_kfs()`、`module.side_suck()`。
- 切换挑战赛数量模式必须在启动 QR 扫描/规划前调用 `race.configure_competition_mode(1|2)`。一次成功的场外预吸取规划会将当前进程中的 `R2_KFS_COUNT` 和 `REQUIRED_R2_PICKUP_COUNT` 各减 `1`；规划失败会恢复原值。
- 不要绕过 `frame_thread` 直接 `sock.sendall()`；动作只通过 `sender.set_*()`、`move.set_channel_values()`、`move.set_motion_channels()` 更新发送状态。
- 不要恢复旧的整帧覆盖方式。每个动作必须显式管理自己结束后的通道状态。
- KFS 姿态、KFS 吸盘释放、KFS 回 0、上楼、下楼的“模式设置 + 触发边沿”必须使用 `tools.AUTO_TRIGGER_LOCK`。锁只覆盖触发临界区，触发后的等待阶段不持锁。
- KFS 机械臂姿态接口只修改 `ch5/ch6/ch7`，不再写入 `ch4`；吸取/释放流程独立维护 `ch4`，吸取后的发送状态应继续保持 `ch4=3`。
- KFS 姿态统一调用 `lib2/kfs.py`；不要恢复已删除的 `move.control_kfs_pose()` 或让机械臂姿态方法重新修改 `ch4`。
- 锁轮使用升降模式 `ch5=1,ch6=2`，`ch7:1->3` 上升沿进入锁轮，`ch7:3->1` 下降沿离开锁轮；触发临界区必须使用 `tools.AUTO_TRIGGER_LOCK`。
- 比赛区域流程中的规划队列统一存放动作矩阵 `action_matrix`，不要再向该队列混入完整 `ChallengePlanResult` 对象。
- 发现规划语义、动作矩阵、真实场地假设之间矛盾时，先指出问题；硬件动作相关逻辑要保守，宁可报错退出，不要静默跳过危险动作。
- 当前工作区可能有用户改动或未跟踪文件，不要回退无关内容。

## 通信协议

控制器当前为 TCP Server：

```text
IP:   192.168.2.199
Port: 5000
Mode: TCP Server
```

下位机当前仍兼容旧帧，但上位机主线应统一发送 V3。旧帧兼容关系：

```text
payload_len = 0x16：seq + ch[10]
payload_len = 0x18：seq + ch[10] + yaw_now
payload_len = 0x1A：seq + ch[10] + yaw_now + target_yaw，固件会把 cylinderSelect 置 0
payload_len = 0x1C：V3，seq + ch[10] + yaw_now + target_yaw + cylinderSelect
```

推荐持续发送 70Hz 左右。NET 输入超时阈值为 `500ms`，NET 从断开/SAFE 恢复时需要连续收到约 3 帧后才会重新接管；发送频率不要低于 10Hz。V3 帧为 34 字节：

```text
[0]      SOF1 = 0xA5
[1]      SOF2 = 0x5A
[2]      payload_len = 0x1C
[3]      frame_type = 0x01
[4:5]    seq, uint16 little-endian
[6:25]   ch[0] ~ ch[9], int16 little-endian
[26:27]  yaw_now_cdeg, int16 little-endian
[28:29]  target_yaw_cdeg, int16 little-endian
[30:31]  cylinderSelect, int16 little-endian
[32:33]  crc16_ccitt, uint16 little-endian
```

CRC16-CCITT：

```text
poly = 0x1021
init = 0xFFFF
范围 = bytes[2:32]，即 payload_len + frame_type + payload 到 cylinderSelect
```

`cylinderSelect` 是方块模式抽气泵选择字段：

```text
0 = 同时控制 PF2/PF3
1 = 只控制 PF2
2 = 只控制 PF3

小端字节：
0 -> 00 00
1 -> 01 00
2 -> 02 00
```

注意：按当前文档文字规则和旧版同一 CRC 算法，默认 V3 停止帧计算出的 CRC 为 `D4 0B`；新版 docx 示例尾部 `45 2A` 与该规则不一致，当前代码以文字规则为准。

下位机解析流程：

- W5500 按 TCP 字节流读取，内部做流式缓存、找帧头、长度校验、CRC 校验和字段解码；不要假设一次 `sendall()` 就等于下位机一次完整读取。
- `ch0~ch3` 在 NET 输入层会被限幅到 `[-992, 992]` 并做约 `20` 的死区。
- `ch4~ch8` 原样进入模式/边沿逻辑；`ch9` 用于吸盘旋转。
- NET 新鲜时优先 NET；NET 超时后若遥控器新鲜则回退 RC，否则进入 SAFE 默认通道。

安全默认通道：

```python
[0, 0, 0, 0, 1, 1, 1, 1, 1, 1]
```

通道语义：

- `ch0`：左右平移 `vy`。
- `ch1`：模式相关；升降模式下映射 ID5/ID6 小轮速度，weapon 模式下用于 4305 位置动作。
- `ch2`：前后移动 `vx`，正数前进，负数后退。
- `ch3`：仅 RC 模式映射旋转；NET 模式下忽略该通道，旋转由 `yaw_now_cdeg` 和 `target_yaw_cdeg` 在下位机 PID 闭环。
- `ch4`：当前模式下的边沿触发，`1 -> 3` 为打开/置位边沿，`3 -> 1` 为关闭/复位边沿。升降模式控制 PF0，方块模式按 `cylinderSelect` 控制 PF2/PF3 抽气泵，weapon 模式控制 PF1。实机已确认 PF1 下 `1->3` 为夹爪打开，`3->1` 为夹爪闭合。
- `ch5`：模式选择，`1=升降`，`2=方块/KFS`，`3=武器`。
- `ch6`：模式子功能。升降模式下 `1=自动上楼`，`2=底盘四轮锁角`，`3=自动下楼`；方块模式下用于姿态编号。
- `ch7`：自动动作触发边沿。升降模式下 `ch7:1->3` 按当前 `ch6` 触发上楼/下楼；方块模式下 `ch7=3` 作为一次性姿态触发。
- `ch8`：当前零点写入任务未启用；若后续启用，`ch8` 从非 `3` 到 `3` 会写达妙/4305 零点。主流程不要误触发。
- `ch9`：吸盘角度选择。业务层固定使用 `800/300/-300/-800` 四个值，按项目逆时针为正的角度体系分别对应 `0/90/180/-90deg`；不要使用未定义的中间区间。

推荐接口：

```python
sender.set_channel_values({4: 3, 5: 2})
move.set_channel_values(sender, channel_values={4: 3, 5: 2})
move.set_channel_values(sender, channel_values={4: 3, 5: 2}, cylinder_select=1)
move.set_motion_channels(sender, lateral_cmd=0, forward_cmd=300, rotation_cmd=0, des_yaw_i16=...)
sender.set_cylinder_select(0)  # PF2/PF3
sender.set_cylinder_select(1)  # PF2
sender.set_cylinder_select(2)  # PF3
```

- `set_channel_values(...)`：只修改传入通道，不复位其他通道。
- `set_motion_channels(...)`：只修改 `ch0/ch2/ch3/des_yaw_i16`。
- `cylinder_select`：只接受 `0/1/2`；不传时保持当前发送线程里的值，默认初始值为 `0`。
- `sender.set_safe_stop(...)`：恢复安全默认通道，只在明确需要全局复位时使用。

航向字段：

```text
target_yaw_cdeg 单位为 0.01deg
90deg  -> 9000
-90deg -> -9000
```

协议特殊值：

```text
target_yaw_cdeg = 0 表示关闭航向 PID
```

真实目标为 `0deg` 时应使用 `move.encode_target_yaw_i16(0.0)`，它会编码成 `1`，避免控制器当作关闭 PID。停止旋转控制时才显式发送 `target_yaw_cdeg=0`。

## 位姿后端与场地

雷达后端：

```python
module.init(lidar_type=1)  # odin
module.init(lidar_type=2)  # mid360
```

不要只调用 `position_backend.set_lidar_type()` 后直接跑动作；`position_resource.py` 和 `move.py` 都缓存了 `position_lib`，必须用 `module.configure_position_backend()` 或 `module.init()` 统一刷新。

场地半场：

```python
from lib2 import position_backend

position_backend.set_field_type(position_backend.FIELD_TYPE_RED)
position_backend.set_field_type(position_backend.FIELD_TYPE_BLUE)
```

红/蓝半场影响：

- `utils/race.py` 的 `pos_to_coord`
- `position_resource.build_stair_height_relation_matrix()`
- `position_odin.py` 的入口、入口前点、weapon 目标点
- `module.fetch_weapon()` 的接近方向、最终 yaw 和释放前 yaw

业务层读取坐标优先使用：

```python
module.get_stair_matrix()
module.get_stair_xy(stair_id)
module.get_stair_height_relation(stair_id, direction)
```

`position_resource.get_stair_matrix()` 按当前后端入口坐标和当前半场动态生成台阶矩阵。矩阵行格式：

```text
[台阶编号, 前方关系, +90deg方向关系, -90deg方向关系, x, y]
```

关系值：

```text
0 = 该方向不相邻/无衔接
1 = 该方向台阶比当前台阶高
2 = 该方向台阶比当前台阶低
```

不要使用 `STAIR_HEIGHT_RELATION_MATRIX[stair_id - 1]`，因为当前矩阵包含 `-1`、`13`、`15` 等非连续编号。

## 挑战赛地图

红场逻辑地图：

```text
y=2: 1:40  4:60  7:40  10:20
y=1: 2:20  5:40  8:60  11:40
y=0: 3:40  6:20  9:40  12:20
```

蓝场逻辑地图当前是 y 方向镜像：

```text
y=0: 1:40  4:60  7:40  10:20
y=1: 2:20  5:40  8:60  11:40
y=2: 3:40  6:20  9:40  12:20
```

`lib2/` 执行层方向编码按红/蓝半场语义解释：

```text
0 = 原地

红场：
1 = 90deg   / map y+
2 = 180deg  / map x-
3 = 0.01deg / map x+
4 = -90deg  / map y-

蓝场：
1 = 90deg   / map y-
2 = 0.01deg / map x+
3 = 180deg  / map x-
4 = -90deg  / map y+
```

`tools.direction_int_to_yaw_deg()` 映射：

```python
红场: {1: 90.0, 2: 180.0, 3: 0.01, 4: -90.0}
蓝场: {1: 90.0, 2: 0.01, 3: 180.0, 4: -90.0}
```

`tools.stair_id_to_direction()` 和 `position_resource.get_stair_height_relation()` 均按上述红/蓝语义工作。`race.pos_to_coord` 仍保持逻辑前/左网格，不直接表示真实地图 x/y；路径规划生成端的方向码同步另行处理。

## 坐标修正

当前现场打点约定：梅林台阶坐标以雷达左上角作为参考点；0/90/180/-90deg 的角度修正来自“左上角对准同一物理点后旋转到对应角度时，定位读数多出来的偏移”。因此 `tools.py` 中保存的是抵消量，即测量偏移的反号：

```python
RED_LIDAR_CORRECTION_BY_YAW = {
    0.01: (0.000, 0.000),
    90.00: (-0.050, -0.067),
    180.00: (0.025, -0.151),
    -90.00: (0.104, -0.081),
}
```

蓝场 `BLUE_LIDAR_CORRECTION_BY_YAW` 当前暂用同一组值，后续需要按蓝场实测值替换。业务入口使用：

```python
tools.deg_correction(target_deg, x, y)
```

修正角度必须是“移动到该坐标时车体/雷达实际保持的朝向”，不是到点后额外旋转到的展示角度。该修正只适用于坐标按 `0.01deg/0deg` 基准打点、但动作以其它朝向到点的场景；如果某个坐标本来就是按目标朝向实测的，不应再套对应角度修正。

`module.execute_action_row()` 当前只在梅林动作行内集中修正坐标：

- `grab_action=1` 的 KFS 吸取分支：`from/to` 坐标都按当前任务方向 `move_dir` 修正；`fetch_and_store_kfs()` 和吸取后回 `from_pos` 中心共用修正后的 `from_x/from_y`。
- 上楼分支：按当前任务方向 `move_dir` 修正 `from/to` 坐标。
- 下楼分支：按当前任务方向的反方向修正，映射为 `1<->4`、`2<->3`。
- 返回结果同时保留 `raw_from_x/raw_from_y/raw_to_x/raw_to_y` 和修正后的 `from_x/from_y/to_x/to_y`，便于现场判断误差。

九宫格/对抗区当前也在 `move.py` 的坐标出口处统一套 `180deg` 修正：`move_to_2rd_place()`、`enter_battlefield()`、`climb_R1_movement()` 读取 `position_odin.py` 的 red/blue 坐标后先调用 `tools.deg180_correction()`；`ultimate_test_script.py` 的二层箱子硬编码坐标也同样套 `180deg` 修正。注意：如果这些九宫格坐标本来就是机器面朝 `180deg` 时实测的，则该修正可能会造成二次偏移，应按现场打点来源决定是否保留。

挑战赛 KFS 数量配置：

```python
race.configure_competition_mode(1)  # R1=2, R2=2, Fake=1
race.configure_competition_mode(2)  # R1=3, R2=3, Fake=1
race.REQUIRED_R2_PICKUP_COUNT = 2   # 必须拿取数，与场上R2总数独立
```

`race.validate_kfs_layout(kfs)` 会校验格子编号、KFS 类型及当前模式数量。`race.plan_path()` 当前使用带代价的 Dijkstra：基础移动代价为 `1`，吸取基础代价为 `0`，转向代价默认为 `1`，移动到带 R1 的格子额外 `+0.5`；这些代价由 `PATH_*` 全局参数和 `configure_path_costs(...)` 调整。

场外预吸取规划只在 QR 前三位含 `2` 时触发：如第二位为 `2` 则硬优先选择2号；否则在1/3号候选中规划，两者同时存在时比较内部路径总代价，相同代价选1号。选中的格子在有效布局中被置 `0`，并将场上 R2 数和剩余必须抓取数各减 `1`。

## 动作矩阵

动作矩阵为 `n*5`：

```text
[from_pos, to_pos, move_dir, height_action, grab_action]
```

- `from_pos`：动作起点格编号。
- `to_pos`：动作目标格编号。
- `move_dir`：方向编码。
- `height_action`：`0=不用上下楼`，`1=需要上下楼`；上/下由执行层根据真实台阶矩阵判断。
- `grab_action`：`0=不抓取`，`1=R2 方块/KFS 抓取`。R1-KFS 按规则由 R1 移除，R2 经过 R1 格时必须输出 `0`。

`utils/challenge_lib.py::build_action_matrix_with_pre_entry_pickup()` 生成的挑战赛完整矩阵会包含入口和出口动作。以下方向码示例按红场写出；蓝场会按当前半场动态生成方向码。无场外吸取时，第一行是：

```python
[-1, 2, 1, 1, 0]
```

有场外吸取时，前两行是：

```python
[-1, target, 1, 0, 1]  # target 为 1/2/3，场外吸取
[-1, 2,      1, 1, 0]  # 入口上楼
```

最后根据规划出口追加：

```python
10 -> [10, 13, 1, 1, 0]
12 -> [12, 15, 1, 1, 0]
```

`module.execute_action_matrix(...)` 会逐行调用 `execute_action_row(...)`，并把下一行上下文传给当前行，用于 KFS 抓取后是否需要回中心的判断。每行统一返回 `implemented/completed/failed_step`；默认在首个 `completed=False` 的已实现动作处停止，矩阵结果会返回 `failed_row_index`、`failure_reason` 和 `executed_row_count`。仅调试穷举时才应显式传入 `stop_on_failed=False`。

`execute_action_row(...)` 在普通相邻性/方向校验前优先精确识别 `[-1,1,1,0,1]` 和 `[-1,3,1,0,1]`，直接调用 `module.side_suck(target_stair_id=1|3)`。只有这两种完整字段组合会启动侧吸；其他 `-1->1/3` 组合会进入常规校验并因非相邻而拒绝执行。`[-1,2,1,0,1]` 仍走普通方向抓取分支。

普通梅林行进入 `move_dir != 0` 分支后会先读取原始 `from/to` 台阶坐标，再按当前动作类型套角度修正。KFS 吸取和上楼按 `move_dir` 修正，下楼按 `move_dir` 反方向修正；修正前后的坐标都会写入行结果。

当前行的最终朝向默认取下一行 `from_pos -> to_pos` 的方向；如果下一行实际执行下楼（`height_action=1`、`grab_action=0` 且高低关系为 `2`），则使用该方向的反方向，使机器在进入 `descend()` 前已经按倒退下楼姿态对正。最后一行仍使用 `execute_action_matrix(final_direction=...)` 的传入值。

连续上下楼的回中心优化：

- 上楼：如果下一行同方向且同样是上楼，当前行跳过回到 `to_pose` 中心，`return_center_skip_reason="next_same_direction_higher_stair"`。
- 下楼：只有下一行也是上下楼、方向相同且同样是下楼时跳过回中心。
- 如果上一行因连续上楼跳过回中心，下一行上楼前的相对高台阶微调自动传 `high_stair_long_adjust=1`，即把高台阶前探从 `ch2=300,1s` 改为 `ch2=300,2s`。
- KFS 吸取分支如果下一行是对同一目标台阶上楼，会跳过吸取后的回中心；否则吸取成功后回到 `from_pos` 中心。

执行层当前只接入：

- 场外特殊行 `-1->1/-1->3`：调用 `module.side_suck(...)`。
- `grab_action=1`：调用 `module.fetch_and_store_kfs(...)`，必要时回到当前格中心。
- `height_action=1`：按高低关系调用 `climb(...)` 或 `descend(...)`。
- 出口行 `10->13`、`12->15`：按普通上下楼/离场动作处理。

当前不支持 `move_dir != 0` 且 `height_action=0` 且 `grab_action=0` 的相邻等高普通移动；如果后续地图允许这种情况，需要先补真实移动分支。

## 上下楼流程

升降模式：

```text
ch5 = 1
ch6 = 子功能选择
ch7 = 自动动作触发边沿
```

新版下位机协议：

```text
上楼/抬升：
ch5 = 1
ch6 = 1
ch7: 1 -> 3 -> 1

四轮锁角：
ch5 = 1
ch6 = 2

下楼：
ch5 = 1
ch6 = 3
ch7: 1 -> 3 -> 1
```

当前代码事实：

- `lib2/move.py::climb()` 当前显式预置 `ch5=1, ch6=1, ch7=1`，再通过 `ch7:1->3->1` 触发上楼。
- 上楼触发完成后固定等待 `DEFAULT_CLIMB_POST_TRIGGER_DELAY_SEC=3.7s`，再开始检测水平位移和高度条件。
- `lib2/move.py::descend()` 当前显式预置 `ch5=1, ch6=3, ch7=1`，触发阶段只更新 `ch7=3`，结束后恢复 `ch5=1, ch6=1, ch7=1`。
- `move.lock_wheel()` 预置 `ch5=1,ch6=2,ch7=1` 后触发 `ch7:1->3` 进入四轮锁角，并保持 `ch7=3`；`move.unlock_wheel()` 保持 `ch5=1,ch6=2` 并触发 `ch7:3->1` 离开锁轮。

## KFS 流程

KFS 模式：

```text
ch5 = 2
ch6 = 姿态编号
ch7 = 触发通道
ch4 = 吸盘边沿
cylinderSelect = 抽气泵选择
ch9 = 吸盘旋转
```

当前下位机方块模式姿态编号：

```text
0 = 初始姿态
1 = 吸取高处姿态
2 = 吸取低处姿态
3 = 过渡姿态
4 = 放置方块姿态
5 = 侧吸姿态
6 = 三层 KFS 放置姿态
```

方块模式下 `ch4` 通过 `1->3` / `3->1` 控制抽气泵边沿，实际控制哪个泵由 V3 字段 `cylinderSelect` 决定：

```text
cylinderSelect = 0：同时控制 PF2/PF3
cylinderSelect = 1：只控制 PF2
cylinderSelect = 2：只控制 PF3
```

吸盘旋转由 `ch9` 控制：

```text
ch9 > 700：吸盘朝向 0deg，业务固定值为 800
ch9 = 200~500：吸盘朝向 90deg，业务固定值为 300
ch9 = -500~-200：吸盘朝向 180deg，业务固定值为 -300
ch9 < -700：吸盘朝向 -90deg，业务固定值为 -800
```

业务代码只应发送 `800/300/-300/-800`，避免落入 `500~700`、`-700~-500`、`-200~200` 和边界值 `+/-700` 等未定义区间。新版姿态把“大臂姿态”和“吸盘角度”拆开配合使用；侧吸时使用 `ch5=2, ch6=5` 触发侧吸大臂姿态，三层放置姿态使用 `kfs.place_3rd_kfs_pose()` 触发 `ch5=2,ch6=6,ch7` 边沿，再根据目标方向选择对应的 `ch9` 固定值。

对应接口为 `kfs.sucker_0deg()`、`kfs.sucker_neg90deg()`、`kfs.sucker_180deg()`、`kfs.sucker_90deg()`；接口只更新 `ch9`，不切换 KFS 模式或触发大臂姿态。

也可调用 `kfs.sucker_to_direction(sender, direction)` 按项目统一方向编号设置吸盘：`1->0deg`、`2->90deg`、`3->-90deg`、`4->180deg`；其他输入会报错并终止程序。

抽气泵选择接口为 `kfs.sucker_select_both()`、`kfs.sucker_select_pf2()`、`kfs.sucker_select_pf3()`，分别设置 `cylinderSelect=0/1/2`；这些接口只选择 PF2/PF3，不触发 `ch4` 吸取或释放边沿。

也可调用 `kfs.sucker_select_cylinder(sender, cylinder_select)`，输入 `0/1/2` 时分派到对应气泵选择接口；省略 `cylinder_select` 时动态使用当前全局 `suck_count`（初始值为 `1`），最终值不在 `0/1/2` 时会报错并终止程序。

当前双头吸盘约定：PF2/PF3 的朝向相反，`cylinderSelect=1` 对应 PF2，`cylinderSelect=2` 对应 PF3。`suck_count` 表示当前要执行的吸取次序：

```text
suck_count = 1：吸盘头 0deg（ch9=800），选择 PF2
suck_count = 2：吸盘头 180deg（ch9=-300），选择 PF3
```

`kfs.kfs_grab_pose()` 在入口完成上述旋转和气缸选择，等待 `0.5s` 后再触发高/低吸取姿态。该方法只读取 `suck_count`，不在入口自增。

`kfs.kfs_suck_preparation(sender,count)` 用于按准备吸取数量预先打开气缸：

- `count=0`：不执行动作，直接返回。
- `count=1`：调用 `_select_sucker_cylinder(...PF2...)` 选择第一个气缸，再复用 `_release_kfs_suction_with_lock(...keep_suction_on=True)` 触发并保持吸气，成功后 `suck_count += 1`。
- `count=2`：调用 `sucker_select_both()` 选择双气缸，再复用 `_release_kfs_suction_with_lock(...keep_suction_on=True)` 触发并保持吸气，成功后 `suck_count += 2`。

侧吸大臂姿态接口为 `kfs.kfs_side_pose()`，按 `ch5=2, ch6=5, ch7:1->3` 触发，默认预置 `0.1s`、触发保持 `1.0s`，且不修改 `ch4`。这是时间保持，不是控制器到位 ACK。

`module.fetch_and_store_kfs(...)` 当前流程：

1. 根据 `stair_id + direction` 正向微调到吸取位置。
2. 根据高低关系选择高位/低位抓取姿态。
3. `kfs.kfs_grab_pose(...)` 根据 `suck_count` 设置吸盘方向和气缸，等待 `0.5s` 后触发抓取姿态。
4. `ch4:1->3` 吸取，并保持一段时间。
5. 复位 KFS 模式通道但保持 `ch4=3`。
6. 启动 `kfs.start_kfs_post_suction_thread(...)`。
7. 后续线程异步执行 pose3，默认等待 `3.0s` 确保过渡动作完成；然后 `suck_count=1` 时将吸盘转到 180deg，`suck_count=2` 时转到 0deg；旋转后等待 `0.5s`，执行 pose0 回初态，成功后 `suck_count += 1`并复位 `ch5/ch6/ch7`。全程保持 `ch4=3`；pose4 是放置姿态，不参与吸取后续流程。

后续线程只管理机械臂/吸盘相关通道，不执行底盘回中心。底盘回中心由 `module.execute_action_row()` 根据下一行上下文同步决定，避免线程并发写 `ch0/ch2/des_yaw_i16`。

`module.side_suck(...)` 当前流程：

1. 校验目标台阶只能为 `1/3`。
2. 按原 `target_stair_id + suck_count` 分支先旋转吸盘头：目标1时 `count1->90deg`、`count2->-90deg`；目标3时 `count1->-90deg`、`count2->90deg`。
3. 调用 `kfs.kfs_side_pose()` 切换 pose5，再按当前 `suck_count` 选择气缸：`1->PF2`、`2->PF3`。
4. 调用 `move.side_suck_movement(...)`：先以 `move_speed=350` 移动到 `-1` 台阶左右偏移 `0.5m` 的侧吸准备点，再以 `ch2=300` 前进 `1s` 做高位微调。红场目标 yaw 为 `90deg`，蓝场目标 yaw 为 `-90deg`，均通过四方向函数得到。该移动速度作为 `move_to_target()` 的平移向量幅值上限，按方向分配到 `ch0/ch2`，不是两个通道各给 `350`。
5. 调用 `set_kfs_suction(suction_on=True,pose_id=5)` 对已选气缸吸取，保持 `ch4=3`。
6. 调用 `move.side_suck_lateral_movement(...)` 做定时左右平移：目标1向左、目标3向右；默认 `ch0` 绝对值 `100`，持续 `4s`，对应 `move.DEFAULT_SIDE_SUCK_LATERAL_CMD_ABS` 和 `move.DEFAULT_SIDE_SUCK_LATERAL_DURATION_SEC`。
7. 横移完成后等待 `0.5s`，启动 `kfs.start_kfs_post_suction_thread()`，并同步将底盘移动回 `-1` 号台阶中心，回程航向沿用侧吸准备点 yaw。

侧吸收尾线程与普通 KFS 共用同一逻辑：pose3 -> 等待默认 `3.0s` -> 根据 `suck_count` 预旋转 -> 等待 `0.5s` -> pose0 -> `suck_count += 1`。收尾线程异步运行，底盘回 `-1` 不等待线程完成。

当前 KFS 未接入的组合业务：

- 普通高/低吸取已按 `suck_count` 自动选择 PF2/PF3 和 0/180deg；90/-90deg + pose5 侧吸已接入场外 `-1->1/-1->3` 特殊动作行。
- pose4 已明确为放置姿态，但尚未封装“进入放置姿态、选择气缸、释放吸盘、回初态”的完整放置流程。

## Weapon 流程

`module.fetch_weapon(...)` 从当前 `position_lib` 读取 `WEAPON_TARGETS`。红场默认（当前地图坐标系）：

- 接近点：`(weapon_x + 1.0, weapon_y)`
- 抓取 yaw：`0.01deg`
- 返回接近点 yaw：`0.01deg`
- 释放前旋转 yaw：`180deg`

蓝场默认（当前地图坐标系）：

- 接近点：`(weapon_x - 1.0, weapon_y)`
- 抓取 yaw：`180deg`
- 返回接近点 yaw：`180deg`
- 释放前旋转 yaw：`0.01deg`

当前流程：

1. 以 `weapon` 参考点移动到接近点。
2. 以 `weapon` 参考点移动到 weapon 目标点，默认 `stop_distance=0.01`。
3. 调用 `weapon.weapon_down()` 放下夹爪结构，并等待 `drop_before_grab_wait_sec=0.8s`。
4. 调用 `weapon.weapon_seize()`，通过 `ch4:3->1` 闭合夹爪，并复位模式通道。
5. 调用 `weapon.weapon_up()`，通过 `ch1:0->100->0` 触发抬起并复位模式通道。
6. 保持 `ch4=1` 的夹取状态，以 `robot` 参考点回到接近点。
7. 调用 `move.move_to_connection(...)` 预留衔接点移动；当前该方法是占位未实现，返回 `implemented=False` 时不会阻断 `fetch_weapon()`。
8. 原地旋转到释放前 yaw。
9. 返回时保持 `ch1=0,ch4=1,ch5=1`；夹取气缸维持闭合状态。

`fetch_weapon(..., move_to_approach_point=0)` 会跳过第 1 步接近点移动，直接从当前位置以 `weapon` 参考点移动到 weapon 目标点；默认 `1` 保持原流程。

释放/放下由调用方显式执行：

```python
move.reset_weapon_after_fetch(sender)
```

该接口现按顺序调用 `weapon.weapon_down()`、等待机械结构放下，再调用 `weapon.weapon_loose()` 执行 `ch4:1->3` 打开夹爪。

## 完整流程入口

当前区域级入口在 `lib2/compete_logic.py`：

1. `rigion_1(sender, position_runtime, odom_runtime, weapon_id=4, ...)`
   启动后台 QR 识别线程，确认 `challenge_lib.SCANNER_RUNNING_LOCK` 已被扫码线程持有，然后调用 `module.fetch_weapon(...)` 抓 weapon。抓取完成后 `move.lock_wheel()`，等待 QR 线程释放全局锁；拿到锁后 `move.unlock_wheel()`、`weapon.weapon_loose()` 松开夹爪并原地等待 `5s`。扫码线程以 `put_action_matrix_only=True` 向队列放入动作矩阵。
2. `rigion_2(sender, position_runtime, odom_runtime, action_matrix_queue, ...)`
   从动作矩阵队列中取出 `action_matrix`，直接调用 `module.execute_action_matrix(...)` 执行完整梅林矩阵。
3. `rigion_2_retry_plan(r1_count,r2_count,required_r2_pickup_count,qr_string,result_queue=None)`
   调用方通过参数传入当前 R1/R2 数量、还要抓取的 R2 数量和 12 位场地字符串；函数内部做整数转换和 QR payload 校验，调用完整规划入口（含前三位预处理），并把新的 `action_matrix` 推入队列。该函数不再执行终端 `input()` 录入。
4. `rigion_3(sender, position_runtime, odom_runtime, final_strategy=1, ...)`
   先调用 `move.enter_battlefield()` 依次到 `pre_entrance9` 和 `entrance9`，再按 `final_strategy=1` 执行 `module.high_score190()`，按 `final_strategy=2` 执行 `module.totally_win()`。

当前还没有一个根脚本把 `rigion_1 -> rigion_2 -> rigion_3` 自动串起来；需要在比赛脚本中显式创建并传递同一个动作矩阵队列。

红场赛后三点：

```text
0.01deg -> (5.437, -4.156)
0.01deg -> (8.378, -3.797)
90.0deg -> (7.537, 0.897)
```

蓝场赛后三点当前为镜像/占位：

```text
0.01deg -> (5.437, 4.156)
0.01deg -> (8.378, 3.797)
-90.0deg -> (7.537, -0.897)
```

## 未完成与风险

- 蓝场的入口、weapon、赛后三点坐标有占位/镜像性质，必须按实测场地复核。
- `position_mid360.py` 的半场拆分不如 odin 完整；当前主入口仍以 odin 为主。
- KFS、weapon、上下楼仍有大量按时间等待；没有控制器状态 ACK，硬件完成只能依赖时间或位姿条件。
- V3 帧格式和新版上下楼触发语义已接入；旧 `place_kfs()` 和 `move.control_kfs_pose()` 已删除，放置动作当前使用 `kfs.place_kfs_pose()`、`kfs.sucker_release_pose()`、`kfs.release_kfs()` 和 `kfs.place_3rd_kfs()` 等新接口组合。
- 方块模式 `cylinderSelect` 和四档 `ch9` 已有底层接口，普通双头吸取、场外 1/3 号侧吸和九宫格二/三层释放测试已接入；`compete_logic.rigion_3()` 已封装进入九宫格后的高分/大胜策略选择，但九宫格坐标仍需实测复核。
- 侧吸的 pose5 `1.0s`、横移默认 `ch0=100,4s`、横移后 `0.5s`、收尾 pose3 默认 `3.0s` 都是时间等待，没有机械臂到位 ACK；侧吸、横移持物和并发回 `-1` 尚未完整实机验证。
- RealSense D455 在小电脑上若日志显示 `USB=2.1`，即使插在标称 USB3 口也可能因实际链路降级导致 `Frame didn't arrive within 5000`；现场已确认恢复到 `USB=3.2` 后 `640x480@30` 彩色流可预热成功。后续排查以 `utils/process.py` 打印的 USB 模式和 `lsusb -t` 实际速率为准。
- `challenge_lib` 场外规划成功后会在当前进程内将 R2 布局数和剩余抓取数各减 `1`；当前约定一次流程只进行一次成功规划。如后续支持多次规划，需要增加显式重置或局部规划上下文。
- `execute_action_matrix()` 已在首个 `completed=False` 动作行停止并返回失败上下文；各主流入口会在矩阵失败后直接结束，不再继续赛后移动。当前仍无硬件 ACK，“成功”只能表示现有时间/位姿条件未报错。
- 锁轮协议使用 `ch5=1,ch6=2` 配合 `ch7` 边沿触发；当前实现按上升沿进入、下降沿离开处理，切换到其他模式后的锁轮保持行为仍需实机确认。
- 旧 `lib/` 仍使用 `payload_len=0x1A`、旧 IP 且无 `cylinderSelect`；`test/level_1.py` 仍导入该旧库。`catch.py` 仍有 weapon `ch4=-100` 时序，这些都不属于当前协议主线。
- 当前旋转关闭了位置保持，可能出现旋转漂移；这是当前代码事实，不是长期闭环方案。
- 如果后续允许相邻等高普通移动，需要补 `height_action=0, grab_action=0` 的真实移动分支。
- 如果调整 `race.pos_to_coord` 格子数量，必须同步 QR 长度、位置集合、出口行和台阶矩阵。
- 九宫格坐标在 `position_odin.py` 中仍有多处 `0.0` 占位；红场 `pre_entrance9/entrance9/column2` 已填入现场值但仍需实机复核。`ultimate_test_script.py` 的二层箱子测试目前写死红场原始坐标 `(-0.950, -4.46, 180deg)` 并套 `deg180_correction()`，蓝场或实测变更前不能直接视为通用流程。
- 大多数硬件动作只能通过实机确认；语法检查和规划穷举不能证明控制器业务层已经执行正确。

## README 更新要求

当用户要求“更新 README”或需要向下一个工作窗口交接时，至少同步：

- 已完成修改的文件路径、函数名、通道时序和最终复位状态。
- 尚未完成的组合动作、动作矩阵分支及建议继续修改的函数。
- 规划层、工具层、组合动作层、底层通道层的关系，以及当前真实入口脚本路径。
- 动作矩阵五列含义、方向编码、上下楼、KFS、weapon 等关键方法的职责和输入。
- 用户已确认的现场假设和协议补充，包括双头吸盘、四档 `ch9`、锁轮持续状态及不存在相邻等高普通移动。
- 已知风险、旧协议文件/脚本、仅做语法或干跑验证的部分，以及尚未实机验证的部分。
- 保留协作约定：优先复用已有封装，发现规划/协议/机械动作矛盾时先明确指出，不静默跳过可能危险的硬件动作。

## 已知验证

- 已读当前源码确认：`lib2/module.py`、`lib2/move.py`、`lib2/kfs.py`、`lib2/weapon.py`、`lib2/compete_logic.py`、`lib2/position_backend.py`、`lib2/position_resource.py`、`utils/race.py`、`utils/challenge_lib.py`、`utils/utils.py`。
- `R2H操作指南.docx` 的 KFS/ch9 内容相对滞后；当前 README 以 V3 帧、`cylinderSelect`、现场补充的 `ch6=5` 侧吸姿态、四档 `ch9` 角度及双头吸盘流程为准。
- 已对 `ultimate_test_script.py`、`d455.py`、`lib2/module.py`、`lib2/move.py`、`lib2/kfs.py`、`lib2/compete_logic.py`、`lib2/position_odin.py`、`lib2/position_mid360.py`、`lib2/position_resource.py`、`lib2/tools.py`、`utils/challenge_lib.py`、`utils/race.py`、`utils/process.py` 和 `utils/utils.py` 做过语法检查。
- 已用模拟 sender 检查双头吸盘的 `suck_count=1/2` 分派、后续旋转/回初态/计数时序，以及 `lock_wheel()` / `unlock_wheel()` 的通道组合。
- 已用模拟上下文检查 `ultimate_test_script.py` 的锁轮/解锁菜单分派、初始区域 `fetch_weapon -> lock_wheel` 顺序、上下楼/普通 KFS 行生成，以及侧吸测试的 `current_stair_id=-1`校验和 `[-1,1|3,1,0,1]` 分派。
- 已用模拟 sender/runtime 检查 `module.side_suck()` 四种目标/次数角度分派、气缸 -> 旋转 -> pose5 -> 吸取 -> 实时坐标横移 -> 等待 -> 收尾线程 -> 回 `-1` 的调用顺序。
- 已对 `lib2/kfs.py` 新增 `kfs_suck_preparation()` 做语法检查；该方法复用已有 `_release_kfs_suction_with_lock(...keep_suction_on=True)` 打开并保持吸气，尚未做实机气缸联调。
- 已将 `lib2/compete_logic.py::rigion_2_retry_plan(...)` 改为参数输入并做语法检查；调用方需要显式传入 `r1_count`、`r2_count`、`required_r2_pickup_count` 和 `qr_string`，不再等待终端录入。
- 已将 `utils/process.py` 的 RealSense 打开流程改为设备枚举、候选流 fallback 和 warmup 重试；已用现场日志确认 `USB=2.1` 时 D455 可能首帧超时，`USB=3.2` 时 `640x480@30` 可预热成功。
- 已新增 `d455.py` 独立后台线程式二维码测试脚本，用于区分 RealSense 取流、OpenCV 窗口和主流程扫码问题。
- 已检查比赛模式 1/2、KFS 数量校验、场外 1/2/3 号候选规划，以及“无场外吸取时入口行在第一行，有场外吸取时在第二行”的矩阵顺序。
- 已用模拟行结果检查 `execute_action_matrix()` 的全部成功、首行失败即停止、`stop_on_failed=False` 继续执行三种路径；失败行索引、原因和已执行行数均符合预期。
- 已对本轮涉及的 `ultimate_test_script.py`、`lib2/tools.py`、`lib2/module.py`、`lib2/move.py` 做过语法检查；顶层恢复初态菜单、角度坐标修正、梅林行执行修正和九宫格 `180deg` 修正尚需实机复核。
- 上述 KFS 双头旋转、普通/侧吸、`kfs_suck_preparation()`、锁轮/解锁、完整梅林和九宫格放置/释放相关动作尚未在当前修改后做完整实机联调。
- `utils/race.py` 的 matplotlib 可视化有 SVG 兜底；matplotlib 不可用时会生成 `race_visualization.svg`。
- 挑战赛完整矩阵已验证自动插入入口行 `[-1,2,1,1,0]`，并在最后到 `10/12` 时追加 `[10,13,1,1,0]` / `[12,15,1,1,0]`。
- README 只描述当前代码状态；实机验证状态需要以现场日志和硬件反馈为准。
