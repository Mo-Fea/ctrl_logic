# R2_H 上位机控制工程：实现思路与方法记录

这是 R2_H 机器人的上位机控制与比赛编排工程。它通过 ROS 2 获取定位/图像信息，通过 TCP 持续向下位机发送控制帧，完成武器抓取、梅林（KFS）路径规划与执行、九宫格策略等动作。

本文以一次比赛研发中的实际实现为样本，记录项目如何拆分定位、感知、规划和执行，以及其中一些或许值得复盘的工程取舍。具体参数和坐标受场地、机械和下位机固件影响，适合结合代码与实机情况理解。

## 1. 先看什么

当前主线只有两个目录：

- `lib2/`：硬件控制、定位、组合动作和比赛区域编排。
- `utils/`：二维码取流/识别与梅林路径规划。

根目录的主要入口：

| 文件 | 用途 | 是否可直接操作实机 |
| --- | --- | --- |
| `competition_script.py` | 当前比赛主菜单与区域重试入口 | 是，正式入口 |
| `ultimate_test_script.py` | 分项动作、梅林和九宫格调试菜单 | 是，调试入口 |
| `d455.py` | RealSense 相机二维码识别调试 | 会访问相机 |
| `measure.py`、`measure_lidar.py`、`measure_weapon.py` | Odin 定位与坐标打点 | 会连接 ROS |
| `race.py` | 早期/独立规划调试入口 | 仅在确认用途后使用 |

旧的 `lib/`、历史脚本和缓存文件不属于当前主线。代码中的 `rigion` 是历史拼写，阅读调用链时可将其理解为 `region`。

## 2. 系统全貌

```text
ROS 2 定位标志、/tf 或 odom ─┐
RealSense D435i/D455 彩色图 ─┼─> 定位/二维码/规划
                              │
competition_script.py ─> lib2/compete_logic.py ─> lib2/module.py
                              │                         │
                              │                         ├─ move.py：底盘、上下楼、锁轮
                              │                         ├─ kfs.py：吸盘、KFS 姿态
                              │                         └─ weapon.py：武器机构
                              │
二维码 ─> utils/process.py ─> utils/challenge_lib.py ─> utils/race.py
                                      │
                                      └─ 动作矩阵 queue

lib2/tools.py::frame_thread ─> TCP V3 控制帧 ─> 下位机
```

控制代码不应直接调用 `socket.sendall()`。`frame_thread` 是唯一持续发帧者，业务动作只能通过 `sender.set_*()` 或 `move.set_*()` 修改它维护的状态。

## 3. 运行环境与外部依赖

工程运行于安装 ROS 2 的 Linux 主机，当前没有锁定的 `requirements.txt`。部署新机器时，至少需要核对：

- Python 3、`numpy`、OpenCV（`cv2`）。
- ROS 2 Python 环境：`rclpy`、`std_msgs`、`sensor_msgs`；运行前需 source 对应 ROS/工作区环境。
- Odin 定位时应提供 `/tf`、`/odin1/flag1`；二维码图像源为 Odin 时应提供 `/odin1/image/undistorted`。
- RealSense 扫码时需要 `pyrealsense2`、D435i/D455 彩色相机和稳定的 USB 3 链路。
- 下位机网络可达：默认地址 `192.168.2.199:5000`（定义于 `lib2/tools.py`）。

在新电脑或新场地复现时，可以先执行 `python3 -m py_compile competition_script.py lib2/*.py utils/*.py`，再分别确认 ROS topic、雷达坐标和 TCP 连通性。代码可导入只说明软件环境大致齐备，并不代表真实机器已经适合运行。

## 4. 启动顺序与定位初始化的设计

正式入口：

```bash
python3 competition_script.py
```

启动后的顺序体现了本项目对“配置先于坐标资源”的处理：

1. 选择红/蓝场、赛制、最终策略，以及 **odom 模式下雷达的启动点**。
2. `module.init()` 连接下位机，启动控制帧线程和定位模式标志监听，并执行机器初始姿态动作。
3. 在菜单内完成本次流程的参数输入；此时不会启动坐标资源线程。
4. 用户输入 `1` 确认执行后，程序才等待 `/odin1/flag1`：`True` 表示重定位，`False` 表示 odom。
5. 如果为 odom，先将第 1 步保存的 **雷达坐标** 写入定位后端，再启动 position/odom 资源线程。
6. 最后进入所选区域流程。

这样做是为了避免 odom 首帧先按默认原点建立坐标、随后才切换到第二启动点。把“确定 odom 起点”放在“启动坐标资源”之前，能让下游模块从一开始看到一致的参考系。

`/odin1/flag1` 一直没有消息时，流程会等待。这个等待点把“定位模式尚未确认”显式暴露给操作者，而不是让控制逻辑带着不确定的定位状态继续运行。

## 5. 比赛流程与二维码

`competition_script.py` 的比赛逻辑分为三块：

- 区域 1（武林）：抓取 weapon，同时等待二维码规划完成。
- 区域 2（梅林）：从队列取出动作矩阵并逐行执行。
- 区域 3（九宫格）：按“190 分”或“大胜”策略执行放置/最终动作。

挑战赛可单独执行武林或九宫格；对抗赛完整流程会串联区域 1、2、3。区域重试会让操作者重新输入当前 KFS 数量或梅林二维码状态，并重新规划。

二维码线程只在本次比赛参数确定后启动。启动校验包含：

1. 能打开图像源；
2. 能取得有效 BGR 彩色首帧；
3. 能完成至少一轮识别调用。

任一项失败或 30 秒超时，程序会退出，而不是带着“没有扫码能力”的状态继续执行区域 1。这是本项目对关键感知依赖采用的 fail-fast 处理。当前正式流程固定使用 RealSense 图像源；相机调试可参考 `d455.py`，Odin 图像源的支持位于 `utils/process.py`。

二维码 payload 为 12 位，字符集为 `0/1/2/3`。识别稳定后，`utils/challenge_lib.py` 将结果转为动作矩阵并放入 `lib2/compete_logic.py` 的 `ACTION_MATRIX_QUEUE`；该队列的约定是只传递 `action_matrix`，以减少区域编排层对规划内部对象的依赖。

## 6. 梅林规划与动作矩阵

规划核心在 `utils/race.py`：它维护逻辑网格、红蓝场编号映射、KFS 数量校验和带转向代价的路径规划。对外入口在 `utils/challenge_lib.py`。

动作矩阵每一行是：

```text
[from_pos, to_pos, move_dir, height_action, grab_action]
```

- `from_pos` / `to_pos`：逻辑台阶编号。
- `move_dir`：统一方向编码，`1=90°`、`2=180°`、`3≈0°`、`4=-90°`。
- `height_action`：`1` 表示该行需要执行上下楼，具体上/下由当前台阶高度关系决定。
- `grab_action`：`1` 表示执行 R2 KFS 吸取。

完整矩阵会补齐入口/出口动作。执行入口为 `lib2/module.py::execute_action_matrix()`，它逐行解释矩阵并在失败时默认停止。这个中间表示把“规划得到什么”与“机构如何执行”分开；若调整路径或规则，建议同时核对 `utils/race.py` 的方向、`lib2/tools.py` 的方向转换、`position_resource.py` 的台阶关系和 `module.py` 的执行语义。

## 7. 定位、坐标与红蓝场

`lib2/position_backend.py` 统一选择两个维度：

- 雷达后端：`1=odin`（`position_odin.py`，主要使用 `/tf`），`2=mid360`（`position_mid360.py`，主要使用里程计 topic）。
- 场地：`1=red/right`，`2=blue/left`。

切换雷达后端时，本项目通过 `module.init(lidar_type=...)` 或 `module.configure_position_backend(...)` 统一刷新；仅修改全局变量会遗漏 `move.py` 与 `position_resource.py` 已缓存的后端模块。

红蓝场的差异由定位/坐标层处理：蓝场使用镜像后的逻辑坐标与 yaw，动作矩阵方向码本身不再按红蓝场重复镜像。这种集中处理减少了执行层分支，但新增坐标或改动方向时仍需要在两侧场地实测验证。

Odom 选项中的“第一/第二启动点”是 **雷达在逻辑地图中的初始坐标**，不是机器人几何中心。雷达到车体中心的固定外参由定位后端换算；当前流程不额外叠加相对旋转。

现场重新打点可使用 `measure.py`、`measure_lidar.py`、`measure_weapon.py`，并明确指定 `--field red|blue`。坐标、weapon 目标和台阶关系属于高风险配置，通常应配合低速、空载实测后再投入完整流程。

## 8. 控制协议与硬件约束

`lib2/tools.py` 持续发送 V3 TCP 帧（约 70 Hz）。帧包含 `ch0~ch9`、当前/目标 yaw 和 `cylinderSelect`；下位机按 TCP 字节流解析，不能假设一次发送就是一次完整接收。

常用语义：

| 字段 | 作用 |
| --- | --- |
| `ch0` / `ch2` | 横移 / 前后底盘运动 |
| `ch4` | 机构边沿触发；常见开/置位为 `1 -> 3`，关/复位为 `3 -> 1` |
| `ch5` | 模式：升降、KFS、weapon |
| `ch6` / `ch7` | 子功能与自动动作触发 |
| `ch9` | 吸盘朝向，业务层固定使用 `800/300/-300/-800` |
| `cylinderSelect` | KFS 吸盘选择：`0=双`、`1=PF2`、`2=PF3` |

目标 yaw 的 `0` 是“关闭航向 PID”的协议特殊值。若真实目标就是 0°，应通过 `move.encode_target_yaw_i16(0.0)` 编码，不能直接裸写 0。

上下楼、KFS 姿态/吸取、weapon 和锁轮都依赖模式切换及边沿触发。项目以 `tools.AUTO_TRIGGER_LOCK` 保护这些临界区，避免多个线程交错写入触发通道。上楼触发后目前会等待 4 秒再检查后续状态；这类时间参数来自当时的实机条件，迁移时值得重新验证。

## 9. 模块边界与可复用的做法

| 需求 | 优先查看/修改 |
| --- | --- |
| TCP 帧、连接、定位标志、清理 | `lib2/tools.py` |
| 雷达后端、红蓝场、坐标变换 | `lib2/position_*.py`、`lib2/position_backend.py`、`lib2/position_resource.py` |
| 底盘移动、旋转、上下楼、锁轮 | `lib2/move.py` |
| 吸盘、姿态、KFS 数量状态 | `lib2/kfs.py` |
| weapon 抓取/释放 | `lib2/weapon.py`、`lib2/module.py` |
| 组合动作与动作矩阵执行 | `lib2/module.py` |
| 区域 1/2/3 串联、队列与重试 | `lib2/compete_logic.py` |
| 相机、二维码检测 | `utils/process.py` |
| QR 到规划结果/动作矩阵 | `utils/challenge_lib.py` |
| 梅林规则、地图、规划代价 | `utils/race.py` |

从这套实现中，可以看到几项较容易迁移到其他机器人项目的做法：

- **单一发帧入口**：业务动作只更新自己负责的通道状态，由后台线程统一发帧，避免不同模块各自发送完整控制帧。
- **显式的失败边界**：二维码首帧、规划和动作执行都返回或抛出明确状态；调用链可以据此停止，而不把异常伪装成成功。
- **规划与执行解耦**：用五列动作矩阵连接路径规划和机械动作，便于先检查规划输出，再考虑执行细节。
- **对可变环境的延后初始化**：红蓝场、odom 起点和定位模式先确认，再建立坐标资源，减少默认值在启动阶段的影响。
- **对共享触发的串行保护**：模式设置和边沿触发使用同一把锁，减少多线程下“状态写对了但触发给错对象”的问题。

这些做法也有边界：TCP 协议仍需和下位机固件共同确认；场地坐标、台阶映射与时间参数高度依赖实测；结束运行时应走现有清理流程（见 `tools.handle_ctrl_c()`），避免遗留发送线程、ROS spin 线程或吸气状态。

## 10. 可按层次阅读和验证

1. 先阅读本 README、`competition_script.py`、`lib2/module.py` 和 `lib2/compete_logic.py`，再从入口沿调用链查看各模块。
2. 在不驱动机构的条件下，检查 Python/ROS 导入、topic 可见性、相机首帧和 TCP 连通性。
3. 若要验证动作，可按“底盘 → 机构 → 单个组合动作 → 区域”的层次使用 `ultimate_test_script.py` 逐步进行。
4. 对场地相关问题，重新确认红蓝场、weapon 点、梅林台阶和 odom 启动点，并记录测量日期、场地条件与误差。
5. 每次较大改动保留变更原因、涉及的坐标/参数、静态检查、空载测试和实机结果，会让后续复盘更容易。

本 README 只是对当前实现的结构与取舍的整理，不替代机械限位、下位机固件说明和现场安全规范。若代码描述与实机观测不一致，宜先停机核对，再判断是参数、实现还是环境假设需要调整。
