# NERO Neo 遥操作系统

本项目为 AgileX NERO 七轴机械臂提供基于 PICO Neo 3 的双臂遥操作与
LeRobot v3 数据采集能力。系统将 OpenXR 手柄位姿映射为末端笛卡尔目标，
使用 Pinocchio 求解速度级 IK，再通过 NERO CPV 接口发送受保护的关节指令。

> **安全警告：** 本软件会直接控制真实机械臂。实机前必须低速单臂验证，
> 保证急停可触达，并检查 Home、关节限位和周围人员。执行前请阅读
> [安全说明](docs/SAFETY.md)。

## 主要功能

- PICO Neo 3 Unity/OpenXR 客户端与低开销二进制 UDP 传输
- 单臂、双臂离合式笛卡尔遥操作
- Pinocchio 微分 IK、位姿滤波、有限领先和 CPV 安全保护
- 模拟量夹爪控制与可选向下接触力保护
- SocketCAN 自动绑定和双臂受控回位
- 三相机、双机械臂 LeRobot v3 数采与断点续采

## 系统架构

```text
PICO 手柄（OpenXR；UDP 发送目标 60 Hz）
        | UDP :50150
        v
PICO 输入解析 + 坐标系映射
        | 末端笛卡尔目标
        v
Pinocchio 速度 IK + 安全保护
        | 7 关节目标，30-40 Hz
        v
NERO CPV -> SocketCAN -> 左右机械臂

CAN 反馈 + 三路相机 + 实际执行指令 -> LeRobot v3 数据集
```

完整数据流和控制链见[架构说明](docs/ARCHITECTURE.md)；需要继续维护代码时，
从[核心代码与调用链](docs/CODE_GUIDE.zh-CN.md)开始阅读。

## 目录结构

| 目录 | 用途 |
| --- | --- |
| `src/nero_neo_teleop/` | 主机端 Python 包 |
| `pico_client/` | Unity/OpenXR PICO 应用 |
| `scripts/control/` | 回位和遥操入口 |
| `scripts/recording/` | LeRobot 托管数采入口 |
| `scripts/can/` | gs_usb 和 SocketCAN 配置 |
| `scripts/pico/` | APK 构建、安装、输入检查 |
| `tests/` | 映射、控制器、回位和数采测试 |
| `artifacts/` | 本机 APK、日志等，不进入 Git |

交接文档：

- [核心代码与调用链](docs/CODE_GUIDE.zh-CN.md)：入口、进程、核心类和逐 tick 执行顺序。
- [运行入口](docs/RUNTIME.md)：日常命令和 Python 包职责。
- [交接清单](docs/HANDOFF.zh-CN.md)：外部依赖、本机配置和验证步骤。
- [安全说明](docs/SAFETY.md)：实机操作边界。

## 环境要求

- Linux、SocketCAN 和两个 `gs_usb` CAN 适配器
- Python 3.11+、NumPy、Pinocchio 和 NERO SDK（`pyAgxArm`）
- 已配置的 `nero_ws`，提供 `nero_vla`、NERO URDF 和 SDK 依赖
- PICO Neo 3、Unity 6 Android 构建环境和 ADB
- 数采需要三路 V4L2 相机及 LeRobot Python 环境

机械臂 SDK、LeRobot 和 PICO Unity OpenXR SDK 均为外部依赖，本仓库不复制
这些项目。

## 安装配置

```bash
git clone https://github.com/ssyly4/PICO-NEO3-AGILE-ARM-TELEOP.git nero_neo_teleop
cd nero_neo_teleop
cp .env.example .env
# 修改 .env：SDK 路径、CAN USB 路径、相机和 PICO 主机地址。
python3 -m pip install -e .
```

此命令只安装本仓库的 Python 包，不会安装 NERO SDK、Pinocchio、LeRobot 环境或
PICO OpenXR 本地包。实机交接前按[交接清单](docs/HANDOFF.zh-CN.md)逐项核对。

从 [PICO 官方开发者网站](https://developer.picoxr.com/zh/document/unity-openxr/)
下载 PICO Unity OpenXR SDK，并将其包目录放到：

```text
pico_client/LocalPackages/com.unity.xr.openxr.picoxr/
```

该 SDK 的上游许可证没有授予本仓库再分发权限，因此此目录不会进入 Git。

构建安装 PICO 客户端：

```bash
./scripts/pico/build.sh
./scripts/pico/install_and_launch.sh
./scripts/pico/check_input.sh
```

## 实机操作

先只预览回位轨迹，确认输出无误后再执行：

```bash
./scripts/control/run_dual_home.sh
./scripts/control/run_dual_home.sh --execute
```

启动单臂或双臂遥操：

```bash
./scripts/control/run_servo_v3_experiment.sh --duration 120 --execute
./scripts/control/run_dual_servo_v3_experiment.sh --duration 120 --execute
```

单臂入口默认使用配置的右臂；`--can-port` 必须是已配置的左臂或右臂接口，
并决定对应的 Home 和 PICO 手柄。执行前应单独预览 Home 目标。

按住 **Grip** 进入跟随，松开后机械臂保持并重新锚定；**Trigger** 控制夹爪。

## 数据采集

```bash
./scripts/recording/run_recording.sh \
  --task "任务自然语言描述" \
  --dataset nero_demo_v1 \
  --episodes 50 \
  --auto-stop off

# 确认预览后再执行
./scripts/recording/run_recording.sh \
  --task "任务自然语言描述" \
  --dataset nero_demo_v1 \
  --episodes 50 \
  --auto-stop off \
  --execute
```

所有任务共用这一个入口。任务文本、数据集名称、episode 数、action 来源、最长时长和
自动停止方式均通过命令行或 `NERO_RECORD_*` 环境变量配置；不加 `--execute` 只打印
最终配置。执行时按 Enter 准备，检测到运动后开始录制，每集结束后选择保存或丢弃。

## 当前边界

这是在特定双 NERO 平台上验证的研究原型，不是认证安全系统。Home、CAN 拓扑、
URDF、相机和接触力阈值均与硬件有关，换设备后必须重新标定。

本项目原创代码采用 Apache-2.0 许可证；外部 PICO SDK 遵循其上游许可证且不随本仓库
分发。

## 代码架构详解

### 1. 输入到机械臂的完整链路

```text
PICO Neo 3 OpenXR
  -> Unity 客户端读取头显/手柄状态
  -> 二进制 UDP :50150
  -> pico_input 解包并保存最新快照
  -> pose_mapper 将手柄位姿转换成目标
  -> Servo v3 根据 Grip 离合状态生成笛卡尔增量
  -> Pinocchio FK/Jacobian 与速度级 IK 求七关节目标
  -> 关节步长、限位、过期输入和接触力保护
  -> pyAgxArm CPV position API
  -> SocketCAN -> NERO 机械臂
```

遥操主循环通常按 30 到 40 Hz 运行；PICO 输入源可能以更高频率发送。每次循环使用
带时间戳的最新有效输入，网络过期时进入 hold，而不是重放旧位姿。

### 2. Python 包的职责

| 路径 | 主要文件 | 职责 |
|---|---|---|
| `src/nero_neo_teleop/pico/` | `pico_input.py` | 解析 UDP、校验序号/时间戳、提供左右手快照 |
|  | `udp_fanout.py` | 将 PICO 输入分发给单臂或双臂控制进程 |
|  | `pose_mapper.py` | 坐标变换、平移/姿态增益和左右镜像 |
| `src/nero_neo_teleop/control/` | `servo_v3_controller.py` | Servo v3 状态机、离合、hold、重锚定和命令节流 |
|  | `servo_v3_core.py` | 目标差分、姿态误差、速度/加速度限制和 IK 输入 |
|  | `gripper.py` | Trigger 到夹爪宽度的映射和限速 |
|  | `contact_force_guard.py` | 可选接触力保护，检测下探受力 |
| `src/nero_neo_teleop/robot/` | `nero_io.py` | SDK 对象、反馈快照和 CPV/夹爪通信封装 |
|  | `dual_home.py` | 双臂反馈、Home 执行授权和误差检查 |
|  | `single_home.py` | 单臂 Home 流程 |
|  | `home_config.py` | 左右 Home、镜像关系和硬件参数 |
| `src/nero_neo_teleop/recording/` | `action_command_stream.py` | 保存实际发出的 command 和时间戳 |
|  | `bimanual_lerobot_recorder.py` | 30 Hz 采集 CAN、三路视频和 command，写 LeRobot v3 |
| `src/nero_neo_teleop/runtime.py` | - | 环境变量、路径和运行时初始化 |
| `src/nero_neo_teleop/diagnostics/` | `pico_input_probe.py` | 只读检查 PICO 左右手数据是否持续到达 |

IK 的机器人模型和底层 SDK 依赖 `nero_ws` 提供的环境；本仓库不复制一套手写机械臂
模型。详见 `/home/dev/nero_ws/README.md`。

### 3. Shell 入口职责

| 路径 | 功能 |
|---|---|
| `scripts/pico/build.sh` | 使用 Unity 构建 PICO Android 客户端 |
| `scripts/pico/install_and_launch.sh` | 通过 ADB 安装并启动客户端 |
| `scripts/pico/check_input.sh` | 查看 PICO UDP 包和左右手追踪状态 |
| `scripts/can/ensure_can_interface.sh` | 按 USB 拓扑恢复 gs_usb、创建接口并设置 bitrate |
| `scripts/can/reset_gs_usb_adapter.sh` | 重置卡住的 gs_usb 适配器 |
| `scripts/control/run_dual_home.sh` | 双臂 Home 预览/执行，执行需要授权 |
| `scripts/control/run_servo_v3_experiment.sh` | 单臂 Servo v3 遥操 |
| `scripts/control/run_dual_servo_v3_experiment.sh` | 双臂 Servo v3 遥操 |
| `scripts/recording/run_recording.sh` | 通用双臂遥操、三相机采集、episode 保存/丢弃和回位 |
| `scripts/check.sh` | 运行仓库级检查和测试 |

### 4. 相机、反馈和数据边界

相机应通过 V4L2 稳定路径传入，不能把 `/dev/videoN` 编号当成永久身份。当前数采配置
约定为世界相机 video6、左腕相机 video2、右腕相机 video4；启动脚本会打印最终设备，
插拔后必须以打印结果和现场画面为准。

数据不写入本 Git 仓库，正式数据根目录为：

```text
/home/dev/nero_data/raw/
```

`artifacts/` 只保存 APK、运行日志和诊断产物。训练转换、筛选、划分和 checkpoint
应放到 `nero_data` 或训练服务器。

### 5. 一次运行的顺序

```bash
cd /home/dev/nero_neo_teleop
./scripts/pico/check_input.sh
./scripts/control/run_dual_home.sh
./scripts/control/run_dual_home.sh --execute
./scripts/control/run_dual_servo_v3_experiment.sh --duration 30 --execute
```

数采时先检查三路相机、两路 CAN 和 PICO 输入，再运行：

```bash
./scripts/recording/run_recording.sh --task "任务描述" --dataset nero_demo_v1 --episodes 50 --execute
```

录制频率是 30 Hz。每帧应包含三路图像、左右臂七关节反馈、夹爪反馈、时间戳以及实际
发送的 `controller_command`。异常停止时先让控制器 hold，再由录制器决定保存或丢弃。

### 6. 外部依赖

```text
本仓库
  ├─ PICO Unity/OpenXR SDK（本地构建，不随仓库分发）
  ├─ /home/dev/nero_ws/src/pyAgxArm（NERO SDK）
  ├─ /home/dev/nero_ws/src/piper_ros（URDF/ROS 描述）
  ├─ Pinocchio / NumPy / Python 环境
  └─ /home/dev/lerobot_ws/lerobot（LeRobot 工具）
```

改变 USB 拓扑、Home 姿态、相机角色、夹爪标定或保护参数后，必须重新进行单臂低速
验证。这个仓库是研究原型，不是认证安全系统。
