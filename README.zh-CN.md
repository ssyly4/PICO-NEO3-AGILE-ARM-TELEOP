# NERO Neo 遥操作系统

简体中文 | [English](README.md)

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
PICO 手柄（OpenXR，72 Hz）
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

完整数据流和控制链见[架构说明](docs/ARCHITECTURE.md)。

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
git clone <repository-url> nero_neo_teleop
cd nero_neo_teleop
cp .env.example .env
# 修改 .env：SDK 路径、CAN USB 路径、相机和 PICO 主机地址。
python3 -m pip install -e .
```

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

按住 **Grip** 进入跟随，松开后机械臂保持并重新锚定；**Trigger** 控制夹爪。

## 数据采集

```bash
./scripts/recording/run_bimanual_record.sh --workflow fullflow
```

支持 `custom`、`fullflow`、`stage1`、`stage23`。按 Enter 进入准备，检测到
运动后才开始录制；脚本按工作流条件结束本集、双臂回位，再询问保存或丢弃。
数据写入仓库外的 `NERO_BIMANUAL_DATA_DIR`。

## 当前边界

这是在特定双 NERO 平台上验证的研究原型，不是认证安全系统。Home、CAN 拓扑、
URDF、相机和接触力阈值均与硬件有关，换设备后必须重新标定。

本项目原创代码采用 MIT 许可证；外部 PICO SDK 遵循其上游许可证且不随本仓库
分发。
