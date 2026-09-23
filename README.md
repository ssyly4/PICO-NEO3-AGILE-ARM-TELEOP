# NERO Neo 3 双臂遥操与数据采集

本仓库负责两件事：

1. 将 PICO Neo 3 OpenXR 手柄位姿转换为 NERO 七轴机械臂 CPV 指令。
2. 以 30 Hz 录制三路相机、双臂状态和已发送控制指令，生成 LeRobot v3 数据集。

训练、数据转换和 VLA 策略执行不在本仓库。代码调用关系见
[系统架构](docs/ARCHITECTURE.md)，现场问题见[故障排查](docs/TROUBLESHOOTING.md)。

> 本软件会直接控制真实机械臂。急停必须可触达，Home 和左右 CAN 角色必须先预览核对，首次运行必须短时、低速、无负载。

## 仓库边界

| 路径 | 内容 |
|---|---|
| `pico_client/` | Unity/OpenXR Android 客户端 |
| `src/nero_neo_teleop/pico/` | UDP 解包、坐标映射和双臂分发 |
| `src/nero_neo_teleop/control/` | 低通滤波、Pinocchio IK、状态机与 CPV 控制 |
| `src/nero_neo_teleop/robot/` | CAN/SDK 边界、单臂与双臂 Home |
| `src/nero_neo_teleop/recording/` | 三相机双臂 LeRobot v3 录制 |
| `scripts/` | 现场操作的统一入口 |
| `artifacts/` | APK 和运行日志，不保存数据集 |

原始数据应保存到仓库外的 `/home/dev/nero_data`。NERO SDK、URDF、Pinocchio、LeRobot 和 PICO OpenXR SDK 是外部依赖，不随本仓库分发。

## 1. 准备环境

已验证的组合为 Debian、Python 3.13 遥操环境、Python 3.12 LeRobot 环境、Unity `6000.0.80f1`、LeRobot `0.5.2` 和 PICO Neo 3。

```bash
git clone git@github.com:ssyly4/PICO-NEO3-AGILE-ARM-TELEOP.git nero_neo_teleop
cd nero_neo_teleop
cp .env.example .env
```

需要两个已存在的 Python 解释器：

- `NERO_TELEOP_PYTHON`：可导入 `numpy`、`pinocchio`、`pyAgxArm`、`trac_ik`、`nero_vla`。
- `NERO_LEROBOT_PYTHON`：可导入 `lerobot`、`pyarrow`、`cv2`、`nero_vla`。

先在 `.env` 中填写两个解释器路径，再将本仓库安装到两个环境：

```bash
source .env
"$NERO_TELEOP_PYTHON" -m pip install -e .
"$NERO_LEROBOT_PYTHON" -m pip install -e .
```

PICO Unity OpenXR SDK 需从 PICO 官方获取并放到：

```text
pico_client/LocalPackages/com.unity.xr.openxr.picoxr/
```

## 2. 配置 `.env`

至少核对以下项目：

```text
NERO_WS                    NERO 外部工作区
NERO_TELEOP_PYTHON         遥操 Python
NERO_LEROBOT_PYTHON        数采 Python
NERO_URDF                  NERO URDF
PICO_LEFT_CAN_USB_BUS      左臂 gs_usb 物理路径
PICO_RIGHT_CAN_USB_BUS     右臂 gs_usb 物理路径
NERO_WORLD_CAMERA          世界相机 by-path
NERO_LEFT_WRIST_CAMERA     左腕相机 by-path
NERO_RIGHT_WRIST_CAMERA    右腕相机 by-path
NERO_RECORD_DATA_DIR       仓库外的数据根目录
UNITY_EDITOR               Unity 可执行文件
ADB                        adb 可执行文件
NERO_PICO_HOST             PICO 发送 UDP 的主机 IP
```

硬件路径用以下命令获取：

```bash
lsusb -t
find /dev/v4l/by-path -type l -print
ip -details link show type can
```

相机必须用 `/dev/v4l/by-path/...`，不要将易变的 `/dev/videoN` 写入配置。左右 CAN 的接口名和 USB 路径不得相同。

## 3. 构建并连接 PICO

`NERO_PICO_HOST` 会在构建时写入 APK；主机 IP 变更后必须重新构建。

```bash
./scripts/pico/build.sh
./scripts/pico/install_and_launch.sh
./scripts/pico/check_input.sh
```

`check_input.sh` 必须能持续看到左右手柄包，再进入机械臂步骤。

## 4. 核对 Home

先预览双臂起点、目标和最大行程：

```bash
./scripts/control/run_dual_home.sh
```

确认左右角色、目标关节角和周围净空后才执行：

```bash
./scripts/control/run_dual_home.sh --execute
```

Home 本身就是实机运动。现场 Home 不同时，只在重新测量后使用 `.env` 中的 `NERO_LEFT_HOME_DEG` 和 `NERO_RIGHT_HOME_DEG` 覆盖。

## 5. 启动遥操

双臂正式入口：

```bash
./scripts/control/run_dual_servo_v3_experiment.sh --duration 120 --execute
```

单臂短时验证：

```bash
./scripts/control/run_servo_v3_experiment.sh --show-selection
./scripts/control/run_servo_v3_experiment.sh --duration 30 --execute
```

按住 **Grip** 后机械臂跟随；松开 Grip 后保持并重新锚定。**Trigger** 控制夹爪。双臂入口会把 UDP `50150` 分发到左右控制进程，并根据 `.env` 自动绑定 CAN。

## 6. 录制 LeRobot v3 数据

所有任务共用一个录制入口。先预览最终配置：

```bash
./scripts/recording/run_recording.sh \
  --task "fold the towel" \
  --dataset nero_towel_fullflow_v1 \
  --episodes 50 \
  --auto-stop off
```

确认数据目录、三路相机角色、episode 数和 action 来源后执行：

```bash
./scripts/recording/run_recording.sh \
  --task "fold the towel" \
  --dataset nero_towel_fullflow_v1 \
  --episodes 50 \
  --auto-stop off \
  --action-source controller_command \
  --execute
```

录制器会：

1. 检查双 CAN 和三路相机。
2. 双臂 Home 后启动托管遥操。
3. 按 Enter 准备，检测到运动后开始记录。
4. 每集结束后选择保存或丢弃。
5. 保存后自动 Home，继续下一集。

同名且结构健康的数据集会断点续采；失败 attempt 不计入成功 episode 数。默认 `controller_command` action 是遥操控制器实际发送给 CPV 的受保护关节目标，而不是原始手柄目标或 CAN 实测位置。

## 7. 输出位置

```text
$NERO_RECORD_DATA_DIR/<dataset>/         LeRobot v3 原始数据
$NERO_TELEOP_ARTIFACTS_DIR/logs/         遥操、Home 和数采日志
artifacts/builds/                         PICO APK
```

完整的数采→转换→训练→VLA 执行命令见
[NERO VLA 端到端流程](https://github.com/ssyly4/NERO_VLA_training/blob/main/docs/END_TO_END_VLA_WORKFLOW.zh-CN.md)。

## 安全底线

- 机械臂运动时人员不得进入可达空间，急停始终可触达。
- 任何 CAN/CPV 失联、反馈过期、非预期运动或碰撞都应立即停止。
- 不得通过增大速度、关节限位或关闭固件保护来掩盖硬件故障。
- 软件 IK、滤波和力估计不能替代硬件急停与现场监护。

## 许可证

本项目原创代码采用 Apache-2.0。PICO OpenXR SDK、NERO SDK 和 LeRobot 遵循各自上游许可证，不随本仓库重新分发。
