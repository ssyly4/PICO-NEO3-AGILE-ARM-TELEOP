# 安装与依赖

本项目包含 PICO 客户端、主机端遥操和 LeRobot v3 数采代码。机械臂厂商 SDK、
机器人描述、PICO OpenXR SDK 和 LeRobot 不随仓库分发，因此安装分为两套 Python
环境和一个 Unity 工程。

## 1. 已验证环境

以下版本是当前实机环境的已验证组合，不表示唯一可用版本：

| 组件 | 已验证版本 |
|---|---|
| Linux | Debian 系 |
| Python（遥操） | 3.13.13 |
| NumPy（遥操） | 2.5.1 |
| Pinocchio | 4.1.0（PyPI 分发名 `pin`） |
| `pyAgxArm` | 1.0.0 |
| `pytracik` / `trac_ik` | 0.0.3 |
| `nero_vla` | 0.1.0 |
| Python（数采） | 3.12.13 |
| LeRobot | 0.5.2 |
| PyArrow | 24.0.0 |
| Unity | 6000.0.80f1 |
| Unity OpenXR | 1.16.1 |

`pyAgxArm`、`pytracik`、`nero_vla` 和 NERO URDF 来自外部 NERO 工作区。它们不是
本仓库原创依赖，也没有假设可从公共 PyPI 获得，因此没有写入 `project.dependencies`。

## 2. 克隆和安装本仓库

```bash
git clone https://github.com/ssyly4/PICO-NEO3-AGILE-ARM-TELEOP.git nero_neo_teleop
cd nero_neo_teleop
python3 -m venv .venv
. .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -e '.[dev]'
cp .env.example .env
```

这一步只能运行公开的纯逻辑测试。实机运行还需要下面两套外部环境。

## 3. 遥操 Python 环境

准备一个可以同时导入以下模块的解释器，并把路径写入 `.env` 的
`NERO_TELEOP_PYTHON`：

```text
numpy
pinocchio
pyAgxArm
trac_ik
nero_vla
nero_neo_teleop
```

机器人工作区通过 `NERO_WS` 指定。默认期望其中包含：

```text
$NERO_WS/nero_vla/
$NERO_WS/src/pyAgxArm/
$NERO_WS/src/pytracik/
$NERO_WS/src/piper_ros/.../nero_description.urdf
```

安装本仓库到该解释器，并验证导入：

```bash
"$NERO_TELEOP_PYTHON" -m pip install -e .
"$NERO_TELEOP_PYTHON" -c \
  'import pinocchio, pyAgxArm, trac_ik, nero_vla, nero_neo_teleop; print("teleop imports OK")'
```

如果 URDF 不在默认位置，在 `.env` 中显式设置 `NERO_URDF`。

## 4. 数采 Python 环境

数采解释器由 `NERO_LEROBOT_PYTHON` 指定，需要能够导入：

```text
lerobot
numpy
pyarrow
cv2
nero_vla
nero_neo_teleop
```

LeRobot 的安装方式和版本兼容性以其上游项目为准。安装完成后，同样把本仓库以
editable 方式安装到该解释器；`dev` extra 提供 `scripts/check.sh` 使用的 Ruff：

```bash
"$NERO_LEROBOT_PYTHON" -m pip install -e '.[dev]'
"$NERO_LEROBOT_PYTHON" -c \
  'import lerobot, pyarrow, cv2, nero_vla, nero_neo_teleop; print("recording imports OK")'
```

## 5. PICO Unity 环境

仓库固定使用 `pico_client/ProjectSettings/ProjectVersion.txt` 中记录的 Unity 版本。
从 PICO 官方渠道取得 Unity OpenXR SDK，并放到：

```text
pico_client/LocalPackages/com.unity.xr.openxr.picoxr/
```

该目录受上游许可证约束并被 `.gitignore` 排除。设置 `.env` 中的 `UNITY_EDITOR`、
`ADB` 和 `NERO_PICO_HOST` 后执行：

```bash
./scripts/pico/build.sh
./scripts/pico/install_and_launch.sh
./scripts/pico/check_input.sh
```

## 6. 配置硬件

使用稳定 USB 拓扑和 V4L2 by-path 路径，不能把动态 `/dev/videoN` 当作永久配置：

```bash
lsusb -t
find /dev/v4l/by-path -type l -print
ip -details link show type can
```

把两路 CAN、三路相机、数据目录和 Home 覆盖值写入 `.env`。禁止提交现场 `.env`。

## 7. 验证层级

```bash
# 公开依赖即可运行
python -m ruff check src tests
python -m unittest discover -s tests -p 'test_pose_mapper.py' -q
python -m unittest discover -s tests -p 'test_action_command_stream.py' -q
python -m unittest discover -s tests -p 'test_gripper.py' -q

# 完整外部环境，无机器人运动
./scripts/check.sh

# 只打印左右角色，不连接 CAN
./scripts/control/run_servo_v3_experiment.sh --show-selection
```

完成上述验证不等于允许实机运动。运动前继续执行[交接清单](HANDOFF.zh-CN.md)和
[安全说明](SAFETY.md)。
