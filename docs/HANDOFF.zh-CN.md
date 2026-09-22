# PICO Neo3 遥操交接清单

本仓库包含 PICO 客户端、主机遥操和数采代码；训练与策略推理属于独立项目。
这里的检查不替代实机风险评估。不要仅因代码测试通过就直接执行回位。

## 1. 固定版本与外部依赖

- 记录本仓库和 `nero_ws` 的 Git 提交及工作区状态；交接使用已提交版本。
- 用 `pico_client/ProjectSettings/ProjectVersion.txt`、
  `pico_client/Packages/manifest.json` 核对 Unity/OpenXR 版本。
- 从 PICO 官方渠道获取 Unity OpenXR SDK，安装到
  `pico_client/LocalPackages/com.unity.xr.openxr.picoxr/`。该包不随本仓库分发。
- 准备两个 Python 环境：`NERO_TELEOP_PYTHON` 应能导入 `nero_vla`、
  `pyAgxArm`、`pinocchio`；`NERO_LEROBOT_PYTHON` 应能导入 `lerobot`、
  `pyarrow` 和相机所需依赖。`pip install -e .` 只安装本仓库，不负责这些外部环境。
- 核对 `NERO_URDF` 指向当前 NERO 型号的 URDF；默认路径在
  `src/nero_neo_teleop/runtime.py` 中。

## 2. 本机配置

从 `.env.example` 复制 `.env`，再按现场设备填写；`.env` 不进入 Git。
重点核对两侧 CAN 接口名与 USB 拓扑、三路相机的稳定 V4L2 路径、
`NERO_PICO_HOST`、数据目录和两套 Python 解释器。
`PICO_LEFT_CAN_PORT` 与 `PICO_RIGHT_CAN_PORT` 不得指向同一个接口。
单臂入口默认选择右臂；用 `--can-port can_left` 时改为左臂 Home 与左手柄。
Home 覆盖值只可依据现场测量修改，不要照搬其他安装位置的关节角。

## 3. 无运动验证

```bash
cd /home/dev/nero_neo_teleop
./scripts/check.sh
./scripts/control/run_servo_v3_experiment.sh --show-selection
./scripts/control/run_servo_v3_experiment.sh --can-port can_left --show-selection
./scripts/pico/check_input.sh
./scripts/control/run_dual_home.sh
```

`check.sh` 运行静态检查和单元测试，不发送机械臂运动指令。
`--show-selection` 只打印单臂 CAN、USB、手柄和 Home 角色，不连接 CAN；
其中 `can_left` 应替换为现场 `.env` 配置的左臂接口名。
`check_input.sh` 检查 PICO 数据是否到达；Home 命令不带 `--execute`
时只预览目标。后两步仍可能使用网络或读取 CAN，必须在现场核对输出。
确认 Home 的左右角色、目标关节角、起点、预计行程和周围净空。

## 4. 实机与数采

首次实机测试使用单臂、低速、短时长，急停保持可触达；确认手柄的
前后、左右、上下和姿态方向后再运行双臂。自动 Home 本身是实机运动。
数采前用相机实时画面确认 world、left_wrist、right_wrist 角色，不能仅靠
`/dev/videoN` 编号；同时确认数据根目录、episode 目标数和 action 来源。
正式数采入口为 `scripts/recording/run_recording.sh`，任务和 action 来源通过参数
配置，具体见 `README.md`。数采文件存放在仓库外的
`NERO_RECORD_DATA_DIR`，运行日志存放在 `artifacts/`。

CAN/CPV 失联、反馈过期、非预期运动或接触异常时立即停止并检查日志；
不要通过增大软件限位来掩盖故障。运行安全要求见 `SAFETY.md`。
