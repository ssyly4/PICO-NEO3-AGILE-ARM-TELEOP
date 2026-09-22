# 运行入口

日常操作从仓库根目录的 `scripts/` 启动。机器路径与硬件绑定在不入库的 `.env` 中，
主机 Python 代码位于 `src/nero_neo_teleop/`，生成物位于 `artifacts/`。

## 常用入口

- `scripts/control/run_servo_v3_experiment.sh`：单臂 PICO Servo v3 控制；默认右臂。
- `scripts/control/run_dual_servo_v3_experiment.sh`：双臂 PICO 控制。
- `scripts/control/run_dual_home.sh`：双臂 Home，默认只预览。
- `scripts/recording/run_recording.sh`：配置驱动的通用双臂 LeRobot 数采入口。
- `scripts/can/ensure_can_interface.sh`：按 USB 拓扑绑定稳定 CAN 名称。
- `scripts/pico/build.sh`、`install_and_launch.sh`、`check_input.sh`：构建、安装和检查 PICO 客户端。

单臂可先运行 `scripts/control/run_servo_v3_experiment.sh --show-selection`；
指定左臂时加 `--can-port` 并填入 `.env` 中的左臂接口名。
该选项只打印 CAN、USB、手柄和 Home 角色，不连接 CAN、不执行运动。

## Python 包职责

- `pico/`：UDP 输入、双臂分发和位姿映射。
- `control/`：Pinocchio 速度级 IK、Servo v3 控制循环和夹爪。
- `robot/`：SDK 反馈、Home 及 CPV 辅助接口。
- `recording/`：控制命令时间戳与 LeRobot 数采。
- `diagnostics/`：PICO 输入只读检查。

## 可选平移前馈

`servo_v3_controller` 的 `--translation-feedforward-gain` 范围为 `[0, 1]`，默认 `0`，
即沿用仅靠位置误差驱动的行为。启用后，对滤波且限幅后的末端目标做差分，
把估计的平移速度加入末端位置反馈项；后续笛卡尔、关节和 CPV 限制仍然生效。
当前日常 Shell 入口未主动启用该参数。
