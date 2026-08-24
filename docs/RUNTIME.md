# Runtime Reference / 运行入口

The live host stack is installed from `src/nero_neo_teleop`. Operator commands
are kept under the repository-level `scripts/` directory.

## Daily entrypoints

- `../scripts/control/run_servo_v3_experiment.sh`: one-arm PICO Servo v3 control.
- `../scripts/control/run_dual_servo_v3_experiment.sh`: two-arm PICO control.
- `../scripts/control/run_dual_home.sh`: guarded dual-arm Home.
- `../scripts/recording/run_bimanual_record.sh`: managed LeRobot
  recording launcher; select `--workflow custom`, `fullflow`, `stage1`, or
  `stage23`.
- `../scripts/can/ensure_can_interface.sh`: bind gs_usb to a stable CAN name.

## Live Python packages

- `../src/nero_neo_teleop/pico/`: UDP input, fanout, and mapping.
- `../src/nero_neo_teleop/control/`: Pinocchio/CPV control and gripper.
- `../src/nero_neo_teleop/robot/`: SDK helpers and Home routines.
- `../src/nero_neo_teleop/recording/`: action stream and LeRobot recorder.

## Layout Rules

- `scripts/` contains all executable shell entrypoints, grouped by function.
- Generated rollouts, captures, and APKs belong under `../artifacts/`.
- Machine configuration belongs in the ignored root `.env` file.
- Retired implementations are not distributed in the public source tree.

中文：日常只应从根目录 `scripts/` 启动。机器路径和硬件绑定写入 `.env`，
运行代码位于 `src/nero_neo_teleop`，生成物统一进入 `artifacts/`。
