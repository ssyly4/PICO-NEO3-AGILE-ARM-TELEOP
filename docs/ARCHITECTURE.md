# Architecture / 系统架构

## Runtime Path / 实时控制链

1. Unity samples head and controller poses through OpenXR and sends binary UDP
   packets to port `50150`.
2. `pico_input.py` validates timing; `udp_fanout.py` duplicates the stream for
   independent left and right controllers.
3. `pose_mapper.py` performs clutch anchoring and maps OpenXR coordinates into
   the NERO base frame.
4. `servo_v3_core.py` filters targets and uses Pinocchio Jacobians for
   position-priority differential IK with null-space posture control.
5. `FiniteLeadCommandFollower` bounds velocity, acceleration, CPV step, and
   feedback lead before the seven-joint target is sent.
6. `nero_vla.cpv_backend` and `pyAgxArm` encode CPV commands for SocketCAN.

中文：Unity 获取 OpenXR 位姿；主机完成 UDP 校验、离合锚定和坐标映射；
Pinocchio 根据雅可比矩阵求速度级 IK；随后施加速度、加速度、单帧 CPV 步长和
反馈领先限制，最后由 `pyAgxArm` 通过 SocketCAN 下发。

## Recording Path / 数采链

The recorder samples two CAN states and three cameras on one 30 Hz schedule.
In `controller_command` mode, each controller publishes its actual guarded
target with a monotonic timestamp over a Unix datagram socket. The recorder
matches these commands to frames and writes LeRobot v3 state, action, image,
timestamp, task, and episode metadata.

中文：录制器以统一 30 Hz 时钟采样双臂 CAN 和三路图像。使用
`controller_command` 标签时，两侧控制器通过 Unix 数据报发布带单调时钟的实际
受控目标，录制器按时间匹配后写入 LeRobot v3，避免记录未执行的手柄目标。

## Configuration / 配置

Machine-specific settings live in `.env`. `scripts/common.sh` loads them for
launchers, while `runtime.py` provides Python defaults. Logs, APKs, and datasets
are excluded from tracked source.

机器相关的 SDK 路径、CAN USB 拓扑、相机和 Home 覆盖值统一放在 `.env`；
日志、APK 和数据集不进入源码版本管理。
