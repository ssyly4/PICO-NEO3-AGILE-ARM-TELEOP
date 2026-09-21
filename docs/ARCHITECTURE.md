# 系统架构

## 实时遥操链路

1. Unity 通过 OpenXR 读取头显和手柄状态，客户端按目标 60 Hz 向主机 UDP `50150` 发送二进制快照。
2. `pico_input.py` 解包并检查时序；双臂时，`udp_fanout.py` 把输入分发到左右控制进程。
3. `pose_mapper.py` 根据 Grip 建立离合锚点，将手柄的相对位姿映射到 NERO 基坐标系。
4. `servo_v3_controller.py` 对目标位姿滤波、限制目标领先量，并读取 CAN 实测关节位置。
5. `servo_v3_core.py` 使用 Pinocchio 正运动学、雅可比和自适应阻尼速度级 IK，计算七关节期望速度及近似零空间的限位回避速度。
6. `FiniteLeadCommandFollower` 限制关节速度、加速度和反馈领先量；CPV 发送前再次限制单帧关节步长。
7. `nero_vla.cpv_backend` 与 `pyAgxArm` 通过 SocketCAN 向 NERO 发送 CPV 位置命令。

这是闭环链路：每个控制 tick 重新读取 CAN 状态，不把 PICO 位移直接当成关节角。
输入过期或跟踪丢失时保持当前命令，并在恢复后重新锚定。

## 数采链路

录制器以 30 Hz 采样双臂 CAN 反馈和三路相机最近的有效帧。
在 `controller_command` 模式下，两侧控制器通过 Unix 数据报发布**已发送的受保护关节目标**及单调时间戳；
录制器按时间匹配后写入 LeRobot v3 的 observation、action、图像、时间戳、任务与 episode 元数据。
`controller_command` 是已发送命令，不等同于电机实际到达的位置；CAN 反馈另行记录。
不同工作流的 action 来源见 `scripts/recording/run_bimanual_record.sh`。

## 配置和项目边界

机器相关路径、CAN USB 拓扑和相机角色写在不入库的 `.env` 中，由 `scripts/common.sh` 加载。
`runtime.py` 提供 Python 侧默认路径。APK、日志存入 `artifacts/`；数据集存于仓库外。
机械臂 SDK、URDF、Pinocchio、LeRobot 环境和 PICO OpenXR 包是外部依赖。
