# 系统架构与代码调用链

本文档只说明当前正式路径。历史实验和 VLA 训练不在本仓库。

## 实时遥操调用链

```text
scripts/control/run_dual_servo_v3_experiment.sh
  ├─ scripts/common.sh                         读取 .env
  ├─ scripts/can/ensure_can_interface.sh       绑定左右 SocketCAN
  ├─ nero_neo_teleop.pico.udp_fanout           :50150 -> :50151/:50152
  ├─ nero_neo_teleop.robot.dual_home            可选双臂 Home
  └─ 两个 servo_v3_controller 进程
       ├─ left  / UDP :50151 / can_left
       └─ right / UDP :50152 / can_right
```

Shell 入口只负责配置、硬件准备和进程生命周期。实时控制主程序是
`src/nero_neo_teleop/control/servo_v3_controller.py`。

### PICO 客户端

```text
PicoControllerProbe.Update()
  -> Unity XR InputDevice / CommonUsages
  -> PicoInputPacket.LatestPacket
  -> PicoUdpSender.Update()
  -> PicoUdpSender.SenderLoop() 按目标 60 Hz 发送 UDP
```

| 文件 | 职责 |
|---|---|
| `PicoControllerProbe.cs` | 读取头显、手柄位姿、速度和按键 |
| `PicoInputPacket.cs` | 定义一帧输入快照 |
| `PicoUdpSender.cs` | 编码 `NQ01` 二进制报文并发往主机 |

PICO 端不计算机械臂坐标、IK 或关节指令。

### 主机每个控制 tick

```text
PicoUdpStream 解包与时序检查
  -> head_yaw_world_to_view / transform_state_to_view
  -> ClutchedPoseMapper             Grip 锚定和相对位姿
  -> PoseLowPassFilter              位置与 SO(3) 低通
  -> bounded_pose_target            限制目标领先 TCP
  -> PinocchioVelocityServo.pose    CAN 关节反馈 FK
  -> PinocchioVelocityServo.solve   自适应 DLS + 零空间限位回避
  -> FiniteLeadCommandFollower      限关节速度、加速度与领先量
  -> bounded_transport_step         限制单个 CPV tick 步长
  -> NeroCpvPositionBackend.send
  -> pyAgxArm -> SocketCAN -> NERO
```

IK 每个 tick 都重新读取 CAN 实测关节。`SE(3).log6` 计算末端误差，Pinocchio 提供 FK 和 LOCAL Jacobian，最小奇异值决定 DLS 阻尼，`I - J#J` 投影关节限位 barrier 梯度。PICO delta 只表示人的运动意图，不会直接当成关节角。

输入过期、跟踪丢失或 Grip 松开时，控制器保持当前指令并在恢复后重新锚定。

## 数采调用链

```text
scripts/recording/record_single.sh
  -> 单右臂 CAN 与两路相机预检
  -> single_home
  -> 启动托管单臂遥操
  -> single_arm_lerobot_recorder.main()

scripts/recording/record_dual.sh
  -> CAN 与相机预检
  -> dual_home
  -> 启动托管双臂遥操
  -> bimanual_lerobot_recorder.main()
       ├─ NeroCanStateSource       双臂 CAN 状态缓存
       ├─ CameraReader             三路最新图像缓存
       ├─ ArmCommandReceiver       已发送指令与单调时间戳
       ├─ take_sample() @ 30 Hz
       └─ LeRobotDataset.add_frame/save_episode
```

`controller_command` 模式下，左右遥操进程用 Unix 数据报发布已经通过 IK、follower 和 CPV 限制的关节目标。录制器根据单调时间戳匹配指令、CAN 反馈和三路最新相机帧，写入 observation、action、task 和 episode 元数据。

## 核心源码

| 路径 | 职责 |
|---|---|
| `pico/pico_input.py` | UDP 解码、时序、最新快照 |
| `pico/pose_mapper.py` | 坐标转换、Grip 离合锚定、平移和姿态增益 |
| `control/servo_v3_controller.py` | 遥操状态机和实时主循环 |
| `control/servo_v3_core.py` | 低通滤波、FK、DLS IK、零空间和 follower |
| `control/gripper.py` | Trigger 到夹爪指令 |
| `robot/nero_io.py` | SDK 反馈和 CPV 辅助边界 |
| `robot/home_config.py` | 左右 Home 和环境覆盖 |
| `recording/action_command_stream.py` | 已发送 action 的进程间传输 |
| `recording/single_arm_lerobot_recorder.py` | 两相机单右臂 LeRobot v3 录制器 |
| `recording/bimanual_lerobot_recorder.py` | 三相机双臂 LeRobot v3 录制器 |

## 配置优先级

```text
命令行参数
  > 当前 Shell 环境变量
  > 仓库根目录 .env
  > scripts/common.sh 与 Python 默认值
```

硬件拓扑和本机路径只写入 `.env`。任务文本、数据集名和 episode 数由统一录制入口的命令行参数提供，不再为任务复制 Shell 脚本。
