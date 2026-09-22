# 核心代码与调用链

本文面向需要继续维护代码的开发者，说明当前正式入口、进程关系、关键类和每个控制 tick 的执行顺序。
项目只包含 PICO 遥操与 LeRobot v3 数采；VLA 训练和策略执行不属于本仓库。

## 1. 双臂遥操启动链

```text
scripts/control/run_dual_servo_v3_experiment.sh
  ├─ scripts/common.sh                         读取 .env
  ├─ scripts/can/ensure_can_interface.sh       绑定并启动左右 SocketCAN
  ├─ nero_neo_teleop.pico.udp_fanout           :50150 -> :50151/:50152
  ├─ nero_neo_teleop.robot.dual_home            可选自动 Home
  └─ 两个 servo_v3_controller 进程
       ├─ left  / UDP :50151 / can_left
       └─ right / UDP :50152 / can_right
```

Shell 入口负责硬件准备和进程生命周期；实时控制逻辑在
`src/nero_neo_teleop/control/servo_v3_controller.py`。单臂入口
`run_servo_v3_experiment.sh` 根据 `--can-port` 同时选择 CAN、USB、PICO 手柄和对应 Home，
未知接口在连接 CAN 前拒绝。

## 2. PICO 客户端

```text
PicoControllerProbe.Update()
  -> Unity XR InputDevice / CommonUsages
  -> PicoInputPacket.LatestPacket
  -> PicoUdpSender.Update() 复制最新状态
  -> PicoUdpSender.SenderLoop() 按目标 60 Hz 发送 244 字节 UDP 报文
```

- `PicoControllerProbe.cs`：读取头显、左右手柄的位置、四元数、线速度、角速度和按键。
- `PicoInputPacket.cs`：定义单个状态快照的数据字段。
- `PicoUdpSender.cs`：写入 `NQ01` 报文头、序号、Unix/单调时间戳及三个 72 字节状态块。
- `PicoProjectBuilder.cs`：Unity 批处理构建，并把 `NERO_PICO_HOST` 写入场景中的发送组件。
- `PicoAndroidManifestPostprocessor.cs`：构建时加入 Wi-Fi 高性能锁所需权限。

PICO 端不做机械臂坐标映射，也不生成关节命令；它只发送 OpenXR/Unity 坐标下的输入快照。

## 3. 主机输入与位姿映射

`PicoUdpStream` 解码二进制报文并估计传输时延；`PicoInputMonitor` 在控制线程外持续保存最新快照。
控制器每个 tick 获取最新状态并判断 `tracked` 与 packet age。

位姿处理顺序：

```text
原始手柄状态
  -> head_yaw_world_to_view()                   只使用头显水平朝向
  -> transform_state_to_view()                  转到操作者视角坐标
  -> ClutchedPoseMapper.update()                Grip 锚定与相对位姿映射
  -> PoseLowPassFilter.update()                 位置和 SO(3) 一阶低通
  -> bounded_pose_target()                      限制目标领先实测 TCP 的距离
```

`OPENXR_TO_NERO` 把 Unity 的右/上/前轴转换到 NERO 的前/左/上轴。
Grip 按下时记录手柄锚点与机器人 TCP 锚点；后续只使用相对变化，因此重新佩戴头显或松开 Grip 不会直接产生绝对位置跳变。

## 4. 每个 Servo v3 控制 tick

当前 Shell 入口以 40 Hz 调用以下流程：

1. 从 CAN 读取完整七关节角与电机力矩。
2. `PinocchioVelocityServo.pose()` 用 FK 得到实测 TCP 位姿。
3. 获取最新 PICO 快照；输入过期或失去跟踪时进入 hold/re-anchor。
4. 生成、滤波并限制末端目标；可选接触力保护只阻止进一步下探。
5. `PinocchioVelocityServo.solve()` 计算速度型 IK：
   - 使用 `SE(3).log6` 求位置和姿态误差；
   - Pinocchio 计算 LOCAL frame Jacobian；
   - 最小奇异值决定 DLS 阻尼；
   - 加权阻尼伪逆得到主任务关节速度；
   - `I - J#J` 投影关节限位 barrier 梯度，得到近似零空间速度；
   - 合并后施加关节速度与关节限位约束。
6. `FiniteLeadCommandFollower.step()` 根据期望关节速度限制加速度及命令相对反馈的领先量。
7. `bounded_transport_step()` 限制单个 CPV tick 的最大关节步长。
8. `NeroCpvPositionBackend.send()` 通过 `pyAgxArm` 和 SocketCAN 发送七关节位置。
9. `ArmCommandPublisher` 可把已发送关节目标与时间戳发布给数采进程。

这是一条反馈闭环。PICO delta 只定义人的相对运动意图；Pinocchio FK 和 CAN 反馈定义机器人当前实际状态。

## 5. 关键控制类

| 文件 | 类或函数 | 责任 |
|---|---|---|
| `pico/pico_input.py` | `PicoUdpStream`、`PicoInputMonitor` | UDP 解码、时序与最新快照 |
| `pico/pose_mapper.py` | `ClutchedPoseMapper` | 离合锚定、坐标和位姿增益 |
| `control/servo_v3_core.py` | `PoseLowPassFilter` | 位置和姿态一阶低通 |
| 同上 | `bounded_pose_target` | 限制末端目标领先量 |
| 同上 | `PinocchioVelocityServo` | FK、自适应 DLS、零空间限位回避 |
| 同上 | `FiniteLeadCommandFollower` | 关节速度积分、加速度和命令领先限制 |
| `control/servo_v3_controller.py` | `main` | 实时状态机与完整控制循环 |
| `robot/nero_io.py` | 反馈和健康检查函数 | SDK/CAN 边界 |
| `robot/home_config.py` | Home 与硬件默认配置 | 左右臂固定参数及环境覆盖 |

## 6. 数采调用链

```text
scripts/recording/run_recording.sh
  -> CAN 与相机预检
  -> 可选 run_dual_home.sh
  -> bimanual_lerobot_recorder.main()
       ├─ 启动托管双臂遥操进程
       ├─ NeroCanStateSource              双臂状态缓存
       ├─ CameraReader                    三路图像缓存
       ├─ ArmCommandReceiver              已发送控制目标缓存
       ├─ 30 Hz take_sample()
       ├─ 自动结束条件
       └─ LeRobotDataset.add_frame/save_episode
```

每条记录包含三路图像、双臂关节和夹爪 observation、action、任务文本及时间信息。
`controller_command` action 来自控制器已发送的受保护关节目标；它不是手柄原始目标，也不等于同一时刻的实测反馈。
`next_feedback` 是兼容旧数据的标签模式；公开入口默认记录实际发送的 `controller_command`。

## 7. 配置来源与优先级

```text
命令行参数
  > 当前 shell 导出的环境变量
  > 仓库根目录 .env
  > scripts/common.sh / Python 模块默认值
```

硬件拓扑、相机路径和本机 Python 路径必须写入 `.env`；不要提交现场凭据和易变设备路径。
Home 默认值在 `home_config.py`，可由 `NERO_LEFT_HOME_DEG`、`NERO_RIGHT_HOME_DEG` 覆盖。
修改控制增益时优先使用已有 `PICO_*` 环境变量，不要再复制一份近似相同的启动脚本。

## 8. 代码阅读顺序

```text
PicoControllerProbe.cs
-> PicoUdpSender.cs
-> pico_input.py
-> pose_mapper.py
-> servo_v3_controller.py
-> servo_v3_core.py
-> nero_io.py
-> action_command_stream.py
-> bimanual_lerobot_recorder.py
```

修改实时控制前先运行 `scripts/check.sh`。涉及 Home、左右角色、CAN、滤波、IK 或限幅的修改，必须先完成无运动检查，再做单臂低速实机验证。
