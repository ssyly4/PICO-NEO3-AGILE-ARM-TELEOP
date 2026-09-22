# 故障排查

所有排查都应先停止机器人运动，并确保急停可用。不要通过提高速度、加速度或保护阈值
来掩盖连接和反馈故障。

## Python 导入失败

先确认脚本实际使用的解释器：

```bash
grep -E 'NERO_(TELEOP|LEROBOT)_PYTHON|NERO_WS' .env
"$NERO_TELEOP_PYTHON" -c 'import pinocchio, pyAgxArm, trac_ik, nero_vla'
"$NERO_LEROBOT_PYTHON" -c 'import lerobot, pyarrow, cv2, nero_vla'
```

`pip install -e .` 只安装本项目，不会自动安装厂商 SDK 和 NERO 工作区。

## CAN 接口不存在或角色错误

```bash
lsusb -t
ip -details link show type can
grep -E 'PICO_(LEFT|RIGHT)_CAN' .env
```

确认 `.env` 中左右 USB 路径与当前物理端口一致，再运行
`scripts/can/ensure_can_interface.sh`。如果适配器频繁消失，应先排查 USB 供电、线束、
拓展坞和固件状态，不要无限重绑接口。

## PICO 没有 UDP 输入

```bash
./scripts/pico/check_input.sh
adb devices
ip address
```

确认 PICO 应用正在前台运行、APK 内嵌的 `NERO_PICO_HOST` 是主机当前地址、主机与 PICO
处于可互通网络，并允许 UDP `50150`。更换主机 IP 后必须重新构建 APK。

## 相机打不开或帧过期

```bash
find /dev/v4l/by-path -type l -print
v4l2-ctl --list-devices
v4l2-ctl --device "$NERO_WORLD_CAMERA" --list-formats-ext
```

插拔后重新确认 world、left_wrist、right_wrist 的实际画面。三路 1280x720@30 MJPEG
需要足够 USB 带宽；必要时把相机分散到不同 USB 控制器，而不是只换 `/dev/videoN`。

## 数采无法继续或数据集被拒绝

录制器只恢复结构完整、Parquet 可读的数据集。检查目标目录、剩余空间、episode 元数据和
视频文件；不要手动只删除其中一个 Parquet 或视频文件。失败 attempt 会被丢弃，不计入
成功 episode 数。

## 机械臂不跟随或进入 hold

查看 `artifacts/logs/` 中的输入 age、控制状态、CAN 反馈和 CPV 健康信息：

- `input_hold`：Grip 未按下、追踪无效或控制器正在重锚定。
- 网络 age 超限：PICO UDP 延迟或丢包。
- CAN/CPV 状态异常：停止运行并检查机械臂、适配器和使能状态。
- follower 受限：命令领先量、速度、加速度或单帧 CPV 步长正在生效。

修改参数前先保存日志并复现单臂低速测试。
