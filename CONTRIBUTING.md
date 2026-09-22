# 贡献指南

硬件相关配置必须写入 `.env`。不要提交数据集、日志、APK、凭据、用户绝对路径或
Unity 生成目录。新增实机运动功能必须提供不执行运动的预检，并通过显式
`--execute` 参数授权。不要提交 `pico_client/LocalPackages/` 下的 PICO Unity
OpenXR SDK。

提交合并请求前请运行：

```bash
python -m pip install -e '.[dev]'
python -m ruff check src tests
./scripts/check.sh
git diff --check
```

公开 CI 只运行不依赖厂商 SDK 的纯逻辑测试；`scripts/check.sh` 需要完整 NERO 和
LeRobot 环境。合并请求必须明确说明运行过哪些检查、是否做过实机验证，以及是否修改
了 Home、左右角色、CAN、IK、限幅或安全保护。没有实机验证时必须明确写“未实机验证”。

不要为了通过公开 CI 而伪造 `pyAgxArm`、`nero_vla` 或机械臂反馈。硬件相关测试应保留
在完整环境中运行，并保证默认测试命令不会发送运动指令。
