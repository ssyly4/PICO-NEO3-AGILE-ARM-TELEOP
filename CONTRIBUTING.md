# 贡献指南

硬件相关配置必须写入 `.env`。不要提交数据集、日志、APK、凭据、用户绝对路径或
Unity 生成目录。新增实机运动功能必须提供不执行运动的预检，并通过显式
`--execute` 参数授权。不要提交 `pico_client/LocalPackages/` 下的 PICO Unity
OpenXR SDK。

提交合并请求前，请运行 Python 编译检查、单元测试、Shell 启动脚本的 `bash -n`
检查以及 `git diff --check`，并说明是否完成过实机验证。
