# Safety / 安全说明

This repository is research software, not a safety-rated controller.

- Keep a tested physical emergency stop within reach.
- Verify CAN roles, feedback, Home targets, and free workspace before motion.
- Start with one arm, low speed, no payload, and a short duration.
- Never bypass firmware limits or collision protection without the manufacturer.
- Do not stand inside the reachable workspace or operate near another person.
- Stop after CAN loss, CPV loss, stale feedback, unexpected motion, or impact.
- Treat automatic Home as motion; preview it without `--execute` first.

本仓库是研究软件，不是安全认证控制器。

- 急停必须经过验证并始终可触达。
- 运动前检查 CAN 角色、反馈、Home 目标和工作空间。
- 首次验证使用单臂、低速、无负载和短时长。
- 未经厂家确认，不要关闭固件限位或碰撞保护。
- 人员不得进入机械臂可达空间。
- CAN/CPV 丢失、反馈过期、异常运动或碰撞后立即停止。
- 自动回位同样是实机运动，必须先在无 `--execute` 时预览。
