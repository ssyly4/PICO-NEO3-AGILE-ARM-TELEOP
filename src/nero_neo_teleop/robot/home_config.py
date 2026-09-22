"""NERO 双臂 PICO 遥操的固定硬件与 Home 配置。"""

from __future__ import annotations

import os

import numpy as np


LEFT_CAN = os.environ.get("PICO_LEFT_CAN_PORT", "can_left")
RIGHT_CAN = os.environ.get("PICO_RIGHT_CAN_PORT", "can_right")
LEFT_USB_BUS = os.environ.get("PICO_LEFT_CAN_USB_BUS", "1-2.2:1.0")
RIGHT_USB_BUS = os.environ.get("PICO_RIGHT_CAN_USB_BUS", "3-1.2:1.0")

# 2026-08-10 在左臂允许的最低毛巾接触姿态下标定的 base_link 坐标系 TCP 下限。
MIN_TCP_HEIGHT_MM = 169.331829

# 2026-08-10 在左臂允许的最高毛巾任务姿态下标定的 base_link 坐标系 TCP 上限。
MAX_TCP_HEIGHT_MM = 391.729529

# 2026-08-10 根据手动摆放姿态做笛卡尔修正后得到的双臂 Home。两臂保留各自原始
# XY，共用 link7 Z=267.434464 mm，并将工具物理前向轴（+X）对齐 base_link -Z。
def _home_from_env(name: str, default: list[float]) -> np.ndarray:
    raw = os.environ.get(name)
    values = default if raw is None else [float(item.strip()) for item in raw.split(",")]
    result = np.asarray(values, dtype=np.float64)
    if result.shape != (7,) or not np.isfinite(result).all():
        raise ValueError(f"{name} must contain seven finite comma-separated degrees")
    return np.deg2rad(result)


LEFT_HOME_RAD = _home_from_env(
    "NERO_LEFT_HOME_DEG",
    [-30.156019, 19.382340, -8.120762, 107.970206, -6.658290, -12.953593, 51.727847],
)

RIGHT_HOME_RAD = _home_from_env(
    "NERO_RIGHT_HOME_DEG",
    [27.183993, 19.491612, 5.759689, 107.696200, 2.522327, 6.487110, 52.642158],
)
