"""Fixed hardware and Home configuration for NERO dual-arm PICO teleoperation."""

from __future__ import annotations

import os

import numpy as np


LEFT_CAN = os.environ.get("PICO_LEFT_CAN_PORT", "can_left")
RIGHT_CAN = os.environ.get("PICO_RIGHT_CAN_PORT", "can_right")
LEFT_USB_BUS = os.environ.get("PICO_LEFT_CAN_USB_BUS", "1-2.3:1.0")
RIGHT_USB_BUS = os.environ.get("PICO_RIGHT_CAN_USB_BUS", "3-1.2:1.0")

# Shared base_link-frame TCP floor captured from the left arm at its lowest
# permitted towel-contact pose on 2026-08-10.
MIN_TCP_HEIGHT_MM = 169.331829

# Shared base_link-frame TCP ceiling captured from the left arm at its highest
# permitted towel-task pose on 2026-08-10.
MAX_TCP_HEIGHT_MM = 391.729529

# Cartesian-refined dual-arm Home generated from the hand-placed poses on
# 2026-08-10. Both preserve their original XY, share link7 Z=267.434464 mm,
# and align the physical tool-forward (+X) axis with base_link -Z.
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
