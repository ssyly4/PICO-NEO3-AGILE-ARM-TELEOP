"""Hardware-independent state helpers for single-arm recording."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


ACTIVE_CONTROL_STATES = frozenset({"velocity_tracking", "network_predict"})


def controller_motion_started(control_state: str | None) -> bool:
    return control_state in ACTIVE_CONTROL_STATES


@dataclass(frozen=True)
class InactivityResult:
    should_stop: bool
    state: str
    joint_speed_deg_s: float
    gripper_speed_per_s: float


class CommandInactivityAutoStop:
    """Detect a continuous interval without arm or gripper target changes."""

    def __init__(
        self,
        *,
        inactivity_seconds: float,
        joint_speed_deg_s: float,
        gripper_speed_per_s: float,
    ) -> None:
        if min(inactivity_seconds, joint_speed_deg_s, gripper_speed_per_s) <= 0:
            raise ValueError("inactivity timing and speed thresholds must be positive")
        self.inactivity_ns = round(inactivity_seconds * 1e9)
        self.joint_speed_deg_s = float(joint_speed_deg_s)
        self.gripper_speed_per_s = float(gripper_speed_per_s)
        self._previous_vector: np.ndarray | None = None
        self._previous_ns: int | None = None
        self._inactive_since_ns: int | None = None

    def update(self, vector: np.ndarray, monotonic_ns: int) -> InactivityResult:
        value = np.asarray(vector, dtype=np.float64)
        if value.shape != (8,) or not np.isfinite(value).all():
            raise ValueError("command must be a finite eight-vector")
        now_ns = int(monotonic_ns)
        if self._previous_vector is None or self._previous_ns is None:
            self._previous_vector = value.copy()
            self._previous_ns = now_ns
            return InactivityResult(False, "activity_armed", 0.0, 0.0)

        previous_ns = self._previous_ns
        elapsed = (now_ns - previous_ns) / 1e9
        if elapsed <= 0:
            raise ValueError("command timestamps must increase")
        joint_delta_deg = np.abs(np.rad2deg(value[:7] - self._previous_vector[:7]))
        joint_speed = float(np.max(joint_delta_deg) / elapsed)
        gripper_speed = float(abs(value[7] - self._previous_vector[7]) / elapsed)
        self._previous_vector = value.copy()
        self._previous_ns = now_ns

        active = (
            joint_speed > self.joint_speed_deg_s
            or gripper_speed > self.gripper_speed_per_s
        )
        if active:
            self._inactive_since_ns = None
            return InactivityResult(False, "input_active", joint_speed, gripper_speed)
        if self._inactive_since_ns is None:
            self._inactive_since_ns = previous_ns
        should_stop = now_ns - self._inactive_since_ns >= self.inactivity_ns
        return InactivityResult(
            should_stop,
            "input_inactive_stop" if should_stop else "waiting_inactivity",
            joint_speed,
            gripper_speed,
        )
