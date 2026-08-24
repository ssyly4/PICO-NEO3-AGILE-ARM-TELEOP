#!/usr/bin/env python3
"""Controller-button gripper state shared by single- and dual-arm teleop."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class GripperToggleOutput:
    toggled: bool
    is_open: bool
    target_m: float


class GripperToggleController:
    """Toggle on a clean button press edge and latch the requested width."""

    def __init__(
        self,
        *,
        open_m: float,
        closed_m: float,
        initial_feedback_m: float,
    ) -> None:
        if not 0.0 <= closed_m < open_m:
            raise ValueError("gripper widths must satisfy 0 <= closed < open")
        self.open_m = float(open_m)
        self.closed_m = float(closed_m)
        midpoint = 0.5 * (self.open_m + self.closed_m)
        self.is_open = float(initial_feedback_m) > midpoint
        self.target_m = self.open_m if self.is_open else self.closed_m
        self._armed = False

    def update(self, *, clicked: bool, input_valid: bool = True) -> GripperToggleOutput:
        if not input_valid:
            self._armed = False
            return self.output(False)
        if not clicked:
            self._armed = True
            return self.output(False)
        if not self._armed:
            return self.output(False)

        self._armed = False
        self.is_open = not self.is_open
        self.target_m = self.open_m if self.is_open else self.closed_m
        return self.output(True)

    def output(self, toggled: bool) -> GripperToggleOutput:
        return GripperToggleOutput(toggled, self.is_open, self.target_m)


@dataclass(frozen=True)
class GripperAnalogOutput:
    command: bool
    target_m: float
    shaped_trigger: float


class GripperAnalogController:
    """Continuously map a trigger to gripper width with noise suppression."""

    def __init__(
        self,
        *,
        open_m: float,
        closed_m: float,
        initial_feedback_m: float,
        trigger_deadzone: float = 0.03,
        trigger_gamma: float = 1.0,
        min_command_step_m: float = 0.001,
    ) -> None:
        if not 0.0 <= closed_m < open_m:
            raise ValueError("gripper widths must satisfy 0 <= closed < open")
        if not 0.0 <= trigger_deadzone < 0.5:
            raise ValueError("trigger_deadzone must be in [0, 0.5)")
        if trigger_gamma <= 0.0:
            raise ValueError("trigger_gamma must be positive")
        if min_command_step_m <= 0.0:
            raise ValueError("min_command_step_m must be positive")
        self.open_m = float(open_m)
        self.closed_m = float(closed_m)
        self.trigger_deadzone = float(trigger_deadzone)
        self.trigger_gamma = float(trigger_gamma)
        self.min_command_step_m = float(min_command_step_m)
        self.target_m = float(initial_feedback_m)
        self._last_command_m = float(initial_feedback_m)

    def _shape(self, trigger: float) -> float:
        value = min(1.0, max(0.0, float(trigger)))
        if value <= self.trigger_deadzone:
            return 0.0
        upper = 1.0 - self.trigger_deadzone
        if value >= upper:
            return 1.0
        normalized = (value - self.trigger_deadzone) / (upper - self.trigger_deadzone)
        return normalized ** self.trigger_gamma

    def update(self, *, trigger: float, input_valid: bool = True) -> GripperAnalogOutput:
        if not input_valid:
            return GripperAnalogOutput(False, self.target_m, 0.0)
        shaped = self._shape(trigger)
        self.target_m = self.open_m + shaped * (self.closed_m - self.open_m)
        at_endpoint = shaped in (0.0, 1.0)
        changed = abs(self.target_m - self._last_command_m) >= self.min_command_step_m
        endpoint_changed = at_endpoint and self.target_m != self._last_command_m
        command = changed or endpoint_changed
        if command:
            self._last_command_m = self.target_m
        return GripperAnalogOutput(command, self.target_m, shaped)
