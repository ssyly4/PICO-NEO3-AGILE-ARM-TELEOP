"""Small robot-feedback helpers shared by active controllers."""

from __future__ import annotations

import time

import numpy as np


def flange_pose(robot) -> np.ndarray:
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        message = robot.get_flange_pose()
        if message is not None:
            value = np.asarray(message.msg, dtype=np.float64)
            if value.shape == (6,) and np.isfinite(value).all():
                return value
            raise RuntimeError(f"Invalid flange pose feedback: {value}")
        time.sleep(0.01)
    raise RuntimeError("No flange-pose feedback within 2 seconds")


def wait_complete_joint_feedback(robot, timeout_sec: float = 5.0) -> np.ndarray:
    deadline = time.monotonic() + timeout_sec
    samples: list[np.ndarray] = []
    timestamps: set[float] = set()
    while time.monotonic() < deadline:
        feedback = robot.get_joint_angles()
        if feedback is not None and feedback.timestamp not in timestamps:
            value = np.asarray(feedback.msg, dtype=np.float64)
            if value.shape == (7,) and np.isfinite(value).all():
                timestamps.add(feedback.timestamp)
                samples.append(value.copy())
                if len(samples) >= 8:
                    recent = np.stack(samples[-8:])
                    if float(np.max(np.ptp(np.rad2deg(recent), axis=0))) <= 0.25:
                        return recent[-1]
        time.sleep(0.01)
    raise RuntimeError("No stable, complete seven-joint CAN feedback snapshot")


def wait_enabled(robot, timeout_sec: float = 3.0) -> None:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if robot.enable():
            return
        time.sleep(0.03)
    raise RuntimeError("All seven joints did not report enabled")


def wait_cpv_mode(robot, timeout_sec: float = 3.0) -> None:
    deadline = time.monotonic() + timeout_sec
    last_status = None
    while time.monotonic() < deadline:
        status = robot.get_arm_status()
        if status is not None:
            last_status = status
        if status is not None and (
            int(status.msg.ctrl_mode) == 0x01
            and int(status.msg.mode_feedback) == 0x05
            and int(status.msg.arm_status) == 0x00
        ):
            return
        time.sleep(0.01)
    if last_status is None:
        detail = "no arm status feedback"
    else:
        detail = (
            f"ctrl_mode=0x{int(last_status.msg.ctrl_mode):02x} "
            f"mode_feedback=0x{int(last_status.msg.mode_feedback):02x} "
            f"arm_status=0x{int(last_status.msg.arm_status):02x}"
        )
    raise RuntimeError(f"CAN/CPV mode was not confirmed: {detail}")


def wait_gripper_status(gripper, timeout_sec: float = 3.0):
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        status = gripper.get_gripper_status()
        if status is not None:
            return status
        time.sleep(0.02)
    raise RuntimeError("No AGX gripper feedback received")


def check_gripper_health(status) -> None:
    flags = status.msg.foc_status
    fault_names = (
        "voltage_too_low", "motor_overheating", "driver_overcurrent",
        "driver_overheating", "sensor_status", "driver_error_status",
    )
    active = [name for name in fault_names if bool(getattr(flags, name, False))]
    if active:
        raise RuntimeError(f"Gripper driver fault: {active}")


def check_driver_health(robot) -> None:
    fault_names = (
        "voltage_too_low", "motor_overheating", "driver_overcurrent",
        "driver_overheating", "collision_status", "driver_error_status", "stall_status",
    )
    for joint_index in range(1, 8):
        state = robot.get_driver_states(joint_index)
        if state is None:
            raise RuntimeError(f"Missing driver state for joint {joint_index}")
        active = [name for name in fault_names if bool(getattr(state.msg.foc_status, name, False))]
        if active:
            raise RuntimeError(f"Joint {joint_index} driver fault: {active}")
