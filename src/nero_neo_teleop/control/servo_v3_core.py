#!/usr/bin/env python3
"""Minimal velocity-level Cartesian servo for NERO teleoperation."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pinocchio as pin

from nero_neo_teleop.pico.pose_mapper import matrix_to_quaternion, quaternion_to_matrix


@dataclass(frozen=True)
class CartesianServoResult:
    joint_velocity: np.ndarray
    position_error_mm: float
    orientation_error_deg: float
    minimum_singular_value: float
    damping: float
    nullspace_velocity_norm: float
    joint_limit_margin_deg: float
    orientation_limit_scale: float


@dataclass(frozen=True)
class CommandFollowerResult:
    command: np.ndarray
    joint_velocity: np.ndarray
    command_lead_deg: float
    lead_limited: bool
    limit_clamped: bool


@dataclass(frozen=True)
class BoundedPoseTargetResult:
    position: np.ndarray
    rotation: np.ndarray
    position_debt_mm: float
    orientation_debt_deg: float
    limited: bool


class PoseLowPassFilter:
    """Light first-order filtering directly on position and SO(3)."""

    def __init__(self, *, position_cutoff_hz: float, rotation_cutoff_hz: float) -> None:
        if position_cutoff_hz <= 0.0 or rotation_cutoff_hz <= 0.0:
            raise ValueError("pose filter cutoffs must be positive")
        self.position_cutoff_hz = float(position_cutoff_hz)
        self.rotation_cutoff_hz = float(rotation_cutoff_hz)
        self._position: np.ndarray | None = None
        self._rotation: np.ndarray | None = None

    def reset(self, position: np.ndarray, rotation: np.ndarray) -> None:
        self._position = np.asarray(position, dtype=np.float64).copy()
        self._rotation = np.asarray(rotation, dtype=np.float64).copy()

    def update(
        self, position: np.ndarray, rotation: np.ndarray, dt: float
    ) -> tuple[np.ndarray, np.ndarray]:
        position = np.asarray(position, dtype=np.float64)
        rotation = np.asarray(rotation, dtype=np.float64)
        if position.shape != (3,) or rotation.shape != (3, 3) or dt <= 0.0:
            raise ValueError("pose filter input has invalid shape or dt")
        if self._position is None or self._rotation is None:
            self.reset(position, rotation)
        position_alpha = self._alpha(self.position_cutoff_hz, dt)
        rotation_alpha = self._alpha(self.rotation_cutoff_hz, dt)
        self._position += position_alpha * (position - self._position)
        rotation_error = pin.log3(self._rotation.T @ rotation)
        self._rotation = self._rotation @ pin.exp3(rotation_alpha * rotation_error)
        return self._position.copy(), self._rotation.copy()

    @staticmethod
    def _alpha(cutoff_hz: float, dt: float) -> float:
        tau = 1.0 / (2.0 * np.pi * cutoff_hz)
        return float(dt / (tau + dt))


def bounded_transport_step(
    current: np.ndarray, target: np.ndarray, maximum_step_rad: float
) -> tuple[np.ndarray, bool]:
    current = np.asarray(current, dtype=np.float64)
    target = np.asarray(target, dtype=np.float64)
    if current.shape != target.shape or maximum_step_rad <= 0.0:
        raise ValueError("transport step input is invalid")
    delta = target - current
    peak = float(np.max(np.abs(delta)))
    if peak <= maximum_step_rad:
        return target.copy(), False
    return current + delta * (maximum_step_rad / peak), True


def bounded_pose_target(
    current_position: np.ndarray,
    current_rotation: np.ndarray,
    desired_position: np.ndarray,
    desired_rotation: np.ndarray,
    *,
    max_position_lead_m: float,
    max_orientation_lead_rad: float,
) -> BoundedPoseTargetResult:
    """Keep the executable Cartesian target within a finite envelope."""
    current_position = np.asarray(current_position, dtype=np.float64)
    current_rotation = np.asarray(current_rotation, dtype=np.float64)
    desired_position = np.asarray(desired_position, dtype=np.float64)
    desired_rotation = np.asarray(desired_rotation, dtype=np.float64)
    if (
        current_position.shape != (3,)
        or desired_position.shape != (3,)
        or current_rotation.shape != (3, 3)
        or desired_rotation.shape != (3, 3)
        or max_position_lead_m <= 0.0
        or max_orientation_lead_rad <= 0.0
    ):
        raise ValueError("bounded pose target input is invalid")

    position_delta = desired_position - current_position
    position_distance = float(np.linalg.norm(position_delta))
    if position_distance > max_position_lead_m:
        position = current_position + position_delta * (
            max_position_lead_m / position_distance
        )
    else:
        position = desired_position.copy()

    rotation_delta = pin.log3(current_rotation.T @ desired_rotation)
    orientation_distance = float(np.linalg.norm(rotation_delta))
    if orientation_distance > max_orientation_lead_rad:
        rotation_delta *= max_orientation_lead_rad / orientation_distance
    rotation = current_rotation @ pin.exp3(rotation_delta)
    return BoundedPoseTargetResult(
        position=position,
        rotation=rotation,
        position_debt_mm=1000.0 * position_distance,
        orientation_debt_deg=float(np.rad2deg(orientation_distance)),
        limited=bool(
            position_distance > max_position_lead_m
            or orientation_distance > max_orientation_lead_rad
        ),
    )


def extrapolate_controller_state(state: dict, horizon_sec: float) -> dict:
    """Constant-twist prediction for a short UDP gap.

    The horizon is intentionally bounded by the caller. Button values are held;
    only pose is predicted from the OpenXR velocity fields.
    """
    if horizon_sec < 0.0:
        raise ValueError("horizon_sec must be non-negative")
    predicted = dict(state)
    position = np.asarray([state["px"], state["py"], state["pz"]], dtype=np.float64)
    velocity = np.asarray(
        [state.get("vx", 0.0), state.get("vy", 0.0), state.get("vz", 0.0)],
        dtype=np.float64,
    )
    quaternion = np.asarray(
        [state["qx"], state["qy"], state["qz"], state["qw"]], dtype=np.float64
    )
    angular_velocity = np.asarray(
        [state.get("avx", 0.0), state.get("avy", 0.0), state.get("avz", 0.0)],
        dtype=np.float64,
    )
    if not all(
        np.isfinite(value).all()
        for value in (position, velocity, quaternion, angular_velocity)
    ):
        raise ValueError("controller state contains non-finite pose or velocity")
    predicted_position = position + velocity * horizon_sec
    predicted_rotation = pin.exp3(angular_velocity * horizon_sec) @ quaternion_to_matrix(
        quaternion
    )
    predicted.update(zip(("px", "py", "pz"), predicted_position.tolist()))
    predicted.update(
        zip(("qx", "qy", "qz", "qw"), matrix_to_quaternion(predicted_rotation).tolist())
    )
    return predicted


class PinocchioVelocityServo:
    """Resolved-rate IK with adaptive damping and projected limit avoidance."""

    def __init__(
        self,
        urdf_path: str | Path,
        *,
        frame_name: str = "link7",
        position_gain_s: float = 6.0,
        rotation_gain_s: float = 4.0,
        max_linear_speed_m_s: float = 0.12,
        max_angular_speed_rad_s: float = np.deg2rad(60.0),
        max_joint_speed_rad_s: float = np.deg2rad(15.0),
        nullspace_gain_s: float = 0.12,
        singular_value_threshold: float = 0.08,
        minimum_damping: float = 1e-4,
        maximum_damping: float = 0.08,
        limit_margin_rad: float = np.deg2rad(2.0),
        orientation_limit_soft_margin_rad: float = np.deg2rad(12.0),
        orientation_limit_hard_margin_rad: float = np.deg2rad(3.0),
    ) -> None:
        self.model = pin.buildModelFromUrdf(str(urdf_path))
        self.data = self.model.createData()
        self.frame_id = self.model.getFrameId(frame_name)
        if self.frame_id == len(self.model.frames):
            raise ValueError(f"URDF has no frame {frame_name!r}")
        if self.model.nq != 7 or self.model.nv != 7:
            raise ValueError("NERO servo expects a seven-DoF fixed-base model")

        self.lower = self.model.lowerPositionLimit.copy()
        self.upper = self.model.upperPositionLimit.copy()
        self.center = 0.5 * (self.lower + self.upper)
        self.half_range = 0.5 * (self.upper - self.lower)
        self.position_gain_s = float(position_gain_s)
        self.rotation_gain_s = float(rotation_gain_s)
        self.max_linear_speed_m_s = float(max_linear_speed_m_s)
        self.max_angular_speed_rad_s = float(max_angular_speed_rad_s)
        self.max_joint_speed_rad_s = float(max_joint_speed_rad_s)
        self.nullspace_gain_s = float(nullspace_gain_s)
        self.singular_value_threshold = float(singular_value_threshold)
        self.minimum_damping = float(minimum_damping)
        self.maximum_damping = float(maximum_damping)
        self.limit_margin_rad = float(limit_margin_rad)
        self.orientation_limit_soft_margin_rad = float(
            orientation_limit_soft_margin_rad
        )
        self.orientation_limit_hard_margin_rad = float(
            orientation_limit_hard_margin_rad
        )
        if min(
            self.position_gain_s,
            self.rotation_gain_s,
            self.max_linear_speed_m_s,
            self.max_angular_speed_rad_s,
            self.max_joint_speed_rad_s,
            self.singular_value_threshold,
            self.minimum_damping,
            self.maximum_damping,
            self.limit_margin_rad,
            self.orientation_limit_soft_margin_rad,
            self.orientation_limit_hard_margin_rad,
        ) <= 0.0:
            raise ValueError("servo gains and limits must be positive")
        if self.maximum_damping < self.minimum_damping:
            raise ValueError("maximum_damping must be >= minimum_damping")
        if self.orientation_limit_soft_margin_rad <= self.orientation_limit_hard_margin_rad:
            raise ValueError("orientation soft margin must exceed hard margin")

    def pose(self, q: np.ndarray) -> pin.SE3:
        joints = self._joints(q)
        pin.forwardKinematics(self.model, self.data, joints)
        pin.updateFramePlacements(self.model, self.data)
        return self.data.oMf[self.frame_id].copy()

    def solve(
        self,
        q: np.ndarray,
        target_position: np.ndarray,
        target_rotation: np.ndarray,
    ) -> CartesianServoResult:
        joints = self._joints(q)
        target_position = np.asarray(target_position, dtype=np.float64)
        target_rotation = np.asarray(target_rotation, dtype=np.float64)
        if target_position.shape != (3,) or target_rotation.shape != (3, 3):
            raise ValueError("target pose has invalid shape")

        current = self.pose(joints)
        target = pin.SE3(target_rotation, target_position)
        current_to_target = current.actInv(target)
        pose_error = pin.log6(current_to_target).vector
        task_velocity = pose_error.copy()
        task_velocity[:3] = self._clamp_norm(
            self.position_gain_s * task_velocity[:3], self.max_linear_speed_m_s
        )
        task_velocity[3:] = self._clamp_norm(
            self.rotation_gain_s * task_velocity[3:], self.max_angular_speed_rad_s
        )

        joint_margins = np.minimum(joints - self.lower, self.upper - joints)
        orientation_limit_scale = self._smooth_limit_scale(float(np.min(joint_margins)))

        jacobian = pin.computeFrameJacobian(
            self.model,
            self.data,
            joints,
            self.frame_id,
            pin.ReferenceFrame.LOCAL,
        )
        error_jacobian = -pin.Jlog6(current_to_target.inverse()) @ jacobian
        singular_values = np.linalg.svd(error_jacobian, compute_uv=False)
        minimum_singular_value = float(singular_values[-1])
        proximity = float(
            np.clip(
                (self.singular_value_threshold - minimum_singular_value)
                / self.singular_value_threshold,
                0.0,
                1.0,
            )
        )
        damping = self.minimum_damping + (
            self.maximum_damping - self.minimum_damping
        ) * proximity * proximity
        task_weights = np.array(
            [1.0, 1.0, 1.0] + [orientation_limit_scale] * 3,
            dtype=np.float64,
        )
        weighted_jacobian = task_weights[:, None] * error_jacobian
        weighted_velocity = task_weights * task_velocity
        normal = (
            weighted_jacobian @ weighted_jacobian.T
            + damping * damping * np.eye(6)
        )
        pseudo_inverse = weighted_jacobian.T @ np.linalg.solve(normal, np.eye(6))
        primary_velocity = -pseudo_inverse @ weighted_velocity

        normalized = np.clip((joints - self.center) / self.half_range, -0.97, 0.97)
        barrier_gradient = -normalized / np.maximum(1.0 - normalized * normalized, 0.05) ** 2
        barrier_gradient /= np.maximum(self.half_range, 1e-6)
        nullspace_projector = (
            np.eye(self.model.nv) - pseudo_inverse @ weighted_jacobian
        )
        task_activity = min(
            1.0,
            float(np.linalg.norm(task_velocity[:3])) / 0.02
            + float(np.linalg.norm(task_velocity[3:])) / 0.20,
        )
        task_activity = max(task_activity, 1.0 - orientation_limit_scale)
        nullspace_velocity = (
            task_activity
            * self.nullspace_gain_s
            * (nullspace_projector @ barrier_gradient)
        )
        nullspace_peak = float(np.max(np.abs(nullspace_velocity)))
        nullspace_limit = 0.4 * self.max_joint_speed_rad_s
        if nullspace_peak > nullspace_limit:
            nullspace_velocity *= nullspace_limit / nullspace_peak
        joint_velocity = primary_velocity + nullspace_velocity
        peak = float(np.max(np.abs(joint_velocity)))
        if peak > self.max_joint_speed_rad_s:
            joint_velocity *= self.max_joint_speed_rad_s / peak

        lower = self.lower + self.limit_margin_rad
        upper = self.upper - self.limit_margin_rad
        outward_lower = (joints <= lower) & (joint_velocity < 0.0)
        outward_upper = (joints >= upper) & (joint_velocity > 0.0)
        joint_velocity[outward_lower | outward_upper] = 0.0
        margin = float(np.min(joint_margins))
        return CartesianServoResult(
            joint_velocity=joint_velocity,
            position_error_mm=1000.0 * float(np.linalg.norm(pose_error[:3])),
            orientation_error_deg=float(np.rad2deg(np.linalg.norm(pose_error[3:]))),
            minimum_singular_value=minimum_singular_value,
            damping=damping,
            nullspace_velocity_norm=float(np.linalg.norm(nullspace_velocity)),
            joint_limit_margin_deg=float(np.rad2deg(margin)),
            orientation_limit_scale=orientation_limit_scale,
        )

    def _smooth_limit_scale(self, margin_rad: float) -> float:
        normalized = np.clip(
            (margin_rad - self.orientation_limit_hard_margin_rad)
            / (
                self.orientation_limit_soft_margin_rad
                - self.orientation_limit_hard_margin_rad
            ),
            0.0,
            1.0,
        )
        return float(normalized * normalized * (3.0 - 2.0 * normalized))

    def _joints(self, q: np.ndarray) -> np.ndarray:
        result = np.asarray(q, dtype=np.float64)
        if result.shape != (self.model.nq,) or not np.isfinite(result).all():
            raise ValueError(f"q must be finite with shape ({self.model.nq},)")
        return result

    @staticmethod
    def _clamp_norm(value: np.ndarray, maximum: float) -> np.ndarray:
        norm = float(np.linalg.norm(value))
        if norm <= maximum or norm == 0.0:
            return value
        return value * (maximum / norm)

class FiniteLeadCommandFollower:
    """Convert joint velocity to CPV targets without accumulating command debt."""

    def __init__(
        self,
        size: int,
        *,
        max_velocity_rad_s: float,
        max_acceleration_rad_s2: float,
        command_lead_sec: float,
        max_command_lead_rad: float,
    ) -> None:
        if size <= 0:
            raise ValueError("size must be positive")
        if min(
            max_velocity_rad_s,
            max_acceleration_rad_s2,
            command_lead_sec,
            max_command_lead_rad,
        ) <= 0.0:
            raise ValueError("follower limits must be positive")
        self.size = int(size)
        self.max_velocity = float(max_velocity_rad_s)
        self.max_acceleration = float(max_acceleration_rad_s2)
        self.command_lead_sec = float(command_lead_sec)
        self.max_command_lead = float(max_command_lead_rad)
        self._velocity = np.zeros(self.size, dtype=np.float64)

    def reset(self) -> None:
        self._velocity.fill(0.0)

    def step(
        self,
        desired_velocity: np.ndarray,
        measured: np.ndarray,
        dt: float,
        lower: np.ndarray,
        upper: np.ndarray,
    ) -> CommandFollowerResult:
        desired = self._vector(desired_velocity, "desired_velocity")
        measured = self._vector(measured, "measured")
        lower = self._vector(lower, "lower")
        upper = self._vector(upper, "upper")
        if dt <= 0.0:
            raise ValueError("dt must be positive")
        desired = np.clip(desired, -self.max_velocity, self.max_velocity)
        velocity_delta = np.clip(
            desired - self._velocity,
            -self.max_acceleration * dt,
            self.max_acceleration * dt,
        )
        velocity = np.clip(
            self._velocity + velocity_delta,
            -self.max_velocity,
            self.max_velocity,
        )
        raw_lead = velocity * self.command_lead_sec
        peak_lead = float(np.max(np.abs(raw_lead)))
        lead_limited = peak_lead > self.max_command_lead
        if lead_limited:
            raw_lead *= self.max_command_lead / peak_lead
        raw_command = measured + raw_lead
        command = np.clip(raw_command, lower, upper)
        limit_clamped = not np.array_equal(command, raw_command)
        blocked = command != raw_command
        velocity[blocked] = 0.0
        self._velocity = velocity
        return CommandFollowerResult(
            command=command,
            joint_velocity=velocity.copy(),
            command_lead_deg=float(np.rad2deg(np.max(np.abs(command - measured)))),
            lead_limited=lead_limited,
            limit_clamped=limit_clamped,
        )

    def _vector(self, value: np.ndarray, label: str) -> np.ndarray:
        result = np.asarray(value, dtype=np.float64)
        if result.shape != (self.size,) or not np.isfinite(result).all():
            raise ValueError(f"{label} must be finite with shape ({self.size},)")
        return result
