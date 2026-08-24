#!/usr/bin/env python3
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


# Unity: +X right, +Y up, +Z forward.
# Candidate NERO base: +X forward, +Y left, +Z up. This orthogonal
# coordinate conversion has determinant -1 because Unity is left-handed.
OPENXR_TO_NERO = np.asarray(
    [
        [0.0, 0.0, 1.0],
        [-1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
    ],
    dtype=np.float64,
)


@dataclass(frozen=True)
class Pose:
    position: np.ndarray
    rotation: np.ndarray


@dataclass(frozen=True)
class MapperOutput:
    state: str
    target: Pose
    trigger: float


@dataclass(frozen=True)
class HybridTranslationOutput:
    target_position: np.ndarray
    mode: str
    commanded_speed_m_s: float


def quaternion_to_matrix(quaternion_xyzw: np.ndarray) -> np.ndarray:
    quaternion = np.asarray(quaternion_xyzw, dtype=np.float64)
    if quaternion.shape != (4,) or not np.isfinite(quaternion).all():
        raise ValueError("quaternion must contain four finite XYZW values")
    norm = float(np.linalg.norm(quaternion))
    if norm < 1e-9:
        raise ValueError("quaternion norm is zero")
    x, y, z, w = quaternion / norm
    return np.asarray(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def matrix_to_quaternion(rotation: np.ndarray) -> np.ndarray:
    matrix = np.asarray(rotation, dtype=np.float64)
    if matrix.shape != (3, 3) or not np.isfinite(matrix).all():
        raise ValueError("rotation must be a finite 3x3 matrix")
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = 2.0 * np.sqrt(trace + 1.0)
        quaternion = np.asarray(
            [
                (matrix[2, 1] - matrix[1, 2]) / scale,
                (matrix[0, 2] - matrix[2, 0]) / scale,
                (matrix[1, 0] - matrix[0, 1]) / scale,
                0.25 * scale,
            ]
        )
    else:
        index = int(np.argmax(np.diag(matrix)))
        if index == 0:
            scale = 2.0 * np.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2])
            quaternion = np.asarray(
                [0.25 * scale, (matrix[0, 1] + matrix[1, 0]) / scale,
                 (matrix[0, 2] + matrix[2, 0]) / scale, (matrix[2, 1] - matrix[1, 2]) / scale]
            )
        elif index == 1:
            scale = 2.0 * np.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2])
            quaternion = np.asarray(
                [(matrix[0, 1] + matrix[1, 0]) / scale, 0.25 * scale,
                 (matrix[1, 2] + matrix[2, 1]) / scale, (matrix[0, 2] - matrix[2, 0]) / scale]
            )
        else:
            scale = 2.0 * np.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1])
            quaternion = np.asarray(
                [(matrix[0, 2] + matrix[2, 0]) / scale,
                 (matrix[1, 2] + matrix[2, 1]) / scale, 0.25 * scale,
                 (matrix[1, 0] - matrix[0, 1]) / scale]
            )
    quaternion /= np.linalg.norm(quaternion)
    return quaternion if quaternion[3] >= 0.0 else -quaternion


def head_yaw_world_to_view(head_state: dict) -> np.ndarray:
    rotation = quaternion_to_matrix(
        np.asarray(
            [head_state["qx"], head_state["qy"], head_state["qz"], head_state["qw"]]
        )
    )
    forward = rotation @ np.asarray([0.0, 0.0, 1.0])
    forward[1] = 0.0
    norm = float(np.linalg.norm(forward))
    if norm < 1e-6:
        raise ValueError("head forward direction is vertical")
    forward /= norm
    up = np.asarray([0.0, 1.0, 0.0])
    right = np.cross(up, forward)
    right /= np.linalg.norm(right)
    return np.stack([right, up, forward])


def transform_state_to_view(state: dict, world_to_view: np.ndarray) -> dict:
    basis = np.asarray(world_to_view, dtype=np.float64)
    transformed = dict(state)
    position = basis @ np.asarray([state["px"], state["py"], state["pz"]])
    rotation = basis @ quaternion_to_matrix(
        np.asarray([state["qx"], state["qy"], state["qz"], state["qw"]])
    )
    velocity = basis @ np.asarray(
        [state.get("vx", 0.0), state.get("vy", 0.0), state.get("vz", 0.0)]
    )
    angular_velocity = basis @ np.asarray(
        [state.get("avx", 0.0), state.get("avy", 0.0), state.get("avz", 0.0)]
    )
    transformed.update(
        zip(("px", "py", "pz"), position.tolist())
    )
    transformed.update(
        zip(("qx", "qy", "qz", "qw"), matrix_to_quaternion(rotation).tolist())
    )
    transformed.update(zip(("vx", "vy", "vz"), velocity.tolist()))
    transformed.update(zip(("avx", "avy", "avz"), angular_velocity.tolist()))
    return transformed


def rotation_vector_to_matrix(vector: np.ndarray) -> np.ndarray:
    value = np.asarray(vector, dtype=np.float64)
    angle = float(np.linalg.norm(value))
    if angle < 1e-12:
        return np.eye(3)
    axis = value / angle
    x, y, z = axis
    skew = np.asarray([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    return np.eye(3) + np.sin(angle) * skew + (1.0 - np.cos(angle)) * (skew @ skew)


def matrix_to_rotation_vector(rotation: np.ndarray) -> np.ndarray:
    matrix = np.asarray(rotation, dtype=np.float64)
    cosine = float(np.clip((np.trace(matrix) - 1.0) / 2.0, -1.0, 1.0))
    angle = float(np.arccos(cosine))
    if angle < 1e-9:
        return np.zeros(3, dtype=np.float64)
    if np.pi - angle < 1e-5:
        eigenvalues, eigenvectors = np.linalg.eig(matrix)
        axis = np.real(eigenvectors[:, np.argmin(np.abs(eigenvalues - 1.0))])
        axis /= np.linalg.norm(axis)
        return axis * angle
    vector = np.asarray(
        [matrix[2, 1] - matrix[1, 2], matrix[0, 2] - matrix[2, 0], matrix[1, 0] - matrix[0, 1]]
    )
    return vector * angle / (2.0 * np.sin(angle))


def align_local_axis(
    reference_rotation: np.ndarray,
    desired_rotation: np.ndarray,
    *,
    local_axis: np.ndarray = np.asarray([1.0, 0.0, 0.0]),
) -> np.ndarray:
    """Align one tool axis while retaining the reference twist about that axis."""
    reference = np.asarray(reference_rotation, dtype=np.float64)
    desired = np.asarray(desired_rotation, dtype=np.float64)
    axis = np.asarray(local_axis, dtype=np.float64).copy()
    if reference.shape != (3, 3) or desired.shape != (3, 3):
        raise ValueError("reference_rotation and desired_rotation must be 3x3")
    if axis.shape != (3,) or not np.isfinite(axis).all():
        raise ValueError("local_axis must contain three finite values")
    axis_norm = float(np.linalg.norm(axis))
    if axis_norm < 1e-9:
        raise ValueError("local_axis must be non-zero")
    axis /= axis_norm

    source = reference @ axis
    target = desired @ axis
    cross = np.cross(source, target)
    sine = float(np.linalg.norm(cross))
    cosine = float(np.clip(np.dot(source, target), -1.0, 1.0))
    if sine > 1e-9:
        swing = rotation_vector_to_matrix(cross / sine * np.arctan2(sine, cosine))
    elif cosine >= 0.0:
        swing = np.eye(3)
    else:
        candidate = np.asarray([1.0, 0.0, 0.0])
        if abs(float(np.dot(candidate, source))) > 0.9:
            candidate = np.asarray([0.0, 1.0, 0.0])
        perpendicular = np.cross(source, candidate)
        perpendicular /= np.linalg.norm(perpendicular)
        swing = rotation_vector_to_matrix(np.pi * perpendicular)
    return swing @ reference


def rpy_to_matrix(rpy: np.ndarray) -> np.ndarray:
    roll, pitch, yaw = np.asarray(rpy, dtype=np.float64)
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.asarray(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def matrix_to_rpy(rotation: np.ndarray) -> np.ndarray:
    matrix = np.asarray(rotation, dtype=np.float64)
    pitch = float(np.arctan2(-matrix[2, 0], np.hypot(matrix[0, 0], matrix[1, 0])))
    if abs(np.cos(pitch)) > 1e-7:
        roll = float(np.arctan2(matrix[2, 1], matrix[2, 2]))
        yaw = float(np.arctan2(matrix[1, 0], matrix[0, 0]))
    else:
        roll = 0.0
        yaw = float(np.arctan2(-matrix[0, 1], matrix[1, 1]))
    return np.asarray([roll, pitch, yaw])


def controller_pose(state: dict) -> Pose:
    return Pose(
        position=np.asarray([state["px"], state["py"], state["pz"]], dtype=np.float64),
        rotation=quaternion_to_matrix(
            np.asarray([state["qx"], state["qy"], state["qz"], state["qw"]])
        ),
    )


class ClutchedPoseMapper:
    def __init__(
        self,
        initial_target: Pose,
        *,
        translation_scale: float = 0.2,
        max_translation_m: float = 0.03,
        max_rotation_rad: float = np.deg2rad(15.0),
        rotation_scale: float = 1.0,
        # Grip is an analog controller signal.  Do not require a full squeeze
        # to enter motion mode, but keep hysteresis so light noise cannot drop
        # and immediately re-enter the clutch.
        engage_threshold: float = 0.25,
        release_threshold: float = 0.12,
        basis: np.ndarray = OPENXR_TO_NERO,
        axis_gain: np.ndarray | None = None,
    ) -> None:
        if (
            translation_scale <= 0
            or max_translation_m <= 0
            or max_rotation_rad <= 0
            or rotation_scale <= 0
        ):
            raise ValueError("mapping scales and limits must be positive")
        if not 0 <= release_threshold < engage_threshold <= 1:
            raise ValueError("clutch thresholds must satisfy 0 <= release < engage <= 1")
        self.translation_scale = translation_scale
        self.max_translation_m = max_translation_m
        self.max_rotation_rad = max_rotation_rad
        self.rotation_scale = rotation_scale
        self.engage_threshold = engage_threshold
        self.release_threshold = release_threshold
        self.basis = np.asarray(basis, dtype=np.float64)
        self.axis_gain = (
            np.ones(3, dtype=np.float64)
            if axis_gain is None
            else np.asarray(axis_gain, dtype=np.float64)
        )
        if self.axis_gain.shape != (3,) or not np.isfinite(self.axis_gain).all() or np.any(self.axis_gain <= 0):
            raise ValueError("axis_gain must contain three positive finite values")
        self.target = initial_target
        self.engaged = False
        self.controller_anchor: Pose | None = None
        self.robot_anchor: Pose | None = None

    def reset_target(self, target: Pose) -> None:
        """Disengage the clutch and make live robot feedback the next anchor."""
        self.target = target
        self.engaged = False
        self.controller_anchor = None
        self.robot_anchor = None

    def update(self, state: dict) -> MapperOutput:
        trigger = float(state["trigger"])
        grip = float(state["grip"])
        if not bool(state["tracked"]):
            self.engaged = False
            return MapperOutput("untracked_hold", self.target, trigger)

        pose = controller_pose(state)
        if self.engaged and grip <= self.release_threshold:
            self.engaged = False
            self.controller_anchor = None
            self.robot_anchor = None
            return MapperOutput("released_hold", self.target, trigger)

        if not self.engaged:
            if grip < self.engage_threshold:
                return MapperOutput("holding", self.target, trigger)
            self.engaged = True
            self.controller_anchor = pose
            self.robot_anchor = self.target
            return MapperOutput("clutch_engaged", self.target, trigger)

        assert self.controller_anchor is not None and self.robot_anchor is not None
        translation = self.translation_scale * self.axis_gain * (
            self.basis @ (pose.position - self.controller_anchor.position)
        )
        translation_norm = float(np.linalg.norm(translation))
        if translation_norm > self.max_translation_m:
            translation *= self.max_translation_m / translation_norm

        # Conjugation by the improper basis conversion still yields a proper
        # rotation. Rotation axes acquire det(B), as required for pseudovectors.
        openxr_delta = pose.rotation @ self.controller_anchor.rotation.T
        robot_delta = self.basis @ openxr_delta @ self.basis.T
        rotation_vector = self.rotation_scale * matrix_to_rotation_vector(robot_delta)
        rotation_norm = float(np.linalg.norm(rotation_vector))
        if rotation_norm > self.max_rotation_rad:
            rotation_vector *= self.max_rotation_rad / rotation_norm
        robot_delta = rotation_vector_to_matrix(rotation_vector)

        self.target = Pose(
            position=self.robot_anchor.position + translation,
            rotation=robot_delta @ self.robot_anchor.rotation,
        )
        return MapperOutput("tracking", self.target, trigger)


class HybridTranslationController:
    """Use direct mapping nearby and feedback-rebased rate control farther away."""

    def __init__(
        self,
        *,
        enter_radius_m: float,
        rate_deadzone_m: float,
        full_rate_radius_m: float,
        max_speed_m_s: float,
        max_target_lead_m: float,
    ) -> None:
        if not 0.0 <= rate_deadzone_m < enter_radius_m < full_rate_radius_m:
            raise ValueError("hybrid radii must satisfy deadzone < enter < full-rate")
        if max_speed_m_s <= 0.0 or max_target_lead_m <= 0.0:
            raise ValueError("hybrid speed and target lead limits must be positive")
        self.enter_radius_m = float(enter_radius_m)
        self.rate_deadzone_m = float(rate_deadzone_m)
        self.full_rate_radius_m = float(full_rate_radius_m)
        self.max_speed_m_s = float(max_speed_m_s)
        self.max_target_lead_m = float(max_target_lead_m)
        self.anchor_position: np.ndarray | None = None
        self.target_position: np.ndarray | None = None
        self.rate_mode = False

    def engage(self, anchor_position: np.ndarray) -> None:
        anchor = np.asarray(anchor_position, dtype=np.float64)
        if anchor.shape != (3,) or not np.isfinite(anchor).all():
            raise ValueError("hybrid anchor must be a finite 3-vector")
        self.anchor_position = anchor.copy()
        self.target_position = anchor.copy()
        self.rate_mode = False

    def reset(self) -> None:
        self.anchor_position = None
        self.target_position = None
        self.rate_mode = False

    def update(
        self,
        direct_target_position: np.ndarray,
        *,
        feedback_position: np.ndarray,
        dt: float,
    ) -> HybridTranslationOutput:
        if self.anchor_position is None or self.target_position is None:
            raise RuntimeError("hybrid controller must be engaged before update")
        direct_target = np.asarray(direct_target_position, dtype=np.float64)
        if direct_target.shape != (3,) or not np.isfinite(direct_target).all():
            raise ValueError("direct target must be a finite 3-vector")
        feedback = np.asarray(feedback_position, dtype=np.float64)
        if feedback.shape != (3,) or not np.isfinite(feedback).all():
            raise ValueError("feedback position must be a finite 3-vector")
        if not np.isfinite(dt) or dt <= 0.0:
            raise ValueError("hybrid dt must be finite and positive")

        deflection = direct_target - self.anchor_position
        magnitude = float(np.linalg.norm(deflection))
        if not self.rate_mode and magnitude < self.enter_radius_m:
            self.target_position = direct_target.copy()
            return HybridTranslationOutput(self.target_position.copy(), "position", 0.0)

        if not self.rate_mode:
            self.rate_mode = True

        speed_fraction = float(
            np.clip(
                (magnitude - self.rate_deadzone_m)
                / (self.full_rate_radius_m - self.rate_deadzone_m),
                0.0,
                1.0,
            )
        )
        speed = self.max_speed_m_s * speed_fraction
        if magnitude > 1e-9 and speed > 0.0:
            # Rate mode must stay closed around the *actual* flange position.
            # The previous implementation accumulated a target from the clutch
            # anchor and could leave it 80 mm ahead of the robot.  That made a
            # reversed hand motion feel ineffective until the old target was
            # consumed.  Rebuild the lead from current feedback every tick.
            direction = deflection / magnitude
            lead_distance = self.max_target_lead_m * (speed / self.max_speed_m_s)
            self.target_position = feedback + direction * lead_distance
        else:
            # Returning the controller to its center stops rate motion at the
            # live flange instead of continuing an old accumulated command.
            self.target_position = feedback.copy()
        mode = "rate" if speed > 0.0 else "rate_hold"
        return HybridTranslationOutput(self.target_position.copy(), mode, speed)
