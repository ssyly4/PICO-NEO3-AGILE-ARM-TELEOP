"""TCP contact-force estimation and downward-only motion guarding."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import pinocchio as pin


@dataclass(frozen=True)
class ContactForceSample:
    wrench_world: np.ndarray
    residual_joint_torque_nm: np.ndarray

    @property
    def vertical_force_n(self) -> float:
        return float(self.wrench_world[2])


class TcpForceEstimator:
    """Estimate an external TCP wrench from gravity-compensated motor torque.

    The NERO does not expose a six-axis wrist force sensor.  Its reported motor
    torques contain a pose-dependent gravity term plus a repeatable static bias.
    Calibration removes that bias at the starting pose; the URDF gravity model
    then tracks the change as the arm moves.
    """

    def __init__(
        self,
        model: pin.Model,
        frame_id: int,
        *,
        cutoff_hz: float = 3.0,
        wrench_damping: float = 0.08,
    ) -> None:
        if cutoff_hz <= 0.0 or wrench_damping <= 0.0:
            raise ValueError("force estimator parameters must be positive")
        self.model = model
        self.data = model.createData()
        self.frame_id = int(frame_id)
        self.cutoff_hz = float(cutoff_hz)
        self.wrench_damping = float(wrench_damping)
        self._bias_nm: np.ndarray | None = None
        self._filtered_wrench = np.zeros(6, dtype=np.float64)

    @property
    def calibrated(self) -> bool:
        return self._bias_nm is not None

    def gravity_torque(self, q: np.ndarray) -> np.ndarray:
        q = self._vector(q, "joint position")
        return np.asarray(
            pin.computeGeneralizedGravity(self.model, self.data, q),
            dtype=np.float64,
        ).copy()

    def calibrate(self, q_samples: np.ndarray, torque_samples_nm: np.ndarray) -> None:
        q_samples = np.asarray(q_samples, dtype=np.float64)
        torque_samples_nm = np.asarray(torque_samples_nm, dtype=np.float64)
        if q_samples.ndim != 2 or q_samples.shape[1] != self.model.nq:
            raise ValueError("calibration joint samples have invalid shape")
        if torque_samples_nm.shape != q_samples.shape or len(q_samples) < 5:
            raise ValueError("calibration torque samples have invalid shape")
        residuals = np.asarray(
            [tau - self.gravity_torque(q) for q, tau in zip(q_samples, torque_samples_nm)]
        )
        self._bias_nm = np.median(residuals, axis=0)
        self._filtered_wrench.fill(0.0)

    def update(
        self,
        q: np.ndarray,
        torque_nm: np.ndarray,
        dt: float,
    ) -> ContactForceSample:
        if self._bias_nm is None:
            raise RuntimeError("force estimator is not calibrated")
        q = self._vector(q, "joint position")
        torque_nm = self._vector(torque_nm, "joint torque")
        residual = torque_nm - self.gravity_torque(q) - self._bias_nm
        jacobian = pin.computeFrameJacobian(
            self.model,
            self.data,
            q,
            self.frame_id,
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
        )
        # residual = J.T @ wrench.  Ridge regularization prevents large wrench
        # estimates around weak/singular Cartesian directions.
        system = jacobian @ jacobian.T + self.wrench_damping**2 * np.eye(6)
        wrench = np.linalg.solve(system, jacobian @ residual)
        alpha = 1.0 - np.exp(-2.0 * np.pi * self.cutoff_hz * max(float(dt), 1e-4))
        self._filtered_wrench += alpha * (wrench - self._filtered_wrench)
        return ContactForceSample(self._filtered_wrench.copy(), residual.copy())

    def _vector(self, value: np.ndarray, label: str) -> np.ndarray:
        result = np.asarray(value, dtype=np.float64)
        if result.shape != (self.model.nv,) or not np.isfinite(result).all():
            raise ValueError(f"{label} must be a finite {self.model.nv}-vector")
        return result


class DownwardContactGuard:
    """Latch out descent on a local force change, preserving XY/upward escape.

    Absolute wrench estimates drift with pose because the URDF and motor-current
    model are imperfect.  A baseline is therefore tracked whenever there is no
    downward intent and frozen for each descent.  The guard operates on the
    magnitude of the change from that local baseline, not on absolute Fz.
    """

    def __init__(
        self,
        *,
        engage_force_n: float,
        release_force_n: float,
        engage_samples: int = 3,
        downward_intent_mm: float = 0.8,
        upward_release_mm: float = 2.0,
        baseline_alpha: float = 0.15,
    ) -> None:
        if engage_force_n <= 0.0 or not 0.0 <= release_force_n < engage_force_n:
            raise ValueError("force thresholds must satisfy 0 <= release < engage")
        if engage_samples < 1:
            raise ValueError("engage_samples must be positive")
        self.engage_force_n = float(engage_force_n)
        self.release_force_n = float(release_force_n)
        self.engage_samples = int(engage_samples)
        self.downward_intent_m = float(downward_intent_mm) / 1000.0
        self.upward_release_m = float(upward_release_mm) / 1000.0
        if not 0.0 < baseline_alpha <= 1.0:
            raise ValueError("baseline_alpha must be in (0, 1]")
        self.baseline_alpha = float(baseline_alpha)
        self.active = False
        self.floor_z_m: float | None = None
        self.baseline_vertical_force_n: float | None = None
        self.relative_load_n = 0.0
        self._confirmations = 0

    def apply(
        self,
        requested_position_m: np.ndarray,
        measured_position_m: np.ndarray,
        vertical_force_n: float,
    ) -> tuple[np.ndarray, str]:
        requested = np.asarray(requested_position_m, dtype=np.float64).copy()
        measured = np.asarray(measured_position_m, dtype=np.float64)
        downward = requested[2] < measured[2] - self.downward_intent_m
        if self.baseline_vertical_force_n is None:
            self.baseline_vertical_force_n = float(vertical_force_n)
        if not self.active and not downward:
            self.baseline_vertical_force_n += self.baseline_alpha * (
                float(vertical_force_n) - self.baseline_vertical_force_n
            )
            self.relative_load_n = 0.0
            self._confirmations = 0
        else:
            self.relative_load_n = abs(
                float(vertical_force_n) - self.baseline_vertical_force_n
            )
        if not self.active:
            self._confirmations = (
                self._confirmations + 1
                if downward and self.relative_load_n >= self.engage_force_n
                else 0
            )
            if self._confirmations >= self.engage_samples:
                self.active = True
                self.floor_z_m = float(measured[2])

        if not self.active:
            return requested, "clear"

        assert self.floor_z_m is not None
        if (
            self.relative_load_n <= self.release_force_n
            and requested[2] >= self.floor_z_m + self.upward_release_m
        ):
            self.active = False
            self.floor_z_m = None
            self.baseline_vertical_force_n = float(vertical_force_n)
            self.relative_load_n = 0.0
            self._confirmations = 0
            return requested, "released_upward"

        requested[2] = max(requested[2], self.floor_z_m)
        return requested, "downward_blocked"
