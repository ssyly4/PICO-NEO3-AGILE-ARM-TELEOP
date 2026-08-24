from pathlib import Path
import unittest

import numpy as np
import pinocchio as pin

from nero_neo_teleop.control.servo_v3_core import (
    bounded_pose_target,
    bounded_transport_step,
    extrapolate_controller_state,
    FiniteLeadCommandFollower,
    PinocchioVelocityServo,
    PoseLowPassFilter,
)
from nero_neo_teleop.runtime import NERO_URDF

URDF = NERO_URDF


class VelocityServoTests(unittest.TestCase):
    def setUp(self):
        self.servo = PinocchioVelocityServo(URDF)
        self.q = np.deg2rad(np.array([5.0, -15.0, -8.0, 90.0, -8.0, -15.0, 35.0]))

    def test_hold_has_zero_velocity(self):
        pose = self.servo.pose(self.q)
        result = self.servo.solve(self.q, pose.translation, pose.rotation)
        self.assertLess(np.max(np.abs(result.joint_velocity)), 1e-9)

    def test_velocity_integration_reduces_cartesian_error(self):
        start = self.servo.pose(self.q)
        target = start.translation + np.array([0.012, -0.006, 0.004])
        q = self.q.copy()
        errors = []
        for _ in range(120):
            result = self.servo.solve(q, target, start.rotation)
            errors.append(result.position_error_mm)
            q = pin.integrate(self.servo.model, q, result.joint_velocity / 60.0)
        self.assertLess(errors[-1], 0.2)
        self.assertLess(errors[-1], errors[0])

    def test_joint_velocity_is_bounded(self):
        pose = self.servo.pose(self.q)
        result = self.servo.solve(
            self.q,
            pose.translation + np.array([0.2, 0.2, -0.2]),
            pin.exp3(np.array([0.5, -0.5, 0.5])) @ pose.rotation,
        )
        self.assertLessEqual(
            np.max(np.abs(result.joint_velocity)), self.servo.max_joint_speed_rad_s + 1e-12
        )

    def test_orientation_has_full_weight_away_from_joint_limits(self):
        pose = self.servo.pose(self.q)
        result = self.servo.solve(self.q, pose.translation, pose.rotation)
        self.assertAlmostEqual(result.orientation_limit_scale, 1.0)

    def test_orientation_is_softened_near_joint_limit_without_disabling_translation(self):
        q = self.q.copy()
        q[5] = self.servo.lower[5] + np.deg2rad(4.0)
        pose = self.servo.pose(q)
        result = self.servo.solve(
            q,
            pose.translation + np.array([0.01, 0.0, 0.0]),
            pin.exp3(np.array([0.0, 0.0, 0.2])) @ pose.rotation,
        )
        self.assertLess(result.orientation_limit_scale, 0.1)
        self.assertGreater(np.linalg.norm(result.joint_velocity), 1e-4)
        self.assertLessEqual(
            result.nullspace_velocity_norm,
            np.sqrt(7.0) * 0.4 * self.servo.max_joint_speed_rad_s + 1e-12,
        )


class FiniteLeadFollowerTests(unittest.TestCase):
    def setUp(self):
        self.follower = FiniteLeadCommandFollower(
            7,
            max_velocity_rad_s=np.deg2rad(15.0),
            max_acceleration_rad_s2=np.deg2rad(45.0),
            command_lead_sec=2.0 / 30.0,
            max_command_lead_rad=np.deg2rad(1.0),
        )

    def test_command_is_formed_from_live_feedback(self):
        measured = np.zeros(7)
        result = self.follower.step(
            np.full(7, np.deg2rad(10.0)), measured, 1.0 / 30.0,
            np.full(7, -2.0), np.full(7, 2.0),
        )
        self.assertGreater(result.command[0], 0.0)
        self.assertLessEqual(result.command_lead_deg, 1.0 + 1e-12)

    def test_feedback_catchup_does_not_accumulate_old_position(self):
        measured = np.zeros(7)
        desired = np.full(7, np.deg2rad(15.0))
        first = self.follower.step(
            desired, measured, 1.0 / 30.0, np.full(7, -2.0), np.full(7, 2.0)
        )
        measured = first.command.copy()
        second = self.follower.step(
            desired, measured, 1.0 / 30.0, np.full(7, -2.0), np.full(7, 2.0)
        )
        self.assertLessEqual(
            np.rad2deg(np.max(np.abs(second.command - measured))), 1.0 + 1e-12
        )

    def test_zero_velocity_decelerates_without_position_debt(self):
        bounds = (np.full(7, -2.0), np.full(7, 2.0))
        measured = np.zeros(7)
        self.follower.step(
            np.full(7, np.deg2rad(15.0)), measured, 1.0 / 30.0, *bounds
        )
        result = self.follower.step(np.zeros(7), measured, 1.0 / 30.0, *bounds)
        self.assertLessEqual(result.command_lead_deg, 1.0 + 1e-12)


class TransportAndFilterTests(unittest.TestCase):
    def test_bounded_pose_target_caps_translation_and_rotation(self):
        result = bounded_pose_target(
            np.zeros(3),
            np.eye(3),
            np.array([0.1, 0.0, 0.0]),
            pin.exp3(np.array([0.0, 0.0, 0.5])),
            max_position_lead_m=0.035,
            max_orientation_lead_rad=0.2,
        )
        self.assertTrue(result.limited)
        self.assertIs(type(result.limited), bool)
        self.assertAlmostEqual(np.linalg.norm(result.position), 0.035)
        self.assertAlmostEqual(np.linalg.norm(pin.log3(result.rotation)), 0.2)
        self.assertAlmostEqual(result.position_debt_mm, 100.0)

    def test_bounded_pose_target_preserves_nearby_target(self):
        desired_position = np.array([0.01, -0.005, 0.002])
        desired_rotation = pin.exp3(np.array([0.01, -0.02, 0.03]))
        result = bounded_pose_target(
            np.zeros(3),
            np.eye(3),
            desired_position,
            desired_rotation,
            max_position_lead_m=0.035,
            max_orientation_lead_rad=0.2,
        )
        self.assertFalse(result.limited)
        np.testing.assert_allclose(result.position, desired_position)
        np.testing.assert_allclose(result.rotation, desired_rotation)

    def test_transport_preserves_joint_direction_ratios(self):
        current = np.zeros(3)
        target = np.deg2rad(np.array([0.4, -1.2, 0.8]))
        result, limited = bounded_transport_step(current, target, np.deg2rad(0.9))
        self.assertTrue(limited)
        np.testing.assert_allclose(result / target, np.full(3, 0.75))

    def test_pose_filter_converges_without_rotation_corruption(self):
        filt = PoseLowPassFilter(position_cutoff_hz=10.0, rotation_cutoff_hz=8.0)
        filt.reset(np.zeros(3), np.eye(3))
        target_rotation = pin.exp3(np.array([0.1, -0.2, 0.05]))
        for _ in range(60):
            position, rotation = filt.update(np.ones(3), target_rotation, 1.0 / 30.0)
        np.testing.assert_allclose(position, np.ones(3), atol=1e-8)
        np.testing.assert_allclose(rotation.T @ rotation, np.eye(3), atol=1e-10)
        self.assertLess(np.linalg.norm(pin.log3(rotation.T @ target_rotation)), 1e-8)

    def test_controller_prediction_uses_linear_and_angular_velocity(self):
        state = {
            "px": 1.0, "py": 2.0, "pz": 3.0,
            "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0,
            "vx": 0.2, "vy": -0.1, "vz": 0.4,
            "avx": 0.0, "avy": 0.0, "avz": 1.0,
            "grip": 0.8,
        }
        predicted = extrapolate_controller_state(state, 0.1)
        np.testing.assert_allclose(
            [predicted["px"], predicted["py"], predicted["pz"]],
            [1.02, 1.99, 3.04],
        )
        rotation = pin.exp3(np.array([0.0, 0.0, 0.1]))
        from nero_neo_teleop.pico.pose_mapper import quaternion_to_matrix
        np.testing.assert_allclose(
            quaternion_to_matrix(
                np.array([predicted["qx"], predicted["qy"], predicted["qz"], predicted["qw"]])
            ),
            rotation,
            atol=1e-10,
        )
        self.assertEqual(predicted["grip"], 0.8)


if __name__ == "__main__":
    unittest.main()
