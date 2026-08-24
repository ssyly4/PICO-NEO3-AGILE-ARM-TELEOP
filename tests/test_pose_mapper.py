import unittest

import numpy as np

from nero_neo_teleop.pico.pico_input import apply_deadzone, joystick_rotation_step
from nero_neo_teleop.pico.pose_mapper import (
    OPENXR_TO_NERO,
    ClutchedPoseMapper,
    HybridTranslationController,
    Pose,
    align_local_axis,
    matrix_to_rotation_vector,
    head_yaw_world_to_view,
    quaternion_to_matrix,
    rotation_vector_to_matrix,
    transform_state_to_view,
)


def state(*, position=(0, 0, 0), quaternion=(0, 0, 0, 1), grip=0.0, tracked=True):
    return {
        "tracked": tracked,
        "px": position[0],
        "py": position[1],
        "pz": position[2],
        "qx": quaternion[0],
        "qy": quaternion[1],
        "qz": quaternion[2],
        "qw": quaternion[3],
        "trigger": 0.0,
        "grip": grip,
    }


class PoseMapperTest(unittest.TestCase):
    def setUp(self):
        self.initial = Pose(np.asarray([0.3, 0.0, 0.3]), np.eye(3))
        self.mapper = ClutchedPoseMapper(
            self.initial,
            translation_scale=1.0,
            max_translation_m=1.0,
            max_rotation_rad=np.pi,
        )
        self.mapper.update(state(grip=1.0))

    def test_axis_mapping(self):
        output = self.mapper.update(state(position=(0.1, 0.2, 0.3), grip=1.0))
        np.testing.assert_allclose(output.target.position - self.initial.position, [0.3, -0.1, 0.2])

    def test_forward_axis_can_be_inverted_independently(self):
        basis = OPENXR_TO_NERO.copy()
        basis[0] *= -1.0
        mapper = ClutchedPoseMapper(
            self.initial,
            translation_scale=1.0,
            max_translation_m=1.0,
            max_rotation_rad=np.pi,
            basis=basis,
        )
        mapper.update(state(grip=1.0))
        output = mapper.update(state(position=(0.1, 0.2, 0.3), grip=1.0))
        np.testing.assert_allclose(
            output.target.position - self.initial.position,
            [-0.3, -0.1, 0.2],
        )

    def test_lateral_axis_can_be_inverted_independently(self):
        basis = OPENXR_TO_NERO.copy()
        basis[1] *= -1.0
        mapper = ClutchedPoseMapper(
            self.initial,
            translation_scale=1.0,
            max_translation_m=1.0,
            max_rotation_rad=np.pi,
            basis=basis,
        )
        mapper.update(state(grip=1.0))
        output = mapper.update(state(position=(0.1, 0.2, 0.3), grip=1.0))
        np.testing.assert_allclose(
            output.target.position - self.initial.position,
            [0.3, 0.1, 0.2],
        )

    def test_axis_gain_scales_robot_forward_only(self):
        mapper = ClutchedPoseMapper(
            self.initial,
            translation_scale=0.25,
            max_translation_m=1.0,
            max_rotation_rad=np.pi,
            basis=np.eye(3),
            axis_gain=np.asarray([1.5, 1.0, 1.0]),
        )
        mapper.update(state(grip=1.0))
        output = mapper.update(state(position=(0.1, 0.1, 0.1), grip=1.0))
        np.testing.assert_allclose(
            output.target.position - self.initial.position,
            [0.0375, 0.025, 0.025],
        )

    def test_y_axis_rotation_maps_to_negative_robot_z_due_to_handedness(self):
        angle = np.deg2rad(30.0)
        quaternion = (0.0, np.sin(angle / 2), 0.0, np.cos(angle / 2))
        output = self.mapper.update(state(quaternion=quaternion, grip=1.0))
        rotation_vector = matrix_to_rotation_vector(output.target.rotation)
        np.testing.assert_allclose(rotation_vector, [0.0, 0.0, -angle], atol=1e-7)

    def test_rotation_scale_reduces_controller_rotation(self):
        mapper = ClutchedPoseMapper(
            self.initial,
            translation_scale=1.0,
            max_translation_m=1.0,
            max_rotation_rad=np.pi,
            rotation_scale=0.5,
        )
        mapper.update(state(grip=1.0))
        angle = np.deg2rad(20.0)
        quaternion = (0.0, np.sin(angle / 2), 0.0, np.cos(angle / 2))
        output = mapper.update(state(quaternion=quaternion, grip=1.0))
        rotation_vector = matrix_to_rotation_vector(output.target.rotation)
        np.testing.assert_allclose(
            rotation_vector,
            [0.0, 0.0, -np.deg2rad(10.0)],
            atol=1e-7,
        )

    def test_release_holds_target(self):
        moved = self.mapper.update(state(position=(0.0, 0.0, 0.1), grip=1.0)).target
        released = self.mapper.update(state(position=(0.0, 0.0, 0.2), grip=0.0))
        self.assertEqual(released.state, "released_hold")
        np.testing.assert_allclose(released.target.position, moved.position)

    def test_reset_target_uses_live_feedback_for_next_clutch(self):
        self.mapper.update(state(position=(0.1, 0.0, 0.0), grip=1.0))
        live_pose = Pose(np.asarray([0.31, -0.12, 0.44]), np.eye(3))

        self.mapper.reset_target(live_pose)
        held = self.mapper.update(state(grip=0.0))
        engaged = self.mapper.update(state(position=(0.4, 0.2, 0.1), grip=1.0))

        self.assertEqual(held.state, "holding")
        self.assertEqual(engaged.state, "clutch_engaged")
        np.testing.assert_allclose(engaged.target.position, live_pose.position)

    def test_candidate_basis_is_orthogonal_reflection(self):
        np.testing.assert_allclose(OPENXR_TO_NERO @ OPENXR_TO_NERO.T, np.eye(3))
        self.assertAlmostEqual(np.linalg.det(OPENXR_TO_NERO), -1.0)

    def test_quaternion_normalizes(self):
        np.testing.assert_allclose(quaternion_to_matrix(np.asarray([0, 0, 0, 2])), np.eye(3))

    def test_head_yaw_turns_world_motion_into_view_forward(self):
        angle = np.deg2rad(90.0)
        head = state(quaternion=(0.0, np.sin(angle / 2), 0.0, np.cos(angle / 2)))
        world_to_view = head_yaw_world_to_view(head)
        np.testing.assert_allclose(
            world_to_view @ np.asarray([1.0, 0.0, 0.0]),
            [0.0, 0.0, 1.0],
            atol=1e-7,
        )

    def test_state_transform_uses_fixed_head_view_basis(self):
        angle = np.deg2rad(90.0)
        head = state(quaternion=(0.0, np.sin(angle / 2), 0.0, np.cos(angle / 2)))
        transformed = transform_state_to_view(
            state(position=(1.0, 2.0, 3.0)),
            head_yaw_world_to_view(head),
        )
        np.testing.assert_allclose(
            [transformed["px"], transformed["py"], transformed["pz"]],
            [-3.0, 2.0, 1.0],
            atol=1e-7,
        )
        np.testing.assert_allclose(
            quaternion_to_matrix(
                np.asarray(
                    [
                        transformed["qx"],
                        transformed["qy"],
                        transformed["qz"],
                        transformed["qw"],
                    ]
                )
            ),
            head_yaw_world_to_view(head),
            atol=1e-7,
        )


class OrientationModeTest(unittest.TestCase):
    def test_axis_alignment_removes_tool_roll(self):
        reference = np.eye(3)
        roll_only = rotation_vector_to_matrix(np.asarray([0.0, 0.0, 0.4]))
        aligned = align_local_axis(
            reference, roll_only, local_axis=np.asarray([0.0, 0.0, 1.0])
        )
        np.testing.assert_allclose(aligned, reference, atol=1e-9)

    def test_axis_alignment_tracks_tool_direction(self):
        reference = np.eye(3)
        desired = rotation_vector_to_matrix(np.asarray([0.2, -0.3, 0.1]))
        aligned = align_local_axis(
            reference, desired, local_axis=np.asarray([0.0, 0.0, 1.0])
        )
        np.testing.assert_allclose(aligned[:, 2], desired[:, 2], atol=1e-9)


class HybridTranslationTest(unittest.TestCase):
    def setUp(self):
        self.controller = HybridTranslationController(
            enter_radius_m=0.05,
            rate_deadzone_m=0.02,
            full_rate_radius_m=0.14,
            max_speed_m_s=0.06,
            max_target_lead_m=0.075,
        )
        self.controller.engage(np.zeros(3))

    def test_small_motion_uses_direct_position_mapping(self):
        output = self.controller.update(
            np.asarray([0.04, 0.0, 0.0]), feedback_position=np.zeros(3), dt=0.1
        )
        self.assertEqual(output.mode, "position")
        np.testing.assert_allclose(output.target_position, [0.04, 0.0, 0.0])

    def test_rate_transition_rebases_ahead_of_live_feedback(self):
        entering = self.controller.update(
            np.asarray([0.05, 0.0, 0.0]),
            feedback_position=np.zeros(3),
            dt=0.0 + 1e-9,
        )
        self.assertEqual(entering.mode, "rate")
        np.testing.assert_allclose(entering.target_position, [0.01875, 0.0, 0.0], atol=1e-9)
        advanced = self.controller.update(
            np.asarray([0.14, 0.0, 0.0]),
            feedback_position=np.zeros(3),
            dt=0.1,
        )
        self.assertEqual(advanced.mode, "rate")
        np.testing.assert_allclose(advanced.target_position, [0.075, 0.0, 0.0], atol=1e-8)
        self.assertAlmostEqual(advanced.commanded_speed_m_s, 0.06)

    def test_returning_to_center_stops_at_live_feedback(self):
        self.controller.update(
            np.asarray([0.14, 0.0, 0.0]), feedback_position=np.zeros(3), dt=0.1
        )
        before = self.controller.update(
            np.asarray([0.14, 0.0, 0.0]), feedback_position=np.zeros(3), dt=0.1
        )
        held = self.controller.update(
            np.zeros(3), feedback_position=np.zeros(3), dt=0.1
        )
        self.assertEqual(held.mode, "rate_hold")
        np.testing.assert_allclose(held.target_position, np.zeros(3))

    def test_rate_direction_reverses_from_live_feedback(self):
        self.controller.update(
            np.asarray([0.14, 0.0, 0.0]), feedback_position=np.zeros(3), dt=0.1
        )
        reversed_output = self.controller.update(
            np.asarray([-0.14, 0.0, 0.0]),
            feedback_position=np.asarray([0.01, 0.0, 0.0]),
            dt=0.1,
        )
        self.assertEqual(reversed_output.mode, "rate")
        np.testing.assert_allclose(
            reversed_output.target_position, [-0.065, 0.0, 0.0], atol=1e-9
        )

    def test_rate_target_is_bounded_ahead_of_live_feedback_not_anchor(self):
        feedback = np.zeros(3)
        for _ in range(100):
            output = self.controller.update(
                np.asarray([0.14, 0.0, 0.0]),
                feedback_position=feedback,
                dt=0.1,
            )
            feedback[0] += 0.004
        self.assertGreater(output.target_position[0], 0.25)
        self.assertLessEqual(
            np.linalg.norm(output.target_position - feedback), 0.075 + 0.004 + 1e-12
        )

    def test_reset_requires_new_engagement(self):
        self.controller.reset()
        with self.assertRaises(RuntimeError):
            self.controller.update(
                np.zeros(3), feedback_position=np.zeros(3), dt=0.1
            )


class JoystickTest(unittest.TestCase):
    def test_deadzone_suppresses_center_noise(self):
        self.assertEqual(apply_deadzone(0.10, 0.18), 0.0)
        self.assertEqual(apply_deadzone(-0.18, 0.18), 0.0)

    def test_deadzone_preserves_direction_and_full_scale(self):
        self.assertAlmostEqual(apply_deadzone(1.0, 0.18), 1.0)
        self.assertAlmostEqual(apply_deadzone(-1.0, 0.18), -1.0)
        self.assertGreater(apply_deadzone(0.5, 0.18), 0.0)

    def test_stick_preserves_established_horizontal_roll_and_vertical_yaw(self):
        np.testing.assert_allclose(
            joystick_rotation_step(0.5, -0.25, rate_rad_s=2.0, dt=0.1),
            [0.1, 0.0, -0.05],
        )


if __name__ == "__main__":
    unittest.main()
