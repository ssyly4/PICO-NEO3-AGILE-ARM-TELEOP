import time
from pathlib import Path
import tempfile
import unittest

import numpy as np

from nero_neo_teleop.recording.bimanual_lerobot_recorder import (
    ArmState,
    BimanualSample,
    InactivityAutoStop,
    NeroCanStateSource,
    SingleReleaseAutoStop,
    STATE_DOF,
    SyntheticStateSource,
    controller_action,
    dataset_features,
    feedback_monotonic_ns,
    max_joint_speed_deg_s,
    resolved_camera_device,
    validate_existing_dataset,
)
from nero_neo_teleop.recording.action_command_stream import ArmCommandPublisher, ArmCommandReceiver
from nero_vla.camera_reader import CameraFrame


def frame(sequence: int) -> CameraFrame:
    now = time.monotonic_ns()
    return CameraFrame(np.zeros((8, 8, 3), dtype=np.uint8), now, time.time_ns(), sequence)


class BimanualRecorderTest(unittest.TestCase):
    def test_timestamped_controller_action_combines_both_executed_targets(self):
        with tempfile.TemporaryDirectory() as directory:
            now = time.monotonic_ns()
            left_receiver = ArmCommandReceiver(Path(directory) / "left.sock", "left")
            right_receiver = ArmCommandReceiver(Path(directory) / "right.sock", "right")
            left_receiver.start()
            right_receiver.start()
            left_publisher = ArmCommandPublisher(str(Path(directory) / "left.sock"), "left")
            right_publisher = ArmCommandPublisher(str(Path(directory) / "right.sock"), "right")
            try:
                left_publisher.publish(
                    joint_target_rad=np.arange(7) * 0.01,
                    gripper_target_normalized=0.25,
                    monotonic_ns=now,
                    sequence=3,
                    control_state="velocity_tracking",
                )
                right_publisher.publish(
                    joint_target_rad=np.arange(7) * -0.02,
                    gripper_target_normalized=0.75,
                    monotonic_ns=now + 1_000_000,
                    sequence=4,
                    control_state="input_hold",
                )
                left_receiver.wait_ready(after_ns=now, timeout_sec=1.0)
                right_receiver.wait_ready(after_ns=now, timeout_sec=1.0)
                state = np.zeros(8, dtype=np.float32)
                image = frame(0)
                observation = BimanualSample(
                    now,
                    ArmState(state, now, time.time_ns()),
                    ArmState(state, now, time.time_ns()),
                    np.zeros(16, dtype=np.float32),
                    image,
                    image,
                    image,
                )
                action = controller_action(observation, left_receiver, right_receiver)
                np.testing.assert_allclose(action.vector[:7], np.arange(7) * 0.01)
                np.testing.assert_allclose(action.vector[8:15], np.arange(7) * -0.02)
                self.assertAlmostEqual(float(action.vector[7]), 0.25)
                self.assertAlmostEqual(float(action.vector[15]), 0.75)
                self.assertEqual(action.source, "controller_command")
                self.assertEqual((action.left_sequence, action.right_sequence), (3, 4))
            finally:
                left_publisher.close()
                right_publisher.close()
                left_receiver.stop()
                right_receiver.stop()

    def test_raw_can_scaling_matches_vendor_codec(self):
        self.assertAlmostEqual(np.deg2rad(90000 * 1e-3), np.pi / 2)
        self.assertAlmostEqual(45000 * 1e-6 / 0.09, 0.5)

    def test_feature_contract(self):
        features = dataset_features(480, 640, "video")
        self.assertEqual(features["observation.state"]["shape"], (STATE_DOF,))
        self.assertEqual(features["action"]["shape"], (STATE_DOF,))
        self.assertEqual(features["observation.images.world"]["shape"], (480, 640, 3))
        self.assertEqual(len(features["observation.state"]["names"]), STATE_DOF)

    def test_synthetic_state_is_finite(self):
        source = SyntheticStateSource(0.0)
        state = source.snapshot()
        self.assertEqual(state.vector.shape, (8,))
        self.assertTrue(np.isfinite(state.vector).all())

    def test_motion_speed_uses_both_arms(self):
        left = SyntheticStateSource(0.0).snapshot()
        right = SyntheticStateSource(0.0).snapshot()
        current_right = type(right)(right.vector.copy(), right.monotonic_ns, right.unix_ns)
        current_right.vector[0] = np.deg2rad(3.0)
        image = frame(0)
        previous = BimanualSample(0, left, right, np.r_[left.vector, right.vector], image, image, image)
        current = BimanualSample(
            100_000_000,
            left,
            current_right,
            np.r_[left.vector, current_right.vector],
            image,
            image,
            image,
        )
        self.assertAlmostEqual(max_joint_speed_deg_s(previous, current), 30.0, places=3)

    def test_feedback_timestamp_accepts_unix_clock(self):
        age_sec = (time.monotonic_ns() - feedback_monotonic_ns(time.time())) / 1e9
        self.assertLess(abs(age_sec), 0.01)

    def test_feedback_timestamp_accepts_monotonic_clock(self):
        age_sec = (time.monotonic_ns() - feedback_monotonic_ns(time.monotonic())) / 1e9
        self.assertLess(abs(age_sec), 0.01)

    def test_camera_path_is_resolved_for_opencv(self):
        self.assertEqual(resolved_camera_device("/dev/null"), "/dev/null")

    def test_right_release_auto_stop_requires_close_then_open_and_stationary(self):
        image = frame(0)

        def sample(timestamp_ns: int, right_gripper: float) -> BimanualSample:
            left = SyntheticStateSource(0.0).snapshot()
            right = SyntheticStateSource(0.0).snapshot()
            left_vector = np.zeros(8, dtype=np.float32)
            right_vector = np.zeros(8, dtype=np.float32)
            left_vector[7] = 1.0
            right_vector[7] = right_gripper
            left = type(left)(left_vector, timestamp_ns, timestamp_ns)
            right = type(right)(right_vector, timestamp_ns, timestamp_ns)
            return BimanualSample(
                timestamp_ns,
                left,
                right,
                np.r_[left_vector, right_vector],
                image,
                image,
                image,
            )

        detector = SingleReleaseAutoStop(
            side="right",
            closed_threshold=0.45,
            open_threshold=0.75,
            stationary_sec=0.5,
            stationary_speed_deg_s=0.5,
        )
        previous = sample(0, 1.0)
        closed = sample(100_000_000, 0.2)
        released = sample(200_000_000, 0.9)
        settled = sample(700_000_000, 0.9)
        self.assertFalse(detector.update(previous, closed)[0])
        self.assertFalse(detector.update(closed, released)[0])
        self.assertTrue(detector.update(released, settled)[0])

    def test_inactivity_auto_stop_resets_when_either_arm_moves(self):
        image = frame(0)

        def sample(timestamp_ns: int, left_joint_deg: float = 0.0) -> BimanualSample:
            left_vector = np.zeros(8, dtype=np.float32)
            right_vector = np.zeros(8, dtype=np.float32)
            left_vector[0] = np.deg2rad(left_joint_deg)
            left = ArmState(left_vector, timestamp_ns, timestamp_ns)
            right = ArmState(right_vector, timestamp_ns, timestamp_ns)
            return BimanualSample(
                timestamp_ns,
                left,
                right,
                np.r_[left_vector, right_vector],
                image,
                image,
                image,
            )

        detector = InactivityAutoStop(stationary_sec=0.8, stationary_speed_deg_s=0.5)
        start = sample(0)
        idle = sample(500_000_000)
        moving = sample(600_000_000, 1.0)
        idle_again = sample(1_000_000_000, 1.0)
        settled = sample(1_800_000_000, 1.0)
        self.assertFalse(detector.update(start, idle)[0])
        self.assertFalse(detector.update(idle, moving)[0])
        self.assertFalse(detector.update(moving, idle_again)[0])
        self.assertTrue(detector.update(idle_again, settled)[0])

    def test_incomplete_dataset_cannot_resume(self):
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(RuntimeError, "incomplete"):
                validate_existing_dataset(Path(directory))


if __name__ == "__main__":
    unittest.main()
    controller_action,
