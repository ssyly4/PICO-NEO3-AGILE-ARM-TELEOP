import tempfile
import time
import unittest
from pathlib import Path

import numpy as np

from nero_neo_teleop.recording.action_command_stream import (
    ArmCommandPublisher,
    ArmCommandReceiver,
)


class ActionCommandStreamTest(unittest.TestCase):
    def test_round_trip_preserves_command_and_timestamp(self):
        with tempfile.TemporaryDirectory() as directory:
            socket_path = Path(directory) / "right.sock"
            receiver = ArmCommandReceiver(socket_path, "right")
            publisher = ArmCommandPublisher(str(socket_path), "right")
            receiver.start()
            try:
                monotonic_ns = time.monotonic_ns()
                joints = np.arange(7, dtype=np.float64) * 0.01
                publisher.publish(
                    joint_target_rad=joints,
                    gripper_target_normalized=0.25,
                    monotonic_ns=monotonic_ns,
                    sequence=7,
                    control_state="velocity_tracking",
                )

                command = receiver.wait_ready(after_ns=monotonic_ns, timeout_sec=1.0)

                np.testing.assert_allclose(command.vector[:7], joints)
                self.assertAlmostEqual(float(command.vector[7]), 0.25)
                self.assertEqual(command.monotonic_ns, monotonic_ns)
                self.assertEqual(command.sequence, 7)
                self.assertEqual(command.control_state, "velocity_tracking")
            finally:
                publisher.close()
                receiver.stop()

    def test_invalid_command_is_rejected_before_transport(self):
        with tempfile.TemporaryDirectory() as directory:
            publisher = ArmCommandPublisher(str(Path(directory) / "missing.sock"), "left")
            try:
                with self.assertRaisesRegex(ValueError, "seven-vector"):
                    publisher.publish(
                        joint_target_rad=np.zeros(6),
                        gripper_target_normalized=0.5,
                        monotonic_ns=time.monotonic_ns(),
                        sequence=1,
                        control_state="test",
                    )
            finally:
                publisher.close()


if __name__ == "__main__":
    unittest.main()
