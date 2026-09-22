import unittest

from nero_neo_teleop.control.gripper import (
    GripperAnalogController,
    GripperToggleController,
)


class GripperControllerTest(unittest.TestCase):
    def test_toggle_uses_one_command_per_rising_edge(self):
        controller = GripperToggleController(
            open_m=0.09,
            closed_m=0.0,
            initial_feedback_m=0.09,
        )

        controller.update(clicked=False)
        closed = controller.update(clicked=True)
        held = controller.update(clicked=True)

        self.assertTrue(closed.toggled)
        self.assertFalse(closed.is_open)
        self.assertEqual(closed.target_m, 0.0)
        self.assertFalse(held.toggled)

    def test_analog_mapping_clamps_and_preserves_hold_on_invalid_input(self):
        controller = GripperAnalogController(
            open_m=0.09,
            closed_m=0.0,
            initial_feedback_m=0.09,
        )

        closed = controller.update(trigger=2.0)
        invalid = controller.update(trigger=0.0, input_valid=False)

        self.assertTrue(closed.command)
        self.assertEqual(closed.target_m, 0.0)
        self.assertFalse(invalid.command)
        self.assertEqual(invalid.target_m, 0.0)


if __name__ == "__main__":
    unittest.main()
