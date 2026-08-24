import unittest
from pathlib import Path

import numpy as np
from trac_ik import TracIK

from nero_neo_teleop.robot.home_config import LEFT_HOME_RAD, MAX_TCP_HEIGHT_MM, MIN_TCP_HEIGHT_MM, RIGHT_HOME_RAD
from nero_neo_teleop.runtime import NERO_URDF


URDF = NERO_URDF


class DualHomeConfigTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.solver = TracIK("base_link", "link7", str(URDF), timeout=0.02)

    def test_home_tcp_heights_match_and_stay_inside_vertical_guards(self):
        left_position, _ = self.solver.fk(LEFT_HOME_RAD)
        right_position, _ = self.solver.fk(RIGHT_HOME_RAD)

        self.assertAlmostEqual(left_position[2], right_position[2], places=7)
        self.assertGreater(1000.0 * left_position[2], MIN_TCP_HEIGHT_MM)
        self.assertLess(1000.0 * left_position[2], MAX_TCP_HEIGHT_MM)

    def test_home_tool_forward_axes_point_down(self):
        expected = np.asarray([0.0, 0.0, -1.0])
        for joints in (LEFT_HOME_RAD, RIGHT_HOME_RAD):
            _, rotation = self.solver.fk(joints)
            np.testing.assert_allclose(rotation[:, 0], expected, atol=2e-7)


if __name__ == "__main__":
    unittest.main()
