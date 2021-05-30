# Logging level must be set before importing any stretch_body class
import stretch_body.robot_params
stretch_body.robot_params.RobotParams.set_logging_level("DEBUG")

import unittest
import stretch_body.arm

import time


class TestArm(unittest.TestCase):

    def test_homing(self):
        """Test arm homes correctly.
        """
        a = stretch_body.arm.Arm()
        a.startup()

        a.home(single_stop=False)
        time.sleep(2)
        a.pull_status()
        self.assertAlmostEqual(a.status['pos'], 0.1, places=3)

        a.stop()
