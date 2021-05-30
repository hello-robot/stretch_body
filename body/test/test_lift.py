# Logging level must be set before importing any stretch_body class
import stretch_body.robot_params
stretch_body.robot_params.RobotParams.set_logging_level("DEBUG")

import unittest
import stretch_body.lift

import time


class TestLift(unittest.TestCase):

    def test_homing(self):
        """Test lift homes correctly.
        """
        l = stretch_body.lift.Lift()
        l.startup()

        l.home()
        time.sleep(2)
        l.pull_status()
        self.assertAlmostEqual(l.status['pos'], 0.58, places=1)

        l.stop()
