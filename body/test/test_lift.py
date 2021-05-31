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
        self.assertTrue(l.startup())
        l.home()
        l.push_command()
        time.sleep(1)
        l.pull_status()
        self.assertAlmostEqual(l.status['pos'], 0.6, places=1)
        l.stop()

    def test_move_lift_with_soft_limits(self):
        """Ensure that soft limits actual clip range of motion.
        """
        l = stretch_body.lift.Lift()
        l.motor.disable_sync_mode()
        self.assertTrue(l.startup())
        l.pull_status()
        if not l.motor.status['pos_calibrated']:
            self.fail('test requires lift to be homed')

        limit_pos = 0.8
        l.set_soft_motion_limits(0,limit_pos)
        l.push_command()
        l.move_to(x_m=0.9)
        l.push_command()
        time.sleep(3.0)
        l.pull_status()
        self.assertAlmostEqual(l.status['pos'], limit_pos, places=1)

        l.stop()
