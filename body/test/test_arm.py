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
        self.assertTrue(a.startup())

        a.home(single_stop=False)
        time.sleep(2)
        a.pull_status()
        self.assertAlmostEqual(a.status['pos'], 0.1, places=3)

        a.stop()

    def test_move_arm_with_soft_limits(self):
        """Ensure that soft limits actual clip range of motion.
        """
        a = stretch_body.arm.Arm()
        a.motor.disable_sync_mode()
        self.assertTrue(a.startup())
        a.pull_status()
        if not a.motor.status['pos_calibrated']:
            self.fail('test requires arm to be homed')

        limit_pos = 0.2
        a.set_soft_motion_limits(0,limit_pos)
        a.push_command()
        a.move_to(x_m=0.3)
        a.push_command()
        time.sleep(3.0)
        a.pull_status()
        self.assertAlmostEqual(a.status['pos'], limit_pos, places=1)

        a.stop()
