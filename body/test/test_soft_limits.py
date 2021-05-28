import unittest
import stretch_body.arm
import time

class TestArmSoftLimitHoming(unittest.TestCase):

    def test_home_arm_with_soft_limits(self):
        """
        Ensure that homing still works.
        Soft limits, when active, can prevent arm and lift from hitting hardstops.
        """
        a = stretch_body.arm.Arm()
        self.assertTrue(a.startup())
        a.home()
        a.pull_status()
        goal_pos=0.1
        self.assertAlmostEqual(a.status['pos'], goal_pos, places=1)
        a.stop()


    def test_move_arm_with_soft_limits(self):
        """
        Ensure that soft limits actual clip range of motion
        """
        a = stretch_body.arm.Arm()
        a.motor.disable_sync_mode()
        self.assertTrue(a.startup())
        a.pull_status()
        self.assertTrue(a.motor.status['pos_calibrated'])
        limit_pos = 0.2
        a.set_soft_motion_limits(0,limit_pos)
        a.push_command()
        a.move_to(x_m=0.3)
        a.push_command()
        time.sleep(3.0)
        a.pull_status()
        self.assertAlmostEqual(a.status['pos'], limit_pos, places=1)
        a.stop()