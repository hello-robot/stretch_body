import unittest
import stretch_body.arm


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


