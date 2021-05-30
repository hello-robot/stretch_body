# Logging level must be set before importing any stretch_body class
import stretch_body.robot_params
stretch_body.robot_params.RobotParams.set_logging_level("DEBUG")

import unittest
import stretch_body.end_of_arm


class TestEndOfArm(unittest.TestCase):

    def test_homing(self):
        """Test end_of_arm homes correctly.
        """
        e = stretch_body.end_of_arm.EndOfArm()
        self.assertTrue(e.startup())

        e.home()
        e.pull_status()
        self.assertAlmostEqual(e.status['wrist_yaw']['pos'], 0.0, places=1)
        self.assertAlmostEqual(e.status['stretch_gripper']['pos'], 0.0, places=1)

        e.stop()

    def test_joints(self):
        """Verify end_of_arm always has atleast one joint: wrist_yaw.
        """
        e = stretch_body.end_of_arm.EndOfArm()
        self.assertTrue(len(e.joints) > 0)
        self.assertTrue('wrist_yaw' in e.joints)
