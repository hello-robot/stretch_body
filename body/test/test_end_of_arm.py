# Logging level must be set before importing any stretch_body class
import stretch_body.robot_params
stretch_body.robot_params.RobotParams.set_logging_level("DEBUG")

import unittest
import stretch_body.end_of_arm
import importlib

class TestEndOfArm(unittest.TestCase):

    def test_homing(self):
        """Test end_of_arm homes correctly.

        """
        e = stretch_body.end_of_arm.EndOfArm()
        tool_name = e.robot_params['robot']['tool']
        module_name = e.robot_params[tool_name]['py_module_name']
        class_name = e.robot_params[tool_name]['py_class_name']
        e = getattr(importlib.import_module(module_name), class_name)()
        self.assertTrue('stretch_gripper' in e.joints)
        self.assertTrue(e.startup(threaded=False))

        e.home()
        e.pull_status()
        self.assertAlmostEqual(e.status['wrist_yaw']['pos'], 0.0, places=1)
        self.assertAlmostEqual(e.status['stretch_gripper']['pos'], 0.0, places=1)

    def test_joints(self):
        """Verify end_of_arm always has at least one joint: wrist_yaw.
        """
        e = stretch_body.end_of_arm.EndOfArm()
        self.assertTrue(len(e.joints) > 0)
        self.assertTrue('wrist_yaw' in e.joints)

    def test_get_joints(self):
        e = stretch_body.end_of_arm.EndOfArm()
        m=e.get_joint('wrist_yaw')
        self.assertEqual('wrist_yaw',m.name)
        m = e.get_joint('foo')
        self.assertEqual(m,None)