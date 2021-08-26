# Logging level must be set before importing any stretch_body class
import stretch_body.robot_params
stretch_body.robot_params.RobotParams.set_logging_level("DEBUG")

import unittest
import stretch_body.head

class TestHead(unittest.TestCase):

    def test_homing(self):
        """Test head doesn't home or complaine
        """
        h = stretch_body.head.Head()
        self.assertTrue(h.startup())
        h.home()
        h.stop()

    def test_joints(self):
        """Verify end_of_arm always has atleast one joint: wrist_yaw.
        """
        h = stretch_body.head.Head()
        self.assertTrue(len(h.joints) ==2)
        self.assertTrue('head_pan' in h.joints)

    def test_get_joints(self):
        h = stretch_body.head.Head()
        m=h.get_joint('head_pan')
        self.assertEqual('head_pan',m.name)
        m = h.get_joint('foo')
        self.assertEqual(m,None)