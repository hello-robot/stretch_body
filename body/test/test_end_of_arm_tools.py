# Logging level must be set before importing any stretch_body class
import stretch_body.robot_params
stretch_body.robot_params.RobotParams.set_logging_level("DEBUG")

import unittest
import stretch_body.end_of_arm_tools
import stretch_body.dynamixel_XL430

class TestEndOfArmTools(unittest.TestCase):

    def test_toolnone_joints(self):
        """Verify ToolNone always has atleast one joint: wrist_yaw.
        """
        e = stretch_body.end_of_arm_tools.ToolNone()
        self.assertTrue(len(e.joints) > 0)
        self.assertTrue('wrist_yaw' in e.joints)

    def test_toolstretchgripper_joints(self):
        """Verify ToolStretchGripper always has atleast one joint: wrist_yaw.
        """
        e = stretch_body.end_of_arm_tools.ToolStretchGripper()
        self.assertTrue(len(e.joints) > 0)
        self.assertTrue('wrist_yaw' in e.joints)

    @unittest.skip(reason='TODO: fails currently')
    def test_catch_startup_exception(self):
        """Verify that cleanly handle wrong baudrate exceptions
        """
        b=stretch_body.dynamixel_XL430.DynamixelXL430.identify_baud_rate(13,'/dev/hello-dynamixel-wrist')
        if b==115200:
            e = stretch_body.end_of_arm_tools.ToolNone()
            e.params['baud']=57600
            self.assertFalse(e.startup())
        else:
            e = stretch_body.end_of_arm_tools.ToolNone()
            e.params['baud'] = 115200
            self.assertFalse(e.startup())
