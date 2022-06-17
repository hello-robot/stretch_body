# Logging level must be set before importing any stretch_body class
#stretch_body.robot_params.RobotParams.set_logging_level("DEBUG")

import unittest
import copy

"""
Note: In order for aaa_test_catch_startup_exception to work we need to temporarily write over the user params prior to
the import of stretch_body.robot_params. As such we prepend aaa so that this test runs first. We also
import locally within the test  and dont configure the logger per usual
"""

class TestEndOfArmTools(unittest.TestCase):
    def test_toolnone_joints(self):
        """Verify ToolNone always has at least one joint: wrist_yaw.
        """
        import stretch_body.end_of_arm_tools
        e = stretch_body.end_of_arm_tools.ToolNone()
        self.assertTrue(len(e.joints) > 0)
        self.assertTrue('wrist_yaw' in e.joints)

    def test_toolstretchgripper_joints(self):
        """Verify ToolStretchGripper always has at least one joint: wrist_yaw.
        """
        import stretch_body.end_of_arm_tools
        e = stretch_body.end_of_arm_tools.ToolStretchGripper()
        self.assertTrue(len(e.joints) > 0)
        self.assertTrue('wrist_yaw' in e.joints)

    def test_aaa_catch_startup_exception(self):
        """Verify that cleanly handle wrong baudrate exceptions
        """
        import stretch_body.hello_utils
        up = stretch_body.hello_utils.read_fleet_yaml('stretch_user_params.yaml')
        up_stash=copy.deepcopy(up)
        if 'tool_none' not in up:
            up['tool_none']={}
        up['tool_none']['baud'] = 1000000 #Set baud to something not supported
        stretch_body.hello_utils.write_fleet_yaml('stretch_user_params.yaml', up)
        import stretch_body.end_of_arm_tools
        e = stretch_body.end_of_arm_tools.ToolNone()
        success = e.startup()
        stretch_body.hello_utils.write_fleet_yaml('stretch_user_params.yaml', up_stash)
        self.assertFalse(success)

