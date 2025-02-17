import unittest
import stretch_body.robot_params


class TestRobotParams(unittest.TestCase):

    def test_getting_params(self):
        """Test class method robot_params.get_params()

        Note: this test will fail if stretch_user_params.yaml doesn't exist.
        """
        # TODO: skip if user yaml doesn't exist
        # if not user_yaml:
        #     raise unittest.case.SkipTest("user yaml file doesn't exist in fleet dir")

        r1 = stretch_body.robot_params.RobotParams()
        r1.get_params()
        r1_motion_params = r1._robot_params['wrist_yaw']['motion']
        self.assertNotEqual(r1_motion_params, {})

        r2 = stretch_body.robot_params.RobotParams()
        _, r2_robot_params = r2.get_params()
        r2_motion_params = r2_robot_params['wrist_yaw']['motion']
        self.assertNotEqual(r2_motion_params, {})
        self.assertDictEqual(r1_motion_params, r2_motion_params)

        _, r3_robot_params = stretch_body.robot_params.RobotParams.get_params()
        r3_motion_params = r3_robot_params['wrist_yaw']['motion']
        self.assertNotEqual(r3_motion_params, {})
        self.assertDictEqual(r1_motion_params, r3_motion_params)
        # stretch_body.robot_params.RobotParams.add_params({'wrist_yaw': {'motion': {'nothing': 123}}})

    def test_setting_log_level(self):
        """Test RobotParams.set_logging_level()
        """
        up, rp = stretch_body.robot_params.RobotParams.get_params()
        self.assertEqual(rp['logging']['handlers']['console_handler']['level'], "INFO")

        stretch_body.robot_params.RobotParams.set_logging_level("NONEXISTENT")
        self.assertEqual(rp['logging']['handlers']['console_handler']['level'], "INFO")

        stretch_body.robot_params.RobotParams.set_logging_level(-1000)
        self.assertEqual(rp['logging']['handlers']['console_handler']['level'], "INFO")

        stretch_body.robot_params.RobotParams.set_logging_level("CRITICAL")
        self.assertEqual(rp['logging']['handlers']['console_handler']['level'], "CRITICAL")

        stretch_body.robot_params.RobotParams.set_logging_level(10)
        self.assertEqual(rp['logging']['handlers']['console_handler']['level'], 10)

    def test_logging_filename_param(self):
        _, rp = stretch_body.robot_params.RobotParams.get_params()
        self.assertTrue(rp['logging']['handlers']['file_handler']['filename'].endswith('.log'))
