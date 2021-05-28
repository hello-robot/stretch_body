import unittest
import stretch_body.robot_params


class TestRobotParams(unittest.TestCase):

    def test_getting_params(self):
        """Test class method robot_params.get_params()

        Note: this test will fail if stretch_re1_user_params.yaml doesn't exist.
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
