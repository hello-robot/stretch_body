# Logging level must be set before importing any stretch_body class
import stretch_body.robot_params
stretch_body.robot_params.RobotParams.set_logging_level("DEBUG")

import unittest
import stretch_body.base

import time


class TestBase(unittest.TestCase):

    def test_fast_base_motion_allowed(self):
        """Verifies fast base motion is allowed at the correct time.
        """
        import stretch_body.robot
        r = stretch_body.robot.Robot()
        r.robot_params['robot_sentry']['base_max_velocity'] = 1 # Enable fast base motion
        r.startup()
        if not r.is_calibrated():
            self.fail("test requires robot to be homed")

        r.stow()
        r.pull_status()
        self.assertTrue(r.base.fast_motion_allowed)

        # check lift
        r.lift.move_to(r.base.params['sentry_max_velocity']['max_lift_height_m'] + 0.05)
        r.push_command()
        time.sleep(3)
        self.assertFalse(r.base.fast_motion_allowed)
        r.lift.move_to(r.base.params['sentry_max_velocity']['max_lift_height_m'] - 0.05)
        r.push_command()
        time.sleep(3)
        self.assertTrue(r.base.fast_motion_allowed)

        # check arm
        r.arm.move_to(r.base.params['sentry_max_velocity']['max_arm_extension_m'] + 0.05)
        r.push_command()
        time.sleep(3)
        self.assertFalse(r.base.fast_motion_allowed)
        r.arm.move_to(r.base.params['sentry_max_velocity']['max_arm_extension_m'] - 0.05)
        r.push_command()
        time.sleep(3)
        self.assertTrue(r.base.fast_motion_allowed)

        # check wrist_yaw
        r.end_of_arm.move_to("wrist_yaw", r.base.params['sentry_max_velocity']['min_wrist_yaw_rad'] - 0.1)
        time.sleep(3)
        self.assertFalse(r.base.fast_motion_allowed)
        r.end_of_arm.move_to("wrist_yaw", r.base.params['sentry_max_velocity']['min_wrist_yaw_rad'] + 0.1)
        time.sleep(3)
        self.assertTrue(r.base.fast_motion_allowed)

        r.stop()
