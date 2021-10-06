# Logging level must be set before importing any stretch_body class
import stretch_body.robot_params
# stretch_body.robot_params.RobotParams.set_logging_level("DEBUG")

import unittest
import stretch_body.base

import time
import numpy as np


class TestBase(unittest.TestCase):

    def test_valid_startup_status(self):
        b = stretch_body.base.Base()
        self.assertTrue(b.startup(threaded=False))
        self.assertNotEqual(b.status['timestamp_pc'],0)

    def test_fast_base_motion_allowed(self):
        """Verifies fast base motion is allowed at the correct time.
        """
        import stretch_body.robot
        r = stretch_body.robot.Robot()
        r.robot_params['robot_sentry']['base_max_velocity'] = 1 # Enable fast base motion
        self.assertTrue(r.startup())
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

    def test_waypoint_trajectory(self):
        """Test a basic waypoint trajectory to verify it works.
        """
        b = stretch_body.base.Base()
        b.left_wheel.disable_sync_mode()
        b.right_wheel.disable_sync_mode()
        self.assertTrue(b.startup(threaded=True))
        b.first_step = True
        b.pull_status()

        b.trajectory.add(0, 0, 0, 0)
        b.trajectory.add(3, 0.1, 0, 0)
        b.trajectory.add(6, 0, 0, 0)
        b.logger.info('Executing {0}'.format(b.trajectory))
        # b.logger.info('Segments:\n{0}\n{1}'.format(
        #     eval(b.trajectory.__repr_segments__(b.translate_to_motor_rad, b.rotate_to_motor_rad))[0],
        #     eval(b.trajectory.__repr_segments__(b.translate_to_motor_rad, b.rotate_to_motor_rad))[1]
        # ))
        self.assertTrue(b.trajectory.is_valid(25, 10, b.translate_to_motor_rad, b.rotate_to_motor_rad))
        b.follow_trajectory()
        time.sleep(7)
        self.assertAlmostEqual(b.status['x'], 0.0, places=1)
        self.assertAlmostEqual(b.status['y'], 0.0, places=1)
        constrained_theta = np.arctan2(np.sin(b.status['theta']), np.cos(b.status['theta'])) # constrains to [-pi, pi]
        self.assertAlmostEqual(constrained_theta, 0.0, places=1)

        b.trajectory.clear()
        b.trajectory.add(0, 0, 0, 0, 0, 0)
        b.trajectory.add(3, 0.1, 0, 0, 0, 0)
        b.trajectory.add(6, 0, 0, 0, 0, 0)
        b.logger.info('Executing {0}'.format(b.trajectory))
        self.assertTrue(b.trajectory.is_valid(25, 10, b.translate_to_motor_rad, b.rotate_to_motor_rad))
        b.follow_trajectory()
        time.sleep(7)
        self.assertAlmostEqual(b.status['x'], 0.0, places=1)
        self.assertAlmostEqual(b.status['y'], 0.0, places=1)
        constrained_theta = np.arctan2(np.sin(b.status['theta']), np.cos(b.status['theta'])) # constrains to [-pi, pi]
        self.assertAlmostEqual(constrained_theta, 0.0, places=1)

        b.trajectory.clear()
        b.trajectory.add(0, 0, 0, 0, 0, 0, 0, 0)
        b.trajectory.add(3, 0.1, 0, 0, 0, 0, 0, 0)
        b.trajectory.add(6, 0, 0, 0, 0, 0, 0, 0)
        b.logger.info('Executing {0}'.format(b.trajectory))
        self.assertTrue(b.trajectory.is_valid(25, 10, b.translate_to_motor_rad, b.rotate_to_motor_rad))
        b.follow_trajectory()
        time.sleep(7)
        self.assertAlmostEqual(b.status['x'], 0.0, places=1)
        self.assertAlmostEqual(b.status['y'], 0.0, places=1)
        constrained_theta = np.arctan2(np.sin(b.status['theta']), np.cos(b.status['theta'])) # constrains to [-pi, pi]
        self.assertAlmostEqual(constrained_theta, 0.0, places=1)

        b.trajectory.clear()
        b.trajectory.add(0, 0, 0, 0, 0, 0)
        b.trajectory.add(6, 0, 0, np.pi / 2.0, 0, 0)
        b.trajectory.add(12, 0, 0, 0, 0, 0)
        b.logger.info('Executing {0}'.format(b.trajectory))
        self.assertTrue(b.trajectory.is_valid(25, 10, b.translate_to_motor_rad, b.rotate_to_motor_rad))
        b.follow_trajectory()
        time.sleep(13)
        self.assertAlmostEqual(b.status['x'], 0.0, places=1)
        self.assertAlmostEqual(b.status['y'], 0.0, places=1)
        constrained_theta = np.arctan2(np.sin(b.status['theta']), np.cos(b.status['theta'])) # constrains to [-pi, pi]
        self.assertAlmostEqual(constrained_theta, 0.0, places=1)

        b.stop()

    def test_multidof_waypoint_trajectory(self):
        b = stretch_body.base.Base()
        b.left_wheel.disable_sync_mode()
        b.right_wheel.disable_sync_mode()
        self.assertTrue(b.startup(threaded=True))
        b.first_step = True
        b.pull_status()

        b.trajectory.clear()
        b.trajectory.add(0, 0, 0, 0, 0, 0, 0, 0)
        b.trajectory.add(6, 0.4, 0, 0, 0, 0, 0, 0)
        b.trajectory.add(12, 0.4, 0, np.pi, 0, 0, 0, 0)
        b.trajectory.add(18, 0.0, 0, np.pi, 0, 0, 0, 0)
        b.trajectory.add(24, 0.0, 0, 0, 0, 0, 0, 0)
        b.logger.info('Executing {0}'.format(b.trajectory))
        self.assertTrue(b.trajectory.is_valid(25, 10, b.translate_to_motor_rad, b.rotate_to_motor_rad))
        b.follow_trajectory()
        time.sleep(25)
        self.assertAlmostEqual(b.status['x'], 0.0, places=1)
        self.assertAlmostEqual(b.status['y'], 0.0, places=1)
        constrained_theta = np.arctan2(np.sin(b.status['theta']), np.cos(b.status['theta'])) # constrains to [-pi, pi]
        self.assertAlmostEqual(constrained_theta, 0.0, places=1)

        b.stop()
