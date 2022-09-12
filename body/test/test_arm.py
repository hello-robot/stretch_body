# Logging level must be set before importing any stretch_body class
import stretch_body.robot_params
stretch_body.robot_params.RobotParams.set_logging_level("DEBUG")

import unittest
import stretch_body.arm

import time


class TestArm(unittest.TestCase):

    def test_vel_guarded_contact(self):
        print('\nUnittest: test_vel_guarded_contact')
        """Test `set_velocity` API and check that guarded contact
        events are triggered at arm's end of range.
        """
        a = stretch_body.arm.Arm()
        a.motor.disable_sync_mode()
        self.assertTrue(a.startup(threaded=False))
        a.pull_status()
        if not a.motor.status['pos_calibrated']:
            self.fail('test requires arm to be homed')

        a.move_to(0.0)
        a.push_command()
        a.motor.wait_until_at_setpoint(timeout=5.0)
        a.pull_status()
        self.assertAlmostEqual(a.status['pos'], 0.0, places=1)
        g1=a.motor.status['guarded_event']
        vel_des=.15 #m/s

        # Move to extend hardstop
        a.set_velocity(v_m=vel_des)
        a.push_command()
        time.sleep(6.0)
        a.pull_status()
        g2=a.motor.status['guarded_event']
        self.assertNotEqual(g1,g2)
        self.assertNotAlmostEqual(a.status['pos'], 0.0, places=1)

        # Move to retract hardstop
        a.set_velocity(v_m=-1*vel_des)
        a.push_command()
        time.sleep(6.0)
        a.pull_status()
        g3 = a.motor.status['guarded_event']
        self.assertNotEqual(g2, g3)

        a.motor.enable_safety()
        a.push_command()
        a.pull_status()
        self.assertAlmostEqual(a.status['pos'], 0.0, places=1)
        a.stop()

    def test_valid_startup_status(self):
        print('\nUnittest: test_valid_startup_status')
        a = stretch_body.arm.Arm()
        self.assertTrue(a.startup(threaded=False))
        self.assertNotEqual(a.status['pos'],0)

    def test_homing(self):
        print('\nUnittest: test_homing')
        """Test arm homes correctly.
        """
        a = stretch_body.arm.Arm()
        self.assertTrue(a.startup(threaded=False))

        a.home()
        time.sleep(2)
        a.pull_status()
        self.assertAlmostEqual(a.status['pos'], 0.1, places=3)

        a.stop()

    def test_move_arm_with_soft_limits(self):
        """Ensure that soft limits actual clip range of motion.
        """
        print('\nUnittest: test_move_arm_with_soft_limits')
        a = stretch_body.arm.Arm()
        a.motor.disable_sync_mode()
        self.assertTrue(a.startup(threaded=False))
        a.pull_status()
        if not a.motor.status['pos_calibrated']:
            self.fail('test requires arm to be homed')

        # First a simple user limit test
        limit_pos = 0.3
        a.set_soft_motion_limit_min(0, limit_type='user')
        a.set_soft_motion_limit_max(limit_pos, limit_type='user')
        a.push_command()
        a.move_to(x_m=0.4)
        a.push_command()
        time.sleep(3.0)
        a.pull_status()
        self.assertAlmostEqual(a.status['pos'], limit_pos, places=1)

        # Now set a collision limit
        limit_pos = 0.2
        a.set_soft_motion_limit_min(0, limit_type='collision')
        a.set_soft_motion_limit_max(limit_pos, limit_type='collision')
        a.push_command()  # This should move to the new more restricted limt
        time.sleep(2.0)
        a.pull_status()
        self.assertAlmostEqual(a.status['pos'], limit_pos, places=1)

        # Now remove the collision limit
        limit_pos = 0.3
        a.set_soft_motion_limit_min(None, limit_type='collision')
        a.set_soft_motion_limit_max(None, limit_type='collision')
        a.move_to(x_m=0.4)
        a.push_command()  # This should move the old user limit
        time.sleep(2.0)
        a.pull_status()
        self.assertAlmostEqual(a.status['pos'], limit_pos, places=1)

        a.stop()

    def test_waypoint_trajectory(self):
        print('\nUnittest: test_waypoint_trajectory')
        a = stretch_body.arm.Arm()
        a.motor.disable_sync_mode()
        self.assertTrue(a.startup(threaded=True))
        if not a.motor.status['pos_calibrated']:
            self.fail('test requires arm to be homed')

        a.trajectory.add(0, 0.1)
        a.trajectory.add(3, 0.2)
        a.trajectory.add(6, 0.15)
        a.logger.debug('Executing {0}'.format(a.trajectory.__repr_segments__()))
        self.assertTrue(a.trajectory.is_valid(0.1, 0.15))
        a.follow_trajectory()
        time.sleep(6)
        self.assertAlmostEqual(a.status['pos'], 0.15, places=2)

        a.trajectory.clear()
        a.trajectory.add(0, 0.1, 0.0)
        a.trajectory.add(3, 0.2, 0.0)
        a.trajectory.add(6, 0.15, 0.0)
        a.logger.debug('Executing {0}'.format(a.trajectory.__repr_segments__()))
        self.assertTrue(a.trajectory.is_valid(0.1, 0.15))
        a.follow_trajectory()
        time.sleep(6)
        self.assertAlmostEqual(a.status['pos'], 0.15, places=2)

        a.trajectory.clear()
        a.trajectory.add(0, 0.1, 0.0, 0.0)
        a.trajectory.add(3, 0.2, 0.0, 0.0)
        a.trajectory.add(6, 0.15, 0.0, 0.0)
        a.logger.debug('Executing {0}'.format(a.trajectory.__repr_segments__()))
        self.assertTrue(a.trajectory.is_valid(0.1, 0.15))
        a.follow_trajectory()
        time.sleep(6)
        self.assertAlmostEqual(a.status['pos'], 0.15, places=2)

        a.stop()
