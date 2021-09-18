# Logging level must be set before importing any stretch_body class
import stretch_body.robot_params
stretch_body.robot_params.RobotParams.set_logging_level("DEBUG")

import unittest
import stretch_body.lift

import time


class TestLift(unittest.TestCase):

    def test_vel_guarded_contact(self):
        l = stretch_body.lift.Lift()
        l.motor.disable_sync_mode()
        self.assertTrue(l.startup())
        l.pull_status()
        if not l.motor.status['pos_calibrated']:
            self.fail('test requires lift to be homed')

        l.move_to(0.5)
        l.push_command()
        l.motor.wait_until_at_setpoint(timeout=8.0)
        l.pull_status()
        self.assertAlmostEqual(l.status['pos'], 0.5, places=1)

        g1=l.motor.status['guarded_event']
        vel_des=.10 #m/s
        l.set_velocity(v_m=vel_des)
        l.push_command()
        time.sleep(10.0) #Move to top hardstop
        l.pull_status()
        g2=l.motor.status['guarded_event']
        self.assertNotEqual(g1,g2)

        l.move_to(0.5)
        l.push_command()
        l.motor.wait_until_at_setpoint(timeout=8.0)
        l.pull_status()
        self.assertAlmostEqual(l.status['pos'], 0.5, places=1)

        l.stop()

    def test_homing(self):
        """Test lift homes correctly.
        """
        l = stretch_body.lift.Lift()
        self.assertTrue(l.startup())
        l.home()
        l.push_command()
        time.sleep(1)
        l.pull_status()
        self.assertAlmostEqual(l.status['pos'], 0.6, places=1)
        l.stop()

    def test_move_lift_with_soft_limits(self):
        """Ensure that soft limits actual clip range of motion.
        """
        l = stretch_body.lift.Lift()
        l.motor.disable_sync_mode()
        self.assertTrue(l.startup())
        l.pull_status()
        if not l.motor.status['pos_calibrated']:
            self.fail('test requires lift to be homed')

        #First a simple user limit test
        limit_pos = 0.8
        l.set_soft_motion_limit_min(0,limit_type='user')
        l.set_soft_motion_limit_max(limit_pos,limit_type='user')
        l.push_command()
        l.move_to(x_m=0.9)
        l.push_command()
        time.sleep(4.0)
        l.pull_status()
        self.assertAlmostEqual(l.status['pos'], limit_pos, places=1)

        #Now set a collision limit
        limit_pos = 0.6
        l.set_soft_motion_limit_min(0, limit_type='collision')
        l.set_soft_motion_limit_max(limit_pos, limit_type='collision')
        l.push_command() #This should move to the new more restricted limt
        time.sleep(4.0)
        l.pull_status()
        self.assertAlmostEqual(l.status['pos'], limit_pos, places=1)

        # Now remove the collision limit
        limit_pos = 0.8
        l.set_soft_motion_limit_min(None, limit_type='collision')
        l.set_soft_motion_limit_max(None, limit_type='collision')
        l.move_to(x_m=0.9)
        l.push_command()  # This should move the old user limit
        time.sleep(4.0)
        l.pull_status()
        self.assertAlmostEqual(l.status['pos'], limit_pos, places=1)

        l.stop()

