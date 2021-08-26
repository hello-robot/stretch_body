# Logging level must be set before importing any stretch_body class
import stretch_body.robot_params
stretch_body.robot_params.RobotParams.set_logging_level("DEBUG")

import unittest
import stretch_body.dynamixel_hello_XL430
import stretch_body.hello_utils as hu
import math
import time
from concurrent.futures import ThreadPoolExecutor


class TestDynamixelHelloXL430(unittest.TestCase):

    def test_soft_motion_limits(self):
        print('test_soft_motion_limits')
        servo = stretch_body.dynamixel_hello_XL430.DynamixelHelloXL430(name="head_pan", chain=None)
        self.assertTrue(servo.startup())
        servo.enable_pos()

        # Test user limits in both directions
        limit_pos = hu.deg_to_rad(60.0)
        servo.set_soft_motion_limit_max(limit_pos, limit_type='user')
        servo.set_soft_motion_limit_min(-1*limit_pos, limit_type='user')
        servo.move_to(x_des=hu.deg_to_rad(90.0))
        time.sleep(2.0)
        servo.pull_status()
        self.assertAlmostEqual(servo.status['pos'], limit_pos, places=1)
        servo.move_to(x_des=hu.deg_to_rad(-90.0))
        time.sleep(2.0)
        servo.pull_status()
        self.assertAlmostEqual(servo.status['pos'], -1*limit_pos, places=1)



        # Now set collision limits in both directions
        limit_pos = hu.deg_to_rad(40.0)
        servo.set_soft_motion_limit_min(-1*limit_pos, limit_type='collision')
        servo.set_soft_motion_limit_max(limit_pos, limit_type='collision')
        servo.move_to(x_des=hu.deg_to_rad(90.0))
        time.sleep(2.0)
        servo.pull_status()
        self.assertAlmostEqual(servo.status['pos'], limit_pos, places=1)
        servo.move_to(x_des=hu.deg_to_rad(-90.0))
        time.sleep(2.0)
        servo.pull_status()
        self.assertAlmostEqual(servo.status['pos'], -1*limit_pos, places=1)

        # # Now remove the collision limits and check user limits still work
        limit_pos = hu.deg_to_rad(60.0)
        servo.set_soft_motion_limit_max(None, limit_type='collision')
        servo.set_soft_motion_limit_min(None, limit_type='collision')
        servo.move_to(x_des=hu.deg_to_rad(90.0))
        time.sleep(2.0)
        servo.pull_status()
        self.assertAlmostEqual(servo.status['pos'], limit_pos, places=1)
        servo.move_to(x_des=hu.deg_to_rad(-90.0))
        time.sleep(2.0)
        servo.pull_status()
        self.assertAlmostEqual(servo.status['pos'], -1 * limit_pos, places=1)

        servo.stop()

    def test_non_multiturn_move_after_enable_pos(self):
        """Verify non-multiturn servo responds to move_to commands after enable_pos.
        """
        print('test_non_multiturn_move_after_enable_pos')
        servo = stretch_body.dynamixel_hello_XL430.DynamixelHelloXL430(name="head_tilt", chain=None)
        servo.params['use_multiturn'] = False
        self.assertTrue(servo.startup())
        servo.enable_pos()

        goal_rad = -math.pi / 2
        servo.move_to(goal_rad)
        time.sleep(1)
        servo.pull_status()
        self.assertAlmostEqual(servo.status['pos'], goal_rad, places=1)

        goal_rad = 0.0
        servo.move_to(goal_rad)
        time.sleep(1)
        servo.pull_status()
        self.assertAlmostEqual(servo.status['pos'], goal_rad, places=1)

        servo.stop()

    def test_non_multiturn_move_after_home(self):
        """Verify non-multiturn servo responds to move_to commands after homing.
        """
        print('test_non_multiturn_move_after_home')
        servo = stretch_body.dynamixel_hello_XL430.DynamixelHelloXL430(name="head_tilt", chain=None)
        servo.params['use_multiturn'] = False
        servo.params['req_calibration']=True
        servo.params['pwm_homing']=[-200,200]
        self.assertTrue(servo.startup())

        servo.home(single_stop=True) # calls servo.enable_pos() internally
        servo.pull_status()
        self.assertAlmostEqual(servo.status['pos'], 0.0, places=1)

        servo.stop()

    def test_two_hardstop_homing(self):
        """Verify servo hits two hardstops during homing when single_stop=False.
        """
        print('test_two_hardstop_homing')
        servo = stretch_body.dynamixel_hello_XL430.DynamixelHelloXL430(name="wrist_yaw", chain=None)
        self.assertTrue(servo.startup())

        servo.home(single_stop=False, move_to_zero=True)
        servo.pull_status()
        self.assertAlmostEqual(servo.status['pos'], 0.0, places=1)

        servo.stop()

    def test_runstop(self):
        print('test_runstop')
        class R():
            class P():
                status = {'runstop_event': False}

            pimu = P()

        r = R()
        servo = stretch_body.dynamixel_hello_XL430.DynamixelHelloXL430(name="head_pan", chain=None)
        self.assertTrue(servo.startup())
        servo.robot_params['robot_sentry']['dynamixel_stop_on_runstop'] = True
        servo.params['enable_runstop'] = True
        servo.move_to(0.0)
        time.sleep(2)

        servo.move_to(servo.get_soft_motion_limits()[0])
        time.sleep(1.0)
        r.pimu.status['runstop_event'] = True
        servo.step_sentry(robot=r)
        time.sleep(2.0)
        servo.pull_status()
        self.assertNotAlmostEqual(servo.status['pos'], servo.get_soft_motion_limits()[0], places=1)

    @unittest.skip(reason='TODO: Not working yet.')
    def test_runstop_multithread(self):
        """Verify dynamixel_hello respect runstop via step_sentry
        """
        print('test_runstop_multithread')

        servo = stretch_body.dynamixel_hello_XL430.DynamixelHelloXL430(name="head_pan", chain=None)
        self.assertTrue(servo.startup())
        servo.robot_params['robot_sentry']['dynamixel_stop_on_runstop'] = True
        servo.params['enable_runstop'] = True
        servo.move_to(0.0)
        time.sleep(2)
        to_save = {'do_interrupt': None, 'pos1': None, 'pos2': None, 'pos3': None, 'pos4': None, 'vel1': None, 'vel2': None, 'interrupts': 0}

        def swivel(to_save):
            print('interrupted swivel 1')
            to_save['do_interrupt'] = True
            servo.move_to(servo.get_soft_motion_limits()[0])
            time.sleep(3)
            servo.pull_status()
            to_save['pos1'] = servo.status['pos']

            print('interrupted swivel 2')
            to_save['do_interrupt'] = True
            servo.move_to(servo.get_soft_motion_limits()[1])
            time.sleep(3)
            servo.pull_status()
            to_save['pos2'] = servo.status['pos']

            print('uninterrupted swivel 3')
            to_save['do_interrupt'] = False
            servo.move_to(servo.get_soft_motion_limits()[0])
            time.sleep(3)
            servo.pull_status()
            to_save['pos3'] = servo.status['pos']

            print('uninterrupted swivel 4')
            to_save['do_interrupt'] = False
            servo.move_to(0.0)
            time.sleep(3)
            servo.pull_status()
            to_save['pos4'] = servo.status['pos']

        def runstop_interrupter(to_save):
            class R():
                class P():
                    status = {'runstop_event': False}
                pimu = P()
            r = R()
            while to_save['do_interrupt']:
                servo.pull_status()
                if servo.status['vel'] > 0.0:
                    time.sleep(1.0)
                    print('interrupt at {0} rad/s'.format(servo.status['vel']))
                    to_save['interrupts'] += 1
                    to_save['vel1'] = servo.status['vel']
                    r.pimu.status['runstop_event'] = True
                    servo.step_sentry(robot=r)
                    time.sleep(0.5) # TODO: value of 0.1 fails
                    servo.pull_status()
                    to_save['vel2'] = servo.status['vel']
                    r.pimu.status['runstop_event'] = False
                    servo.step_sentry(robot=r)


        with ThreadPoolExecutor(max_workers=2) as executor:
            executor.submit(swivel, to_save)
            executor.submit(runstop_interrupter, to_save)

        self.assertEqual(to_save['interrupts'], 2)
        self.assertNotAlmostEqual(to_save['pos1'], servo.get_soft_motion_limits()[0], places=1)
        self.assertNotAlmostEqual(to_save['pos2'], servo.get_soft_motion_limits()[1], places=1)
        self.assertAlmostEqual(to_save['pos3'], servo.get_soft_motion_limits()[0], places=1)
        self.assertAlmostEqual(to_save['pos4'], 0.0, places=1)
        self.assertNotAlmostEqual(to_save['vel1'], 0.0, places=2)
        self.assertAlmostEqual(to_save['vel2'], 0.0, places=2)

        servo.stop()

    def test_set_motion_profile(self):
        """Verify set_motion_params sets the Dynamixel's vel/accel profile correctly.
        """
        print('test_set_motion_profile')
        servo = stretch_body.dynamixel_hello_XL430.DynamixelHelloXL430(name="wrist_yaw", chain=None)
        self.assertTrue(servo.startup())

        # Check servo starts with profile equal to motion defaults
        vel_ticks = servo.rad_per_sec_to_ticks(servo.params['motion']['default']['vel'])
        accel_ticks = servo.rad_per_sec_sec_to_ticks(servo.params['motion']['default']['accel'])
        self.assertEqual(servo.motor.get_profile_velocity(), vel_ticks)
        self.assertEqual(servo.motor.get_profile_acceleration(), accel_ticks)

        # Set profile to zero
        vel_rad_sec = 0.0
        accel_rad_sec_sec = 0.0
        vel_ticks = servo.rad_per_sec_to_ticks(vel_rad_sec)
        accel_ticks = servo.rad_per_sec_sec_to_ticks(accel_rad_sec_sec)
        servo.set_motion_params(vel_rad_sec, accel_rad_sec_sec)
        self.assertEqual(servo.motor.get_profile_velocity(), vel_ticks)
        self.assertEqual(servo.motor.get_profile_acceleration(), accel_ticks)

        # Set profile to slow
        vel_rad_sec = servo.params['motion']['slow']['vel']
        accel_rad_sec_sec = servo.params['motion']['slow']['accel']
        vel_ticks = servo.rad_per_sec_to_ticks(vel_rad_sec)
        accel_ticks = servo.rad_per_sec_sec_to_ticks(accel_rad_sec_sec)
        servo.set_motion_params(vel_rad_sec, accel_rad_sec_sec)
        self.assertEqual(servo.motor.get_profile_velocity(), vel_ticks)
        self.assertEqual(servo.motor.get_profile_acceleration(), accel_ticks)

        # Set profile to fast
        vel_rad_sec = servo.params['motion']['fast']['vel']
        accel_rad_sec_sec = servo.params['motion']['fast']['accel']
        vel_ticks = servo.rad_per_sec_to_ticks(vel_rad_sec)
        accel_ticks = servo.rad_per_sec_sec_to_ticks(accel_rad_sec_sec)
        servo.set_motion_params(vel_rad_sec, accel_rad_sec_sec)
        self.assertEqual(servo.motor.get_profile_velocity(), vel_ticks)
        self.assertEqual(servo.motor.get_profile_acceleration(), accel_ticks)

        # Set profile to max
        vel_rad_sec = servo.params['motion']['max']['vel']
        accel_rad_sec_sec = servo.params['motion']['max']['accel']
        vel_ticks = servo.rad_per_sec_to_ticks(vel_rad_sec)
        accel_ticks = servo.rad_per_sec_sec_to_ticks(accel_rad_sec_sec)
        servo.set_motion_params(vel_rad_sec, accel_rad_sec_sec)
        self.assertEqual(servo.motor.get_profile_velocity(), vel_ticks)
        self.assertEqual(servo.motor.get_profile_acceleration(), accel_ticks)

        # Set profile to max plus more
        vel_rad_sec = servo.params['motion']['max']['vel'] + 1e5
        accel_rad_sec_sec = servo.params['motion']['max']['accel'] + 1e5
        vel_ticks = servo.rad_per_sec_to_ticks(vel_rad_sec)
        accel_ticks = servo.rad_per_sec_sec_to_ticks(accel_rad_sec_sec)
        servo.set_motion_params(vel_rad_sec, accel_rad_sec_sec)
        self.assertNotEqual(servo.motor.get_profile_velocity(), vel_ticks)
        self.assertNotEqual(servo.motor.get_profile_acceleration(), accel_ticks)

        servo.stop()

    def test_motion_profile_doesnt_persist(self):
        """Verify move_to/move_by with vel/accel doesn't set a motion profile that
        persists to subsequent move_to/move_by commands.
        """
        print('test_motion_profile_doesnt_persist')
        servo = stretch_body.dynamixel_hello_XL430.DynamixelHelloXL430(name="wrist_yaw", chain=None)
        self.assertTrue(servo.startup())

        servo.move_to(0.0)
        time.sleep(5)

        servo.move_to(1.0, v_des=0.0, a_des=0.0)
        move1_vel_ticks = servo.motor.get_profile_velocity()
        move1_accel_ticks = servo.motor.get_profile_acceleration()
        time.sleep(5)

        servo.move_to(0.0)
        move2_vel_ticks = servo.motor.get_profile_velocity()
        move2_accel_ticks = servo.motor.get_profile_acceleration()
        time.sleep(5)

        self.assertNotEqual(move1_vel_ticks, move2_vel_ticks)
        self.assertNotEqual(move1_accel_ticks, move2_accel_ticks)

        servo.stop()
