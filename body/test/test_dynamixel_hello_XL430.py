# Logging level must be set before importing any stretch_body class
import stretch_body.robot_params
stretch_body.robot_params.RobotParams.set_logging_level("DEBUG")

import unittest
import stretch_body.dynamixel_hello_XL430

import math
import time
from concurrent.futures import ThreadPoolExecutor


class TestDynamixelHelloXL430(unittest.TestCase):

    def test_non_multiturn_move_after_enable_pos(self):
        """Verify non-multiturn servo responds to move_to commands after enable_pos.
        """
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
        servo = stretch_body.dynamixel_hello_XL430.DynamixelHelloXL430(name="wrist_yaw", chain=None)
        self.assertTrue(servo.startup())

        servo.home(single_stop=False, move_to_zero=True)
        servo.pull_status()
        self.assertAlmostEqual(servo.status['pos'], 0.0, places=1)

        servo.stop()

    def test_runstop(self):
        """Verify dynamixel_hello respect runstop via step_sentry
        """
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
            servo.move_to(servo.soft_motion_limits[0])
            time.sleep(3)
            servo.pull_status()
            to_save['pos1'] = servo.status['pos']

            print('interrupted swivel 2')
            to_save['do_interrupt'] = True
            servo.move_to(servo.soft_motion_limits[1])
            time.sleep(3)
            servo.pull_status()
            to_save['pos2'] = servo.status['pos']

            print('uninterrupted swivel 3')
            to_save['do_interrupt'] = False
            servo.move_to(servo.soft_motion_limits[0])
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
        self.assertNotAlmostEqual(to_save['pos1'], servo.soft_motion_limits[0], places=1)
        self.assertNotAlmostEqual(to_save['pos2'], servo.soft_motion_limits[1], places=1)
        self.assertAlmostEqual(to_save['pos3'], servo.soft_motion_limits[0], places=1)
        self.assertAlmostEqual(to_save['pos4'], 0.0, places=1)
        self.assertNotAlmostEqual(to_save['vel1'], 0.0, places=2)
        self.assertAlmostEqual(to_save['vel2'], 0.0, places=2)

        servo.stop()
