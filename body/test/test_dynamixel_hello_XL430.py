import math
import time
import unittest
import stretch_body.dynamixel_hello_XL430


class TestDynamixelHelloXL430(unittest.TestCase):

    def test_non_multiturn_move_after_enable_pos(self):
        """Verify non-multiturn servo responds to move_to commands after enable_pos.
        """
        servo = stretch_body.dynamixel_hello_XL430.DynamixelHelloXL430(name="head_tilt", chain=None, verbose=True)
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
        servo = stretch_body.dynamixel_hello_XL430.DynamixelHelloXL430(name="wrist_yaw", chain=None, verbose=True)
        servo.params['use_multiturn'] = False
        self.assertTrue(servo.startup())

        servo.home(single_stop=True) # calls servo.enable_pos() internally
        time.sleep(2)
        servo.pull_status()
        self.assertAlmostEqual(servo.status['pos'], 0.0, places=1)

        servo.stop()
