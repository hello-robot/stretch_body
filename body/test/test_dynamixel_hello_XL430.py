import unittest
import stretch_body.dynamixel_hello_XL430


class TestDynamixelHelloXL430(unittest.TestCase):

    def test_startup_wo_multiturn(self):
        servo = stretch_body.dynamixel_hello_XL430.DynamixelHelloXL430(name="head_tilt", chain=None, verbose=True)
        servo.params['use_multiturn'] = False
        self.assertTrue(servo.startup())
        servo.stop()
