import unittest
import stretch_body.dynamixel_XL430

from concurrent.futures import ThreadPoolExecutor


class TestDynamixelXL430(unittest.TestCase):

    def test_concurrent_access(self):
        """Verify zero comms errors in concurrent access.
        """
        servo = stretch_body.dynamixel_XL430.DynamixelXL430(dxl_id=12, usb="/dev/hello-dynamixel-head", verbose=True)
        self.assertTrue(servo.startup())

        def ping_n(n):
            # servo.pretty_print() # causes many more servo communications
            servo.do_ping(verbose=False)

        ns = [1,2,3,4,5]
        with ThreadPoolExecutor(max_workers = 2) as executor:
            results = executor.map(ping_n, ns)
        self.assertEqual(servo.comm_errors, 0)
        self.assertTrue(servo.last_comm_success)

        servo.stop()

    def test_handle_comm_result(self):
        """Verify comm results correctly handled.
        """
        servo = stretch_body.dynamixel_XL430.DynamixelXL430(dxl_id=12, usb="/dev/hello-dynamixel-head", verbose=False)
        self.assertTrue(servo.startup())

        ret = servo.handle_comm_result('DXL_TEST', 0, 0)
        self.assertTrue(ret)
        self.assertTrue(servo.last_comm_success)
        self.assertEqual(servo.comm_errors, 0)

        ret = servo.handle_comm_result('DXL_TEST', -1000, 0) # -1000 = PORT BUSY
        self.assertFalse(ret)
        self.assertFalse(servo.last_comm_success)
        self.assertEqual(servo.comm_errors, 1)

        ret = servo.handle_comm_result('DXL_TEST', -3002, 1) # -3002 = RX Corrupt
        self.assertFalse(ret)
        self.assertFalse(servo.last_comm_success)
        self.assertEqual(servo.comm_errors, 2)

        servo.stop()

    def test_change_baud_rate(self, dxl_id=13, usb="/dev/hello-dynamixel-wrist"):
        """Verify can change baud rate.
        """
        start_baud = stretch_body.dynamixel_XL430.DynamixelXL430.identify_baud_rate(dxl_id=dxl_id, usb=usb)
        servo1 = stretch_body.dynamixel_XL430.DynamixelXL430(dxl_id=dxl_id, usb=usb, baud=start_baud, verbose=False)
        self.assertTrue(servo1.do_ping(verbose=False))

        curr_baud = servo1.get_baud_rate()
        self.assertEqual(curr_baud, start_baud)
        self.assertTrue(servo1.do_ping(verbose=False))

        # invalid baud goal
        goal_baud = 9000
        succeeded = servo1.set_baud_rate(goal_baud)
        self.assertFalse(succeeded)
        curr_baud = servo1.get_baud_rate()
        self.assertNotEqual(curr_baud, goal_baud)
        self.assertTrue(servo1.do_ping(verbose=False))

        # change the baud
        goal_baud = 115200 if start_baud != 115200 else 57600
        succeeded = servo1.set_baud_rate(goal_baud)
        self.assertTrue(succeeded)
        servo2 = stretch_body.dynamixel_XL430.DynamixelXL430(dxl_id=dxl_id, usb=usb, baud=goal_baud, verbose=False)
        curr_baud = servo2.get_baud_rate()
        self.assertEqual(curr_baud, goal_baud)
        self.assertTrue(servo2.do_ping(verbose=False))
        servo3 = stretch_body.dynamixel_XL430.DynamixelXL430(dxl_id=dxl_id, usb=usb, baud=start_baud, verbose=False)
        curr_baud = servo3.get_baud_rate()
        self.assertNotEqual(curr_baud, goal_baud)
        self.assertFalse(servo3.do_ping(verbose=False))

        # reset baud to its starting baud
        servo4 = stretch_body.dynamixel_XL430.DynamixelXL430(dxl_id=dxl_id, usb=usb, baud=goal_baud, verbose=False)
        self.assertTrue(servo4.do_ping(verbose=False))
        succeeded = servo4.set_baud_rate(start_baud)
        self.assertTrue(succeeded)
        servo5 = stretch_body.dynamixel_XL430.DynamixelXL430(dxl_id=dxl_id, usb=usb, baud=start_baud, verbose=False)
        curr_baud = servo5.get_baud_rate()
        self.assertEqual(curr_baud, start_baud)
        self.assertTrue(servo5.do_ping(verbose=False))

        servo1.stop()
        servo2.stop()
        servo3.stop()
        servo4.stop()
        servo5.stop()
