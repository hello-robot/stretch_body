# Logging level must be set before importing any stretch_body class
import stretch_body.robot_params
stretch_body.robot_params.RobotParams.set_logging_level("DEBUG")

import unittest
import stretch_body.pimu

import time


class TestPimu(unittest.TestCase):

    def test_motor_sync_rate(self):
        print('test_motor_sync_rate')
        p = stretch_body.pimu.Pimu()
        p.startup()
        for i in range(100):
            p.trigger_motor_sync()
            time.sleep(1/p.params['max_sync_rate_hz'])
        self.assertTrue(p.status['motor_sync_drop']==0)
        for i in range(100):
            p.trigger_motor_sync()
            time.sleep(0.01)
        self.assertTrue(p.status['motor_sync_drop']>75)

    def test_invalid_protocol(self):
        """Simulate an invalid protocol and verify the correct error.
        """
        p = stretch_body.pimu.Pimu()
        p.valid_firmware_protocol = 'p-1' # valid protocols are p0 and up
        self.assertFalse(p.startup())

        p.stop()
