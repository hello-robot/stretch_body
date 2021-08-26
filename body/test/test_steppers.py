# Logging level must be set before importing any stretch_body class
import stretch_body.robot_params
stretch_body.robot_params.RobotParams.set_logging_level("DEBUG")

import unittest
import stretch_body.stepper

import time


class TestSteppers(unittest.TestCase):


    def test_is_moving_filtered(self):
        """Test that is_moving_filtered is False when no motion
        """
        motors=['/dev/hello-motor-left-wheel','/dev/hello-motor-right-wheel','/dev/hello-motor-arm','/dev/hello-motor-lift']
        for m in motors:
            print('Testing is moving filtered for %s'%m)
            s = stretch_body.stepper.Stepper(m)
            self.assertTrue(s.startup())
            for i in range(50):
                s.pull_status()
                s.step_sentry(robot=None)
                self.assertFalse(s.status['is_moving_filtered'])
                time.sleep(0.1)
            s.stop()
