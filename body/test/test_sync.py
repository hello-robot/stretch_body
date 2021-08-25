# Logging level must be set before importing any stretch_body class
import stretch_body.robot_params
stretch_body.robot_params.RobotParams.set_logging_level("DEBUG")

import unittest
import stretch_body.arm
import stretch_body.pimu
import stretch_body.robot
import time


class TestSync(unittest.TestCase):

    def test_sync_stream(self):
        #Check that streaming sync pulses to
        #Stepper doesnt place in runstop
        r = stretch_body.robot.Robot()
        r.startup()
        r.arm.move_to(0.0)
        r.push_command()
        r.arm.motor.wait_until_at_setpoint(timeout=3.0)
        #r.arm.motor.disable_sync_mode()
        r.arm.move_to(0.3)
        r.push_command()
        ts=time.time()
        while time.time()-ts<5.0:
            r.pimu.trigger_motor_sync()
            r.arm.pretty_print()
            self.assertFalse(r.arm.motor.status['runstop_on'])
            #time.sleep(0.01)
        self.assertAlmostEqual(r.arm.status['pos'], 0.3, places=1)
        r.stop()

