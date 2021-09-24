# Logging level must be set before importing any stretch_body class
import stretch_body.robot_params
stretch_body.robot_params.RobotParams.set_logging_level("DEBUG")

import unittest
import stretch_body.arm
import stretch_body.pimu
import stretch_body.robot
import time


class TestSync(unittest.TestCase):

    def test_motor_sync(self):
        """Verify that the motor sync enable/disable works as expected
        """
        p = stretch_body.pimu.Pimu()
        p.startup()
        a = stretch_body.arm.Arm()
        a.startup()

        #First verify can move w/o sync_mode
        a.motor.disable_sync_mode()
        a.move_to(0.1)
        a.push_command()
        a.motor.wait_until_at_setpoint(timeout=5.0)
        a.pull_status()
        self.assertAlmostEqual(a.status['pos'], 0.1, places=1)

        #Now verify in sync mode it doesn't move until the Pimu triggers
        a.motor.enable_sync_mode()
        a.push_command()
        a.move_to(0.05)
        a.push_command()
        a.motor.wait_until_at_setpoint(timeout=2.0)
        a.pull_status()
        self.assertAlmostEqual(a.status['pos'], 0.1, places=1)

        p.trigger_motor_sync()
        time.sleep(2.0)
        a.motor.wait_until_at_setpoint(timeout=2.0)
        a.pull_status()
        self.assertAlmostEqual(a.status['pos'], 0.05, places=1)

        #Now turn sync mode off and verify that moves still
        a.motor.disable_sync_mode()
        a.move_to(0.15)
        a.push_command()
        a.motor.wait_until_at_setpoint(timeout=2.0)
        a.pull_status()
        self.assertAlmostEqual(a.status['pos'], 0.15, places=1)

        p.stop()
        a.stop()

    def test_sync_stream(self):
        """Check that streaming sync pulses to
        Stepper doesnt place it in runstop
        """
        r = stretch_body.robot.Robot()
        self.assertTrue(r.startup())
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
