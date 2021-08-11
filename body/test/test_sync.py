# Logging level must be set before importing any stretch_body class
import stretch_body.robot_params
stretch_body.robot_params.RobotParams.set_logging_level("DEBUG")

import unittest
import stretch_body.arm
import stretch_body.pimu

import time


class TestSync(unittest.TestCase):

    def test_runstop_sync_disabled(self):
        a = stretch_body.arm.Arm()
        self.assertTrue(a.startup())
        a.motor.disable_sync_mode()
        a.push_command()

        p = stretch_body.pimu.Pimu()
        self.assertTrue(p.startup())

        # Reset runstop and verify state
        p.runstop_event_reset()
        p.push_command()
        p.pull_status()
        a.pull_status()
        self.assertFalse(p.status['runstop_event'])
        time.sleep(0.1)  # Give time for arm to exit stop
        self.assertFalse(a.motor.status['runstop_on'])
        print('Runstop reset done')

        # Should move right away
        if abs(a.status['pos'] - 0.1) < .01:  # Make a small motion at least if already at 0.1
            x_target = 0.11
        else:
            x_target = 0.1
        a.move_to(x_target)
        a.push_command()
        self.assertTrue(a.motor.wait_until_at_setpoint(timeout=5.0))

        # Trigger runstop
        p.runstop_event_trigger()
        p.push_command()
        p.pull_status()
        self.assertTrue(p.status['runstop_event'])
        time.sleep(0.1)  # Give time for arm to enter stop
        a.pull_status()
        self.assertTrue(a.motor.status['runstop_on'])
        print('Runstop trigger done')

        # Should not move
        a.move_to(0.15)
        a.push_command()
        self.assertFalse(a.motor.wait_until_at_setpoint(timeout=2.0))
        a.pull_status()
        self.assertAlmostEqual(a.status['pos'], x_target, places=2)
        print('Not move done')

        # Reset runstop
        p.runstop_event_reset()
        p.push_command()
        p.pull_status()
        self.assertFalse(p.status['runstop_event'])
        time.sleep(0.1)  # Give time for arm to exit stop
        a.pull_status()
        self.assertFalse(a.motor.status['runstop_on'])
        print('Runstop reset done')

        # Should move
        a.move_to(0.15)
        a.push_command()
        self.assertTrue(a.motor.wait_until_at_setpoint(timeout=2.0))
        print("Move to done")

        a.stop()
        p.stop()

    def test_runstop_sync_enabled(self):
        """
        Check that runstop still works when using motor sync enable/disable
        """
        a = stretch_body.arm.Arm()
        self.assertTrue(a.startup())
        a.motor.enable_sync_mode()
        a.push_command()

        p = stretch_body.pimu.Pimu()
        self.assertTrue(p.startup())

        #Reset runstop and verify state
        p.runstop_event_reset()
        p.push_command()
        p.pull_status()
        a.pull_status()
        self.assertFalse(p.status['runstop_event'])
        time.sleep(0.1)  # Give time for arm to exit stop
        self.assertFalse(a.motor.status['runstop_on'])
        print('Runstop reset done')

        #Should not move until sync signal is sent
        if abs(a.status['pos']-0.1)<.01: #Make a small motion at least if already at 0.1
            x_target=0.11
        else:
            x_target=0.1
        a.move_to(x_target)
        a.push_command()
        self.assertFalse(a.motor.wait_until_at_setpoint(timeout=5.0))


        #Trigger motion
        p.trigger_motor_sync()
        self.assertTrue(a.motor.wait_until_at_setpoint(timeout=2.0))
        print('Move to done')

        #Trigger runstop
        p.runstop_event_trigger()
        p.push_command()
        p.pull_status()
        self.assertTrue(p.status['runstop_event'])
        time.sleep(0.1) #Give time for arm to enter stop
        a.pull_status()
        self.assertTrue(a.motor.status['runstop_on'])
        print('Runstop trigger done')

        #Should not move
        a.move_to(0.15)
        a.push_command()
        p.trigger_motor_sync()
        self.assertFalse(a.motor.wait_until_at_setpoint(timeout=2.0))
        a.pull_status()
        self.assertAlmostEqual(a.status['pos'], x_target, places=2)
        print('Not move done')

        #Reset runstop
        p.runstop_event_reset()
        p.push_command()
        p.pull_status()
        self.assertFalse(p.status['runstop_event'])
        time.sleep(0.1) #Give time for arm to exit stop
        a.pull_status()
        self.assertFalse(a.motor.status['runstop_on'])
        print('Runstop reset done')

        #Should not move as the runstop will have nullifed the last move_to
        p.trigger_motor_sync()
        self.assertFalse(a.motor.wait_until_at_setpoint(timeout=2.0))
        print('Not move done')

        #Should move
        a.move_to(0.15)
        a.push_command()
        p.trigger_motor_sync()
        self.assertTrue(a.motor.wait_until_at_setpoint(timeout=2.0))
        print("Move to done")

        a.stop()
        p.stop()
