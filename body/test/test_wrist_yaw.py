# Logging level must be set before importing any stretch_body class
import stretch_body.robot_params
# stretch_body.robot_params.RobotParams.set_logging_level("DEBUG")

import unittest
import stretch_body.wrist_yaw

import time


class TestWristYaw(unittest.TestCase):

    def test_waypoint_trajectory(self):
        w = stretch_body.wrist_yaw.WristYaw()
        self.assertTrue(w.startup())
        if not w.is_calibrated:
            self.fail('test requires wrist yaw to be homed')

        w.trajectory.add(0, 3.14)
        w.trajectory.add(3, 2.14)
        w.trajectory.add(6, 3.14)
        w.logger.info('Executing {0}'.format(w.trajectory.__repr_segments__()))
        self.assertTrue(w.trajectory.is_valid(4, 8))
        w.follow_trajectory()
        time.sleep(7)
        self.assertAlmostEqual(w.status['pos'], 3.14, places=1)

        w.trajectory.clear()
        w.trajectory.add(0, 3.14, 0.0)
        w.trajectory.add(3, 2.14, 0.0)
        w.trajectory.add(6, 3.14, 0.0)
        w.logger.info('Executing {0}'.format(w.trajectory.__repr_segments__()))
        self.assertTrue(w.trajectory.is_valid(4, 8))
        w.follow_trajectory()
        time.sleep(7)
        self.assertAlmostEqual(w.status['pos'], 3.14, places=1)

        w.trajectory.clear()
        w.trajectory.add(0, 3.14, 0.0, 0.0)
        w.trajectory.add(3, 2.14, 0.0, 0.0)
        w.trajectory.add(6, 3.14, 0.0, 0.0)
        w.logger.info('Executing {0}'.format(w.trajectory.__repr_segments__()))
        self.assertTrue(w.trajectory.is_valid(4, 8))
        w.follow_trajectory()
        time.sleep(7)
        self.assertAlmostEqual(w.status['pos'], 3.14, places=1)

        w.stop()

    def test_runstopped_waypoint_trajectory(self):
        """Simulate a runstop to see if the trajectory stops
        """
        w = stretch_body.wrist_yaw.WristYaw()
        self.assertTrue(w.startup())
        if not w.is_calibrated:
            self.fail('test requires wrist yaw to be homed')
        w.robot_params['robot_sentry']['dynamixel_stop_on_runstop'] = True
        w.params['enable_runstop'] = True

        class P:
            def __init__(self):
                self.status = {'runstop_event': False}

        class R:
            def __init__(self):
                self.pimu = P()

        robot = R()

        w.trajectory.add(0, 3.14)
        w.trajectory.add(3, 2.14)
        w.trajectory.add(6, 3.14)
        w.logger.info('Executing {0}'.format(w.trajectory.__repr_segments__()))
        self.assertTrue(w.trajectory.is_valid(4, 8))
        w.follow_trajectory()
        w.step_sentry(robot)
        time.sleep(2)
        robot.pimu.status['runstop_event'] = True
        w.step_sentry(robot)
        self.assertTrue(w.was_runstopped)
        time.sleep(0.1)
        w.pull_status()
        self.assertAlmostEqual(w.status['vel'], 0.0, places=3)

        w.stop()
