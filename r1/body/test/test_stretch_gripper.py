# Logging level must be set before importing any stretch_body class
import stretch_body.robot_params
# stretch_body.robot_params.RobotParams.set_logging_level("DEBUG")

import unittest
import stretch_body.stretch_gripper

import time


class TestStretchGripper(unittest.TestCase):

    def test_waypoint_trajectory(self):
        g = stretch_body.stretch_gripper.StretchGripper()
        self.assertTrue(g.startup())
        if not g.is_calibrated:
            self.fail('test requires stretch gripper to be homed')

        g.trajectory.add(0, 0)
        g.trajectory.add(3, 50)
        g.trajectory.add(6, 0)
        g.logger.info('Executing {0}'.format(g.trajectory.__repr_segments__()))
        self.assertTrue(g.trajectory.is_valid(50, 100))
        g.follow_trajectory()
        time.sleep(7)
        self.assertAlmostEqual(g.status['pos'], 0.0, places=1)

        g.trajectory.clear()
        g.trajectory.add(0, 0, 0)
        g.trajectory.add(3, 50, 0)
        g.trajectory.add(6, 0, 0)
        g.logger.info('Executing {0}'.format(g.trajectory.__repr_segments__()))
        self.assertTrue(g.trajectory.is_valid(50, 100))
        g.follow_trajectory()
        time.sleep(7)
        self.assertAlmostEqual(g.status['pos'], 0.0, places=1)

        g.trajectory.clear()
        g.trajectory.add(0, 0, 0, 0)
        g.trajectory.add(3, 50, 0, 0)
        g.trajectory.add(6, 0, 0, 0)
        g.logger.info('Executing {0}'.format(g.trajectory.__repr_segments__()))
        self.assertTrue(g.trajectory.is_valid(50, 100))
        g.follow_trajectory()
        time.sleep(7)
        self.assertAlmostEqual(g.status['pos'], 0.0, places=1)

        g.stop()
