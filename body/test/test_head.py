# Logging level must be set before importing any stretch_body class
import stretch_body.robot_params
stretch_body.robot_params.RobotParams.set_logging_level("DEBUG")

import unittest
import stretch_body.head

import time


class TestHead(unittest.TestCase):

    def test_homing(self):
        """Test head doesn't home or complaine
        """
        h = stretch_body.head.Head()
        self.assertTrue(h.startup())
        h.home()
        h.stop()

    def test_joints(self):
        """Verify end_of_arm always has at least one joint: wrist_yaw.
        """
        h = stretch_body.head.Head()
        self.assertTrue(len(h.joints) ==2)
        self.assertTrue('head_pan' in h.joints)

    def test_get_joints(self):
        h = stretch_body.head.Head()
        m=h.get_joint('head_pan')
        self.assertEqual('head_pan',m.name)
        m = h.get_joint('foo')
        self.assertEqual(m,None)


    def test_waypoint_trajectory(self):
        h = stretch_body.head.Head()
        self.assertTrue(h.startup())

        h.get_joint('head_tilt').trajectory.add(0, 0)
        h.get_joint('head_tilt').trajectory.add(3, -1.0)
        h.get_joint('head_tilt').trajectory.add(6, 0)
        h.get_joint('head_pan').trajectory.add(0, 0.1)
        h.get_joint('head_pan').trajectory.add(3, -0.9)
        h.get_joint('head_pan').trajectory.add(6, 0.1)
        h.logger.info('Executing tilt {0}'.format(h.get_joint('head_tilt').trajectory.__repr_segments__()))
        h.logger.info('Executing pan {0}'.format(h.get_joint('head_pan').trajectory.__repr_segments__()))
        self.assertTrue(h.get_joint('head_tilt').trajectory.is_valid(4, 8))
        h.follow_trajectory()
        time.sleep(10)
        self.assertAlmostEqual(h.get_joint('head_tilt').status['pos'], 0.0, places=1)
        self.assertAlmostEqual(h.get_joint('head_pan').status['pos'], 0.1, places=1)

        h.get_joint('head_tilt').trajectory.clear()
        h.get_joint('head_pan').trajectory.clear()
        h.get_joint('head_tilt').trajectory.add(0, 0, 0)
        h.get_joint('head_tilt').trajectory.add(3, -1.0, 0)
        h.get_joint('head_tilt').trajectory.add(6, 0, 0)
        h.get_joint('head_pan').trajectory.add(0, 0.1, 0)
        h.get_joint('head_pan').trajectory.add(3, -0.9, 0)
        h.get_joint('head_pan').trajectory.add(6, 0.1, 0)
        h.logger.info('Executing tilt {0}'.format(h.get_joint('head_tilt').trajectory.__repr_segments__()))
        h.logger.info('Executing pan {0}'.format(h.get_joint('head_pan').trajectory.__repr_segments__()))
        self.assertTrue(h.get_joint('head_tilt').trajectory.is_valid(4, 8))
        h.follow_trajectory()
        time.sleep(7)
        self.assertAlmostEqual(h.get_joint('head_tilt').status['pos'], 0.0, places=1)
        self.assertAlmostEqual(h.get_joint('head_pan').status['pos'], 0.1, places=1)

        h.get_joint('head_tilt').trajectory.clear()
        h.get_joint('head_pan').trajectory.clear()
        h.get_joint('head_tilt').trajectory.add(0, 0, 0, 0)
        h.get_joint('head_tilt').trajectory.add(3, -1.0, 0, 0)
        h.get_joint('head_tilt').trajectory.add(6, 0, 0, 0)
        h.get_joint('head_pan').trajectory.add(0, 0.1, 0, 0)
        h.get_joint('head_pan').trajectory.add(3, -0.9, 0, 0)
        h.get_joint('head_pan').trajectory.add(6, 0.1, 0, 0)
        h.logger.info('Executing tilt {0}'.format(h.get_joint('head_tilt').trajectory.__repr_segments__()))
        h.logger.info('Executing pan {0}'.format(h.get_joint('head_pan').trajectory.__repr_segments__()))
        self.assertTrue(h.get_joint('head_tilt').trajectory.is_valid(4, 8))
        h.follow_trajectory()
        time.sleep(7)
        self.assertAlmostEqual(h.get_joint('head_tilt').status['pos'], 0.0, places=1)
        self.assertAlmostEqual(h.get_joint('head_pan').status['pos'], 0.1, places=1)

        h.stop()
