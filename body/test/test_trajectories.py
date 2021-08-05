import unittest
import stretch_body.trajectories


class TestTrajectories(unittest.TestCase):

    def test_waypoint(self):
        print('equal')
        w1 = stretch_body.trajectories.Waypoint(time=0.0, position=0.0); print('w1: {0}'.format(w1))
        w2 = stretch_body.trajectories.Waypoint(time=0.0, position=0.0); print('w2: {0}'.format(w2))
        self.assertEqual(w1, w2)

        print('not equal')
        w1 = stretch_body.trajectories.Waypoint(time=1.0, position=0.0); print('w1: {0}'.format(w1))
        w2 = stretch_body.trajectories.Waypoint(time=0.0, position=0.0); print('w2: {0}'.format(w2))
        self.assertNotEqual(w1, w2)

        print('equal')
        w1 = stretch_body.trajectories.Waypoint(time=0.0, position=0.0, velocity=0.0); print('w1: {0}'.format(w1))
        w2 = stretch_body.trajectories.Waypoint(time=0.0, position=0.0); print('w2: {0}'.format(w2))
        self.assertEqual(w1, w2)

        print('equal')
        w1 = stretch_body.trajectories.Waypoint(time=1.0, position=0.0, velocity=1000, acceleration=43, contact_threshold=10); print('w1: {0}'.format(w1))
        w2 = stretch_body.trajectories.Waypoint(time=1.0, position=0.0, velocity=1000, acceleration=43, contact_threshold=10); print('w2: {0}'.format(w2))
        self.assertEqual(w1, w2)

        print('not equal')
        w1 = stretch_body.trajectories.Waypoint(time=1.0, position=0.0, velocity=3249, acceleration=43, contact_threshold=10); print('w1: {0}'.format(w1))
        w2 = stretch_body.trajectories.Waypoint(time=1.1, position=0.0, velocity=1000, acceleration=43); print('w2: {0}'.format(w2))
        self.assertNotEqual(w1, w2)

    def test_invalid_waypoints(self):
        with self.assertRaises(ValueError):
            stretch_body.trajectories.Waypoint(time=None, position=None)

        with self.assertRaises(ValueError):
            stretch_body.trajectories.Waypoint(time=0.0, position=None)

        with self.assertRaises(ValueError):
            stretch_body.trajectories.Waypoint(time=1.0, position=0.0, acceleration=43)

    def test_waypoint_operations(self):
        w1 = stretch_body.trajectories.Waypoint(time=0.0, position=0.0)
        w2 = stretch_body.trajectories.Waypoint(time=0.0, position=0.0)
        self.assertTrue(w1 == w2)
        self.assertFalse(w1 != w2)
        self.assertFalse(w1 < w2)
        self.assertFalse(w1 > w2)
        self.assertTrue(w1 <= w2)
        self.assertTrue(w1 >= w2)

        w1 = stretch_body.trajectories.Waypoint(time=0.0, position=0.0)
        w2 = stretch_body.trajectories.Waypoint(time=0.001, position=0.0)
        self.assertTrue(w1 == w2)
        self.assertFalse(w1 != w2)
        self.assertTrue(w1 < w2)
        self.assertFalse(w1 > w2)
        self.assertTrue(w1 <= w2)
        self.assertFalse(w1 >= w2)

        w1 = stretch_body.trajectories.Waypoint(time=0.0, position=0.0)
        w2 = stretch_body.trajectories.Waypoint(time=0.1, position=0.0)
        self.assertFalse(w1 == w2)
        self.assertTrue(w1 != w2)
        self.assertTrue(w1 < w2)
        self.assertFalse(w1 > w2)
        self.assertTrue(w1 <= w2)
        self.assertFalse(w1 >= w2)
