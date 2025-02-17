import unittest
import stretch_body.trajectories

import numpy as np

stretch_body.trajectories.WAYPOINT_ISCLOSE_ATOL = 0.25


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
        w1 = stretch_body.trajectories.Waypoint(time=1.0, position=0.0, velocity=1000, acceleration=43); print('w1: {0}'.format(w1))
        w2 = stretch_body.trajectories.Waypoint(time=1.0, position=0.0, velocity=1000, acceleration=43); print('w2: {0}'.format(w2))
        self.assertEqual(w1, w2)

        print('not equal')
        w1 = stretch_body.trajectories.Waypoint(time=1.0, position=0.0, velocity=3249, acceleration=43); print('w1: {0}'.format(w1))
        w2 = stretch_body.trajectories.Waypoint(time=1.01 + stretch_body.trajectories.WAYPOINT_ISCLOSE_ATOL, position=0.0, velocity=1000, acceleration=43); print('w2: {0}'.format(w2))
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
        self.assertFalse(w1 >= w2) # this one is a bit weird considering w1 == w2

        w1 = stretch_body.trajectories.Waypoint(time=0.0, position=0.0)
        w2 = stretch_body.trajectories.Waypoint(time=0.01 + stretch_body.trajectories.WAYPOINT_ISCLOSE_ATOL, position=0.0)
        self.assertFalse(w1 == w2)
        self.assertTrue(w1 != w2)
        self.assertTrue(w1 < w2)
        self.assertFalse(w1 > w2)
        self.assertTrue(w1 <= w2)
        self.assertFalse(w1 >= w2)

    def test_segment(self):
        print('equal')
        s1 = stretch_body.trajectories.Segment(segment_id=0, duration=0.0, a0=0.0, a1=0.0, a2=0.0, a3=0.0, a4=0.0, a5=0.0); print('s1: {0}'.format(s1))
        s2 = stretch_body.trajectories.Segment(segment_id=1, duration=0.0, a0=0.0, a1=0.0, a2=0.0, a3=0.0, a4=0.0, a5=0.0); print('s2: {0}'.format(s2))
        self.assertEqual(s1, s2)

        print('not equal')
        s1 = stretch_body.trajectories.Segment(segment_id=0, duration=0.0, a0=0.0, a1=0.0, a2=0.0, a3=0.0, a4=0.0, a5=0.0); print('s1: {0}'.format(s1))
        s2 = stretch_body.trajectories.Segment(segment_id=1, duration=0.0, a0=0.0, a1=100.0, a2=-34.0, a3=0.0, a4=0.0, a5=0.0); print('s2: {0}'.format(s2))
        self.assertNotEqual(s1, s2)

        print('not equal')
        segment_arr = [3.0, 62.425, 0, 0, -3.731, 1.866, -0.249]
        s1 = stretch_body.trajectories.Segment.from_array(segment_arr, segment_id=0); print('s1: {0}'.format(s1))
        s2 = stretch_body.trajectories.Segment(segment_id=1, duration=0.0, a0=0.0, a1=100.0, a2=-34.0, a3=0.0, a4=0.0, a5=0.0); print('s2: {0}'.format(s2))
        self.assertNotEqual(s1, s2)

        print('equal')
        segment_arr = [3.0, 62.425, 0, 0, -3.731, 1.866, -0.249]
        s1 = stretch_body.trajectories.Segment.from_array(segment_arr, segment_id=0); print('s1: {0}'.format(s1))
        s2 = stretch_body.trajectories.Segment(segment_id=1, duration=3.0, a0=62.425, a1=0.0, a2=0.0, a3=-3.731, a4=1.866, a5=-0.249); print('s2: {0}'.format(s2))
        self.assertEqual(s1, s2)

        print('equal')
        segment_arr = [62.425, 0, 0, -3.731, 1.866, -0.249]
        s1 = stretch_body.trajectories.Segment.from_array(segment_arr, segment_id=0, duration=3.0); print('s1: {0}'.format(s1))
        s2 = stretch_body.trajectories.Segment(segment_id=1, duration=3.0, a0=62.425, a1=0.0, a2=0.0, a3=-3.731, a4=1.866, a5=-0.249); print('s2: {0}'.format(s2))
        self.assertEqual(s1, s2)

        print('equal')
        s1_arr = [3.0, 62.425, 0, 0, -3.731, 1.866, -0.249, 0]; print('s1: {0}'.format(s1_arr))
        s2_arr = stretch_body.trajectories.Segment(segment_id=0, duration=3.0, a0=62.425, a1=0.0, a2=0.0, a3=-3.731, a4=1.866, a5=-0.249).to_array(); print('s2: {0}'.format(s2_arr))
        self.assertEqual(s1_arr, s2_arr)
        for s1_elem, s2_elem in zip(s1_arr, s2_arr):
            self.assertAlmostEqual(s1_elem, s2_elem, places=8)

        print('equal')
        s1_arr = [62.425, 0, 0, -3.731, 1.866, -0.249]; print('s1: {0}'.format(s1_arr))
        s2_arr = stretch_body.trajectories.Segment(segment_id=1, duration=3.0, a0=62.425, a1=0.0, a2=0.0, a3=-3.731, a4=1.866, a5=-0.249).to_array(only_coeffs=True); print('s2: {0}'.format(s2_arr))
        self.assertEqual(s1_arr, s2_arr)
        for s1_elem, s2_elem in zip(s1_arr, s2_arr):
            self.assertAlmostEqual(s1_elem, s2_elem, places=8)

        print('equal')
        s1 = stretch_body.trajectories.Segment.zeros(); print('s1: {0}'.format(s1))
        s2 = stretch_body.trajectories.Segment(segment_id=1, duration=0.0, a0=0.0, a1=0.0, a2=0.0, a3=0.0, a4=0.0, a5=0.0); print('s2: {0}'.format(s2))
        self.assertEqual(s1, s2)

    def test_from_invalid_segment_arr(self):
        segment_arr = [4, 65]
        with self.assertRaises(ValueError):
            stretch_body.trajectories.Segment.from_array(segment_arr)

        segment_arr = [4, 65, 4, 56, 36, 346, 345, 345, 346, 346, 234, 23]
        with self.assertRaises(ValueError):
            stretch_body.trajectories.Segment.from_array(segment_arr)

    def test_segment_evaluate_at(self):
        """Verify correctness of the Segment.evaluate_at method

        Spline coefficients calculated in Desmos at:
        https://www.desmos.com/calculator/atv5ilhodq
        """
        # Segment b
        b1 = stretch_body.trajectories.Waypoint(time=0.0, position=62.425, velocity=0.0, acceleration=0.0)
        b2 = stretch_body.trajectories.Waypoint(time=3.0, position=52.350, velocity=0.0, acceleration=0.0)
        b  = stretch_body.trajectories.Segment(segment_id=2, duration=3.0, a0=62.425, a1=0, a2=0, a3=-3.7314815, a4=1.8657407, a5=-0.2487654)

        pos, vel, accel = b.evaluate_at(b1.time - b1.time)
        self.assertAlmostEqual(pos, b1.position, places=4)
        self.assertAlmostEqual(vel, b1.velocity, places=4)
        self.assertAlmostEqual(accel, b1.acceleration, places=4)

        pos, vel, accel = b.evaluate_at(b2.time - b1.time)
        self.assertAlmostEqual(pos, b2.position, places=4)
        self.assertAlmostEqual(vel, b2.velocity, places=4)
        self.assertAlmostEqual(accel, b2.acceleration, places=4)

        pos, vel, accel = b.evaluate_at(1.3 - b1.time)
        self.assertAlmostEqual(pos, 58.632, places=3)
        self.assertNotAlmostEqual(vel, 0.0, places=4)
        self.assertNotAlmostEqual(accel, 0, places=4)

        # Segment c
        c1 = stretch_body.trajectories.Waypoint(time=3.0, position=52.350, velocity=0.0, acceleration=0.0)
        c2 = stretch_body.trajectories.Waypoint(time=6.0, position=62.425, velocity=0.0, acceleration=0.0)
        c  = stretch_body.trajectories.Segment(segment_id=3, duration=3.0, a0=52.350, a1=0, a2=0, a3=3.7314815, a4=-1.8657407, a5=0.2487654)
        pos, vel, accel = c.evaluate_at(c1.time - c1.time)
        self.assertAlmostEqual(pos, c1.position, places=4)
        self.assertAlmostEqual(vel, c1.velocity, places=4)
        self.assertAlmostEqual(accel, c1.acceleration, places=4)

        pos, vel, accel = c.evaluate_at(c2.time - c1.time)
        self.assertAlmostEqual(pos, c2.position, places=4)
        self.assertAlmostEqual(vel, c2.velocity, places=4)
        self.assertAlmostEqual(accel, c2.acceleration, places=4)

        pos, vel, accel = c.evaluate_at(4.584 - c1.time)
        self.assertAlmostEqual(pos, 57.915, places=3)
        self.assertNotAlmostEqual(vel, 0.0, places=4)
        self.assertNotAlmostEqual(accel, 0, places=4)

    def test_invalid_segments(self):
        """Ensure invalid segments fail as expected.

        Dynamic limits visualized on Desmos at:
        https://www.desmos.com/calculator/h6qlt496sr
        """
        b  = stretch_body.trajectories.Segment(segment_id=2, duration=3.0, a0=22.425, a1=0, a2=0, a3=-3.7314815, a4= 1.8657407, a5=-0.2487654)
        c  = stretch_body.trajectories.Segment(segment_id=3, duration=3.0, a0=12.350, a1=0, a2=0, a3= 3.7314815, a4=-1.8657407, a5= 0.2487654)
        self.assertTrue(b.is_valid(v_des=7.0, a_des=10.0))
        self.assertTrue(c.is_valid(v_des=7.0, a_des=10.0))
        self.assertFalse(b.is_valid(v_des=6.28, a_des=10.0))
        self.assertFalse(b.is_valid(v_des=7.0, a_des=6.4))
        self.assertFalse(c.is_valid(v_des=6.28, a_des=10.0))
        self.assertFalse(c.is_valid(v_des=7.0, a_des=6.4))

    def test_segment_from_two_waypoints(self):
        """Verify correctness of the Segment.from_two_waypoints method

        Spline coefficients calculated in Desmos at:
        https://www.desmos.com/calculator/lc8dyfouay
        """
        b_waypoint1 = stretch_body.trajectories.Waypoint(time=0.148, position=0.307, velocity=-0.026, acceleration=0.1320)
        b_waypoint2 = stretch_body.trajectories.Waypoint(time=0.512, position=0.246, velocity= 0.070, acceleration=0.1943)
        expected_b_segment = [0.364, 0.307, -0.026, 0.066, -13.8610492077, 57.9964235233, -64.1494569109]
        expected_b_segment = stretch_body.trajectories.Segment.from_array(expected_b_segment, segment_id=0)
        calculated_b_segment = stretch_body.trajectories.Segment.from_two_waypoints(b_waypoint1, b_waypoint2, segment_id=0)
        self.assertTrue(expected_b_segment == calculated_b_segment)
        self.assertFalse(expected_b_segment != calculated_b_segment)

        f_waypoint1 = stretch_body.trajectories.Waypoint(time=0.512, position=0.246, velocity=0.07)
        f_waypoint2 = stretch_body.trajectories.Waypoint(time=0.964, position=0.170, velocity=0.06)
        expected_f_segment = [0.452, 0.246, 0.07, -1.55846189991, 2.28230081565, 0, 0]
        expected_f_segment = stretch_body.trajectories.Segment.from_array(expected_f_segment, segment_id=0)
        calculated_f_segment = stretch_body.trajectories.Segment.from_two_waypoints(f_waypoint1, f_waypoint2, segment_id=0)
        self.assertTrue(expected_f_segment == calculated_f_segment)
        self.assertFalse(expected_f_segment != calculated_f_segment)

        d_waypoint1 = stretch_body.trajectories.Waypoint(time=0.964, position=0.170, velocity=0.06, acceleration=-0.145)
        d_waypoint2 = stretch_body.trajectories.Waypoint(time=1.270, position=0.303, velocity=0.30, acceleration= 0.068)
        expected_d_segment = [0.306, 0.17, 0.06, -0.0725, 30.5797367333, -140.544613558, 177.975073164]
        expected_d_segment = stretch_body.trajectories.Segment.from_array(expected_d_segment, segment_id=0)
        calculated_d_segment = stretch_body.trajectories.Segment.from_two_waypoints(d_waypoint1, d_waypoint2, segment_id=0)
        self.assertTrue(expected_d_segment == calculated_d_segment)
        self.assertFalse(expected_d_segment != calculated_d_segment)

        e_waypoint1 = stretch_body.trajectories.Waypoint(time=1.27, position=0.303)
        e_waypoint2 = stretch_body.trajectories.Waypoint(time=2.00, position=0.500)
        expected_e_segment = [0.73, 0.303, 0.269863013699, 0, 0, 0, 0]
        expected_e_segment = stretch_body.trajectories.Segment.from_array(expected_e_segment, segment_id=0)
        calculated_e_segment = stretch_body.trajectories.Segment.from_two_waypoints(e_waypoint1, e_waypoint2, segment_id=0)
        self.assertTrue(expected_e_segment == calculated_e_segment)
        self.assertFalse(expected_e_segment != calculated_e_segment)

        g_waypoint1 = stretch_body.trajectories.Waypoint(time=2.00, position=0.500, velocity=0.00, acceleration=0.10)
        g_waypoint2 = stretch_body.trajectories.Waypoint(time=2.49, position=0.005, velocity=0.78, acceleration=0.51)
        expected_g_segment = [0.49, 0.5, 0, 0.05, -54.854605649, 173.708754214, -143.990651018]
        expected_g_segment = stretch_body.trajectories.Segment.from_array(expected_g_segment, segment_id=0)
        calculated_g_segment = stretch_body.trajectories.Segment.from_two_waypoints(g_waypoint1, g_waypoint2, segment_id=0)
        self.assertTrue(expected_g_segment == calculated_g_segment)
        self.assertFalse(expected_g_segment != calculated_g_segment)

    def test_spline_operations(self):
        """Construct and use a spline
        """
        traj = stretch_body.trajectories.Spline()
        print('traj: {0}'.format(traj))
        print('traj segments: ' + traj.__repr_segments__())
        self.assertEqual(len(traj), 0)
        self.assertEqual(len(traj.waypoints), 0)

        b_waypoint1 = stretch_body.trajectories.Waypoint(time=0.148, position=0.307, velocity=-0.026, acceleration=0.1320)
        traj.add(time=b_waypoint1.time, pos=b_waypoint1.position, vel=b_waypoint1.velocity, accel=b_waypoint1.acceleration)
        print('traj: {0}'.format(traj))
        print('traj segments: ' + traj.__repr_segments__())
        self.assertEqual(len(traj), 1)
        self.assertEqual(len(traj.waypoints), 1)
        self.assertEqual(traj[0], b_waypoint1)

        b_waypoint2 = stretch_body.trajectories.Waypoint(time=0.512, position=0.246, velocity= 0.070, acceleration=0.1943)
        traj.add(time=b_waypoint2.time, pos=b_waypoint2.position, vel=b_waypoint2.velocity, accel=b_waypoint2.acceleration)
        print('traj: {0}'.format(traj))
        print('traj segments: ' + traj.__repr_segments__())
        self.assertEqual(len(traj), 2)
        self.assertEqual(len(traj.waypoints), 2)
        self.assertEqual(traj[0], b_waypoint1)
        self.assertEqual(traj[1], b_waypoint2)

        f_waypoint2 = stretch_body.trajectories.Waypoint(time=0.964, position=0.170, velocity=0.06)
        traj.add(time=f_waypoint2.time, pos=f_waypoint2.position, vel=f_waypoint2.velocity)
        print('traj: {0}'.format(traj))
        print('traj segments: ' + traj.__repr_segments__())
        self.assertEqual(len(traj), 3)
        self.assertEqual(len(traj.waypoints), 3)
        self.assertEqual(traj[0], b_waypoint1)
        self.assertEqual(traj[1], b_waypoint2)
        self.assertEqual(traj[2], f_waypoint2)

        traj[1].time = 1000.0
        print('traj: {0}'.format(traj))
        print('traj segments: ' + traj.__repr_segments__())
        self.assertEqual(len(traj), 3)
        self.assertEqual(len(traj.waypoints), 3)
        self.assertEqual(traj[0], b_waypoint1)
        self.assertNotEqual(traj[1], b_waypoint2)
        self.assertEqual(traj[2], f_waypoint2)

        for waypoint in traj:
            print(waypoint)

        traj.clear()
        print('traj: {0}'.format(traj))
        print('traj segments: ' + traj.__repr_segments__())
        self.assertEqual(len(traj), 0)
        self.assertEqual(len(traj.waypoints), 0)

    def test_spline_get_segment(self):
        """Verify correctness of Spline.get_segment

        Spline coefficients calculated in Desmos at:
        https://www.desmos.com/calculator/lc8dyfouay
        """
        b_waypoint1 = stretch_body.trajectories.Waypoint(time=0.148, position=0.307, velocity=-0.026, acceleration=0.1320)
        b_waypoint2 = stretch_body.trajectories.Waypoint(time=0.512, position=0.246, velocity= 0.070, acceleration=0.1943)
        expected_b_coeffs = [0.307, -0.026, 0.066, -13.8610492077, 57.9964235233, -64.1494569109]
        # f_waypoint1 = stretch_body.trajectories.Waypoint(time=0.512, position=0.246, velocity=0.07)
        # f_waypoint2 = stretch_body.trajectories.Waypoint(time=0.964, position=0.170, velocity=0.06)
        # expected_f_coeffs = [0.246, 0.07, -1.55846189991, 2.28230081565, 0, 0]
        d_waypoint1 = stretch_body.trajectories.Waypoint(time=0.964, position=0.170, velocity=0.06, acceleration=-0.145)
        d_waypoint2 = stretch_body.trajectories.Waypoint(time=1.270, position=0.303, velocity=0.30, acceleration= 0.068)
        expected_d_coeffs = [0.17, 0.06, -0.0725, 30.5797367333, -140.544613558, 177.975073164]
        # e_waypoint1 = stretch_body.trajectories.Waypoint(time=1.27, position=0.303)
        # e_waypoint2 = stretch_body.trajectories.Waypoint(time=2.00, position=0.500)
        # expected_e_coeffs = [0.303, 0.269863013699, 0, 0, 0, 0]
        g_waypoint1 = stretch_body.trajectories.Waypoint(time=2.00, position=0.500, velocity=0.00, acceleration=0.10)
        g_waypoint2 = stretch_body.trajectories.Waypoint(time=2.49, position=0.005, velocity=0.78, acceleration=0.51)
        expected_g_coeffs = [0.5, 0, 0.05, -54.854605649, 173.708754214, -143.990651018]

        traj = stretch_body.trajectories.Spline()
        self.assertEqual(traj.get_num_segments(), 0)
        traj.add(time=b_waypoint1.time, pos=b_waypoint1.position, vel=b_waypoint1.velocity, accel=b_waypoint1.acceleration)
        self.assertEqual(traj.get_num_segments(), 0)
        traj.add(time=b_waypoint2.time, pos=b_waypoint2.position, vel=b_waypoint2.velocity, accel=b_waypoint2.acceleration)
        self.assertEqual(traj.get_num_segments(), 1)
        traj.add(time=d_waypoint1.time, pos=d_waypoint1.position, vel=d_waypoint1.velocity, accel=d_waypoint1.acceleration)
        self.assertEqual(traj.get_num_segments(), 2)
        traj.add(time=d_waypoint2.time, pos=d_waypoint2.position, vel=d_waypoint2.velocity, accel=d_waypoint2.acceleration)
        self.assertEqual(traj.get_num_segments(), 3)
        traj.add(time=g_waypoint1.time, pos=g_waypoint1.position, vel=g_waypoint1.velocity, accel=g_waypoint1.acceleration)
        self.assertEqual(traj.get_num_segments(), 4)
        traj.add(time=g_waypoint2.time, pos=g_waypoint2.position, vel=g_waypoint2.velocity, accel=g_waypoint2.acceleration)
        self.assertEqual(traj.get_num_segments(), 5)
        traj.add(time=100.0, pos=g_waypoint2.position)
        self.assertEqual(traj.get_num_segments(), 6)
        print('traj: {0}'.format(traj))
        print('traj segments: ' + traj.__repr_segments__())
        print('traj segments: ' + traj.__repr_segments__(to_motor_rad=lambda pos: 2.0 * pos))
        self.assertEqual(len(traj), 7)
        self.assertEqual(len(traj.waypoints), 7)
        traj.pop()
        self.assertEqual(traj.get_num_segments(), 5)
        self.assertEqual(len(traj), 6)
        self.assertEqual(len(traj.waypoints), 6)

        for s1_elem, s2_elem in zip(traj.get_segment(0).to_array(only_coeffs=True), expected_b_coeffs):
            self.assertAlmostEqual(s1_elem, s2_elem, places=8)
        for s1_elem, s2_elem in zip(traj.get_segment(2).to_array(only_coeffs=True), expected_d_coeffs):
            self.assertAlmostEqual(s1_elem, s2_elem, places=8)
        for s1_elem, s2_elem in zip(traj.get_segment(4).to_array(only_coeffs=True), expected_g_coeffs):
            self.assertAlmostEqual(s1_elem, s2_elem, places=8)

        traj.clear()

    def test_spline_evaluate_at(self):
        """Verify correctness of Spline.evaluate_at

        Spline coefficients calculated in Desmos at:
        https://www.desmos.com/calculator/lc8dyfouay
        """
        b_waypoint1 = stretch_body.trajectories.Waypoint(time=0.148, position=0.307, velocity=-0.026, acceleration=0.1320)
        b_waypoint2 = stretch_body.trajectories.Waypoint(time=0.512, position=0.246, velocity= 0.070, acceleration=0.1943)
        d_waypoint1 = stretch_body.trajectories.Waypoint(time=0.964, position=0.170, velocity=0.06, acceleration=-0.145)
        d_waypoint2 = stretch_body.trajectories.Waypoint(time=1.270, position=0.303, velocity=0.30, acceleration= 0.068)
        g_waypoint1 = stretch_body.trajectories.Waypoint(time=2.00, position=0.500, velocity=0.00, acceleration=0.10)
        g_waypoint2 = stretch_body.trajectories.Waypoint(time=2.49, position=0.005, velocity=0.78, acceleration=0.51)

        traj = stretch_body.trajectories.Spline()
        traj.add(time=b_waypoint1.time, pos=b_waypoint1.position, vel=b_waypoint1.velocity, accel=b_waypoint1.acceleration)
        traj.add(time=b_waypoint2.time, pos=b_waypoint2.position, vel=b_waypoint2.velocity, accel=b_waypoint2.acceleration)
        traj.add(time=d_waypoint1.time, pos=d_waypoint1.position, vel=d_waypoint1.velocity, accel=d_waypoint1.acceleration)
        traj.add(time=d_waypoint2.time, pos=d_waypoint2.position, vel=d_waypoint2.velocity, accel=d_waypoint2.acceleration)
        traj.add(time=g_waypoint1.time, pos=g_waypoint1.position, vel=g_waypoint1.velocity, accel=g_waypoint1.acceleration)
        traj.add(time=g_waypoint2.time, pos=g_waypoint2.position, vel=g_waypoint2.velocity, accel=g_waypoint2.acceleration)
        # print(traj.__repr_segments__())
        pos, vel, acc = traj.evaluate_at(-1.0)
        self.assertAlmostEqual(pos, 0.307, places=3)
        pos, vel, acc = traj.evaluate_at(0.263)
        self.assertAlmostEqual(pos, 0.2927, places=4)
        pos, vel, acc = traj.evaluate_at(0.339)
        self.assertAlmostEqual(pos, 0.2687, places=4)
        pos, vel, acc = traj.evaluate_at(0.4693)
        self.assertAlmostEqual(pos, 0.2441, places=4)
        pos, vel, acc = traj.evaluate_at(1.003)
        self.assertAlmostEqual(pos, 0.1737, places=4)
        pos, vel, acc = traj.evaluate_at(1.1)
        self.assertAlmostEqual(pos, 0.2139, places=4)
        pos, vel, acc = traj.evaluate_at(1.184)
        self.assertAlmostEqual(pos, 0.2678, places=4)
        pos, vel, acc = traj.evaluate_at(2.023)
        self.assertAlmostEqual(pos, 0.4994, places=4)
        pos, vel, acc = traj.evaluate_at(2.3543)
        self.assertAlmostEqual(pos, 0.0000, places=4)
        pos, vel, acc = traj.evaluate_at(2.4146)
        self.assertAlmostEqual(pos, -0.032, places=4)
        pos, vel, acc = traj.evaluate_at(4.0)
        self.assertAlmostEqual(pos, 0.005, places=4)
        pos, vel, acc = traj.evaluate_at(1e100)
        self.assertAlmostEqual(pos, 0.005, places=4)

    def test_invalid_spline(self):
        """Test invalid splines fail as expected.

        Plotted with acceleration curve at:
        https://www.desmos.com/calculator/l5mebpssk6
        """
        b_waypoint1 = stretch_body.trajectories.Waypoint(time=0.0, position=0.307, velocity=-0.026, acceleration=0.1320)
        b_waypoint2 = stretch_body.trajectories.Waypoint(time=0.512, position=0.246, velocity= 0.070, acceleration=0.1943)
        d_waypoint1 = stretch_body.trajectories.Waypoint(time=0.964, position=0.170, velocity=0.06, acceleration=-0.145)
        d_waypoint2 = stretch_body.trajectories.Waypoint(time=1.270, position=0.303, velocity=0.30, acceleration= 0.068)
        g_waypoint1 = stretch_body.trajectories.Waypoint(time=2.00, position=0.500, velocity=0.00, acceleration=0.10)
        g_waypoint2 = stretch_body.trajectories.Waypoint(time=2.49, position=0.005, velocity=0.78, acceleration=0.51)

        traj = stretch_body.trajectories.Spline()
        traj.add(time=b_waypoint1.time, pos=b_waypoint1.position, vel=b_waypoint1.velocity, accel=b_waypoint1.acceleration)
        traj.add(time=b_waypoint2.time, pos=b_waypoint2.position, vel=b_waypoint2.velocity, accel=b_waypoint2.acceleration)
        traj.add(time=d_waypoint1.time, pos=d_waypoint1.position, vel=d_waypoint1.velocity, accel=d_waypoint1.acceleration)
        traj.add(time=d_waypoint2.time, pos=d_waypoint2.position, vel=d_waypoint2.velocity, accel=d_waypoint2.acceleration)
        traj.add(time=g_waypoint1.time, pos=g_waypoint1.position, vel=g_waypoint1.velocity, accel=g_waypoint1.acceleration)
        traj.add(time=g_waypoint2.time, pos=g_waypoint2.position, vel=g_waypoint2.velocity, accel=g_waypoint2.acceleration)
        self.assertTrue(traj.is_valid(10.0, 20.0)[0])

        traj[0].time = -10.0
        self.assertFalse(traj.is_valid(10.0, 20.0)[0])
        traj[0].time = 0.0
        self.assertTrue(traj.is_valid(10.0, 20.0)[0])

        traj[1].time = 1.0
        self.assertFalse(traj.is_valid(10.0, 20.0)[0])
        traj[1].time = 0.512
        self.assertTrue(traj.is_valid(10.0, 20.0)[0])

        traj[1].time = 0.964
        self.assertFalse(traj.is_valid(10.0, 20.0)[0])
        traj[1].time = 0.512
        self.assertTrue(traj.is_valid(10.0, 20.0)[0])

        self.assertFalse(traj.is_valid(10.0, 10.0)[0])
        self.assertTrue(traj.is_valid(10.0, 20.0)[0])

    def test_eval_of_repr(self):
        b_waypoint1 = stretch_body.trajectories.Waypoint(time=0.148, position=0.307, velocity=-0.026, acceleration=0.1320)
        b_waypoint2 = stretch_body.trajectories.Waypoint(time=0.512, position=0.246, velocity= 0.070, acceleration=0.1943)

        from stretch_body.trajectories import Waypoint, Spline, RevoluteTrajectory, PrismaticTrajectory
        traj0 = Spline()
        traj0.add(time=b_waypoint1.time, pos=b_waypoint1.position, vel=b_waypoint1.velocity, accel=b_waypoint1.acceleration)
        traj0.add(time=b_waypoint2.time, pos=b_waypoint2.position, vel=b_waypoint2.velocity, accel=b_waypoint2.acceleration)
        print(traj0)
        traj0_evaled = eval(repr(traj0))
        self.assertTrue(isinstance(traj0_evaled, Spline))
        for i, j in zip(traj0, traj0_evaled):
            self.assertEqual(i, j)

        traj1 = RevoluteTrajectory()
        traj1.add(t_s=b_waypoint1.time, x_r=b_waypoint1.position, v_r=b_waypoint1.velocity, a_r=b_waypoint1.acceleration)
        traj1.add(t_s=b_waypoint2.time, x_r=b_waypoint2.position, v_r=b_waypoint2.velocity, a_r=b_waypoint2.acceleration)
        print(traj1)
        traj1_evaled = eval(repr(traj1))
        self.assertTrue(isinstance(traj1_evaled, RevoluteTrajectory))
        for i, j in zip(traj1, traj1_evaled):
            self.assertEqual(i, j)

        traj2 = PrismaticTrajectory()
        traj2.add(t_s=b_waypoint1.time, x_m=b_waypoint1.position, v_m=b_waypoint1.velocity, a_m=b_waypoint1.acceleration)
        traj2.add(t_s=b_waypoint2.time, x_m=b_waypoint2.position, v_m=b_waypoint2.velocity, a_m=b_waypoint2.acceleration)
        print(traj2)
        traj2_evaled = eval(repr(traj2))
        self.assertTrue(isinstance(traj2_evaled, PrismaticTrajectory))
        for i, j in zip(traj2, traj2_evaled):
            self.assertEqual(i, j)

    def test_diffdrivetrajectory_to_wheel_space(self):
        def translate_to_motor_rad(x_m):
            circ=0.1016*np.pi
            return np.radians(360*3.4*x_m/circ)

        def rotate_to_motor_rad(x_r):
            r = 0.3153 / 2.0
            c = r * x_r #distance wheel travels (m)
            return translate_to_motor_rad(c)

        traj = stretch_body.trajectories.DiffDriveTrajectory()
        left, right = traj._to_wheel_space(0.1, 0.0, translate_to_motor_rad, rotate_to_motor_rad)
        self.assertEqual(left, right)

        left, right = traj._to_wheel_space(0.0, 1.57, translate_to_motor_rad, rotate_to_motor_rad)
        self.assertEqual(left, -right)

    def test_diffdrivetrajectory_compute_wheel_waypoints(self):
        def translate_to_motor_rad(x_m):
            circ=0.1016*np.pi
            return np.radians(360*3.4*x_m/circ)

        def rotate_to_motor_rad(x_r):
            r = 0.3153 / 2.0
            c = r * x_r #distance wheel travels (m)
            return translate_to_motor_rad(c)

        traj = stretch_body.trajectories.DiffDriveTrajectory()
        traj.add(0, 0, 0, 0)
        traj.add(3, 0.05, 0, 0)
        traj.add(6, 0, 0, 0)
        left, right = traj._compute_wheel_waypoints(translate_to_motor_rad, rotate_to_motor_rad)
        self.assertEqual(len(left), 3)
        self.assertAlmostEqual(left[0].position, 0.0)
        self.assertNotAlmostEqual(left[1].position, 0.0)
        self.assertAlmostEqual(left[2].position, 0.0)
        self.assertEqual(len(right), 3)
        self.assertAlmostEqual(right[0].position, 0.0)
        self.assertNotAlmostEqual(right[1].position, 0.0)
        self.assertAlmostEqual(right[2].position, 0.0)

        traj.clear()
        traj.add(0, 0, 0, 0)
        traj.add(3, 0, 0, 1.0)
        traj.add(6, 0, 0, 0)
        left, right = traj._compute_wheel_waypoints(translate_to_motor_rad, rotate_to_motor_rad)
        self.assertEqual(len(left), 3)
        self.assertAlmostEqual(left[0].position, 0.0)
        self.assertNotAlmostEqual(left[1].position, 0.0)
        self.assertAlmostEqual(left[2].position, 0.0)
        self.assertEqual(len(right), 3)
        self.assertAlmostEqual(right[0].position, 0.0)
        self.assertNotAlmostEqual(right[1].position, 0.0)
        self.assertAlmostEqual(right[2].position, 0.0)
