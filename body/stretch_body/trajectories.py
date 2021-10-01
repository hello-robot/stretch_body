from __future__ import print_function
import stretch_body.hello_utils as hu

import numpy as np


class Waypoint:

    def __init__(self, time, position, velocity=None, acceleration=None):
        """Represents one waypoint in a spline

        A linear spline is constructed from position waypoints,
        a cubic spline is constructed from position and velocity waypoints, and
        a quintic spline is constructed from position, velocity, and accleration waypoints.

        Attributes
        ----------
        time : float
            time in seconds
        position : float
            unitless position
        velocity : float
            unitless velocity
        acceleration : float
            unitless acceleration
        """
        if time is None or position is None:
            raise ValueError("time and position must be defined")
        self.time = time
        self.position = position
        self.velocity = velocity
        if velocity is None and acceleration is not None:
            raise ValueError("velocity must be defined if acceleration is defined")
        self.acceleration = acceleration

    def __repr__(self):
        return "Waypoint(time={0}, position={1}{2}{3})".format(self.time, self.position,
            ', velocity={0}'.format(self.velocity) if self.velocity is not None else '',
            ', acceleration={0}'.format(self.acceleration) if self.acceleration is not None else '')

    def __eq__(self, other):
        return np.isclose(self.time, other.time, atol=1e-2)

    def __ne__(self, other):
        return not np.isclose(self.time, other.time, atol=1e-2)

    def __lt__(self, other):
        return np.less(self.time, other.time)

    def __le__(self, other):
        return np.less_equal(self.time, other.time)

    def __gt__(self, other):
        return np.greater(self.time, other.time)

    def __ge__(self, other):
        return np.greater_equal(self.time, other.time)

    def apply_transform(self, transform=lambda pos: pos):
        """Apply a transform to the waypoint, e.g. to convert waypoint into motor space

        Parameters
        ----------
        transform : func or lambda
            used to transform waypoint

        Returns
        -------
        Waypoint
            transformed waypoint in ``Waypoint`` class
        """
        return Waypoint(self.time, transform(self.position), \
            transform(self.velocity) if self.velocity is not None else None, \
            transform(self.acceleration) if self.velocity is not None else None)


class Segment:

    def __init__(self, segment_id, duration, a0, a1, a2, a3, a4, a5):
        """A polynomial segment of a spline

        Coefficients a0-a5 can represent up to a quintic polynomial as
        f(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5

        Attributes
        ----------
        segment_id : int
            The required segment ID for pushing to the hardware
        duration : float
            The duration in seconds the polynomial is tracked
        """
        self.segment_id = segment_id
        self.duration = duration
        self.a0 = a0
        self.a1 = a1
        self.a2 = a2
        self.a3 = a3
        self.a4 = a4
        self.a5 = a5

    def __repr__(self):
        return "Segment(segment_id={0}, duration={1}, a0={2}, a1={3}, a2={4}, a3={5}, a4={6}, a5={7})".format(
            self.segment_id, self.duration, self.a0, self.a1, self.a2, self.a3, self.a4, self.a5)

    def __eq__(self, other):
        return np.isclose(self.duration, other.duration, atol=1e-2) and \
               np.isclose(self.a0, other.a0, atol=1e-2) and \
               np.isclose(self.a1, other.a1, atol=1e-2) and \
               np.isclose(self.a2, other.a2, atol=1e-2) and \
               np.isclose(self.a3, other.a3, atol=1e-2) and \
               np.isclose(self.a4, other.a4, atol=1e-2) and \
               np.isclose(self.a5, other.a5, atol=1e-2)

    def __ne__(self, other):
        return not self.__eq__(other)

    @classmethod
    def zeros(cls, segment_id=2):
        return cls(segment_id=segment_id, duration=0, a0=0, a1=0, a2=0, a3=0, a4=0, a5=0)

    @classmethod
    def from_array(cls, segment_arr, segment_id=None, duration=None):
        if len(segment_arr) == 8:
            segment_id = segment_arr.pop()
        if len(segment_arr) == 7:
            duration = segment_arr.pop(0)
        if len(segment_arr) != 6:
            raise ValueError("invalid number of elements in segment array")
        return cls(segment_id, duration, *segment_arr)

    def to_array(self, only_coeffs=False):
        if only_coeffs:
            return [self.a0, self.a1, self.a2, self.a3, self.a4, self.a5]
        return [self.duration, self.a0, self.a1, self.a2, self.a3, self.a4, self.a5, self.segment_id]

    @classmethod
    def from_two_waypoints(cls, waypoint1, waypoint2, segment_id):
        if waypoint1.acceleration is not None and waypoint2.acceleration is not None:
            i1_waypoint = [waypoint1.time, waypoint1.position, waypoint1.velocity, waypoint1.acceleration]
            i2_waypoint = [waypoint2.time, waypoint2.position, waypoint2.velocity, waypoint2.acceleration]
            segment_arr = hu.generate_quintic_polynomial(i1_waypoint, i2_waypoint)
        elif waypoint1.velocity is not None and waypoint2.velocity is not None:
            i1_waypoint = [waypoint1.time, waypoint1.position, waypoint1.velocity]
            i2_waypoint = [waypoint2.time, waypoint2.position, waypoint2.velocity]
            segment_arr = hu.generate_cubic_polynomial(i1_waypoint, i2_waypoint)
        else:
            i1_waypoint = [waypoint1.time, waypoint1.position]
            i2_waypoint = [waypoint2.time, waypoint2.position]
            segment_arr = hu.generate_linear_polynomial(i1_waypoint, i2_waypoint)
        return cls.from_array(segment_arr, segment_id=segment_id)

    def evaluate_at(self, t):
        return hu.evaluate_polynomial_at(self.to_array(only_coeffs=True), t)


class Spline:

    def __init__(self):
        """Spline representing class

        Presents a interface to create splines from waypoints
        and generate polynomial segments. This class should be
        extended to support trajectories and enforce continuity
        contraints.

        Attributes
        ----------
        waypoints : List(Waypoint)
            a set of waypoints defining the spline
        """
        self.waypoints = []

    def __repr__(self):
        return repr(self.waypoints)

    def print_segments(self, to_motor_rad=lambda pos: pos):
        if len(self.waypoints) < 2:
            return repr([])
        return repr([Segment.from_two_waypoints(w0.apply_transform(to_motor_rad), w1.apply_transform(to_motor_rad), segment_id=i+2) \
            for i, (w0, w1) in enumerate(zip(self.waypoints, self.waypoints[1:]))])

    def __len__(self):
        return len(self.waypoints)

    def __getitem__(self, index):
        return self.waypoints[index]

    def __setitem__(self, index, waypoint):
        self.waypoints[index] = waypoint

    def __delitem__(self, index):
        del self.waypoints[index]

    def __iter__(self):
        for waypoint in self.waypoints:
            yield waypoint

    def pop(self, index=-1):
        return self.waypoints.pop(index)

    def clear(self):
        self.waypoints = []

    def add(self, time, pos, vel=None, accel=None):
        """Add a waypoint to the spline.

        This method will sort through the existing waypoints
        in the spline to insert the waypoint such that
        waypoint time increases with index in the array.

        Parameters
        ----------
        time : float
            time in seconds
        pos : float
            unitless position
        vel : float
            unitless velocity
        accel : float
            unitless acceleration
        """
        new_waypoint = Waypoint(time=time, position=pos, velocity=vel, acceleration=accel)
        if len(self.waypoints) == 0:
            self.waypoints.append(new_waypoint)
            return

        # Cannot have two waypoints scheduled for the same time
        if new_waypoint in self.waypoints:
            return

        # Prepend or append if before first or after last waypoint
        if new_waypoint < self.waypoints[0]:
            self.waypoints.insert(0, new_waypoint)
            return
        if new_waypoint > self.waypoints[0]:
            self.waypoints.append(new_waypoint)
            return

        # Insert before first later waypoint
        for i, other_waypoint in enumerate(self.waypoints):
            if new_waypoint < other_waypoint:
                self.waypoints.insert(i, new_waypoint)
                return

    def get_num_segments(self):
        return max(0, len(self.waypoints)-1)

    def get_segment(self, index, to_motor_rad=lambda pos: pos):
        """Retrieves a segment in the spline by index

        Num of segments is one less than number of waypoints in
        the trajectory. Index bounds are [-1 * num_seg, num_seg).

        Parameters
        ----------
        index : int
            index of segment to return
        to_motor_rad : func or lambda
            used to convert waypoint into motor space

        Returns
        -------
        Segment
            coefficients + duration encapsulated in ``Segment`` class
        """
        if index < -1 * len(self.waypoints) + 1 or index >= len(self.waypoints) - 1:
            return None

        index = index - 1 if index < 0 else index
        w0 = self.waypoints[index].apply_transform(to_motor_rad)
        w1 = self.waypoints[index + 1].apply_transform(to_motor_rad)
        return Segment.from_two_waypoints(w0, w1, segment_id=index + 2)

    def evaluate_at(self, t_s, to_motor_rad=lambda pos: pos):
        """Evaluate a point along the curve at a given time.

        Parameters
        ----------
        t_s : float
            time in seconds
        to_motor_rad : func or lambda
            used to convert waypoint into motor space

        Returns
        -------
        Waypoint
            a ``Waypoint`` class with populated position, velocity, and acceleration.
        """
        if len(self.waypoints) < 2:
            return

        # Return bounds for early or late t_s
        if t_s < self.waypoints[0].time:
            return self.waypoints[0]
        if t_s > self.waypoints[-1].time:
            return self.waypoints[-1]

        # Find segment indices
        for i in range(len(self.waypoints) - 1):
            if t_s >= self.waypoints[i].time and t_s <= self.waypoints[i + 1].time:
                i0, i1 = i, i+1
                i0_t = self.waypoints[i0].time
                break

        # Generate segment and evaluate at t_s
        w0 = self.waypoints[i0]
        w1 = self.waypoints[i1]
        if w0.acceleration is not None and w1.acceleration is not None:
            w0_arr = [w0.time, w0.position, w0.velocity, w0.acceleration]
            w1_arr = [w1.time, w1.position, w1.velocity, w1.acceleration]
            seg = generate_quintic_spline_segment(w0_arr, w1_arr)
        elif w0.velocity is not None and w1.velocity is not None:
            w0_arr = [w0.time, w0.position, w0.velocity]
            w1_arr = [w1.time, w1.position, w1.velocity]
            seg = generate_cubic_spline_segment(w0_arr, w1_arr)
        else:
            w0_arr = [w0.time, w0.position]
            w1_arr = [w1.time, w1.position]
            seg = generate_linear_segment(w0_arr, w1_arr)

        return evaluate_polynomial_at(seg, t_s - i0_t)
