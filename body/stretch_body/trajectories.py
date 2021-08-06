from __future__ import print_function
import stretch_body.hello_utils as hu

import numpy as np


class Waypoint:

    def __init__(self, time, position, velocity=None, acceleration=None, contact_threshold=None):
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
        contact_threshold : float
            force threshold to stop motion in ~Newtons
        """
        if time is None or position is None:
            raise ValueError("time and position must be defined")
        self.time = time
        self.position = position
        self.velocity = velocity
        if velocity is None and acceleration is not None:
            raise ValueError("velocity must be defined if acceleration is defined")
        self.acceleration = acceleration
        self.contact_threshold = contact_threshold

    def __repr__(self):
        return "Waypoint(t={0}, pos={1}{2}{3}{4})".format(self.time, self.position,
            ', vel={0}'.format(self.velocity) if self.velocity is not None else '',
            ', accel={0}'.format(self.acceleration) if self.acceleration is not None else '',
            ', contact_threshold={0}'.format(self.contact_threshold) if self.contact_threshold is not None else '')

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
        return "Segment(id={0}, duration={1}, a0={2}, a1={3}, a2={4}, a3={5}, a4={6}, a5={7})".format(
            self.segment_id, self.duration, self.a0, self.a1, self.a2, self.a3, self.a4, self.a5)

    def __eq__(self, other):
        return np.isclose(self.duration, other.duration, atol=1e-2) and \
               np.isclose(self.a0, other.a0, atol=1e-2) and \
               np.isclose(self.a1, other.a1, atol=1e-2) and \
               np.isclose(self.a2, other.a2, atol=1e-2) and \
               np.isclose(self.a3, other.a3, atol=1e-2) and \
               np.isclose(self.a4, other.a4, atol=1e-2) and \
               np.isclose(self.a5, other.a5, atol=1e-2)

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

    def evaluate_at(self, t):
        return hu.evaluate_polynomial_at(self.to_array(only_coeffs=True), t)
