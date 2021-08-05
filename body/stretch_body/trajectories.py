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
