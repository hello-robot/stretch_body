class Waypoint:
    def __init__(self, time=None, position=None, velocity=None, acceleration=None):
        self.t = time
        self.p = position
        self.v = velocity
        self.a = acceleration

    def apply_conversion(self, conversion_function):
        np = None if self.p is None else conversion_function(self.p)
        nv = None if self.v is None else conversion_function(self.v)
        na = None if self.a is None else conversion_function(self.a)
        return Waypoint(self.t, np, nv, na)

    def __repr__(self):
        return "Waypoint(t={0}, pos={1}, vel={2}, accel={3})".format(self.t, self.p, self.v, self.a)


class QuinticPolynomialSegment:
    """Represents a quintic polynomial of the form f(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5."""

    def __init__(self, start_t, end_t, a0=0.0, a1=0.0, a2=0.0, a3=0.0, a4=0.0, a5=0.0):
        self.start_t = start_t
        self.dt = end_t - start_t
        self.coefficients = [a0, a1, a2, a3, a4, a5]

    def evaluate_at(self, full_t):
        """Evaluate a quintic polynomial at a given time.

        Parameters
        ----------
        t : float
            the time in seconds at which to evaluate the polynomial

        Returns
        -------
        List(float)
            array with three elements: evaluated position, velocity, and acceleration.

        """
        t = full_t - self.start_t
        c = self.coefficients  # Just for brevity
        pos = c[0] + (c[1] * t) + (c[2] * t**2) + (c[3] * t**3) + (c[4] * t**4) + (c[5] * t**5)
        vel = c[1] + (2 * c[2] * t) + (3 * c[3] * t**2) + (4 * c[4] * t**3) + (5 * c[5] * t**4)
        accel = 2 * c[2] + (6 * c[3] * t) + (12 * c[4] * t**2) + (20 * c[5] * t**3)
        return Waypoint(full_t, pos, vel, accel)

    def __repr__(self):
        coefficient_s = ', '.join('a{0}={1}'.format(i, v) for i, v in enumerate(self.coefficients))
        return 'Segment(duration={0}, {1})'.format(self.dt, coefficient_s)

    def __eq__(self, other):
        return self.dt == other.dt and self.coefficients == other.coefficients

    def __ne__(self, other):
        return not self.__eq__(other)


def generate_quintic_spline_segment(wp0, wp1):
    dt = wp1.t - wp0.t
    a0 = wp0.p
    a1 = wp0.v
    a2 = wp0.a / 2
    a3_num = 20 * wp1.p - 20 * wp0.p - (8 * wp1.v + 12 * wp0.v) * dt - (3 * wp0.a - wp1.a) * (dt ** 2)
    a3_den = 2 * (dt ** 3)
    a3 = a3_num / a3_den
    a4_num = 30 * wp0.p - 30 * wp1.p + (14 * wp1.v + 16 * wp0.v) * dt + (3 * wp0.a - 2 * wp1.a) * (dt ** 2)
    a4_den = 2 * (dt ** 4)
    a4 = a4_num / a4_den
    a5 = (12 * wp1.p - 12 * wp0.p - (6 * wp1.v + 6 * wp0.v) * dt - (wp0.a - wp1.a) * (dt ** 2)) / (2 * (dt ** 5))
    return QuinticPolynomialSegment(wp0.t, wp1.t, a0, a1, a2, a3, a4, a5)


def generate_cubic_spline_segment(wp0, wp1):
    dt = wp1.t - wp0.t
    a0 = wp0.p
    a1 = wp0.v
    a2 = (3 / dt ** 2) * (wp1.p - wp0.p) - (2 / dt) * wp0.v - (1 / dt) * wp1.v
    a3 = (-2 / dt ** 3) * (wp1.p - wp0.p) + (1 / dt ** 2) * (wp1.v + wp0.v)
    return QuinticPolynomialSegment(wp0.t, wp1.t, a0, a1, a2, a3)


def generate_linear_segment(wp0, wp1):
    dt = wp1.t - wp0.t
    a0 = wp0.p
    a1 = (wp1.p - wp0.p) / dt
    return QuinticPolynomialSegment(wp0.t, wp1.t, a0, a1)


class Trajectory:
    """Basic trajectory representing class.

    Attributes
    ----------
    waypoints : list(Waypoint)
        a set of waypoints defining the trajectory

    """

    def __init__(self):
        self.waypoints = []

    def __len__(self):
        return len(self.waypoints)

    def __repr__(self):
        return str(self.waypoints)

    def add_waypoint(self, wp):
        if len(self.waypoints) == 0 or wp.t > self.waypoints[-1].t:
            # Empty set or late waypoint
            self.waypoints.append(wp)
            return
        elif wp.t < self.waypoints[0].t:
            # Early waypoint
            self.waypoints.insert(0, wp)
            return

        for i, existing_wp in enumerate(self.waypoints):
            # Insert before first later waypoint
            if wp.t < existing_wp.t:
                self.waypoints.insert(i, wp)
            elif wp.t == existing_wp.t:
                return

    def clear_waypoints(self):
        self.waypoints = []

    def get_waypoint(self, index):
        if index >= -1 * len(self.waypoints) and index < len(self.waypoints):
            return self.waypoints[index]

    def get_num_segments(self):
        return max(0, len(self.waypoints) - 1)

    def get_segment(self, index, to_motor_rad=None):
        """Retrieve a segment in the trajectory by index.

        Num of segments is one less than number of waypoints in
        the trajectory. Index bounds are [-1 * num_seg, num_seg).

        Parameters
        ----------
        index : int
            index of segment to return
        to_motor_rad : func or none
            used to convert into motor space if not None

        Returns
        -------
        QuinticPolynomialSegment
            coefficients + duration encapsulated in ``QuinticPolynomialSegment`` class

        """
        if index < -1 * len(self.waypoints) + 1 or index >= len(self.waypoints) - 1:
            # Invalid index
            return QuinticPolynomialSegment()

        index = index - 1 if index < 0 else index
        waypoint0 = self.get_waypoint(index)
        waypoint1 = self.get_waypoint(index + 1)

        if to_motor_rad is not None:
            waypoint0 = waypoint0.apply_conversion(to_motor_rad)
            waypoint1 = waypoint1.apply_conversion(to_motor_rad)

        if waypoint0.v is None:
            # Have just time and position (no velocity or acceleration) --> Generate Linear
            return generate_linear_segment(waypoint0, waypoint1)
        elif waypoint0.a is None:
            # Have time, position and velocity, but no acceleration --> Generate Cubic
            return generate_cubic_spline_segment(waypoint0, waypoint1)
        else:
            # Have time, position, velocity and acceleration --> Generate Quintic
            return generate_quintic_spline_segment(waypoint0, waypoint1)

    def delete_waypoint(self, index):
        if index >= -1 * len(self.waypoints) and index < len(self.waypoints):
            return self.waypoints.pop(index)

    def get_index(self, t_s):
        for i in range(len(self.waypoints) - 1):
            if t_s >= self.get_waypoint(i).t and t_s <= self.get_waypoint(i + 1).t:
                return i

    def evaluate_at(self, t_s):
        """Evaluate a point along the curve at a given time.

        Parameters
        ----------
        t_s : float
            time in seconds

        Returns
        -------
        Waypoint
            a ``Waypoint`` class with populated position, velocity, and acceleration.

        """
        if len(self.waypoints) < 2:
            return

        # Return bounds for early or late t_s
        if t_s < self.get_waypoint(0).t:
            return self.get_waypoint(0)
        if t_s > self.get_waypoint(-1).t:
            return self.get_waypoint(-1)

        # Find segment indices
        index = self.get_index(t_s)
        segment = self.get_segment(index)
        return segment.evaluate_at(t_s)

    def get_duration(self):
        if not self.waypoints:
            return 0.0
        else:
            return self.waypoints[-1].t - self.waypoints[0].t
