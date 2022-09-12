from __future__ import print_function
import stretch_body.hello_utils as hu

import numpy as np

# Limits how close together waypoints can be planned
WAYPOINT_ISCLOSE_ATOL = 0.8


class Waypoint:

    def __init__(self, time, position, velocity=None, acceleration=None):
        """Represents one waypoint in a spline

        A linear spline is constructed from position waypoints,
        a cubic spline is constructed from position and velocity waypoints, and
        a quintic spline is constructed from position, velocity, and acceleration waypoints.

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
        if time < 0.0:
            raise ValueError("time cannot be negative")
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
        return np.isclose(self.time, other.time, atol=WAYPOINT_ISCLOSE_ATOL)

    def __ne__(self, other):
        return not self.__eq__(other)

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
            transform(self.acceleration) if self.acceleration is not None else None)


class SE2Waypoint(Waypoint):

    def __init__(self, time, pose, vel_twist=None, accel_twist=None):
        """Represents one waypoint in a translation/rotation restricted base spline

        Attributes
        ----------
        time : float
            time in seconds
        pose : Tuple(float, float, float)
            unitless pose as tuple (x, y, theta)
        vel_twist : Tuple(float, float)
            unitless velocity twist as tuple (translational_velocity, rotational_velocity)
        accel_twist : Tuple(float, float)
            unitless acceleration twist as tuple (translational_acceleration, rotational_acceleration)
        """
        if len(pose) != 3:
            raise ValueError("pose must be tuple (x, y, theta)")
        if vel_twist is not None and len(vel_twist) != 2:
            raise ValueError("vel_twist must be tuple (translational_velocity, rotational_velocity)")
        if accel_twist is not None and len(accel_twist) != 2:
            raise ValueError("accel_twist must be tuple (translational_acceleration, rotational_acceleration)")
        Waypoint.__init__(self, time, pose, vel_twist, accel_twist)
        self.pose = pose
        self.vel_twist = vel_twist
        self.accel_twist = accel_twist

    def __repr__(self):
        return "SE2Waypoint(time={0}, pose={1}{2}{3})".format(self.time, self.pose,
            ', vel_twist={0}'.format(self.vel_twist) if self.vel_twist is not None else '',
            ', accel_twist={0}'.format(self.accel_twist) if self.accel_twist is not None else '')

    def apply_transform(self, transform=lambda pos: pos):
        raise NotImplementedError('transform are not applied to SE2 waypoints directly')


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
        """Evaluates segment at a given time.

        Parameters
        ----------
        t : float
            the time in seconds at which to evaluate the polynomial

        Returns
        -------
        Tuple(float)
            tuple with three elements: evaluated position, velocity, and acceleration.
        """
        return hu.evaluate_polynomial_at(self.to_array(only_coeffs=True), t)

    def is_valid(self, v_des, a_des):
        """Determines whether segment adheres to dynamic limits.

        Parameters
        ----------
        v_des : float
            Velocity limit that the segment shouldn't exceed
        a_des : float
            Acceleration limit that the segment shouldn't exceed

        Returns
        -------
        bool
            whether the segment is valid
        """
        success,v_max,a_max=hu.is_segment_feasible(self.to_array(), v_des, a_des)
        return success


class Spline:

    def __init__(self, init_waypoints=None):
        """Spline representing class

        Presents a interface to create splines from waypoints
        and generate polynomial segments. This class should be
        extended to support trajectories and enforce continuity
        constraints.

        Parameters
        ----------
        init_waypoints : List(Waypoint)
            optional, starting list of waypoints e.g. for eval-ing repr

        Attributes
        ----------
        waypoints : List(Waypoint)
            a set of waypoints defining the spline
        """
        self.waypoints = init_waypoints if init_waypoints != None else []

    def __repr__(self):
        return "Spline({0})".format(repr(self.waypoints))

    def __repr_segments__(self, to_motor_rad=lambda pos: pos):
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
        self.add_waypoint(new_waypoint)

    def add_waypoint(self, new_waypoint):
        """Add a waypoint to the spline.

        This method will sort through the existing waypoints
        in the spline to insert the waypoint such that
        waypoint time increases with index in the array.

        Parameters
        ----------
        waypoint : ``Waypoint``
            with time, position, [velocity, [and acceleration]] attributes set
        """
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
            optional, used to convert waypoint into motor space

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

    def evaluate_at(self, t, to_motor_rad=lambda pos: pos):
        """Evaluate a point along the curve at a given time.

        Parameters
        ----------
        t : float
            time in seconds
        to_motor_rad : func or lambda
            optional, used to convert waypoint into motor space

        Returns
        -------
        Tuple(float)
            tuple with three elements: evaluated position, velocity, and acceleration.
        """
        if len(self.waypoints) < 2:
            return

        # Return bounds for early or late t
        if t < self.waypoints[0].time:
            return (self.waypoints[0].position, self.waypoints[0].velocity, self.waypoints[0].acceleration)
        if t > self.waypoints[-1].time:
            return (self.waypoints[-1].position, self.waypoints[-1].velocity, self.waypoints[-1].acceleration)

        # Find segment indices
        for i in range(self.get_num_segments()):
            if t >= self.waypoints[i].time and t <= self.waypoints[i + 1].time:
                w0 = self.waypoints[i]
                w1 = self.waypoints[i + 1]
                return hu.evaluate_polynomial_at(Segment.from_two_waypoints(w0, w1, segment_id=None).to_array(only_coeffs=True), t - w0.time)

    def is_valid(self, v_des, a_des):
        """Determines whether spline is well-formed and adheres to dynamic limits.

        Parameters
        ----------
        v_des : float
            Velocity limit that the spline shouldn't exceed
        a_des : float
            Acceleration limit that the spline shouldn't exceed

        Returns
        -------
        Tuple(bool, str)
            whether the segment is valid, and error message if not
        """
        # only check if valid if there are enough waypoints
        if len(self.waypoints) < 2:
            return True, "must have at least two waypoints"

        # verify that spline starts at time zero
        if not np.isclose(self.waypoints[0].time, 0.0, atol=WAYPOINT_ISCLOSE_ATOL):
            return False, "first waypoint must be planned for time zero"

        # verify that waypoint time increases with index in the array
        t = -1.0
        for waypoint in self.waypoints:
            if waypoint.time < 0.0:
                return False, "waypoint cannot be planned for negative time"
            if np.isclose(waypoint.time, t, atol=WAYPOINT_ISCLOSE_ATOL):
                return False, "two waypoints cannot be planned for the same time"
            if waypoint.time < t:
                return False, "time must increase for each subsequent waypoint"
            t = waypoint.time
        # verify spline adheres to joint dynamics limits
        for i in range(self.get_num_segments()):
            success, v_max, a_max=hu.is_segment_feasible(self.get_segment(i).to_array(), v_des, a_des)
            if not success:
                return False, "segment %d exceeds dynamic bounds of (%f vel | %f acc ) with max of (%f vel | %f acc )"%(i,v_des,a_des,v_max,a_max)

        return True, ""


class RevoluteTrajectory(Spline):

    def __repr__(self):
        return "RevoluteTrajectory({0})".format(repr(self.waypoints))

    def add(self, t_s, x_r, v_r=None, a_r=None):
        """Add a waypoint to the trajectory.

        This method will sort through the existing waypoints
        in the trajectory to insert the waypoint such that
        waypoint time increases with index in the array.

        Parameters
        ----------
        t_s : float
            time in seconds
        x_r : float
            position in radians
        v_r : float
            velocity in radians per second
        a_r : float
            acceleration in radians per second squared
        """
        Spline.add(self, t_s, x_r, v_r, a_r)


class PrismaticTrajectory(Spline):

    def __repr__(self):
        return "PrismaticTrajectory({0})".format(repr(self.waypoints))

    def add(self, t_s, x_m, v_m=None, a_m=None):
        """Add a waypoint to the trajectory.

        This method will sort through the existing waypoints
        in the trajectory to insert the waypoint such that
        waypoint time increases with index in the array.

        Parameters
        ----------
        t_s : float
            time in seconds
        x_m : float
            position in meters
        v_m : float
            velocity in meters per second
        a_m : float
            acceleration in meters per second squared
        """
        Spline.add(self, t_s, x_m, v_m, a_m)


class DiffDriveTrajectory(Spline):

    def __init__(self, init_waypoints=None):
        """Differential drive trajectory representing class

        Presents a interface to create differential drive
        trajectories for the mobile base from `SE2Waypoints`
        and generate polynomial segments.

        These waypoints must allow only either translation or
        rotation at a time. This is checked in `is_valid()`.

        Parameters
        ----------
        init_waypoints : List(SE2Waypoint)
            optional, starting list of waypoints e.g. for eval-ing repr

        Attributes
        ----------
        waypoints : List(SE2Waypoint)
            a set of base waypoints defining the mobile base's trajectory
        """
        Spline.__init__(self, init_waypoints=init_waypoints)

    def __repr__(self):
        return "DiffDriveTrajectory({0})".format(repr(self.waypoints))

    def __repr_segments__(self, translate_to_motor_rad, rotate_to_motor_rad):
        if len(self.waypoints) < 2:
            return repr([])
        return repr([self.get_wheel_segments(i, translate_to_motor_rad, rotate_to_motor_rad) \
            for i in range(self.get_num_segments())])

    def add(self, time, x, y, theta, translational_vel=None, rotational_vel=None, translational_accel=None, rotational_accel=None):
        """Add a waypoint to the diff drive trajectory.

        This method will sort through the existing waypoints
        in the trajectory to insert the waypoint such that
        waypoint time increases with index in the array.

        Parameters
        ----------
        time : float
            time in seconds
        x : float
            x component of SE2 pose in meters
        y : float
            y component of SE2 pose in meters
        theta : float
            theta component of SE2 pose in radians
        translational_vel : float
            translational velocity component of twist in meters per second
        rotational_vel : float
            rotational velocity component of twist in radians per second
        translational_accel : float
            translational acceleration component of twist in meters per second squared
        rotational_accel : float
            rotational acceleration component of twist in radians per second squared
        """
        new_waypoint = SE2Waypoint(time=time, pose=(x, y, theta),
            vel_twist=(translational_vel, rotational_vel) if translational_vel is not None and rotational_vel is not None else None,
            accel_twist=(translational_accel, rotational_accel) if translational_accel is not None and rotational_accel is not None else None)
        self.add_waypoint(new_waypoint)

    def get_segment(self, index, to_motor_rad=lambda pos: pos):
        raise NotImplementedError('Use get_wheel_segments instead for DiffDriveTrajectory.')

    def _to_wheel_space(self, x, theta, translate_to_motor_rad, rotate_to_motor_rad):
        """Convert translation/rotation units into left and right motor units.

        Parameters
        ----------
        x : float
            Translational motion in meters
        theta : float
            Rotational motion in radians

        Returns
        -------
        float, float
            left and right wheel motion (respectively) in motor units
        """
        left = right = translate_to_motor_rad(x)
        rot = rotate_to_motor_rad(theta)
        left -= rot
        right += rot
        return left, right

    def _compute_wheel_waypoints(self, translate_to_motor_rad, rotate_to_motor_rad,
                                 lwpos=0.0, rwpos=0.0):
        left_wheel_waypoints = []
        right_wheel_waypoints = []

        # zero-th left and right waypoints
        lvel = rvel = None
        laccel = raccel = None
        if self.waypoints[0].vel_twist is not None:
            lvel, rvel = self._to_wheel_space(*self.waypoints[0].vel_twist, \
                translate_to_motor_rad=translate_to_motor_rad, rotate_to_motor_rad=rotate_to_motor_rad)
        if self.waypoints[0].accel_twist is not None:
            laccel, raccel = self._to_wheel_space(*self.waypoints[0].accel_twist, \
                translate_to_motor_rad=translate_to_motor_rad, rotate_to_motor_rad=rotate_to_motor_rad)
        lw0 = Waypoint(time=self.waypoints[0].time, position=lwpos, velocity=lvel, acceleration=laccel)
        rw0 = Waypoint(time=self.waypoints[0].time, position=rwpos, velocity=rvel, acceleration=raccel)
        left_wheel_waypoints.append(lw0)
        right_wheel_waypoints.append(rw0)

        # remaining left and right waypoints
        for se2_w0, se2_w1 in zip(self.waypoints, self.waypoints[1:]):
            lvel = rvel = None
            laccel = raccel = None
            if se2_w1.vel_twist is not None:
                lvel, rvel = self._to_wheel_space(*se2_w1.vel_twist, \
                    translate_to_motor_rad=translate_to_motor_rad, rotate_to_motor_rad=rotate_to_motor_rad)
            if se2_w1.accel_twist is not None:
                laccel, raccel = self._to_wheel_space(*se2_w1.accel_twist, \
                    translate_to_motor_rad=translate_to_motor_rad, rotate_to_motor_rad=rotate_to_motor_rad)
            lprev, rprev = left_wheel_waypoints[-1].position, right_wheel_waypoints[-1].position
            ldelta, rdelta = self._to_wheel_space(*hu.get_pose_diff(se2_w0.pose, se2_w1.pose), \
                translate_to_motor_rad=translate_to_motor_rad, rotate_to_motor_rad=rotate_to_motor_rad)
            lw1 = Waypoint(time=se2_w1.time, position=lprev + ldelta, velocity=lvel, acceleration=laccel)
            rw1 = Waypoint(time=se2_w1.time, position=rprev + rdelta, velocity=rvel, acceleration=raccel)
            left_wheel_waypoints.append(lw1)
            right_wheel_waypoints.append(rw1)

        return left_wheel_waypoints, right_wheel_waypoints

    def get_wheel_segments(self, index, translate_to_motor_rad, rotate_to_motor_rad,
                           lwpos=0.0, rwpos=0.0):
        """Retrieves left/right wheel segments in the trajectory by index

        Num of segments is one less than number of waypoints in
        the trajectory. Index bounds are [-1 * num_seg, num_seg).

        Parameters
        ----------
        index : int
            index of segment to return
        translate_to_motor_rad : func or lambda
            used to convert se2 waypoints into left motor space
        rotate_to_motor_rad : func or lambda
            used to convert se2 waypoints into right motor space
        lwpos : float
            used to account for left wheel start of trajectory position in motor space
        rwpos : float
            used to account for right wheel start of trajectory position in motor space

        Returns
        -------
        Tuple(Segment, Segment)
            left and right wheel coefficients + duration encapsulated within ``Segment`` classes
        """
        if index < -1 * len(self.waypoints) + 1 or index >= len(self.waypoints) - 1:
            return None

        index = index - 1 if index < 0 else index
        left_wheel_waypoints, right_wheel_waypoints = self._compute_wheel_waypoints(
            translate_to_motor_rad, rotate_to_motor_rad, lwpos, rwpos)
        lw0 = left_wheel_waypoints[index]
        lw1 = left_wheel_waypoints[index + 1]
        rw0 = right_wheel_waypoints[index]
        rw1 = right_wheel_waypoints[index + 1]
        return (Segment.from_two_waypoints(lw0, lw1, segment_id=index + 2),
                Segment.from_two_waypoints(rw0, rw1, segment_id=index + 2))

    def evaluate_at(self, t, to_motor_rad=lambda pos: pos):
        raise NotImplementedError('This method not implemented for DiffDriveTrajectory.')

    def is_valid(self, v_des, a_des, translate_to_motor_rad, rotate_to_motor_rad):
        """Determines whether trajectory is well-formed and adheres to dynamic limits.

        Parameters
        ----------
        v_des : float
            Velocity limit in motor space that the trajectory shouldn't exceed
        a_des : float
            Acceleration limit in motor space that the trajectory shouldn't exceed
        translate_to_motor_rad : func or lambda
            used to convert se2 waypoints into left motor space
        rotate_to_motor_rad : func or lambda
            used to convert se2 waypoints into right motor space

        Returns
        -------
        Tuple(bool, str)
            whether the segment is valid, and error message if not
        """
        # only check if valid if there are enough waypoints
        if len(self.waypoints) < 2:
            return True, "must have at least two waypoints"

        # verify that spline starts at time zero
        if not np.isclose(self.waypoints[0].time, 0.0, atol=WAYPOINT_ISCLOSE_ATOL):
            return False, "first waypoint must be planned for time zero"

        # verify that spline starts at pose zero
        if not np.isclose(self.waypoints[0].pose[0], 0.0, atol=WAYPOINT_ISCLOSE_ATOL):
            return False, "first base waypoint must have x component of pose be zero"
        if not np.isclose(self.waypoints[0].pose[1], 0.0, atol=WAYPOINT_ISCLOSE_ATOL):
            return False, "first base waypoint must have y component of pose be zero"
        if not np.isclose(self.waypoints[0].pose[2], 0.0, atol=WAYPOINT_ISCLOSE_ATOL):
            return False, "first base waypoint must have theta component of pose be zero"

        # verify that waypoint time increases with index in the array
        t = -1.0
        for waypoint in self.waypoints:
            if waypoint.time < 0.0:
                return False, "waypoint cannot be planned for negative time"
            if np.isclose(waypoint.time, t, atol=WAYPOINT_ISCLOSE_ATOL):
                return False, "two waypoints cannot be planned for the same time"
            if waypoint.time < t:
                return False, "time must increase for each subsequent waypoint"
            t = waypoint.time

        # verify left and right trajectories adheres to joint dynamics limits
        for i in range(self.get_num_segments()):
            ls, rs = self.get_wheel_segments(i, translate_to_motor_rad, rotate_to_motor_rad)
            success, v_max, a_max =hu.is_segment_feasible(ls.to_array(), v_des, a_des)
            if not success:
                return False, "left wheel segment %d exceeds dynamic bounds of (%f vel | %f acc ) with max of (%f vel | %f acc )"%(i,v_des,a_des,v_max,a_max)
            success, v_max, a_max =hu.is_segment_feasible(rs.to_array(), v_des, a_des)
            if not success:
                return False, "right wheel segment %d exceeds dynamic bounds of (%f vel | %f acc ) with max of (%f vel | %f acc )"%(i,v_des,a_des,v_max,a_max)

        return True, ""
