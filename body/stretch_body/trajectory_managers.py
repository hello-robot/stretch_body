from __future__ import print_function
import threading
from math import atan2, hypot

from stretch_body.hello_utils import *


class Waypoint:

    def __init__(self, time=None, position=None, velocity=None, acceleration=None, effort_threshold=None):
        self.time = time
        self.position = position
        self.velocity = velocity
        self.acceleration = acceleration
        self.effort_threshold = effort_threshold

    def __repr__(self):
        return "Waypoint(t={0}, pos={1}, vel={2}, accel={3}, effort_threshold={4})".format(
            self.time, self.position, self.velocity, self.acceleration, self.effort_threshold)
    def __eq__(self,other):
        return self.time==other.time and self.position==other.position and self.acceleration==other.acceleration and self.effort_threshold==other.effort_threshold

class Segment:
    def __init__(self, duration=0, a0=0, a1=0, a2=0, a3=0, a4=0, a5=0):
        self.duration = duration
        self.a0 = a0
        self.a1 = a1
        self.a2 = a2
        self.a3 = a3
        self.a4 = a4
        self.a5 = a5

    def __repr__(self):
        return "Segment(duration={0}, a0={1}, a1={2}, a2={3}, a3={4}, a4={5}, a5={6})".format(
            self.duration, self.a0, self.a1, self.a2, self.a3, self.a4, self.a5)

    def __eq__(self, other):
        return self.duration == other.duration  and self.a0 == other.a0 and \
               self.a1 == other.a1 and self.a2 == other.a2 and self.a3 == other.a3 and \
               self.a4 == other.a4  and self.a5 == other.a5
    def __ne__(self, other):
        return not self.__eq__(other)

class Trajectory:

    def __init__(self):
        """Basic trajectory representing class.

        Defines a unitless trajectory composed of ``Waypoint``s.
        Revolute and prismatic joints should extend this class
        with appropriate units. Enforcing cubic or quintic curve
        requirements should be done by extending this class.

        Attributes
        ----------
        waypoints : list(Waypoint)
            a set of waypoints defining the trajectory
        """
        self.waypoints = []

    def __len__(self):
        return len(self.waypoints)

    def __repr__(self):
        return str(self.waypoints)

    def clear_waypoints(self):
        self.waypoints = []

    def get_waypoint(self, index):
        if index >= -1 * len(self.waypoints) and index < len(self.waypoints):
            return self.waypoints[index]

    def get_num_segments(self):
        return max(0,len(self.waypoints)-1)

    def get_segment(self, index, to_motor_rad=None):
        """Retrieves a segment in the trajectory by index.

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
        Segment
            coefficients + duration encapsulated in ``Segment`` class
        """
        if index >= -1 * len(self.waypoints) + 1 and index < len(self.waypoints) - 1:
            index = index - 1 if index < 0 else index
            waypoint0 = self.get_waypoint(index)
            waypoint1 = self.get_waypoint(index + 1)
            if waypoint0.acceleration is not None and waypoint1.acceleration is not None:
                if to_motor_rad is not None:
                    i0_waypoint = [waypoint0.time, to_motor_rad(waypoint0.position), to_motor_rad(waypoint0.velocity), to_motor_rad(waypoint0.acceleration)]
                    i1_waypoint = [waypoint1.time, to_motor_rad(waypoint1.position), to_motor_rad(waypoint1.velocity), to_motor_rad(waypoint1.acceleration)]
                else:
                    i0_waypoint = [waypoint0.time, waypoint0.position, waypoint0.velocity, waypoint0.acceleration]
                    i1_waypoint = [waypoint1.time, waypoint1.position, waypoint1.velocity, waypoint1.acceleration]
                segment_arr = generate_quintic_spline_segment(i0_waypoint, i1_waypoint)
            elif waypoint0.velocity is not None and waypoint1.velocity is not None:
                if to_motor_rad is not None:
                    i0_waypoint = [waypoint0.time, to_motor_rad(waypoint0.position), to_motor_rad(waypoint0.velocity)]
                    i1_waypoint = [waypoint1.time, to_motor_rad(waypoint1.position), to_motor_rad(waypoint1.velocity)]
                else:
                    i0_waypoint = [waypoint0.time, waypoint0.position, waypoint0.velocity]
                    i1_waypoint = [waypoint1.time, waypoint1.position, waypoint1.velocity]
                segment_arr = generate_cubic_spline_segment(i0_waypoint, i1_waypoint)
            else:
                if to_motor_rad is not None:
                    i0_waypoint = [waypoint0.time, to_motor_rad(waypoint0.position)]
                    i1_waypoint = [waypoint1.time, to_motor_rad(waypoint1.position)]
                else:
                    i0_waypoint = [waypoint0.time, waypoint0.position]
                    i1_waypoint = [waypoint1.time, waypoint1.position]
                segment_arr = generate_linear_segment(i0_waypoint, i1_waypoint)
            return Segment(*segment_arr)

        return Segment()

    def delete_waypoint(self, index):
        if index >= -1 * len(self.waypoints) and index < len(self.waypoints):
            return self.waypoints.pop(index)

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
        if t_s < self.get_waypoint(0).time:
            return self.get_waypoint(0)
        if t_s > self.get_waypoint(-1).time:
            return self.get_waypoint(-1)

        # Find segment indices
        for i in range(len(self.waypoints) - 1):
            if t_s >= self.get_waypoint(i).time and t_s <= self.get_waypoint(i + 1).time:
                i0, i1 = i, i+1
                i0_t = self.get_waypoint(i0).time

        # Generate segment and evaluate at t_s
        waypoint0 = self.get_waypoint(i0)
        waypoint1 = self.get_waypoint(i1)
        if waypoint0.acceleration is not None and waypoint1.acceleration is not None:
            i0_waypoint = [waypoint0.time, waypoint0.position, waypoint0.velocity, waypoint0.acceleration]
            i1_waypoint = [waypoint1.time, waypoint1.position, waypoint1.velocity, waypoint1.acceleration]
            seg = generate_quintic_spline_segment(i0_waypoint, i1_waypoint)
            ret = evaluate_polynomial_at(seg, t_s - i0_t)
        elif waypoint0.velocity is not None and waypoint1.velocity is not None:
            i0_waypoint = [waypoint0.time, waypoint0.position, waypoint0.velocity]
            i1_waypoint = [waypoint1.time, waypoint1.position, waypoint1.velocity]
            seg = generate_cubic_spline_segment(i0_waypoint, i1_waypoint)
            ret = evaluate_polynomial_at(seg, t_s - i0_t)
        else:
            i0_waypoint = [waypoint0.time, waypoint0.position]
            i1_waypoint = [waypoint1.time, waypoint1.position]
            seg = generate_linear_segment(i0_waypoint, i1_waypoint)
            ret = evaluate_polynomial_at(seg, t_s - i0_t)

        return Waypoint(position=ret[0], velocity=ret[1], acceleration=ret[2])


class PrismaticTrajectory(Trajectory):

    def __init__(self):
        Trajectory.__init__(self)

    def add_waypoint(self, t_s, x_m, v_m=None, a_m=None):
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
        if t_s is None or x_m is None:
            return
        if v_m is None and a_m is not None:
            return
        for i in range(len(self.waypoints)):
            if t_s == self.get_waypoint(i).time:
                return

        if len(self.waypoints) == 0:
            self.waypoints.append(Waypoint(time=t_s, position=x_m, velocity=v_m, acceleration=a_m))
            return

        # Prepend or append for early or late t_s
        if t_s < self.get_waypoint(0).time:
            self.waypoints.insert(0, Waypoint(time=t_s, position=x_m, velocity=v_m, acceleration=a_m))
            return
        if t_s > self.get_waypoint(-1).time:
            self.waypoints.append(Waypoint(time=t_s, position=x_m, velocity=v_m, acceleration=a_m))
            return

        # Insert before first later waypoint
        for i in range(len(self.waypoints)):
            if t_s <= self.get_waypoint(i).time:
                self.waypoints.insert(i, Waypoint(time=t_s, position=x_m, velocity=v_m, acceleration=a_m))
                return


class RevoluteTrajectory(Trajectory):

    def __init__(self):
        Trajectory.__init__(self)

    def add_waypoint(self, t_s, x_r, v_r=None, a_r=None):
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
        if t_s is None or x_r is None:
            return
        if v_r is None and a_r is not None:
            return
        for i in range(len(self.waypoints)):
            if t_s == self.get_waypoint(i).time:
                return

        if len(self.waypoints) == 0:
            self.waypoints.append(Waypoint(time=t_s, position=x_r, velocity=v_r, acceleration=a_r))
            return

        # Prepend or append for early or late t_s
        if t_s < self.get_waypoint(0).time:
            self.waypoints.insert(0, Waypoint(time=t_s, position=x_r, velocity=v_r, acceleration=a_r))
            return
        if t_s > self.get_waypoint(-1).time:
            self.waypoints.append(Waypoint(time=t_s, position=x_r, velocity=v_r, acceleration=a_r))
            return

        # Insert before first later waypoint
        for i in range(len(self.waypoints)):
            if t_s <= self.get_waypoint(i).time:
                self.waypoints.insert(i, Waypoint(time=t_s, position=x_r, velocity=v_r, acceleration=a_r))
                return


def get_base_motion(position_tuple0, position_tuple1):
    """Return the motion required to get from position 0 to position 1.

    Parameters
    ----------
    position_tuple0 : float, float, float
        x, y, theta in meters and radians
    position_tuple1 : float, float, float
        same


    Returns
    -------
    float, float
        translation and rotation required for motion

    """
    x0, y0, theta0 = position_tuple0
    x1, y1, theta1 = position_tuple1

    # TODO: For now, we use a simplified motion model where we assume
    # that every motion is either a translation OR a rotation,
    # and the translation is either straight forward or straight back
    if x0 == x1 and y0 == y1:
        return 0.0, theta1 - theta0
    dx = x1 - x0
    dy = y1 - y0
    drive_angle = atan2(dy, dx)
    distance = hypot(dy, dx)
    if (drive_angle - theta0) < 1e-2:
        return distance, 0.0
    else:
        return -distance, 0.0

class BasicDiffDriveTrajectory(Trajectory):

    def __init__(self, translate_to_motor_rad, rotate_to_motor_rad):
        """Differential drive trajectory representing class.

        Defines a trajectory that allows distinct translation and rotation waypoints but not both simultaneously.

        Each waypoint position is an (x, y, theta) tuple. Assume that the first one is the robot's current position
        Each waypoint velocity/acceleration is a (translation, rotation) tuple.

        Parameters
        ----------
        translate_to_motor_rad : func
            used to convert translation waypoints into motor space
        rotate_to_motor_rad : func
            used to convert rotation waypoints into motor space

        Attributes
        ----------
        trajectory_type : none
            This attribute has been deprecated
        wheel_space_waypoints : list(Waypoint)
            Each position/velocity/acceleration is a tuple for the left and right wheel

        """
        Trajectory.__init__(self)
        self.trajectory_type = None
        self.translate_to_motor_rad = translate_to_motor_rad
        self.rotate_to_motor_rad = rotate_to_motor_rad
        self.wheel_space_waypoints = []

    def to_wheel_units(self, x, theta):
        """Convert translation/rotation units into left and right motor units.

        Parameters
        ----------
        x : float
            Translational motion in m
        theta : float
            Rotational motion in radians

        Returns
        -------
        float, float
            left and right wheel motion (respectively) in motor units

        """
        left = right = self.translate_to_motor_rad(x)
        rot = self.rotate_to_motor_rad(theta)
        left -= rot
        right += rot
        return left, right

    def complete_trajectory(self):
        self.wheel_space_waypoints = []
        for i, waypoint in enumerate(self.waypoints):
            velocity = self.to_wheel_units(*waypoint.velocity) if waypoint.velocity else None
            acceleration = self.to_wheel_units(*waypoint.acceleration) if waypoint.acceleration else None

            if i == 0:
                self.wheel_space_waypoints.append(Waypoint(time=waypoint.time,
                                                           position=(0.0, 0.0),
                                                           velocity=velocity,
                                                           acceleration=acceleration))
            else:
                prev_waypoint = self.waypoints[i - 1]
                prev_wheel_waypoint = self.wheel_space_waypoints[i - 1]
                delta_x, delta_theta = get_base_motion(prev_waypoint.position, waypoint.position)
                delta_left, delta_right = self.to_wheel_units(delta_x, delta_theta)
                prev_left, prev_right = prev_wheel_waypoint.position
                self.wheel_space_waypoints.append(Waypoint(time=waypoint.time,
                                                           position=(prev_left + delta_left, prev_right + delta_right),
                                                           velocity=velocity,
                                                           acceleration=acceleration))

    def get_wheel_segments(self, index, rwpos=0.0, lwpos=0.0):
        """Retrieve left and right wheel segments in the trajectory by index.

        Num of segments is one less than number of waypoints in
        the trajectory. Index bounds are [-1 * num_seg, num_seg).

        Parameters
        ----------
        index : int
            index of segment to return
        rwpos : float
            starting position of right wheel in motor space
        lwpos : float
            starting position of left wheel in motor space

        Returns
        -------
        Segment, Segment
            left and right coefficients + durations encapsulated in ``Segment`` class

        """
        if index < -1 * len(self.waypoints) + 1 or index >= len(self.waypoints) - 1:
            # Invalid index
            return (Segment(), Segment())

        index = index - 1 if index < 0 else index
        waypoint0 = self.wheel_space_waypoints[index]
        waypoint1 = self.wheel_space_waypoints[index + 1]

        i0_lwaypoint = [waypoint0.time, waypoint0.position[0] + lwpos]
        i1_lwaypoint = [waypoint1.time, waypoint1.position[0] + lwpos]
        i0_rwaypoint = [waypoint0.time, waypoint0.position[1] + rwpos]
        i1_rwaypoint = [waypoint1.time, waypoint1.position[1] + rwpos]

        if waypoint0.velocity is not None and waypoint1.velocity is not None:
            i0_lwaypoint.append(waypoint0.velocity[0])
            i1_lwaypoint.append(waypoint1.velocity[0])
            i0_rwaypoint.append(waypoint0.velocity[1])
            i1_rwaypoint.append(waypoint1.velocity[1])

            if waypoint0.acceleration is not None and waypoint1.acceleration is not None:
                i0_lwaypoint.append(waypoint0.acceleration[0])
                i1_lwaypoint.append(waypoint1.acceleration[0])
                i0_rwaypoint.append(waypoint0.acceleration[1])
                i1_rwaypoint.append(waypoint1.acceleration[1])

                # Have position, velocity and acceleration --> Generate Quintic
                lsegment_arr = generate_quintic_spline_segment(i0_lwaypoint, i1_lwaypoint)
                rsegment_arr = generate_quintic_spline_segment(i0_rwaypoint, i1_rwaypoint)
            else:
                # Have position and velocity, but no acceleration --> Generate Cubic
                lsegment_arr = generate_cubic_spline_segment(i0_lwaypoint, i1_lwaypoint)
                rsegment_arr = generate_cubic_spline_segment(i0_rwaypoint, i1_rwaypoint)
        else:
            # Have just position (no velocity or acceleration) --> Generate Linear
            lsegment_arr = generate_linear_segment(i0_lwaypoint, i1_lwaypoint)
            rsegment_arr = generate_linear_segment(i0_rwaypoint, i1_rwaypoint)

        return (Segment(*lsegment_arr), Segment(*rsegment_arr))

    def add_waypoint(self, t_s, x_pos, v_pair=None, a_pair=None):
        """Add a waypoint to the base trajectory.

        This method will sort through the existing waypoints
        to insert the waypoint such that waypoint time increases
        with index in the array.

        Parameters
        ----------
        t_s : float
            time in seconds
        x_pos : float, float, float
            x, y, theta position in meters/radians
        v_pair : float, float or None
            velocity in meters per second and radians per second
        a_pair : float, float or None
            acceleration in meters per second squared and radians per second squared

        """
        if t_s is None or x_pos is None:
            return
        if v_pair is None and a_pair is not None:
            return

        for i in range(len(self.waypoints)):
            if t_s == self.get_waypoint(i).time:
                return

        waypoint = Waypoint(time=t_s, position=x_pos, velocity=v_pair, acceleration=a_pair)

        if len(self.waypoints) == 0:
            self.waypoints.append(waypoint)
            return

        # Prepend or append for early or late t_s
        if t_s < self.get_waypoint(0).time:
            self.waypoints.insert(0, waypoint)
            return
        if t_s > self.get_waypoint(-1).time:
            self.waypoints.append(waypoint)
            return

        # Insert before first later waypoint
        for i in range(len(self.waypoints)):
            if t_s <= self.get_waypoint(i).time:
                self.waypoints.insert(i, waypoint)
                return

    def clear_waypoints(self):
        self.waypoints = []
        self.wheel_space_waypoints = []

class TrajectoryManager:

    def __init__(self):
        """Basic trajectory manager class that should be extended.

        Provides threading and utility functions for managing
        the execution of trajectories. This class is extended
        to support specific joint types.

        Attributes
        ----------
        traj_thread_freq : int
            the frequency in hz that the trajectory is pushed to hardware
        traj_thread_shutdown_flag : threading.Event
            signals shutdown to the trajectory pushing thread
        traj_thread : threading.Thread
            the trajectory pushing thread
        traj_threaded : bool
            if True, trajectory manager takes charge of pushing trajectory
        traj_start_time : float
            wall time at which trajectory execution began
        traj_curr_time : float
            wall time at which trajectory is being evaluated
        """
        self.traj_thread_freq = 100
        self.traj_thread_shutdown_flag = threading.Event()
        self.traj_thread = None
        self.traj_threaded = False
        self.traj_start_time = None
        self.traj_curr_time = None

    def _start_trajectory_thread(self):
        if self.traj_threaded:
            if self.traj_thread is not None:
                self.traj_thread_shutdown_flag.set()
                self.traj_thread.join()
            self.traj_thread = threading.Thread(target=self._push_trajectory_thread)
            self.traj_thread_shutdown_flag.clear()
            self.traj_thread.start()

    def _stop_trajectory_thread(self):
        if self.traj_threaded:
            self.traj_thread_shutdown_flag.set()
            self.traj_thread.join()

    def _push_trajectory_thread(self):
        while not self.traj_thread_shutdown_flag.is_set():
            ts = time.time()
            if not self.push_trajectory():
                return
            te = time.time()
            tsleep = max(0, (1 / self.traj_thread_freq) - (te - ts))
            if not self.traj_thread_shutdown_flag.is_set():
                time.sleep(tsleep)

    def duration_remaining(self):
        if len(self.trajectory) < 2:
            return
        traj_duration = self.trajectory.get_waypoint(-1).time - self.trajectory.get_waypoint(0).time
        if self.traj_start_time is None:
            return traj_duration
        if self.traj_curr_time is None:
            self.traj_curr_time = time.time()
        traj_elapased = self.traj_curr_time - self.traj_start_time
        return max(0.0, traj_duration - traj_elapased)


class DynamixelTrajectoryManager(TrajectoryManager):

    def __init__(self):
        """Trajectory tracking class for ``DynamixelHelloXL430`` joints.

        Manages execution of ``DynamixelTrajectory``
        trajectories. This class **must** be extended by a
        dynamixel joint class because it utilizes the status
        and motor of the extending class.

        Attributes
        ----------
        trajectory : DynamixelTrajectory
            the trajectory that is tracked
        traj_pos_mode : bool
            whether to use position or velocity control to track the traj
        traj_curr_goal : Waypoint
            interpolated point along the trajectory currently being tracked
        """
        TrajectoryManager.__init__(self)
        self.trajectory = RevoluteTrajectory()
        self.traj_pos_mode = True
        self.traj_curr_goal = None

    def start_trajectory(self, position_ctrl=True, threaded=True, watchdog_timeout=None):
        """Starts execution of the trajectory.

        Parameters
        ----------
        position_ctrl : bool
            True uses position control to follow traj, False uses velocity control
        threaded : bool
            True launches a separate thread for ``push_trajectory``, False puts burden on the user
        watchdog_timeout : int
            See ``DynamixelXL430.enable_watchdog()``
        """
        self.traj_pos_mode = position_ctrl
        self.traj_threaded = threaded
        if len(self.trajectory) < 2:
            return
        if not self.hw_valid:
            return
        if self.params['req_calibration'] and not self.is_calibrated:
            print('Dynamixel not homed: {0}'.format(self.name))
            return
        if self.traj_pos_mode:
            self.motor.set_profile_velocity(self.rad_per_sec_to_ticks(self.params['motion']['trajectory_max']['vel']))
            self.motor.set_profile_acceleration(self.rad_per_sec_sec_to_ticks(self.params['motion']['trajectory_max']['accel']))
        else:
            self.disable_torque()
            if watchdog_timeout is not None:
                self.motor.enable_watchdog(watchdog_timeout)
            else:
                self.motor.enable_watchdog()
            self.motor.enable_vel()
            self.motor.set_profile_acceleration(self.rad_per_sec_sec_to_ticks(self.params['motion']['trajectory_max']['accel']))
            self.motor.set_vel_limit(self.rad_per_sec_to_ticks(self.params['motion']['trajectory_max']['vel']))
            self.enable_torque()
        self.traj_start_time = time.time()
        self.traj_curr_time = self.traj_start_time
        self.traj_curr_goal = self.trajectory.evaluate_at(t_s=0.0)
        self.status['trajectory_active'] = True
        TrajectoryManager._start_trajectory_thread(self)

    def push_trajectory(self):
        """Commands goals to the dynamixel hardware.

        If not using threaded mode in ``start_trajectory``, the user is
        responsible for calling this method a regular frequency.
        As low as 60hz is acceptable for velocity control.
        As low as 100hz is acceptable for position control.

        Returns
        -------
        bool
            True if trajectory must continue to be pushed, False otherwise
        """
        if self.traj_start_time is None:
            return False

        self.traj_curr_time = time.time()
        self.traj_curr_goal = self.trajectory.evaluate_at(t_s=self.traj_curr_time - self.traj_start_time)
        if self.duration_remaining() > 0.0:
            if self.traj_pos_mode:
                self.move_to(self.traj_curr_goal.position)
            else:
                v_des = self.world_rad_to_ticks_per_sec(self.traj_curr_goal.velocity)
                #Honor joint limits in velocity mode
                lim_lower=min(self.ticks_to_world_rad(self.params['range_t'][0]),self.ticks_to_world_rad(self.params['range_t'][1]))
                lim_upper=max(self.ticks_to_world_rad(self.params['range_t'][0]),self.ticks_to_world_rad(self.params['range_t'][1]))
                x_curr=self.status['pos']
                v_curr=self.status['vel']
                if self.params['motion']['trajectory_max']['accel']>0:
                    t_brake = abs(v_curr)/self.params['motion']['trajectory_max']['accel'] #How long to brake from current speed (s)
                else:
                    t_brake=0
                d_brake = t_brake* v_curr / 2 #How far it will go before breaking (pos/neg)
                if (self.traj_curr_goal.velocity>0 and x_curr+d_brake>=lim_upper) or (self.traj_curr_goal.velocity<0 and x_curr+d_brake<=lim_lower):
                    v_des=0
                self.motor.go_to_vel(v_des)
        if self.status['trajectory_active'] and self.duration_remaining() == 0.0:
            if not self.traj_pos_mode:
                self.disable_torque()
                self.motor.disable_watchdog()
                self.enable_pos()
                self.enable_torque()
            self.move_to(self.traj_curr_goal.position)
            self.status['trajectory_active'] = False
            return False

        return True

    def stop_trajectory(self):
        """Stops a currently executing trajectory.

        Restores the dynamixel hardware to position mode,
        if necessary. Additionally, stops threaded execution,
        if necessary.
        """
        self.traj_start_time = None
        if not self.traj_pos_mode:
            self.disable_torque()
            self.motor.disable_watchdog()
            self.enable_pos()
            self.enable_torque()
        self.status['trajectory_active'] = False
        TrajectoryManager._stop_trajectory_thread(self)


class StepperTrajectoryManager(TrajectoryManager):

    def __init__(self):
        """Trajectory tracking class for ``Stepper`` joints.

        Manages execution of ``StepperTrajectory`` trajectories.
        This class **must** be extended by a stepper joint class
        because it utilizes the status and motor of the extending class.

        Attributes
        ----------
        trajectory : StepperTrajectory
            the trajectory that is tracked
        """
        TrajectoryManager.__init__(self)
        self.trajectory = PrismaticTrajectory()
        self.segment_last=None
        self.segment_id_last=None

    def start_trajectory(self, threaded=True, req_calibration=True):
        """Starts execution of the trajectory.

        Parameters
        ----------
        threaded : bool
            True launches a separate thread for ``push_trajectory``, False puts burden on the user
        req_calibration : bool
            If True and joint not calibrated, trajectory does not start
        """
        self.traj_threaded = threaded
        if len(self.trajectory) < 2:
            return
        if req_calibration and not self.motor.status['pos_calibrated']:
            print('Joint not homed')
            return

        v_r = self.translate_to_motor_rad(self.params['motion']['trajectory_max']['vel_m'])
        a_r = self.translate_to_motor_rad(self.params['motion']['trajectory_max']['accel_m'])
        self.motor.enable_pos_traj_waypoint()
        self.motor.set_command(v_des=v_r,
                               a_des=a_r,
                               i_feedforward=self.i_feedforward,
                               i_contact_pos=self.i_contact_pos,
                               i_contact_neg=self.i_contact_neg)
        self.motor.push_command()

        s = self.trajectory.get_segment(0, to_motor_rad=self.translate_to_motor_rad)
        self.motor.start_waypoint_trajectory([s.duration, s.a0, s.a1, s.a2, s.a3, s.a4, s.a5, 2])
        if self.motor.traj_curr_seg_id == 0: #Failed to load
            print('Stepper failed to start trajectory. One already executing')
            self.traj_start_time =None
        else:
            self.traj_start_time = time.time()
            self.traj_curr_time = self.traj_start_time
            TrajectoryManager._start_trajectory_thread(self)


    def push_trajectory(self):
        """Commands goals to Hello Robot stepper hardware.

        If not using threaded mode in ``start_trajectory``, the user is
        responsible for calling this method a regular frequency.
        As low as 25hz is acceptable.

        ``motor.traj_curr_seg_id`` is the ID of the currently active trajectory segment as reported
        by the uC. Ids go from 2 to 255 and loop around. Id of 0 marks that the uC is in Idle state.
        Id of 1 marks that a trajectory is loaded but not executing. Id of 2 marks that the first
        segment has begun executing. The segment that is pushed to the uC is the subsequent segment
        after the active segment. All future segments may be modified until the hardware latches it
        into the active segment. A segment is represented by [duration, coefficients a0-a5, seg_id].

        Returns
        -------
        bool
            True if trajectory must continue to be pushed, False otherwise
        """

        if self.traj_threaded:
            self.pull_status()

        if self.traj_start_time is None or not self.status['motor']['trajectory_active']:
            return False
        self.traj_curr_time = time.time()

        next_segment_id=self.motor.traj_curr_seg_id-1 #Offset due to uC starting at ID 2
        if next_segment_id>0 and self.trajectory.get_num_segments()>next_segment_id:
            s = self.trajectory.get_segment(next_segment_id, to_motor_rad=self.translate_to_motor_rad)
        else:
            s=Segment()
        arr = [s.duration, s.a0, s.a1, s.a2, s.a3, s.a4, s.a5, next_segment_id+2]
        self.motor.push_waypoint_trajectory(arr)
        return True

    def stop_trajectory(self):
        """Stop a currently executing trajectory.
        uC controller remains in trajectory mode.
        Additionally, stops threaded execution if necessary.
        """
        self.motor.reset_waypoint_trajectory()
        self.traj_start_time = None
        TrajectoryManager._stop_trajectory_thread(self)


class MobileBaseTrajectoryManager(TrajectoryManager):

    def __init__(self):
        """Trajectory tracking class for ``Base`` joint.

        Manages execution of ``StepperTrajectory`` trajectories.
        This class **must** be extended by a stepper joint class
        because it utilizes the status and motor of the extending class.

        Attributes
        ----------
        trajectory : BasicDiffDriveTrajectory
            the trajectory that is tracked
        """
        TrajectoryManager.__init__(self)
        self.trajectory = BasicDiffDriveTrajectory(self.translate_to_motor_rad, self.rotate_to_motor_rad)

    def start_trajectory(self, threaded=True):
        """Start execution of the mobile base trajectory.

        Parameters
        ----------
        threaded : bool
            True launches a separate thread for ``push_trajectory``, False puts burden on the user

        """
        self.traj_threaded = threaded
        if len(self.trajectory) < 2:
            return

        max_v = min(self.params['motion']['trajectory_translate_max']['vel_m'],
                    self.params['motion']['trajectory_rotate_max']['vel_r'])
        max_a = min(self.params['motion']['trajectory_translate_max']['accel_m'],
                    self.params['motion']['trajectory_rotate_max']['accel_r'])
        v_r = self.translate_to_motor_rad(max_v)
        a_r = self.translate_to_motor_rad(max_a)

        self.left_wheel.enable_pos_traj_waypoint()
        self.left_wheel.set_command(v_des=v_r, a_des=a_r)
        self.right_wheel.enable_pos_traj_waypoint()
        self.right_wheel.set_command(v_des=v_r, a_des=a_r)
        if self.traj_threaded:
            self.left_wheel.pull_status()
            self.right_wheel.pull_status()
        self.traj_start_rwpos = self.right_wheel.status['pos']
        self.traj_start_lwpos = self.left_wheel.status['pos']
        self.left_wheel.push_command()
        self.right_wheel.push_command()
        ls, rs = self.trajectory.get_wheel_segments(0, rwpos=self.traj_start_rwpos,
                                                       lwpos=self.traj_start_lwpos)
        self.left_wheel.start_waypoint_trajectory([ls.duration, ls.a0, ls.a1, ls.a2, ls.a3, ls.a4, ls.a5, 2])
        self.right_wheel.start_waypoint_trajectory([rs.duration, rs.a0, rs.a1, rs.a2, rs.a3, rs.a4, rs.a5, 2])
        if self.left_wheel.traj_curr_seg_id == 0 or self.right_wheel.traj_curr_seg_id == 0: #Failed to load
            print('Base failed to start trajectory. One already executing.')
            self.traj_start_time =None
        else:
            self.traj_start_time = time.time()
            self.traj_curr_time = self.traj_start_time
            TrajectoryManager._start_trajectory_thread(self)

    def push_trajectory(self):
        """Commands goals to Hello Robot mobile base.

        If not using threaded mode in ``start_trajectory``, the user is
        responsible for calling this method a regular frequency.
        As low as 25hz is acceptable.

        Returns
        -------
        bool
            True if trajectory must continue to be pushed, False otherwise
        """
        if self.traj_threaded:
            self.left_wheel.pull_status()
            self.right_wheel.pull_status()

        if self.traj_start_time is None or not self.status['left_wheel']['trajectory_active'] or not self.status['right_wheel']['trajectory_active']:
            return False

        self.traj_curr_time = time.time()

        next_segment_id_rw = self.right_wheel.traj_curr_seg_id - 1  # Offset due to uC starting at ID 2
        next_segment_id_lw = self.left_wheel.traj_curr_seg_id - 1

        if next_segment_id_rw>0 and self.trajectory.get_num_segments()>next_segment_id_rw and next_segment_id_lw>0 and self.trajectory.get_num_segments()>next_segment_id_lw:
            ls, rs = self.trajectory.get_wheel_segments(max(next_segment_id_lw, next_segment_id_rw),
                                                        rwpos=self.traj_start_rwpos,
                                                        lwpos=self.traj_start_lwpos)
        else:
            ls=Segment()
            rs=Segment()

        larr = [ls.duration, ls.a0, ls.a1, ls.a2, ls.a3, ls.a4, ls.a5, next_segment_id_lw + 2]
        rarr = [rs.duration, rs.a0, rs.a1, rs.a2, rs.a3, rs.a4, rs.a5, next_segment_id_rw + 2]
        self.left_wheel.push_waypoint_trajectory(larr)
        self.right_wheel.push_waypoint_trajectory(rarr)
        return True

    def stop_trajectory(self):
        """Stop a currently executing trajectory.
        uC controller remains in trajectory mode.
        Additionally, stops threaded execution if necessary.
        """
        self.traj_start_time = None
        self.left_wheel.reset_waypoint_trajectory()
        self.right_wheel.reset_waypoint_trajectory()
        TrajectoryManager._stop_trajectory_thread(self)
