#! /usr/bin/env python

import time
import copy
import threading

from stretch_body.hello_utils import *

TRAJECTORY_TYPE_INVALID = 0
TRAJECTORY_TYPE_CUBIC_SPLINE = 1
TRAJECTORY_TYPE_QUINTIC_SPLINE = 2


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


class Trajectory:

    def __init__(self):
        """Base trajectory representing class.

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
        if not self.is_valid(ignore_joint_limits=True):
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
            ret = evaluate_quintic_spline(seg, t_s - i0_t)
        elif waypoint0.velocity is not None and waypoint1.velocity is not None:
            i0_waypoint = [waypoint0.time, waypoint0.position, waypoint0.velocity]
            i1_waypoint = [waypoint1.time, waypoint1.position, waypoint1.velocity]
            seg = generate_cubic_spline_segment(i0_waypoint, i1_waypoint)
            ret = evaluate_cubic_spline(seg, t_s - i0_t)
        else:
            i0_waypoint = [waypoint0.time, waypoint0.position]
            i1_waypoint = [waypoint1.time, waypoint1.position]
            seg = generate_linear_segment(i0_waypoint, i1_waypoint)
            ret = evaluate_linear_interpolate(seg, t_s - i0_t)

        return Waypoint(position=ret[0], velocity=ret[1], acceleration=ret[2])


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
        if not self.trajectory.is_valid(ignore_joint_limits=True):
            return
        traj_duration = self.trajectory.get_waypoint(-1).time - self.trajectory.get_waypoint(0).time
        if self.traj_start_time is None:
            return traj_duration
        if self.traj_curr_time is None:
            self.traj_curr_time = time.time()
        traj_elapased = self.traj_curr_time - self.traj_start_time
        return max(0.0, traj_duration - traj_elapased)


class DynamixelTrajectory(Trajectory):

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

    def is_valid(self, ignore_joint_limits=False):
        """Verifies whether the current trajectory is valid.

        Parameters
        ----------
        ignore_joint_limits : bool
            whether to check if waypoints fall within joint limits

        Returns
        -------
        bool
        """
        if len(self.waypoints) < 2:
            return False

        if not ignore_joint_limits:
            pass # TODO

        return True


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
        self.trajectory = DynamixelTrajectory()
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
        if not self.trajectory.is_valid(ignore_joint_limits=True):
            return
        if not self.servo_valid:
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
            self.motor.set_vel_limit(self.rad_per_sec_to_ticks(self.params['motion']['max']['vel']))
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


class StepperTrajectory(Trajectory):

    def __init__(self, joint_utilities_class):
        Trajectory.__init__(self)
        self.utils = joint_utilities_class

    def __repr__(self):
        waypoints_meter = []
        for w in self.waypoints:
            t_s = w.time
            x_m = self.utils.motor_rad_to_translate(w.position)
            v_m = self.utils.motor_rad_to_translate(w.velocity)
            a_m = self.utils.motor_rad_to_translate(w.acceleration)
            waypoints_meter.append(Waypoint(time=t_s, position=x_m, velocity=v_m, acceleration=a_m))
        return str(waypoints_meter)

    def get_waypoint(self, index, motor_rad=False):
        """Get waypoint by index from a ``StepperTrajectory``.

        Parameters
        ----------
        index : int
            indexes the trajectory in range [-1 * len(trajectory), len(trajectory) - 1]
        motor_rad : bool
            returns values as radians in motor joint space if True
        """
        if index >= -1 * len(self.waypoints) and index < len(self.waypoints):
            waypoint_rad = self.waypoints[index]
            if motor_rad:
                return waypoint_rad
            t_s = waypoint_rad.time
            x_m = self.utils.motor_rad_to_translate(waypoint_rad.position)
            v_m = self.utils.motor_rad_to_translate(waypoint_rad.velocity)
            a_m = self.utils.motor_rad_to_translate(waypoint_rad.acceleration)
            return Waypoint(time=t_s, position=x_m, velocity=v_m, acceleration=a_m)

    def delete_waypoint(self, index):
        if index >= -1 * len(self.waypoints) and index < len(self.waypoints):
            waypoint_rad = self.waypoints.pop(index)
            t_s = waypoint_rad.time
            x_m = self.utils.motor_rad_to_translate(waypoint_rad.position)
            v_m = self.utils.motor_rad_to_translate(waypoint_rad.velocity)
            a_m = self.utils.motor_rad_to_translate(waypoint_rad.acceleration)
            return Waypoint(time=t_s, position=x_m, velocity=v_m, acceleration=a_m)

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

        x_r = self.utils.translate_to_motor_rad(x_m)
        v_r = self.utils.translate_to_motor_rad(v_m) if v_m is not None else None
        a_r = self.utils.translate_to_motor_rad(a_m) if a_m is not None else None

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

    def is_valid(self, ignore_joint_limits=False):
        """Verifies whether the current trajectory is valid.

        Parameters
        ----------
        ignore_joint_limits : bool
            whether to check if waypoints fall within joint limits

        Returns
        -------
        bool
        """
        if len(self.waypoints) < 2:
            return False

        if not ignore_joint_limits:
            pass # TODO

        return True


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
        traj_segment_idx : int
            index of the currently active segment on the stepper hardware
        traj_loaded : bool
            whether a trajectory segment is currently loaded on the hardware
        traj_curr_segment : list(float)
            segment represented as list as [seg_duration, a0-a5 coefficients, seg_id]
        """
        TrajectoryManager.__init__(self)
        self.trajectory = StepperTrajectory(self)
        self.traj_segment_idx = 0
        self.traj_loaded = False
        self.traj_curr_segment = None

    def _setup_new_trajectory(self):
        self.traj_segment_idx = 0
        self.traj_loaded = False
        self.traj_curr_segment = None
        self.traj_start_time = None
        self.traj_curr_time = None

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
        if not self.trajectory.is_valid(ignore_joint_limits=True):
            return
        if req_calibration and not self.motor.status['pos_calibrated']:
            print 'Arm not homed'
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
        self.motor.start_waypoint_trajectory()
        self.traj_start_time = time.time()
        self.traj_curr_time = self.traj_start_time
        TrajectoryManager._start_trajectory_thread(self)

    def push_trajectory(self):
        """Commands goals to Hello Robot stepper hardware.

        If not using threaded mode in ``start_trajectory``, the user is
        responsible for calling this method a regular frequency.
        As low as 25hz is acceptable.

        Returns
        -------
        bool
            True if trajectory must continue to be pushed, False otherwise
        """
        if self.traj_start_time is None:
            return False

        self.traj_curr_time = time.time()
        self.motor.push_waypoint_trajectory()
        self.pull_status()
        if not self.status['motor']['trajectory_active']:
            return False

        return True

    def stop_trajectory(self):
        """Stop a currently executing trajectory.

        Restores the stepper hardware to position mode and
        resets trajectory tracking. Additionally, stops
        threaded execution if necessary.
        """
        self.traj_start_time = None
        self.motor.enable_pos_traj()
        self.motor.push_command()
        self._setup_new_trajectory()
        TrajectoryManager._stop_trajectory_thread(self)

    def get_first_segment(self):
        if not self.trajectory.is_valid(ignore_joint_limits=True):
            return

        i0 = 0
        i1 = 1
        waypoint0 = self.trajectory.get_waypoint(i0, motor_rad=True)
        waypoint1 = self.trajectory.get_waypoint(i1, motor_rad=True)
        if waypoint0.acceleration is not None and waypoint1.acceleration is not None:
            i0_waypoint = [waypoint0.time, waypoint0.position, waypoint0.velocity, waypoint0.acceleration]
            i1_waypoint = [waypoint1.time, waypoint1.position, waypoint1.velocity, waypoint1.acceleration]
            self.traj_curr_segment = generate_quintic_spline_segment(i0_waypoint, i1_waypoint)
        elif waypoint0.velocity is not None and waypoint1.velocity is not None:
            i0_waypoint = [waypoint0.time, waypoint0.position, waypoint0.velocity]
            i1_waypoint = [waypoint1.time, waypoint1.position, waypoint1.velocity]
            self.traj_curr_segment = generate_cubic_spline_segment(i0_waypoint, i1_waypoint)
        else:
            i0_waypoint = [waypoint0.time, waypoint0.position]
            i1_waypoint = [waypoint1.time, waypoint1.position]
            self.traj_curr_segment = generate_linear_segment(i0_waypoint, i1_waypoint)

        self.traj_segment_idx = 2
        self.traj_curr_segment.append(self.traj_segment_idx)
        return self.traj_curr_segment

    def get_next_segment(self, active_id):
        """Returns next segment for hardware to execute.

        The active id is the ID of the currently active trajectory segment as reported by the uC.
        Ids go from 2 to 255 and loop around. Id of 0 marks that the uC is in Idle state. Id of 1
        marks that a trajectory is loaded but not executing. Id of 2 marks that the first segment
        has begun executing. The segment that is returned is the subsequent segment after the active
        segment. A segment is represented by [seg_duration, six coefficients a0 through a5, seg_id].
        All future segments may be modified until the hardware latches it into the active segment.
        """
        if active_id == 0 and self.traj_loaded:
            self._setup_new_trajectory()
            self.traj_start_time = None
            return
        if not self.trajectory.is_valid(ignore_joint_limits=True):
            return
        if active_id == 1:
            self.traj_loaded = True
            return self.traj_curr_segment

        if active_id == self.traj_segment_idx:
            self.traj_segment_idx = max(2, (self.traj_segment_idx + 1) % 255)
        if (self.traj_segment_idx - 2) < (len(self.trajectory) - 1):
            i0 = self.traj_segment_idx - 2
            i1 = self.traj_segment_idx - 1
            waypoint0 = self.trajectory.get_waypoint(i0, motor_rad=True)
            waypoint1 = self.trajectory.get_waypoint(i1, motor_rad=True)
            if waypoint0.acceleration is not None and waypoint1.acceleration is not None:
                i0_waypoint = [waypoint0.time, waypoint0.position, waypoint0.velocity, waypoint0.acceleration]
                i1_waypoint = [waypoint1.time, waypoint1.position, waypoint1.velocity, waypoint1.acceleration]
                self.traj_curr_segment = generate_quintic_spline_segment(i0_waypoint, i1_waypoint)
            elif waypoint0.velocity is not None and waypoint1.velocity is not None:
                i0_waypoint = [waypoint0.time, waypoint0.position, waypoint0.velocity]
                i1_waypoint = [waypoint1.time, waypoint1.position, waypoint1.velocity]
                self.traj_curr_segment = generate_cubic_spline_segment(i0_waypoint, i1_waypoint)
            else:
                i0_waypoint = [waypoint0.time, waypoint0.position]
                i1_waypoint = [waypoint1.time, waypoint1.position]
                self.traj_curr_segment = generate_linear_segment(i0_waypoint, i1_waypoint)
            self.traj_curr_segment.append(self.traj_segment_idx)
        else:
            self.traj_curr_segment = [0] * 8
        return self.traj_curr_segment


class WaypointTrajectoryManager:
    """
    Manage joint trajectories
    Trajectories are specified as a list of waypoints
    TRAJECTORY_TYPE_CUBIC_SPLINE:   [[time, pos,vel],...]
    TRAJECTORY_TYPE_QUINTIC_SPLINE: [[time, pos,vel,accel],...]

    These are then represented as a list of continuous trajectory segments (splines or lines)
    TRAJECTORY_TYPE_CUBIC_SPLINE:   [[duration, a0,a1,a2,a3],...]
    TRAJECTORY_TYPE_QUINTIC_SPLINE: [[duration, a0,a1,a2,a3,a4,a5],...]

    New waypoints can be added dynamically (so long as it is in the future),
    and the list of splines will be recomputed.

    The spline trajectory for each pair of waypoints is periodically pushed down to the joint controller
    for execution.
    """
    def __init__(self):
        self.__setup_new_trajectory()
        self.waypoints_last_trajectory = None

    def add_waypoints_to_trajectory(self,v):
        #V is a list of waypoints
        if self.num_segment_left()>0: #Append or insert into an existing trajectory
            if self.get_trajectory_type(v[0])!=self.traj_type:
                print('Invalid waypoint type')
                return
            tstart=self.waypoints[self.idx_waypoint][0]
            tend=self.waypoints[-1][0]
            tnew=v[0][0]
            if tnew<tstart:
                print('Unable to add waypoints to trajectory: tstart %f tnew %f'%(tstart,tnew))
                return
            idx_insert=len(self.waypoints)
            if tnew<tend: #Replace existing waypoints with new ones starting at tnew
                idx_insert=0
                for i in range(len(self.waypoints)):
                    if self.waypoints[i][0]>=tnew:
                        idx_insert=i
                        break
            self.waypoints =self.waypoints[0:idx_insert]+copy.deepcopy(v)
        else: #Start of a new trajectory
            self.__setup_new_trajectory()
            self.waypoints=copy.deepcopy(v)
            self.traj_type=self.get_trajectory_type(v[0])

        self.ts_duration=self.waypoints[-1][0] - self.waypoints[0][0]


    def get_trajectory_type(self,v):
        if len(v) == 4:
            return TRAJECTORY_TYPE_QUINTIC_SPLINE
        if len(v) == 3:
            return TRAJECTORY_TYPE_CUBIC_SPLINE
        return TRAJECTORY_TYPE_INVALID

    def __setup_new_trajectory(self):
        self.waypoints = []
        self.idx_waypoint = 0
        self.ts_start = None
        self.ts_duration = 0
        self.new_trajectory = True
        self.id_seg=0
        self.trajectory_loaded=False
        self.seg=None
        self.traj_type =TRAJECTORY_TYPE_INVALID
        self.last_setpoint_time = 0

    def num_segment_left(self):
        return max(0,len(self.waypoints)-self.idx_waypoint-1)


    def find_waypoint_before_time(self,t):
        #Return idx of waypoint just before/at the time
        for i in range(len(self.waypoints)-1):
            if t>=self.waypoints[i][0] and t<=self.waypoints[i+1][0] :
                return i
        return None

    def get_setpoint_at_time(self,t):
        #Get the target pos, vel, accel at time t
        if t>self.waypoints[-1][0]:
            return self.waypoints[-1][1:]
        idx_0=self.find_waypoint_before_time(t)
        if idx_0 is None:
            return None
        idx_1=idx_0+1
        if(idx_0>0):
            t_prior=self.waypoints[idx_0][0]
        else:
            t_prior=0
        self.last_setpoint_time = t
        if self.traj_type == TRAJECTORY_TYPE_CUBIC_SPLINE:
            #print 'Waypoint0',self.waypoints[idx_0],t-t_prior
            seg = self.generate_cubic_spline_segment(self.waypoints[idx_0],self.waypoints[idx_1])
            e=self.evaluate_cubic_spline(seg,t-t_prior)
            #print 'Vtarget',e[1]
            return e

        if self.traj_type == TRAJECTORY_TYPE_QUINTIC_SPLINE:
            seg = self.generate_quintic_spline_segment(self.waypoints[idx_0],self.waypoints[idx_1])
            return self.evaluate_quintic_spline(seg,t-t_prior)


    def get_first_segment(self):
        #Get first segment to send down, start loading
        #For non dynamixel joints
        self.idx_waypoint=0
        if (self.num_segment_left()<1):
            return None
        if self.traj_type == TRAJECTORY_TYPE_CUBIC_SPLINE:  # Recompute spline in case has changed
            self.seg = self.generate_cubic_spline_segment(self.waypoints[self.idx_waypoint], self.waypoints[self.idx_waypoint + 1])
            self.id_seg = 2
            self.seg.append(self.id_seg) #Id of first segment
        if self.traj_type == TRAJECTORY_TYPE_QUINTIC_SPLINE:  # Recompute spline in case has changed
            self.seg = self.generate_quintic_spline_segment(self.waypoints[self.idx_waypoint], self.waypoints[self.idx_waypoint + 1])
            self.id_seg = 2
            self.seg.append(self.id_seg) #Id of first segment
        return self.seg

    def get_next_segment(self,active_id):
        #The active id is the ID of the currently active trajectory segment as reported by the uC
        #Ids go from 2 to 255 and loop around
        #Id of 0 marks that the uC is in Idle state
        #Id of 1 marks that a trajectory is loaded but not executing
        #Id of 2 marks that the first segment has begun executing
        #This returns the trajectory segment subsequent to the active_id (seg_next)
        #The seg_next can be modified up until active_id==seg_next.id, allow for last minute revision
        #of the next segment to be pushed to the uC
        #An arbitrary number of segments can be sent do the uC sequentially
        #The end of a sequence is marked by sending a [0]*8 segment
        # For non dynamixel joints

        if active_id ==0 and self.trajectory_loaded: #Marks end of last segment of trajectory
            self.waypoints_last_trajectory=copy.deepcopy(self.waypoints) #Store this if want a trajectory history
            self.__setup_new_trajectory()
            return None

        if active_id ==1:   #Marks first segment of trajectory is loaded but not yet executing
            self.trajectory_loaded = True
            return self.seg

        if active_id == 2:
            if self.new_trajectory:
                self.new_trajectory = False
                self.ts_start = time.time()

        if active_id == self.id_seg:  #Marks the last pushed segment is executing, advance counters to push next one
            self.idx_waypoint=self.idx_waypoint+1
            self.id_seg = max(2,(self.id_seg+1)%255)

        if self.num_segment_left()>0 :
            if self.traj_type == TRAJECTORY_TYPE_CUBIC_SPLINE: #Recompute spline in case has changed
                self.seg= self.generate_cubic_spline_segment(self.waypoints[self.idx_waypoint],self.waypoints[self.idx_waypoint+1])
                self.seg.append(self.id_seg)
            if self.traj_type == TRAJECTORY_TYPE_QUINTIC_SPLINE: #Recompute spline in case has changed
                self.seg= self.generate_quintic_spline_segment(self.waypoints[self.idx_waypoint],self.waypoints[self.idx_waypoint+1])
                self.seg.append(self.id_seg)
        else:
            self.seg=[0]*8 #Marks no more segments to the uC
        return self.seg

    def duration_remaining(self):
        if self.ts_start is None:
            return self.ts_duration
        dt_elapased = time.time() - self.ts_start
        return max(0,self.ts_duration - dt_elapased)

    def generate_quintic_spline_segment(self, i,f):
        # waypoints are [[time, pos,vel,accel],...]
        duration = f[0] - i[0]
        a0 = i[1]
        a1 = i[2]
        a2 = i[3] / 2
        a3 = (20 * f[1] - 20 * i[1] - (8 * f[2] + 12 * i[2]) * (f[0] - i[0]) - (3 * i[3] - f[3]) * ((f[0] - i[0]) ** 2)) / (2 * ((f[0] - i[0]) ** 3))
        a4 = (30 * i[1] - 30 * f[1] + (14 * f[2] + 16 * i[2]) * (f[0] - i[0]) + (3 * i[3] - 2 * f[3]) * ((f[0] - i[0]) ** 2)) / (2 * ((f[0] - i[0]) ** 4))
        a5 = (12 * f[1] - 12 * i[1] - (6 * f[2] + 6 * i[2]) * (f[0] - i[0]) - (i[3] - f[3]) * ((f[0] - i[0]) ** 2)) / (2 * ((f[0] - i[0]) ** 5))
        return [duration, a0, a1, a2, a3, a4,a5]

    #See Craig Intro Robotics Ch7
    def generate_cubic_spline_segment(self, i,f):
        # waypoints are [[time, pos,vel],...]
        duration =f[0] - i[0]
        a0 = i[1]
        a1 = i[2]
        a2 = (3 / duration ** 2) * (f[1] - i[1]) - (2 / duration) * i[2] - (1 / duration) * f[2]
        a3 = (-2 / duration ** 3) * (f[1] - i[1]) + (1 / duration ** 2) * (f[2] + i[2])
        return [duration, a0, a1, a2, a3, 0, 0]

    def generate_linear_segment(self,i,f):
        # waypoints are [[time, pos],...]
        duration = f[0] - i[0]
        return [duration, i[1],f[1]]

    def evaluate_cubic_spline(self,s,t):
        #TRAJECTORY_TYPE_CUBIC_SPLINE:   [[duration, a0,a1,a2,a3],...]
        a=s[1:]
        pos= a[0]+(a[1]*t)+(a[2]*t**2)+(a[3]*t**3)
        vel= a[1] +(2*a[2]*t)+(3*a[3]*t**2)
        acc= 2*a[2] + 6*a[3]*t
        return [pos,vel,acc]

    def evaluate_quintic_spline(self,s,t):
        #TRAJECTORY_TYPE_QUINTIC_SPLINE: [[duration, a0,a1,a2,a3,a4,a5],...]
        a=s[1:]
        pos= a[0]+(a[1]*t)+(a[2]*t**2)+(a[3]*t**3)+(a[4]*t**4)+(a[5]*t**5)
        vel = a[1] +(2*a[2]*t)+(3*a[3]*t**2) + (4*a[4]*t**3) + (5*a[5]*t*4)
        accel = 2*a[2] + 6*a[3]*t + 12*a[4]*t**2 + 20*a[5]*t**3
        return [pos,vel,accel]

    def evaluate_linear_interpolate(self,s,t):
        #TRAJECTORY_TYPE_LINEAR: [[duration, pos0, pos1], ...]
        duration = s[0]
        pos0=s[1]
        pos1=s[2]
        pos = pos0 + t*(pos1-pos0)/duration
        vel = (pos1-pos0)/duration
        accel = None
        return [pos, vel, accel]

