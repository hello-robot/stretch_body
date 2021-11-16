from __future__ import print_function
from stretch_body.stepper import Stepper
from stretch_body.device import Device
from stretch_body.trajectories import PrismaticTrajectory

import time
import math


class Lift(Device):
    """
    API to the Stretch RE1 Lift
    """
    def __init__(self):
        Device.__init__(self, 'lift')
        self.motor = Stepper(usb='/dev/hello-motor-lift')
        self.status = {'timestamp_pc':0,'pos': 0.0, 'vel': 0.0, 'force':0.0,'motor': self.motor.status}
        self.trajectory = PrismaticTrajectory()
        self.thread_rate_hz = 5.0

        # Default controller params
        self.stiffness = 1.0
        self.i_feedforward=self.params['i_feedforward']
        self.vel_r = self.translate_to_motor_rad(self.params['motion']['default']['vel_m'])
        self.accel_r = self.translate_to_motor_rad(self.params['motion']['default']['accel_m'])
        self.i_contact_neg = self.translate_force_to_motor_current(self.params['contact_thresh_N'][0])
        self.i_contact_pos = self.translate_force_to_motor_current(self.params['contact_thresh_N'][1])
        self.soft_motion_limits = {'collision': [None, None], 'user': [None, None],
                                   'hard': [self.params['range_m'][0], self.params['range_m'][1]],
                                   'current': [self.params['range_m'][0], self.params['range_m'][1]]}
        self.motor.set_motion_limits(self.translate_to_motor_rad(self.soft_motion_limits['current'][0]),
                                     self.translate_to_motor_rad(self.soft_motion_limits['current'][1]))
    # ###########  Device Methods #############
    def startup(self, threaded=True):
        Device.startup(self, threaded=threaded)
        success= self.motor.startup(threaded=False)
        self.__update_status()
        return success

    def stop(self):
        Device.stop(self)
        if self.motor.hw_valid and int(str(self.motor.board_info['protocol_version'])[1:]) >= 1:
            self.motor.stop_waypoint_trajectory()
        self.motor.stop()

    def pull_status(self):
        self.motor.pull_status()
        self.__update_status()

    def __update_status(self):
        self.status['timestamp_pc'] = time.time()
        self.status['pos'] = self.motor_rad_to_translate_m(self.status['motor']['pos'])
        self.status['vel'] = self.motor_rad_to_translate_m(self.status['motor']['vel'])
        self.status['force'] = self.motor_current_to_translate_force(self.status['motor']['current'])

    def push_command(self):
        self.motor.push_command()

    def _thread_loop(self):
        self.pull_status()
        self.update_trajectory()

    def pretty_print(self):
        print('----- Lift ------ ')
        print('Pos (m): ', self.status['pos'])
        print('Vel (m/s): ', self.status['vel'])
        print('Force (N): ', self.status['force'])
        print('Soft motion limits (m)', self.soft_motion_limits['current'])
        print('Timestamp PC (s):', self.status['timestamp_pc'])
        self.motor.pretty_print()

    # ###################################################


    def get_soft_motion_limits(self):
        """
            Return the currently applied soft motion limits: [min, max]

            The soft motion limit restricts joint motion to be <= its physical limits.

            There are three types of limits:
            Hard: The physical limits
            Collision: Limits set by RobotCollision to avoid collisions
            User: Limits set by the user software

            The joint is limited to the most restrictive range of the Hard / Collision/ User values.
            Specifying a value of None for a limit indicates that no constraint exists for that limit type.
            This allows a User limits and Collision limits to coexist.
            For example, a user can temporarily restrict the range of motion beyond the current collision limits.
            Then, by commanding User limits of None, the joint limits will revert back to the collision settings.
        """
        return self.soft_motion_limits['current']

    def set_soft_motion_limit_min(self,x,limit_type='user' ):
        """
        x: value to set a joints limit to
        limit_type: 'user' or 'collision'
        """
        self.soft_motion_limits[limit_type][0]=x
        xn=max(filter(lambda x: x is not None, [self.soft_motion_limits['collision'][0],self.soft_motion_limits['hard'][0],self.soft_motion_limits['user'][0]]))
        prev=self.soft_motion_limits['current'][:]
        self.soft_motion_limits['current'][0]=xn
        if xn != prev[0]:
            #print('New soft limit on min',xn)
            self.motor.set_motion_limits(self.translate_to_motor_rad(self.soft_motion_limits['current'][0]), self.translate_to_motor_rad(self.soft_motion_limits['current'][1]))

    def set_soft_motion_limit_max(self,x,limit_type='user' ):
        """
        x: value to set a joints limit to
        limit_type: 'user' or 'collision'
        """
        self.soft_motion_limits[limit_type][1]=x
        xn=min(filter(lambda x: x is not None, [self.soft_motion_limits['collision'][1],self.soft_motion_limits['hard'][1],self.soft_motion_limits['user'][1]]))
        prev=self.soft_motion_limits['current'][:]
        self.soft_motion_limits['current'][1]=xn
        if xn != prev[1]:
            #print('New soft limit on max', xn)
            self.motor.set_motion_limits(self.translate_to_motor_rad(self.soft_motion_limits['current'][0]), self.translate_to_motor_rad(self.soft_motion_limits['current'][1]))

    # ###################################################

    def set_velocity(self, v_m, a_m=None,stiffness=None, contact_thresh_pos_N=None, contact_thresh_neg_N=None, req_calibration=True):
        if req_calibration:
            if not self.motor.status['pos_calibrated']:
                self.logger.warning('Lift not calibrated')
                return
        v_m=min(self.params['motion']['max']['vel_m'],v_m) if v_m>=0 else max(-1*self.params['motion']['max']['vel_m'],v_m)
        v_r = self.translate_to_motor_rad(v_m)

        if stiffness is not None:
            stiffness = max(0, min(1.0, stiffness))
        else:
            stiffness = self.stiffness

        if a_m is not None:
            a_r = self.translate_to_motor_rad(min(abs(a_m), self.params['motion']['max']['accel_m']))
        else:
            a_r = self.accel_r

        i_contact_neg = self.translate_force_to_motor_current(
            max(contact_thresh_neg_N, self.params['contact_thresh_max_N'][0])) \
            if contact_thresh_neg_N is not None else self.translate_force_to_motor_current(self.params['contact_thresh_N'][0])
        i_contact_pos = self.translate_force_to_motor_current(
            min(contact_thresh_pos_N, self.params['contact_thresh_max_N'][1])) \
            if contact_thresh_pos_N is not None else self.translate_force_to_motor_current(self.params['contact_thresh_N'][1])


        self.motor.set_command(mode=Stepper.MODE_VEL_TRAJ,
                               v_des=v_r,
                               a_des=a_r,
                               stiffness=stiffness,
                               i_feedforward=self.i_feedforward,
                               i_contact_pos=i_contact_pos,
                               i_contact_neg=i_contact_neg)

    def move_to(self,x_m,v_m=None, a_m=None, stiffness=None, contact_thresh_pos_N=None, contact_thresh_neg_N=None, req_calibration=True):
        """
        x_m: commanded absolute position (meters). x_m=0 is down. x_m=~1.1 is up
        v_m: velocity for trapezoidal motion profile (m/s)
        a_m: acceleration for trapezoidal motion profile (m/s^2)
        stiffness: stiffness of motion. Range 0.0 (min) to 1.0 (max)
        contact_thresh_pos_N: force threshold to stop motion (~Newtons, up direction)
        contact_thresh_neg_N: force threshold to stop motion (~Newtons, down direction)
        req_calibration: Disallow motion prior to homing
        """
        if req_calibration:
            if not self.motor.status['pos_calibrated']:
                self.logger.warning('Lift not calibrated')
                return
            x_m = min(max(self.soft_motion_limits['current'][0], x_m), self.soft_motion_limits['current'][1]) #Only clip motion when calibrated
        if stiffness is not None:
            stiffness = max(0, min(1.0, stiffness))
        else:
            stiffness = self.stiffness

        if v_m is not None:
            v_r=self.translate_to_motor_rad(min(abs(v_m), self.params['motion']['max']['vel_m']))
        else:
            v_r = self.vel_r

        if a_m is not None:
            a_r = self.translate_to_motor_rad(min(abs(a_m), self.params['motion']['max']['accel_m']))
        else:
            a_r = self.accel_r

        i_contact_neg = self.translate_force_to_motor_current(
            max(contact_thresh_neg_N, self.params['contact_thresh_max_N'][0])) \
            if contact_thresh_neg_N is not None else self.translate_force_to_motor_current(self.params['contact_thresh_N'][0])
        i_contact_pos = self.translate_force_to_motor_current(
            min(contact_thresh_pos_N, self.params['contact_thresh_max_N'][1])) \
            if contact_thresh_pos_N is not None else self.translate_force_to_motor_current(self.params['contact_thresh_N'][1])

        self.motor.set_command(mode = Stepper.MODE_POS_TRAJ,
                                x_des=self.translate_to_motor_rad(x_m),
                                v_des=v_r,
                                a_des=a_r,
                                stiffness=stiffness,
                                i_feedforward=self.i_feedforward,
                                i_contact_pos=i_contact_pos,
                                i_contact_neg=i_contact_neg)

    def move_by(self,x_m,v_m=None, a_m=None, stiffness=None, contact_thresh_pos_N=None, contact_thresh_neg_N=None, req_calibration=True):
        """
        x_m: commanded incremental motion (meters).
        v_m: velocity for trapezoidal motion profile (m/s)
        a_m: acceleration for trapezoidal motion profile (m/s^2)
        stiffness: stiffness of motion. Range 0.0 (min) to 1.0 (max)
        contact_thresh_pos_N: force threshold to stop motion (~Newtons, up direction)
        contact_thresh_neg_N: force threshold to stop motion (~Newtons, down direction)
        req_calibration: Disallow motion prior to homing
        """
        if req_calibration:
            if not self.motor.status['pos_calibrated']:
                self.logger.warning('Lift not calibrated')
                return

            if self.status['pos'] + x_m < self.soft_motion_limits['current'][0]:  #Only clip motion when calibrated
                x_m = self.soft_motion_limits['current'][0] - self.status['pos']
            if self.status['pos'] + x_m > self.soft_motion_limits['current'][1]:
                x_m = self.soft_motion_limits['current'][1] - self.status['pos']

        if stiffness is not None:
            stiffness = max(0, min(1.0, stiffness))
        else:
            stiffness = self.stiffness

        if v_m is not None:
            v_r=self.translate_to_motor_rad(min(abs(v_m), self.params['motion']['max']['vel_m']))
        else:
            v_r = self.vel_r

        if a_m is not None:
            a_r = self.translate_to_motor_rad(min(abs(a_m), self.params['motion']['max']['accel_m']))
        else:
            a_r = self.accel_r

        i_contact_neg = self.translate_force_to_motor_current(
            max(contact_thresh_neg_N, self.params['contact_thresh_max_N'][0])) \
            if contact_thresh_neg_N is not None else self.translate_force_to_motor_current(self.params['contact_thresh_N'][0])
        i_contact_pos = self.translate_force_to_motor_current(
            min(contact_thresh_pos_N, self.params['contact_thresh_max_N'][1])) \
            if contact_thresh_pos_N is not None else self.translate_force_to_motor_current(self.params['contact_thresh_N'][1])

        #print('Lift %.2f , %.2f  , %.2f' % (x_m, self.motor_rad_to_translate_m(v_r), self.motor_rad_to_translate_m(a_r)))

        self.motor.set_command(mode = Stepper.MODE_POS_TRAJ_INCR,
                                x_des=self.translate_to_motor_rad(x_m),
                                v_des=v_r,
                                a_des=a_r,
                                stiffness=stiffness,
                                i_feedforward=self.i_feedforward,
                                i_contact_pos=i_contact_pos,
                                i_contact_neg=i_contact_neg)

    # ######### Waypoint Trajectory Interface ##############################
    def is_trajectory_active(self):
        return self.motor.is_trajectory_active()

    def follow_trajectory(self, v_m=None, a_m=None, stiffness=None, contact_thresh_pos_N=None, contact_thresh_neg_N=None,
                          req_calibration=True, move_to_start_point=True):
        """Starts executing a waypoint trajectory

        `self.trajectory` must be populated with a valid trajectory before calling
        this method.

        Parameters
        ----------
        v_m : float
            velocity limit for trajectory in meters per second
        a_m : float
            acceleration limit for trajectory in meters per second squared
        stiffness : float
            stiffness of motion. Range 0.0 (min) to 1.0 (max)
        contact_thresh_pos_N : float
            force threshold to stop motion (~Newtons, up direction)
        contact_thresh_neg_N : float
            force threshold to stop motion (~Newtons, down direction)
        req_calibration : bool
            whether to allow motion prior to homing
        move_to_start_point : bool
            whether to move to the trajectory's start to avoid a jump, this
            time to move doesn't count against the trajectory's timeline
        """
        # check if joint valid, homed, and right protocol
        if not self.motor.hw_valid:
            self.logger.warning('Lift connection to hardware not valid')
            return
        if req_calibration:
            if not self.motor.status['pos_calibrated']:
                self.logger.warning('Lift not calibrated')
                return
        if int(str(self.motor.board_info['protocol_version'])[1:]) < 1:
            self.logger.warning("Lift firmware version doesn't support waypoint trajectories")
            return

        # check if trajectory valid
        vel_limit = v_m if v_m is not None else self.params['motion']['trajectory_max']['vel_m']
        acc_limit = a_m if a_m is not None else self.params['motion']['trajectory_max']['accel_m']
        valid, reason = self.trajectory.is_valid(vel_limit, acc_limit)
        if not valid:
            self.logger.warning('Lift traj not valid: {0}'.format(reason))
            return

        # set defaults
        stiffness = max(0, min(1.0, stiffness)) if stiffness is not None else self.stiffness
        v_r = self.translate_to_motor_rad(min(abs(v_m), self.params['motion']['trajectory_max']['vel_m'])) \
            if v_m is not None else self.translate_to_motor_rad(self.params['motion']['trajectory_max']['vel_m'])
        a_r = self.translate_to_motor_rad(min(abs(a_m), self.params['motion']['trajectory_max']['accel_m'])) \
            if a_m is not None else self.translate_to_motor_rad(self.params['motion']['trajectory_max']['accel_m'])
        i_contact_neg = self.translate_force_to_motor_current(max(contact_thresh_neg_N, self.params['contact_thresh_max_N'][0])) \
            if contact_thresh_neg_N is not None else self.translate_force_to_motor_current(self.params['contact_thresh_N'][0])
        i_contact_pos = self.translate_force_to_motor_current(min(contact_thresh_pos_N, self.params['contact_thresh_max_N'][1])) \
            if contact_thresh_pos_N is not None else self.translate_force_to_motor_current(self.params['contact_thresh_N'][1])

        # move to start point
        if move_to_start_point:
            prev_sync_mode = self.motor.gains['enable_sync_mode']
            self.move_to(self.trajectory[0].position)
            self.motor.disable_sync_mode()
            self.push_command()
            if not self.motor.wait_until_at_setpoint():
                self.logger.warning('Lift unable to reach starting point')
            if prev_sync_mode:
                self.motor.enable_sync_mode()
                self.push_command()

        # start trajectory
        self.motor.set_command(mode=Stepper.MODE_POS_TRAJ_WAYPOINT,
                               v_des=v_r,
                               a_des=a_r,
                               stiffness=stiffness,
                               i_contact_pos=i_contact_pos,
                               i_contact_neg=i_contact_neg)
        self.motor.push_command()
        s0 = self.trajectory.get_segment(0, to_motor_rad=self.translate_to_motor_rad).to_array()
        self.motor.start_waypoint_trajectory(s0)

    def update_trajectory(self):
        """Updates hardware with the next segment of `self.trajectory`

        This method must be called frequently to enable complete trajectory execution
        and preemption of future segments. If used with `stretch_body.robot.Robot` or
        with `self.startup(threaded=True)`, a background thread is launched for this.
        Otherwise, the user must handle calling this method.
        """
        # check if joint valid, right protocol, and right mode
        if not self.motor.hw_valid or int(str(self.motor.board_info['protocol_version'])[1:]) < 1:
            return
        if not self.is_trajectory_active():
            return

        if self.motor.status['waypoint_traj']['state'] == 'active':
            next_segment_id = self.motor.status['waypoint_traj']['segment_id'] - 2 + 1 # subtract 2 due to IDs 0 & 1 being reserved by firmware
            if next_segment_id < self.trajectory.get_num_segments():
                s1 = self.trajectory.get_segment(next_segment_id, to_motor_rad=self.translate_to_motor_rad).to_array()
                self.motor.set_next_trajectory_segment(s1)
        elif self.motor.status['waypoint_traj']['state'] == 'idle' and self.is_trajectory_active():
            self.motor.enable_pos_traj()
            self.push_command()

    def stop_trajectory(self):
        """Stop waypoint trajectory immediately and resets hardware
        """
        self.motor.stop_waypoint_trajectory()

    # ######### Utility ##############################

    def motor_current_to_translate_force(self,i):
        #tq=self.motor.current_to_torque(i)
        #r = (self.params['pinion_t'] * self.params['belt_pitch_m'] / math.pi)/2
        return self.params['force_N_per_A']*(i-self.params['i_feedforward'])
        #return tq/r

    def translate_force_to_motor_current(self,f):
        return (f/self.params['force_N_per_A'])+self.params['i_feedforward']
        #r = (self.params['pinion_t'] * self.params['belt_pitch_m'] / math.pi) / 2
        #tq=f*r
        #return self.motor.torque_to_current(tq)

    def motor_rad_to_translate_m(self,ang): #input in rad
        d=self.params['pinion_t']*self.params['belt_pitch_m']/math.pi
        lift_m = (math.degrees(ang)/180.0)*math.pi*(d/2)
        return lift_m

    def translate_to_motor_rad(self, lift_m):
        d = self.params['pinion_t'] * self.params['belt_pitch_m'] / math.pi
        ang = 180*lift_m/((d/2)*math.pi)
        return math.radians(ang)

    def __wait_for_contact(self, timeout=5.0):
        ts=time.time()
        while (time.time()-ts<timeout):
            self.pull_status()
            if self.motor.status['in_guarded_event']:
                return True
            time.sleep(0.01)
        return False


    def home(self, measuring=False):
        if not self.motor.hw_valid:
            self.logger.warning('Not able to home lift. Hardware not present')
            return
        print('Homing lift...')
        prev_guarded_mode = self.motor.gains['enable_guarded_mode']
        prev_sync_mode = self.motor.gains['enable_sync_mode']
        self.motor.enable_guarded_mode()
        self.motor.disable_sync_mode()
        self.motor.reset_pos_calibrated()
        self.push_command()

        x_up=None
        #Up direction
        self.pull_status()
        xstart=self.status['pos']
        self.move_by(x_m=1.25, contact_thresh_pos_N=self.params['homing_force_N'][1], contact_thresh_neg_N=self.params['homing_force_N'][0], req_calibration=False)
        self.push_command()
        if self.__wait_for_contact(timeout=15.0): #self.__wait_for_contact(timeout=15.0):
            print('Upward contact detected at motor position (rad)', self.motor.status['pos'],self.motor_rad_to_translate_m(self.motor.status['pos']))
            if not measuring:
                x = self.translate_to_motor_rad(self.params['range_m'][1])
                print('Marking lift position to (m)', self.params['range_m'][1])
                self.motor.mark_position(x)
                self.motor.set_pos_calibrated()
                self.push_command()
            else:
                self.pull_status()
                x_up=self.status['pos']-xstart
        else:
            print('Failed to detect contact')
            return
        #Allow to settle
        time.sleep(1.0)

        # Down direction
        self.move_to(x_m=0.6, req_calibration=False)
        self.push_command()
        time.sleep(6.0)

        # Restore previous modes
        if not prev_guarded_mode:
            self.motor.disable_guarded_mode()
        if prev_sync_mode:
            self.motor.enable_sync_mode()
        self.push_command()

        return x_up

    def step_sentry(self,robot):
        self.motor.step_sentry(robot)
