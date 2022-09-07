from __future__ import print_function
from stretch_body.stepper import Stepper
from stretch_body.device import Device
from stretch_body.trajectories import PrismaticTrajectory
import stretch_body.hello_utils as hu
import time
import sys


class PrismaticJoint(Device):
    """
    API to the Stretch Prismatic Joints
    """
    def __init__(self,name):
        Device.__init__(self,name )
        self.motor = Stepper(usb='/dev/hello-motor-'+name)
        self.status = {'timestamp_pc':0,'pos': 0.0, 'vel': 0.0, 'force':0.0,'motor': self.motor.status}
        self.trajectory = PrismaticTrajectory()
        self.thread_rate_hz = 5.0

        # Default controller params
        self.stiffness = 1.0
        self.i_feedforward=self.params['i_feedforward']
        self.vel_r = self.translate_m_to_motor_rad(self.params['motion']['default']['vel_m'])
        self.accel_r = self.translate_m_to_motor_rad(self.params['motion']['default']['accel_m'])
        self.soft_motion_limits = {'collision': [None, None], 'user': [None, None],
                                   'hard': [self.params['range_m'][0], self.params['range_m'][1]],
                                   'current': [self.params['range_m'][0], self.params['range_m'][1]]}
        self.motor.set_motion_limits(self.translate_m_to_motor_rad(self.soft_motion_limits['current'][0]),
                                     self.translate_m_to_motor_rad(self.soft_motion_limits['current'][1]))

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
        self.status['force'] =  0 #Deprecated

    def push_command(self):
        self.motor.push_command()

    def _thread_loop(self):
        self.pull_status()
        self.update_trajectory()

    def pretty_print(self):
        print('----- %s ------ '%self.name.capitalize())
        print('Pos (m): ', self.status['pos'])
        print('Vel (m/s): ', self.status['vel'])
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
            self.motor.set_motion_limits(self.translate_m_to_motor_rad(self.soft_motion_limits['current'][0]), self.translate_m_to_motor_rad(self.soft_motion_limits['current'][1]))

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
            self.motor.set_motion_limits(self.translate_m_to_motor_rad(self.soft_motion_limits['current'][0]), self.translate_m_to_motor_rad(self.soft_motion_limits['current'][1]))

    # ###################################################

    def contact_thresh_to_motor_current(self,contact_thresh_pos, contact_thresh_neg):
        """
        This model converts from a specified percentage effort (-100 to 100) in the motor frame to motor currents
        """
        e_cn = self.params['contact_models']['effort_pct']['contact_thresh_default'][0] if contact_thresh_neg is None else contact_thresh_neg
        e_cp = self.params['contact_models']['effort_pct']['contact_thresh_default'][1] if contact_thresh_neg is None else contact_thresh_pos
        i_contact_neg = self.motor.effort_pct_to_current(max(e_cn, self.params['contact_models']['effort_pct']['contact_thresh_max'][0]))
        i_contact_pos = self.motor.effort_pct_to_current(min(e_cp, self.params['contact_models']['effort_pct']['contact_thresh_max'][1]))
        return i_contact_pos, i_contact_neg



    def set_velocity(self, v_m, a_m=None,stiffness=None, contact_thresh_pos_N=None,contact_thresh_neg_N=None,req_calibration=True,
                     contact_thresh_pos=None, contact_thresh_neg=None):
        """
        v_m: commanded joint velocity (m/s)
        a_m: acceleration for trapezoidal motion profile (m/s^2)
        stiffness: stiffness of motion. Range 0.0 (min) to 1.0 (max)
        contact_thresh_pos_N: Deprecated: positive threshold to stop motion (units: pseudo_N)
        contact_thresh_neg_N: Deprecated: negative threshold to stop motion (units: pseudo_N)
        req_calibration: Disallow motion prior to homing
        contact_thresh_pos: positive threshold to stop motion (units: effort_pct -100:100)
        contact_thresh_neg: negative threshold to stop motion (units: effort_pct -100:100)
        """

        hu.check_deprecated_contact_model_prismatic_joint(self,'set_velocity', contact_thresh_pos_N, contact_thresh_neg_N,contact_thresh_pos,contact_thresh_neg )

        if req_calibration:
            if not self.motor.status['pos_calibrated']:
                self.logger.warning('%s not calibrated'%self.name.capitalize())
                return

        v_m=min(self.params['motion']['max']['vel_m'],v_m) if v_m>=0 else max(-1*self.params['motion']['max']['vel_m'],v_m)
        v_r = self.translate_m_to_motor_rad(v_m)

        if stiffness is not None:
            stiffness = max(0.0, min(1.0, stiffness))
        else:
            stiffness = self.stiffness

        if a_m is not None:
            a_r = self.translate_m_to_motor_rad(min(abs(a_m), self.params['motion']['max']['accel_m']))
        else:
            a_r = self.accel_r

        i_contact_pos,i_contact_neg  = self.contact_thresh_to_motor_current(contact_thresh_pos,contact_thresh_neg )


        self.motor.set_command(mode=Stepper.MODE_VEL_TRAJ,
                               v_des=v_r,
                               a_des=a_r,
                               stiffness=stiffness,
                               i_feedforward=self.i_feedforward,
                               i_contact_pos=i_contact_pos,
                               i_contact_neg=i_contact_neg)



    def move_to(self,x_m,v_m=None, a_m=None, stiffness=None, contact_thresh_pos_N=None,contact_thresh_neg_N=None,
                req_calibration=True,contact_thresh_pos=None, contact_thresh_neg=None):
        """
        x_m: commanded absolute position (meters). x_m=0 is down. x_m=~1.1 is up
        v_m: velocity for trapezoidal motion profile (m/s)
        a_m: acceleration for trapezoidal motion profile (m/s^2)
        stiffness: stiffness of motion. Range 0.0 (min) to 1.0 (max)
        contact_thresh_pos_N: Deprecated: positive threshold to stop motion (units: pseudo_N)
        contact_thresh_neg_N: Deprecated: negative threshold to stop motion (units: pseudo_N)
        req_calibration: Disallow motion prior to homing
        contact_thresh_pos: positive threshold to stop motion (units: effort_pct -100:100)
        contact_thresh_neg: negative threshold to stop motion (units: effort_pct -100:100)
        """
        hu.check_deprecated_contact_model_prismatic_joint(self,'move_to', contact_thresh_pos_N, contact_thresh_neg_N,contact_thresh_pos,contact_thresh_neg )

        if req_calibration:
            if not self.motor.status['pos_calibrated']:
                self.logger.warning('%s not calibrated'%self.name.capitalize())
                return
            old_x_m = x_m
            x_m = min(max(self.soft_motion_limits['current'][0], x_m), self.soft_motion_limits['current'][1]) #Only clip motion when calibrated
            if x_m != old_x_m:
                self.logger.debug('Clipping move_to({0}) with soft limits {1}'.format(old_x_m, self.soft_motion_limits['current']))

        if stiffness is not None:
            stiffness = max(0.0, min(1.0, stiffness))
        else:
            stiffness = self.stiffness

        if v_m is not None:
            v_r=self.translate_m_to_motor_rad(min(abs(v_m), self.params['motion']['max']['vel_m']))
        else:
            v_r = self.vel_r

        if a_m is not None:
            a_r = self.translate_m_to_motor_rad(min(abs(a_m), self.params['motion']['max']['accel_m']))
        else:
            a_r = self.accel_r

        i_contact_pos,i_contact_neg  = self.contact_thresh_to_motor_current(contact_thresh_pos,contact_thresh_neg )

        self.motor.set_command(mode = Stepper.MODE_POS_TRAJ,
                                x_des=self.translate_m_to_motor_rad(x_m),
                                v_des=v_r,
                                a_des=a_r,
                                stiffness=stiffness,
                                i_feedforward=self.i_feedforward,
                                i_contact_pos=i_contact_pos,
                                i_contact_neg=i_contact_neg)



    def move_by(self,x_m,v_m=None, a_m=None, stiffness=None, contact_thresh_pos_N=None,contact_thresh_neg_N=None, req_calibration=True,
                contact_thresh_pos=None,contact_thresh_neg=None):
        """
        x_m: commanded incremental motion (meters).
        v_m: velocity for trapezoidal motion profile (m/s)
        a_m: acceleration for trapezoidal motion profile (m/s^2)
        stiffness: stiffness of motion. Range 0.0 (min) to 1.0 (max)
        contact_thresh_pos_N: Deprecated: positive threshold to stop motion (units: pseudo_N)
        contact_thresh_neg_N: Deprecated: negative threshold to stop motion (units: pseudo_N)
        req_calibration: Disallow motion prior to homing
        contact_thresh_pos: positive threshold to stop motion (units: effort_pct -100:100)
        contact_thresh_neg: negative threshold to stop motion (units: effort_pct -100:100)
        """
        hu.check_deprecated_contact_model_prismatic_joint(self,'move_by', contact_thresh_pos_N, contact_thresh_neg_N,contact_thresh_pos,contact_thresh_neg )

        if req_calibration or self.motor.status['pos_calibrated']:
            if not self.motor.status['pos_calibrated']:
                self.logger.warning('%s not calibrated'%self.name.capitalize())
                return
            old_x_m = x_m
            if self.status['pos'] + x_m < self.soft_motion_limits['current'][0]:  #Only clip motion when calibrated
                x_m = self.soft_motion_limits['current'][0] - self.status['pos']
            if self.status['pos'] + x_m > self.soft_motion_limits['current'][1]:
                x_m = self.soft_motion_limits['current'][1] - self.status['pos']
            if x_m != old_x_m:
                self.logger.debug('Clipping {0} + move_by({1}) with soft limits {2}'.format(self.status['pos'], old_x_m, self.soft_motion_limits['current']))

        if stiffness is not None:
            stiffness = max(0.0, min(1.0, stiffness))
        else:
            stiffness = self.stiffness

        if v_m is not None:
            v_r=self.translate_m_to_motor_rad(min(abs(v_m), self.params['motion']['max']['vel_m']))
        else:
            v_r = self.vel_r

        if a_m is not None:
            a_r = self.translate_m_to_motor_rad(min(abs(a_m), self.params['motion']['max']['accel_m']))
        else:
            a_r = self.accel_r

        i_contact_pos,i_contact_neg  = self.contact_thresh_to_motor_current(contact_thresh_pos,contact_thresh_neg )

        self.motor.set_command(mode = Stepper.MODE_POS_TRAJ_INCR,
                                x_des=self.translate_m_to_motor_rad(x_m),
                                v_des=v_r,
                                a_des=a_r,
                                stiffness=stiffness,
                                i_feedforward=self.i_feedforward,
                                i_contact_pos=i_contact_pos,
                                i_contact_neg=i_contact_neg)

    # ######### Waypoint Trajectory Interface ##############################

    def follow_trajectory(self, v_m=None, a_m=None, stiffness=None, contact_thresh_pos_N=None,contact_thresh_neg_N=None,
                          req_calibration=True, move_to_start_point=True,contact_thresh_pos=None,contact_thresh_neg=None):
        """Starts executing a waypoint trajectory

        `self.trajectory` must be populated with a valid trajectory before calling
        this method.

        Parameters
        ----------
        v_m : float
            velocity limit for trajectory in meters per second
        a_m : float
            acceleration limit for trajectory in meters per second squared
        stiffness: stiffness of motion. Range 0.0 (min) to 1.0 (max)
        contact_thresh_pos_N: Deprecated: positive threshold to stop motion (units: pseudo_N)
        contact_thresh_neg_N: Deprecated: negative threshold to stop motion (units: pseudo_N)
        contact_thresh_pos: positive threshold to stop motion (units: effort_pct -100:100)
        contact_thresh_neg: negative threshold to stop motion (units: effort_pct -100:100)
        req_calibration : bool
            whether to allow motion prior to homing
        move_to_start_point : bool
            whether to move to the trajectory's start to avoid a jump, this
            time to move doesn't count against the trajectory's timeline
        """


        hu.check_deprecated_contact_model_prismatic_joint(self,'follow_trajectory', contact_thresh_pos_N, contact_thresh_neg_N,contact_thresh_pos, contact_thresh_neg)

        # check if joint valid, homed, and right protocol
        if not self.motor.hw_valid:
            self.logger.warning('%s connection to hardware not valid'%self.name.upper)
            return False
        if req_calibration:
            if not self.motor.status['pos_calibrated']:
                self.logger.warning('%s not calibrated'%self.name.capitalize())
                return False
        if int(str(self.motor.board_info['protocol_version'])[1:]) < 1:
            self.logger.warning("%s firmware version doesn't support waypoint trajectories"%self.name.capitalize())
            return False

        # check if trajectory valid
        vel_limit = v_m if v_m is not None else self.params['motion']['trajectory_max']['vel_m']
        acc_limit = a_m if a_m is not None else self.params['motion']['trajectory_max']['accel_m']
        valid, reason = self.trajectory.is_valid(vel_limit, acc_limit)
        if not valid:
            self.logger.warning('Joint traj not valid: {0}'.format(reason))
            return False
        if valid and reason == "must have at least two waypoints":
            # skip this device
            return True

        # set defaults
        stiffness = max(0.0, min(1.0, stiffness)) if stiffness is not None else self.stiffness
        v_r = self.translate_m_to_motor_rad(min(abs(v_m), self.params['motion']['trajectory_max']['vel_m'])) \
            if v_m is not None else self.translate_m_to_motor_rad(self.params['motion']['trajectory_max']['vel_m'])
        a_r = self.translate_m_to_motor_rad(min(abs(a_m), self.params['motion']['trajectory_max']['accel_m'])) \
            if a_m is not None else self.translate_m_to_motor_rad(self.params['motion']['trajectory_max']['accel_m'])

        i_contact_pos,i_contact_neg  = self.contact_thresh_to_motor_current(contact_thresh_pos,contact_thresh_neg )


        # move to start point
        if move_to_start_point:
            prev_sync_mode = self.motor.gains['enable_sync_mode']
            self.move_to(self.trajectory[0].position)
            self.motor.disable_sync_mode()
            self.push_command()
            if not self.motor.wait_until_at_setpoint():
                self.logger.warning('%f unable to reach starting point'%self.name.capitalize())
            if prev_sync_mode:
                self.motor.enable_sync_mode()
                self.push_command()
            #print('WAIT')
            #time.sleep(2.0)

        self.traj_guarded_event=self.motor.status['guarded_event']
        # start trajectory
        self.motor.set_command(mode=Stepper.MODE_POS_TRAJ_WAYPOINT,
                               v_des=v_r,
                               a_des=a_r,
                               stiffness=stiffness,
                               i_feedforward=self.i_feedforward,
                               i_contact_pos=i_contact_pos,
                               i_contact_neg=i_contact_neg)
        self.motor.push_command()
        s0 = self.trajectory.get_segment(0, to_motor_rad=self.translate_m_to_motor_rad).to_array()
        return self.motor.start_waypoint_trajectory(s0)

    def is_trajectory_active(self):
        return self.motor.status['waypoint_traj']['state'] == 'active'

    def get_trajectory_ts(self):
        # Return trajectory execution time
        if self.is_trajectory_active() and self.motor._waypoint_ts is not None:
            return time.time()-self.motor._waypoint_ts
        elif len(self.trajectory.waypoints):
            return self.trajectory.waypoints[-1].time
        else:
            return 0

    def get_trajectory_time_remaining(self):
        if not self.is_trajectory_active():
            return 0
        else:
            return max(0,self.trajectory.waypoints[-1].time - self.get_trajectory_ts())

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
        if self.motor.status['mode'] != self.motor.MODE_POS_TRAJ_WAYPOINT:
            return

        if self.traj_guarded_event!=self.motor.status['guarded_event']:
            self.logger.warning('%s guarded contact. Stopping trajectory'%self.name.upper())
            self.stop_trajectory()
            return

        if self.motor.status['waypoint_traj']['state'] == 'active':
            next_segment_id = self.motor.status['waypoint_traj']['segment_id'] - 2 + 1 # subtract 2 due to IDs 0 & 1 being reserved by firmware
            if next_segment_id < self.trajectory.get_num_segments():
                s1 = self.trajectory.get_segment(next_segment_id, to_motor_rad=self.translate_m_to_motor_rad).to_array()
                self.motor.set_next_trajectory_segment(s1)
        elif self.motor.status['waypoint_traj']['state'] == 'idle' and self.motor.status['mode'] == Stepper.MODE_POS_TRAJ_WAYPOINT:
            self.motor.enable_pos_traj()
            self.push_command()

    def stop_trajectory(self):
        """Stop waypoint trajectory immediately and resets hardware
        """
        self.motor.stop_waypoint_trajectory()

    # ######### Utility ##############################


    def motor_current_to_translate_force(self,i):
        raise DeprecationWarning('Method motor_current_to_translate_force has been deprecated since v0.3.5')

    def translate_force_to_motor_current(self,f):
        raise DeprecationWarning('Method translate_force_to_motor_current has been deprecated since v0.3.5')

    def motor_rad_to_translate_m(self,ang): #Override
        self.logger.warning('motor_rad_to_translate_m not implemented in %s'%self.name)
        pass

    def translate_m_to_motor_rad(self, x):#Override
        self.logger.warning('motor_rad_to_translate_m not implemented in %s' % self.name)
        pass

    def wait_until_at_setpoint(self, timeout=15.0):
        self.motor.wait_until_at_setpoint(timeout)

    def wait_for_contact(self, timeout=5.0):
        ts=time.time()
        while (time.time()-ts<timeout):
            self.pull_status()
            if self.motor.status['in_guarded_event']:
                return True
            time.sleep(0.01)
        return False

    def step_sentry(self,robot):
        self.motor.step_sentry(robot)


    def home(self,end_pos,to_positive_stop, measuring=False):
        """
        end_pos: position to end on
        to_positive_stop:
        -- True: Move to the positive direction stop and mark to range_m[1]
        -- False: Move to the negative direction stop and mark to range_m[0]
        measuring: After homing to stop, move to opposite stop and report back measured distance
        return measured range-of-motion if measuring. Return None if not a valide measurement
        """

        hu.check_deprecated_contact_model_prismatic_joint(self,'home',None,None,None,None)

        if not self.motor.hw_valid:
            self.logger.warning('Not able to home %s. Hardware not present' % self.name.capitalize())
            return None


        contact_thresh_neg = self.params['contact_models']['effort_pct']['contact_thresh_homing'][0]
        contact_thresh_pos = self.params['contact_models']['effort_pct']['contact_thresh_homing'][1]


        success = True
        print('Homing %s...' % self.name.capitalize())
        self.pull_status()
        prev_calibrated=self.motor.status['pos_calibrated']
        prev_guarded_mode = self.motor.gains['enable_guarded_mode']
        prev_sync_mode = self.motor.gains['enable_sync_mode']
        self.motor.enable_guarded_mode()
        self.motor.disable_sync_mode()

        self.motor.reset_pos_calibrated()
        self.push_command()
        self.pull_status()

        if to_positive_stop:
            x_goal_1 = 5.0  # Well past the stop
            x_goal_2 = -5.0
        else:
            x_goal_1 = -5.0
            x_goal_2 = 5.0

        # Move to stop
        self.move_by(x_m=x_goal_1, contact_thresh_pos=contact_thresh_pos, contact_thresh_neg=contact_thresh_neg,req_calibration=False)
        self.push_command()
        if self.wait_for_contact(timeout=15.0):  # timeout=15.0):
            self.pull_status()
            print('Hardstop detected at motor position (rad)', self.motor.status['pos'])
            x_dir_1 = self.status['pos']
            if to_positive_stop:
                x = self.translate_m_to_motor_rad(self.params['range_m'][1] )
                print('Marking %s position to %f (m)' % (self.name.capitalize(), self.params['range_m'][1]))
            else:
                x = self.translate_m_to_motor_rad(self.params['range_m'][0])
            if not measuring:
                print('Marking %s position to %f (m)' % (self.name.capitalize(), self.params['range_m'][0]))
                self.motor.mark_position(x)
                self.motor.set_pos_calibrated()
                self.push_command()
            time.sleep(1.0)  # Allow to settle
            # Second direction
            if measuring:
                # Move to other direction
                self.pull_status()
                self.move_by(x_m=x_goal_2, contact_thresh_pos=contact_thresh_pos,contact_thresh_neg=contact_thresh_neg,req_calibration=False)
                self.push_command()
                if self.wait_for_contact(timeout=15.0):
                    print('Second hardstop detected at motor position (rad)', self.motor.status['pos'])
                    time.sleep(1.0)
                    self.pull_status()
                    x_dir_2 = self.status['pos']
                else:
                    self.logger.warning('%s homing failed. Failed to detect contact' % self.name.capitalize())
                    success = False
        else:
            self.logger.warning('%s homing failed. Failed to detect contact' % self.name.capitalize())
            success = False

        if success:
            self.move_to(x_m=end_pos, req_calibration=False)
            self.push_command()
            time.sleep(1.0)
            if not self.motor.wait_until_at_setpoint():
                self.logger.warning('%s failed to reach final position' % self.name.capitalize())
                success = False

        # Restore previous modes
        if not prev_guarded_mode:
            self.motor.disable_guarded_mode()
        if prev_sync_mode:
            self.motor.enable_sync_mode()
        if measuring and prev_calibrated:
            self.motor.set_pos_calibrated()
        self.push_command()
        if success:
            if measuring:
                print('%s range measuing successful: %f (m)' % (self.name.capitalize(), abs(x_dir_1 - x_dir_2)))
                return abs(x_dir_1 - x_dir_2)
            else:
                print('%s homing successful' % self.name.capitalize())
        return None