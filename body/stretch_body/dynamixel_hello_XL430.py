from __future__ import print_function
from stretch_body.dynamixel_XL430 import *
from stretch_body.device import Device
import time
from stretch_body.hello_utils import *
import termios
import numpy

class DynamixelCommErrorStats(Device):
    def __init__(self, name, logger):
        Device.__init__(self, name='dxl_comm_errors')
        self.name=name
        self.status={'n_rx':0, 'n_tx':0, 'n_gsr': 0, 'error_rate_avg_hz':0}
        self.rate_log=None
        self.n_log=10
        self.log_idx =0
        self.ts_error_last=time.time()
        self.ts_warn_last=time.time()
        self.logger=logger

    def add_error(self,rx=True, gsr=False):
        t = time.time()
        if type(self.rate_log)==type(None): #First error
            self.rate_log=numpy.array([0.0] * self.n_log)
        self.rate_log[self.log_idx]=1/(t-self.ts_error_last)
        self.log_idx = (self.log_idx + 1) % self.n_log
        self.status['error_rate_avg_hz'] = numpy.average(self.rate_log)
        if rx:
            self.status['n_rx']+=1
        else:
            self.status['n_tx'] += 1
        if gsr:
            self.status['n_gsr'] += 1
        if t-self.ts_warn_last>self.params['warn_every_s']:
            self.ts_warn_last=t
            if self.status['error_rate_avg_hz']>self.params['warn_above_rate']:
                self.logger.warning('Device %s generating %f errors per minute'%(self.name,(self.status['error_rate_avg_hz']*60)))
        if self.params['verbose']:
            self.pretty_print()

    def pretty_print(self):
        print('---- Dynamixel Comm Errors %s ----'%self.name)
        print('Rate (Hz): %f' % self.status['error_rate_avg_hz'])
        print('Rate (errors per minute): %f'%(self.status['error_rate_avg_hz']*60))
        print('Num TX: %f'%self.status['n_tx'])
        print('Num RX: %f' % self.status['n_rx'])
        print('Num Group Sync RX: %f' % self.status['n_gsr'])



class DynamixelHelloXL430(Device):
    """
    Abstract the Dynamixel X-Series to handle calibration, radians, etc
    """
    def __init__(self, name, chain=None):
        Device.__init__(self, name)
        self.chain = chain
        self.status={'timestamp_pc':0,'comm_errors':0,'pos':0,'vel':0,'effort':0,'temp':0,'shutdown':0, 'hardware_error':0,
                     'input_voltage_error':0,'overheating_error':0,'motor_encoder_error':0,'electrical_shock_error':0,'overload_error':0,
                     'stalled':0,'stall_overload':0,'pos_ticks':0,'vel_ticks':0,'effort_ticks':0}

        #Share bus resource amongst many XL430s
        self.motor = DynamixelXL430(dxl_id=self.params['id'],
                                    usb=self.params['usb_name'],
                                    port_handler=None if chain is None else chain.port_handler,
                                    pt_lock=None if chain is None else chain.pt_lock,
                                    baud=self.params['baud'],
                                    logger=self.logger)
        self.polarity = -1.0 if self.params['flip_encoder_polarity'] else 1.0
        self.ts_over_eff_start=None
        self.hw_valid=False
        self.is_calibrated=False


        if self.params['flip_encoder_polarity']:
            wr_max = self.ticks_to_world_rad(self.params['range_t'][0])
            wr_min = self.ticks_to_world_rad(self.params['range_t'][1])
        else:
            wr_max = self.ticks_to_world_rad(self.params['range_t'][1])
            wr_min = self.ticks_to_world_rad(self.params['range_t'][0])
        self.soft_motion_limits = {'collision': [None, None], 'user': [None, None],'hard': [wr_min,wr_max],'current': [wr_min,wr_max]}

        self.is_homing=False
        self.status_mux_id = 0
        self.was_runstopped = False
        self.comm_errors = DynamixelCommErrorStats(name,logger=self.logger)

    # ###########  Device Methods #############

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
        self.soft_motion_limits['current'][0]=max(filter(lambda x: x is not None, [self.soft_motion_limits['collision'][0],self.soft_motion_limits['hard'][0],self.soft_motion_limits['user'][0]]))


    def set_soft_motion_limit_max(self,x,limit_type='user' ):
        """
        x: value to set a joints limit to
        limit_type: 'user' or 'collision'
        """
        self.soft_motion_limits[limit_type][1]=x
        self.soft_motion_limits['current'][1]=min(filter(lambda x: x is not None, [self.soft_motion_limits['collision'][1],self.soft_motion_limits['hard'][1],self.soft_motion_limits['user'][1]]))

    # ###################################################

    def do_ping(self, verbose=False):
        return self.motor.do_ping(verbose)

    def startup(self):
        if self.motor.do_ping(verbose=False):
            self.hw_valid = True
            self.motor.disable_torque()
            if self.params['use_multiturn']:
                self.motor.enable_multiturn()
            else:
                self.motor.enable_pos()
                if self.params['range_t'][0]<0 or self.params['range_t'][1]>4095:
                    self.logger.warning('Warning: Invalid position range for %s'%self.name)
            self.motor.set_pwm_limit(self.params['pwm_limit'])
            self.motor.set_temperature_limit(self.params['temperature_limit'])
            self.motor.set_min_voltage_limit(self.params['min_voltage_limit'])
            self.motor.set_max_voltage_limit(self.params['max_voltage_limit'])
            self.motor.set_P_gain(self.params['pid'][0])
            self.motor.set_I_gain(self.params['pid'][1])
            self.motor.set_D_gain(self.params['pid'][2])
            self.motor.set_return_delay_time(self.params['return_delay_time'])
            self.motor.set_profile_velocity(self.rad_per_sec_to_ticks(self.params['motion']['default']['vel']))
            self.motor.set_profile_acceleration(self.rad_per_sec_sec_to_ticks(self.params['motion']['default']['accel']))
            self.v_des=self.params['motion']['default']['vel']
            self.a_des=self.params['motion']['default']['accel']
            self.is_calibrated=self.motor.is_calibrated()
            self.enable_torque()
            return True
        else:
            self.logger.warning('DynamixelHelloXL430 Ping failed... %s' % self.name)
            print('DynamixelHelloXL430 Ping failed...', self.name)
            return False


    def stop(self):
        if self.hw_valid:
            self.disable_torque()
            self.hw_valid = False

    def pull_status(self,data=None):
        if not self.hw_valid:
            return

        pos_valid = True
        vel_valid = True
        eff_valid = True
        temp_valid = True
        err_valid = True

        #First pull new data from servo
        #Or bring in data from a synchronized read
        if data is None:
            try:
                x = self.motor.get_pos()
                if not self.motor.last_comm_success and self.params['retry_on_comm_failure']:
                    x = self.motor.get_pos()
                pos_valid = self.motor.last_comm_success

                v = self.motor.get_vel()
                if not self.motor.last_comm_success and self.params['retry_on_comm_failure']:
                    v = self.motor.get_vel()
                vel_valid = self.motor.last_comm_success

                if self.status_mux_id == 0:
                    eff = self.motor.get_load()
                    if not self.motor.last_comm_success and self.params['retry_on_comm_failure']:
                        eff = self.motor.get_load()
                    eff_valid = self.motor.last_comm_success
                else:
                    eff = self.status['effort_ticks']

                if self.status_mux_id == 1:
                    temp = self.motor.get_temp()
                    if not self.motor.last_comm_success and self.params['retry_on_comm_failure']:
                        temp = self.motor.get_temp()
                    temp_valid = self.motor.last_comm_success
                else:
                    temp = self.status['temp']

                if self.status_mux_id == 2:
                    err = self.motor.get_hardware_error()
                    if not self.motor.last_comm_success and self.params['retry_on_comm_failure']:
                        err = self.motor.get_hardware_error()
                    err_valid = self.motor.last_comm_success
                else:
                    err = self.status['hardware_error']

                self.status_mux_id = (self.status_mux_id + 1) % 3


                if not pos_valid or not vel_valid or not eff_valid or not temp_valid or not err_valid:
                    raise DynamixelCommError
                ts = time.time()
            except(termios.error, DynamixelCommError):
                #self.logger.warning('Dynamixel communication error on %s: '%self.name)
                self.port_handler.ser.reset_output_buffer()
                self.port_handler.ser.reset_input_buffer()
                self.comm_errors.add_error(rx=True,gsr=False)
                return
        else:
            x = data['x']
            v = data['v']
            eff = data['eff']
            temp = data['temp']
            ts = data['ts']
            err = data['err']

        #Now update status dictionary
        if pos_valid:
            self.status['pos_ticks'] = x
            self.status['pos'] = self.ticks_to_world_rad(float(x))
        if vel_valid:
            self.status['vel_ticks'] = v
            self.status['vel'] = self.ticks_to_rad_per_sec(float(v))
        if eff_valid:
            self.status['effort_ticks'] = eff
            self.status['effort'] = self.ticks_to_pct_load(float(eff))
        if temp_valid:
            self.status['temp'] = float(temp)
        if err_valid:
            self.status['hardware_error'] = err

        self.status['timestamp_pc'] = ts

        self.status['hardware_error'] = err
        self.status['input_voltage_error'] = self.status['hardware_error'] & 1 != 0
        self.status['overheating_error'] = self.status['hardware_error'] & 4 != 0
        self.status['motor_encoder_error'] = self.status['hardware_error'] & 8 != 0
        self.status['electrical_shock_error'] = self.status['hardware_error'] & 16 != 0
        self.status['overload_error'] = self.status['hardware_error'] & 32 != 0

        #Finally flag if stalled at high effort for too long
        self.status['stalled']=abs(self.status['vel'])<self.params['stall_min_vel']
        over_eff=abs(self.status['effort']) > self.params['stall_max_effort']

        if self.status['stalled']:
            if not over_eff:
                self.ts_over_eff_start = None
            if over_eff and self.ts_over_eff_start is None: #Mark the start of being stalled and over-effort
                self.ts_over_eff_start = time.time()
            if self.ts_over_eff_start is not None and time.time()-self.ts_over_eff_start>self.params['stall_max_time']:
                self.status['stall_overload'] = True
            else:
                self.status['stall_overload'] = False
        else:
            self.ts_over_eff_start=None
            self.status['stall_overload'] = False

    def mark_zero(self):
        if not self.hw_valid:
            return
        x=self.motor.get_pos()
        print('Marking current position of (ticks)',x,' as 0 (rad)')
        self.params['zero_t']=x
        self.write_device_params(self.name, self.params)



    def pretty_print(self):
        if not self.hw_valid:
            print('----- HelloXL430 ------ ')
            print('Servo not on bus')
            return
        print('----- HelloXL430 ------ ')
        print('Name',self.name)
        print('Position (rad)', self.status['pos'])
        print('Position (deg)', rad_to_deg(self.status['pos']))
        print('Position (ticks)', self.status['pos_ticks'])
        print('Velocity (rad/s)', self.status['vel'])
        print('Velocity (ticks/s)', self.status['vel_ticks'])
        print('Effort (%)', self.status['effort'])
        print('Effort (ticks)', self.status['effort_ticks'])
        print('Temp', self.status['temp'])
        print('Comm Errors', self.motor.comm_errors)
        print('Hardware Error', self.status['hardware_error'])
        print('Hardware Error: Input Voltage Error: ',self.status['input_voltage_error'])
        print('Hardware Error: Overheating Error: ', self.status['overheating_error'])
        print('Hardware Error: Motor Encoder Error: ',self.status['motor_encoder_error'])
        print('Hardware Error: Electrical Shock Error: ', self.status['electrical_shock_error'])
        print('Hardware Error: Overload Error: ', self.status['overload_error'])
        print('Timestamp PC', self.status['timestamp_pc'])
        print('Range (ticks)',self.params['range_t'])
        print('Range (rad) [',self.ticks_to_world_rad(self.params['range_t'][0]), ' , ',self.ticks_to_world_rad(self.params['range_t'][1]),']')
        print('Stalled',self.status['stalled'])
        print('Stall Overload',self.status['stall_overload'])
        print('Is Calibrated',self.is_calibrated)
        #self.motor.pretty_print()

    def step_sentry(self, robot):

        if self.hw_valid and self.robot_params['robot_sentry']['dynamixel_stop_on_runstop'] and self.params['enable_runstop']:
            is_runstopped = robot.pimu.status['runstop_event']
            if is_runstopped is not self.was_runstopped:
                if is_runstopped:
                    self.disable_torque()
                else:
                    self.enable_torque()
            self.was_runstopped = is_runstopped

    # #####################################

    def reboot(self):
        if not self.hw_valid:
            return
        self.motor.do_reboot()

    def enable_torque(self):
        if not self.hw_valid:
            return
        self.motor.enable_torque()

    def disable_torque(self):
        if not self.hw_valid:
            return
        self.motor.disable_torque()

    def move_to(self,x_des, v_des=None, a_des=None):
        if not self.hw_valid:
            return
        if self.params['req_calibration'] and not self.is_calibrated:
            self.logger.warning('Dynamixel not calibrated: %s' % self.name)
            print('Dynamixel not calibrated:', self.name)
            return
        try:
            self.set_motion_params(v_des,a_des)
            x_des = min(max(self.get_soft_motion_limits()[0], x_des), self.get_soft_motion_limits()[1])
            t_des = self.world_rad_to_ticks(x_des)
            t_des = max(self.params['range_t'][0], min(self.params['range_t'][1], t_des))
            self.motor.go_to_pos(t_des)
        except (termios.error, DynamixelCommError):
            #self.logger.warning('Dynamixel communication error on: %s' % self.name)
            self.comm_errors.add_error(rx=False, gsr=False)


    def set_motion_params(self,v_des=None,a_des=None):
        try:
            if not self.hw_valid:
                return

            v_des = v_des if v_des is not None else self.params['motion']['default']['vel']
            v_des = min(self.params['motion']['max']['vel'], v_des)
            if v_des != self.v_des:
                self.motor.set_profile_velocity(self.rad_per_sec_to_ticks(v_des))
                self.v_des = v_des

            a_des = a_des if a_des is not None else self.params['motion']['default']['accel']
            a_des = min(self.params['motion']['max']['accel'], a_des)
            if a_des != self.a_des:
                self.motor.set_profile_acceleration(self.rad_per_sec_sec_to_ticks(a_des))
                self.a_des = a_des
        except (termios.error, DynamixelCommError):
            #self.logger.warning('Dynamixel communication error on: %s' % self.name)
            self.comm_errors.add_error(rx=False, gsr=False)


    def move_by(self,x_des, v_des=None, a_des=None):
        if not self.hw_valid:
            return
        try:
            if abs(x_des) > 0.00002: #Avoid drift
                x=self.motor.get_pos()
                if not self.motor.last_comm_success and self.params['retry_on_comm_failure']:
                    x = self.motor.get_pos()

                if self.motor.last_comm_success:
                    cx=self.ticks_to_world_rad(x)
                    self.move_to(cx + x_des, v_des, a_des)
                else:
                    self.logger.debug('Move_By comm failure on %s' % self.name)
        except (termios.error, DynamixelCommError):
            #self.logger.warning('Dynamixel communication error on: %s' % self.name)
            self.comm_errors.add_error(rx=False, gsr=False)

    def quick_stop(self):
        if not self.hw_valid:
            return
        try:
            self.motor.disable_torque()
            self.motor.enable_torque()
        except (termios.error, DynamixelCommError):
            self.comm_errors.add_error(rx=False, gsr=False)

    def enable_pos(self):
        if not self.hw_valid:
            return
        try:
            self.motor.disable_torque()
            if self.params['use_multiturn']:
                self.motor.enable_multiturn()
            else:
                self.motor.enable_pos()
            self.motor.set_profile_velocity(self.rad_per_sec_to_ticks(self.v_des))
            self.motor.set_profile_acceleration(self.rad_per_sec_sec_to_ticks(self.a_des))
            self.motor.enable_torque()
        except (termios.error, DynamixelCommError):
            self.comm_errors.add_error(rx=False, gsr=False)

    def enable_pwm(self):
        if not self.hw_valid:
            return
        try:
            self.motor.disable_torque()
            self.motor.enable_pwm()
            self.motor.enable_torque()
        except (termios.error, DynamixelCommError):
            self.comm_errors.add_error(rx=False, gsr=False)


    def set_pwm(self,x):
        if not self.hw_valid:
            return
        try:
            self.motor.set_pwm(x)
        except (termios.error, DynamixelCommError):
            self.comm_errors.add_error(rx=False, gsr=False)

# ##########################################
    """
    Servo calibration works by:
    
    Homing
    ==============
    * Move with a fixed PWM to a hardstop
    * Store the current position (ticks) in the EEPROM homing offset such that self.motor.status['pos']==0 at that hardstop
    
    Calibration to SI / joint range:
    ===============
    Once homed, the servo can move to positions within self.params.range_t (ticks).
    The 'Joint Zero' (self.params.zero_t) is different than the 'Servo Zero'. Per the homing procedure, Servo Zero is at one hardstop
    Joint Zero is an offset (ticks) from Servo Zero that is added/subtracted to commanded/reported values
    Thus when self.motor.status['pos']==self.params.zero_t, self.status['pos]=0 (rad)
    
    There is an added complexity with how Dynamixels handle position reporting in multi-turn and single-turn mode
    In multi-turn mode, position values and homing values can be large (+/-2B, +/-1M)
    In single-turn mode, position values are bound to 0-4095 and homing values to -1,024 ~ 1,024
    In addition, the home offset stored during a multi-turn homing procedure may not be valid in single turn (out of range)
    
    The Home Offset(20) adjusts the home position. The offest value is added to the Present Position(132).
    Present Position(132) = Actual Position + Homing Offset(20)
    Range: -1,044,479 ~ 1,044,479
    
    ### NOTE ###
    In case of the Position Control Mode(Joint Mode) that rotates less than 360 degrees, any invalid Homing Offset(20) values will be ignored(valid range : -1,024 ~ 1,024).
    So when homing in single-turn mode, ensure that the homing offset stored is in range, otherwise unexpected values will appear. 
    -- It is possible then to not be able to home a single-turn servo as position can range 0-4096 and depending on hardstop location, a valid homing offset can not be found
    -- Also, homing shouldn't be necessary in single-turn mode as the position reporting is already absolute
    """

    def home(self, single_stop=False, move_to_zero=True,delay_at_stop=0.0,save_calibration=False):
        # Requires at least one hardstop in the pwm_homing[0] direction
        # Can be in multiturn or single turn mode
        # Mark the first hardstop as zero ticks on the Dynammixel
        # Second hardstop is optional

        if not self.hw_valid:
            self.logger.warning('Not able to home %s. Hardware not present'%self.name)
            return
        if not self.params['req_calibration']:
            print('Homing not required for: '+self.name)
            return

        self.pull_status()
        if self.status['overload_error'] or self.status['overheating_error']:
            self.logger.warning('Hardware error, unable to home. Exiting')
            return

        self.is_homing=True
        self.enable_pwm()

        print('Moving to first hardstop...')
        self.set_pwm(self.params['pwm_homing'][0])
        ts=time.time()
        time.sleep(1.0)
        timeout=False
        while self.motor.is_moving() and not timeout:
            timeout=time.time()-ts>15.0
            time.sleep(0.5)
        time.sleep(delay_at_stop)
        self.set_pwm(0)

        if timeout:
            self.logger.warning('Timed out moving to first hardstop. Exiting.')
            return
        if self.status['overload_error'] or self.status['overheating_error']:
            self.logger.warning('Hardware error, unable to home. Exiting')
            return

        print('Contact at position: %d'%self.motor.get_pos())
        print('Hit first hardstop, marking to zero ticks')
        self.motor.disable_torque()
        self.motor.zero_position()
        print("Homing is now  %d"%self.motor.get_homing_offset())

        self.motor.disable_torque()
        self.motor.set_calibrated(1)
        self.is_calibrated=1
        self.motor.enable_torque()

        self.enable_pwm()

        print('Raw position:',self.motor.get_pos())
        if not single_stop:
            #Measure the range and write to YAML
            print('Moving to second hardstop...')
            self.set_pwm(self.params['pwm_homing'][1])
            ts = time.time()
            time.sleep(1.0)
            timeout = False
            while self.motor.is_moving() and not timeout:
                timeout = time.time() - ts > 15.0
                time.sleep(0.5)
            time.sleep(delay_at_stop)
            self.set_pwm(0)

            if timeout:
                self.logger.warning('Timed out moving to second hardstop. Exiting.')
                return
            if self.status['overload_error'] or self.status['overheating_error']:
                self.logger.warning('Hardware error, unable to home. Exiting')
                return

            x_dir_1 = self.motor.get_pos()
            print('Hit second hardstop at: ', x_dir_1)
            self.params['range_t']=[0,x_dir_1]
            print('Homed to range of:')
            print('   Ticks:',self.params['range_t'])
            r1=self.ticks_to_world_rad(0)
            r2=self.ticks_to_world_rad(x_dir_1)
            print('   Radians:',[r1,r2])
            print('   Degrees:',[rad_to_deg(r1),rad_to_deg(r2)])

        if save_calibration:
            self.write_device_params(self.name, self.params)

        self.enable_pos()
        if move_to_zero:
            print('Moving to calibrated zero: (rad)')
            self.move_to(0)
            time.sleep(3.0)
        self.is_homing=False

# ##########################################

    def ticks_to_world_rad_per_sec(self,t):
        rps_servo=self.ticks_to_rad_per_sec(t)
        return self.polarity*rps_servo/self.params['gr']

    def ticks_to_world_rad(self,t):
        t=t-self.params['zero_t']
        rad_servo = self.ticks_to_rad(t)
        return (self.polarity*rad_servo/self.params['gr'])

    def world_rad_to_ticks(self,r):
        rad_servo = r*self.params['gr']*self.polarity
        t= self.rad_to_ticks(rad_servo)
        return t+self.params['zero_t']

    def world_rad_to_ticks_per_sec(self,r):
        rad_per_sec_servo = r*self.params['gr']*self.polarity
        t= self.rad_per_sec_to_ticks(rad_per_sec_servo)
        return t


    def ticks_to_rad(self,t):
        return deg_to_rad((360.0 * t / 4096.0))

    def rad_to_ticks(self,r):
        return int( 4096.0 * rad_to_deg(r) / 360.0)

    def ticks_to_rad_per_sec(self,t):
        rpm= t*0.229
        return deg_to_rad(rpm*360/60.0)

    def rad_per_sec_to_ticks(self,r):
        rpm=rad_to_deg(r)*60/360.0
        return int(rpm/0.229)

    def ticks_to_rad_per_sec_sec(self,t):
        rpmm=t*214.577
        return deg_to_rad(rpmm*360/60.0/60.0)

    def rad_per_sec_sec_to_ticks(self,r):
        rpmm=rad_to_deg(r)*60*60/360.0
        return int(rpmm/214.577)

    def ticks_to_pct_load(self,t):
        #-100 to 100.0
        return t/10.24
