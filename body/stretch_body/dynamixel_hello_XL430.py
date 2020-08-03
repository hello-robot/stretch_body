
from dynamixel_XL430 import *
from stretch_body.device import Device
import time
from stretch_body.hello_utils import *
import termios

class DynamixelHelloXL430(Device):
    """
    Abstract the Dynamixel X-Series to handle calibration, radians, etc
    """
    def __init__(self,name, chain=None):
        Device.__init__(self)
        self.name=name
        self.chain=None
        self.params=self.robot_params[self.name]
        self.status={'timestamp_pc':0,'comm_errors':0,'pos':0,'vel':0,'effort':0,'temp':0,'shutdown':0, 'hardware_error':0,
                     'input_voltage_error':0,'overheating_error':0,'motor_encoder_error':0,'electrical_shock_error':0,'overload_error':0,
                     'stalled':0,'stall_overload':0,'pos_ticks':0,'vel_ticks':0,'effort_ticks':0}
        #Share bus resource amongst many XL430s
        if chain is None:
            self.motor = DynamixelXL430(dxl_id=self.params['id'],usb=self.params['usb_name'],port_handler=None)
        else:
            self.motor = DynamixelXL430(dxl_id=self.params['id'], usb=self.params['usb_name'], port_handler=chain.port_handler, pt_lock=chain.pt_lock)
        if self.params['flip_encoder_polarity']:
            self.polarity=-1.0
        else:
            self.polarity=1.0
        self.ts_over_eff_start=None
        self.servo_valid=False
        self.is_calibrated=False

    # ###########  Device Methods #############

    def do_ping(self, verbose):
        return self.motor.do_ping(verbose)

    def startup(self):
        if self.motor.do_ping(verbose=False):
            self.servo_valid = True
            self.motor.disable_torque()
            if self.params['use_multiturn']:
                self.motor.enable_multiturn()
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
        else:
            print 'DynamixelHelloXL430 Ping failed...', self.name


    def stop(self):
        if self.servo_valid:
            self.disable_torque()

    def pull_status(self,data=None):
        if not self.servo_valid:
            return
        #First pull new data from servo
        #Or bring in data from a synchronized read
        if data is None:
            try:
                x = self.motor.get_pos()
                v = self.motor.get_vel()
                eff = self.motor.get_load()
                temp = self.motor.get_temp()
                ts = time.time()
                err=self.motor.get_hardware_error()
            except termios.error:
                print 'Dynamixel communiation error at time: ',time.time()
        else:
            x = data['x']
            v = data['v']
            eff = data['eff']
            temp = data['temp']
            ts = data['ts']
            err = data['err']

        #Now update status dictionary
        self.status['pos_ticks'] = x
        self.status['vel_ticks'] = v
        self.status['effort_ticks'] = eff
        self.status['pos'] = self.ticks_to_world_rad(float(x))
        self.status['vel'] = self.ticks_to_rad_per_sec(float(v))
        self.status['effort'] = self.ticks_to_pct_load(float(eff))
        self.status['temp'] = float(temp)
        self.status['timestamp_pc'] = ts
        self.status['hardware_error'] = err
        self.status['input_voltage_error'] = self.status['hardware_error'] & 1 is not 0
        self.status['overheating_error'] = self.status['hardware_error'] & 4 is not 0
        self.status['motor_encoder_error'] = self.status['hardware_error'] & 8 is not 0
        self.status['electrical_shock_error'] = self.status['hardware_error'] & 16 is not 0
        self.status['overload_error'] = self.status['hardware_error'] & 32 is not 0

        #Finally flag if stalled at high effort for too long
        self.status['stalled']=abs(self.status['vel'])<self.params['stall_min_vel']
        over_eff=abs(self.status['effort']) > self.params['stall_max_effort']

        if self.status['stalled']:
            if not over_eff:
                self.ts_over_eff_start = None
            if over_eff and self.ts_over_eff_start is None: #Mark the start of being stall and over-effort
                self.ts_over_eff_start = time.time()
            if self.ts_over_eff_start is not None and time.time()-self.ts_over_eff_start>self.params['stall_max_time']:
                self.status['stall_overload'] = True
            else:
                self.status['stall_overload'] = False
        else:
            self.ts_over_eff_start=None
            self.status['stall_overload'] = False

    def mark_zero(self):
        if not self.servo_valid:
            return
        x=self.motor.get_pos()
        print 'Marking current position of (ticks)',x,' as 0 (rad)'
        self.params['zero_t']=x
        self.write_device_params(self.name, self.params)



    def pretty_print(self):
        if not self.servo_valid:
            print '----- HelloXL430 ------ '
            print 'Servo not on bus'
            return
        print '----- HelloXL430 ------ '
        print 'Name',self.name
        print 'Position (rad)', self.status['pos']
        print 'Position (deg)', rad_to_deg(self.status['pos'])
        print 'Position (ticks)', self.status['pos_ticks']
        print 'Velocity (rad/s)', self.status['vel']
        print 'Velocity (ticks/s)', self.status['vel_ticks']
        print 'Effort (%)', self.status['effort']
        print 'Effort (ticks)', self.status['effort_ticks']
        print 'Temp', self.status['temp']
        print 'Comm Errors', self.motor.comm_errors
        print 'Hardware Error', self.status['hardware_error']
        print 'Hardware Error: Input Voltage Error: ',self.status['input_voltage_error']
        print 'Hardware Error: Overheating Error: ', self.status['overheating_error']
        print 'Hardware Error: Motor Encoder Error: ',self.status['motor_encoder_error']
        print 'Hardware Error: Electrical Shock Error: ', self.status['electrical_shock_error']
        print 'Hardware Error: Overload Error: ', self.status['overload_error']
        print 'Timestamp PC', self.status['timestamp_pc']
        print 'Range (ticks)',self.params['range_t']
        print 'Range (rad) [',self.ticks_to_world_rad(self.params['range_t'][0]), ' , ',self.ticks_to_world_rad(self.params['range_t'][1]),']'
        print 'Stalled',self.status['stalled']
        print 'Stall Overload',self.status['stall_overload']
        print 'Is Calibrated',self.is_calibrated
        #self.motor.pretty_print()

    # #####################################

    def reboot(self):
        if not self.servo_valid:
            return
        self.motor.do_reboot()

    def enable_torque(self):
        if not self.servo_valid:
            return
        self.motor.enable_torque()

    def disable_torque(self):
        if not self.servo_valid:
            return
        self.motor.disable_torque()

    def move_to(self,x_des, v_des=None, a_des=None):
        if not self.servo_valid:
            return
        if self.params['req_calibration'] and not self.is_calibrated:
            print 'Dynamixel not calibrated:', self.name
            return
        try:
            self.set_motion_params(v_des,a_des)
            t_des = self.world_rad_to_ticks(x_des)
            t_des = max(self.params['range_t'][0], min(self.params['range_t'][1], t_des))
            self.motor.go_to_pos(t_des)
        except termios.error:
            print 'Dynamixel communication error at time: ',time.time()


    def set_motion_params(self,v_des=None,a_des=None):
        if not self.servo_valid:
            return
        if v_des is not None:
            v_des = min(self.params['motion']['max']['vel'], v_des)

            if v_des != self.v_des:
                self.motor.set_profile_velocity(self.rad_per_sec_to_ticks(v_des))
                self.v_des = v_des
        if a_des is not None:
            a_des = min(self.params['motion']['max']['accel'], a_des)
            if a_des != self.a_des:
                self.motor.set_profile_acceleration(self.rad_per_sec_sec_to_ticks(a_des))
                self.a_des = a_des

    def move_by(self,x_des, v_des=None, a_des=None):
        if not self.servo_valid:
            return
        if abs(x_des) > 0.00002: #Avoid drift
            cx=self.ticks_to_world_rad(self.motor.get_pos())
            self.move_to(cx + x_des, v_des, a_des)

    def quick_stop(self):
        if not self.servo_valid:
            return
            self.motor.disable_torque()
            self.motor.enable_torque()

    def enable_pos(self):
        if not self.servo_valid:
            return
        self.motor.disable_torque()
        if self.params['use_multiturn']:
            self.motor.enable_multiturn()
        self.motor.set_profile_velocity(self.rad_per_sec_to_ticks(self.v_des))
        self.motor.set_profile_acceleration(self.rad_per_sec_sec_to_ticks(self.a_des))
        self.motor.enable_torque()

    def enable_pwm(self):
        if not self.servo_valid:
            return
        self.motor.disable_torque()
        self.motor.enable_pwm()
        self.motor.enable_torque()

    def set_pwm(self,x):
        if not self.servo_valid:
            return
        self.motor.set_pwm(x)

# ##########################################

    def home(self, single_stop=False, move_to_zero=True,delay_at_stop=0.0):
        # Requires at least one hardstop in the pwm_homing[0] direction
        # Can be in multiturn or single turn mode
        # Mark the first hardstop as zero ticks on the Dynammixel
        # Second hardstop is optional
        if not self.servo_valid:
            return
        if not self.params['req_calibration']:
            print('Homing not required for: '+self.name)
            return

        self.pull_status()
        if self.status['overload_error'] or self.status['overheating_error']:
            print 'Hardware error, unable to home. Exiting'
            return

        self.enable_pwm()

        print 'Moving to first hardstop...'
        self.set_pwm(self.params['pwm_homing'][0])
        ts=time.time()
        time.sleep(1.0)
        timeout=False
        while self.motor.is_moving() and not timeout:
            timeout=time.time()-ts>15.0
            time.sleep(0.5)
            #print 'Pos (ticks)',self.motor.get_pos()
        time.sleep(delay_at_stop)
        xs=self.motor.get_pos()
        self.set_pwm(0)

        if timeout:
            print 'Timed out moving to first hardstop. Exiting.'
            return
        if self.status['overload_error'] or self.status['overheating_error']:
            print 'Hardware error, unable to home. Exiting'
            return

        print 'Contact at position:', xs
        print 'Hit first hardstop, marking to zero ticks'
        self.motor.disable_torque()
        self.motor.zero_position(verbose=False)
        self.motor.set_calibrated(1)
        self.is_calibrated=1
        self.motor.enable_torque()

        self.enable_pwm()

        print 'Raw position:',self.motor.get_pos()

        if not single_stop:
            #Measure the range and write to YAML
            print 'Moving to second hardstop...'
            self.set_pwm(self.params['pwm_homing'][1])
            ts = time.time()
            time.sleep(1.0)
            timeout = False
            while self.motor.is_moving() and not timeout:
                timeout = time.time() - ts > 15.0
                time.sleep(0.5)
                #print 'Pos (ticks)', self.motor.get_pos()
            time.sleep(delay_at_stop)
            x_dir_1 = self.motor.get_pos()
            self.set_pwm(0)

            if timeout:
                print 'Timed out moving to second hardstop. Exiting.'
                return
            print 'Hit second hardstop at: ', x_dir_1
            if self.status['overload_error'] or self.status['overheating_error']:
                print 'Hardware error, unable to home. Exiting'
                return

            self.params['range_t']=[0,x_dir_1]
            print 'Homed to range of:'
            print '   Ticks:',self.params['range_t']
            r1=self.ticks_to_world_rad(0)
            r2=self.ticks_to_world_rad(x_dir_1)
            print '   Radians:',[r1,r2]
            print '   Degrees:',[rad_to_deg(r1),rad_to_deg(r2)]
            self.write_device_params(self.name, self.params)

        self.enable_pos()
        if move_to_zero:
            print 'Moving to calibrated zero: (rad)'
            self.move_to(0)
            time.sleep(3.0)
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
