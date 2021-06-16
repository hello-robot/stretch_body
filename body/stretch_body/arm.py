from __future__ import print_function
from stretch_body.stepper import *
from stretch_body.device import Device
from stretch_body.hello_utils import *

class Arm(Device):
    """
    API to the Stretch RE1 Arm
    """
    def __init__(self):
        Device.__init__(self, 'arm')
        self.motor = Stepper(usb='/dev/hello-motor-arm')
        self.status = {'pos': 0.0, 'vel': 0.0, 'force':0.0, 'motor': self.motor.status,'timestamp_pc':0}
        self.motor_rad_2_arm_m = self.params['chain_pitch']*self.params['chain_sprocket_teeth']/self.params['gr_spur']/(math.pi*2)

        # Default controller params
        self.stiffness = 1.0
        self.i_feedforward = self.params['i_feedforward']
        self.vel_r = self.translate_to_motor_rad(self.params['motion']['default']['vel_m'])
        self.accel_r = self.translate_to_motor_rad(self.params['motion']['default']['accel_m'])
        self.i_contact_neg = self.translate_force_to_motor_current(self.params['contact_thresh_N'][0])
        self.i_contact_pos = self.translate_force_to_motor_current(self.params['contact_thresh_N'][1])
        self.motor.set_motion_limits(self.translate_to_motor_rad(self.params['range_m'][0]),self.translate_to_motor_rad(self.params['range_m'][1]))
        self.soft_motion_limits = [self.params['range_m'][0], self.params['range_m'][1]]
    # ###########  Device Methods #############

    def startup(self):
        return self.motor.startup()

    def stop(self):
        self.motor.stop()

    def pull_status(self):
        self.motor.pull_status()
        self.status['timestamp_pc']=time.time()
        self.status['pos']= self.motor_rad_to_translate(self.status['motor']['pos'])
        self.status['vel'] = self.motor_rad_to_translate(self.status['motor']['vel'])
        self.status['force'] = self.motor_current_to_translate_force(self.status['motor']['current'])

    def push_command(self):
        self.motor.push_command()

    def pretty_print(self):
        print('----- Arm ------ ')
        print('Pos (m): ', self.status['pos'])
        print('Vel (m/s): ', self.status['vel'])
        print('Force (N): ', self.status['force'])
        print('Timestamp PC (s):', self.status['timestamp_pc'])
        self.motor.pretty_print()


    # ###################################################
    def set_soft_motion_limits(self,x_min=None,x_max=None):
        x_min = max(x_min,self.params['range_m'][0]) if x_min is not None else self.params['range_m'][0]
        x_max = min(x_max,self.params['range_m'][1]) if x_max is not None else self.params['range_m'][1]
        if x_min != self.soft_motion_limits[0] or x_max != self.soft_motion_limits[1]:
            self.motor.set_motion_limits(self.translate_to_motor_rad(x_min), self.translate_to_motor_rad(x_max))

        self.soft_motion_limits=[x_min, x_max]

    def move_to(self,x_m,v_m=None, a_m=None, stiffness=None, contact_thresh_pos_N=None, contact_thresh_neg_N=None, req_calibration=True):
        """
        x_m: commanded absolute position (meters). x_m=0 is retracted. x_m=~0.5 is extended
        v_m: velocity for trapezoidal motion profile (m/s)
        a_m: acceleration for trapezoidal motion profile (m/s^2)
        stiffness: stiffness of motion. Range 0.0 (min) to 1.0 (max)
        contact_thresh_pos_N: force threshold to stop motion (~Newtons, extension direction)
        contact_thresh_neg_N: force threshold to stop motion (~Newtons, retraction direction)
        req_calibration: Disallow motion prior to homing
        """
        if req_calibration:
            if not self.motor.status['pos_calibrated']:
                self.logger.warning('Arm not calibrated')
                return
            x_m = min(max(self.soft_motion_limits[0], x_m), self.soft_motion_limits[1]) #Only clip motion when calibrated

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

        if contact_thresh_neg_N is not None:
            i_contact_neg = max(self.translate_force_to_motor_current(contact_thresh_neg_N),self.params['contact_thresh_max_N'][0])
        else:
            i_contact_neg = self.i_contact_neg

        if contact_thresh_pos_N is not None:
            i_contact_pos = min(self.translate_force_to_motor_current(contact_thresh_pos_N),self.params['contact_thresh_max_N'][1])
        else:
            i_contact_pos = self.i_contact_pos

        self.motor.set_command(mode = MODE_POS_TRAJ,
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
        contact_thresh_pos_N: force threshold to stop motion (~Newtons, extension direction)
        contact_thresh_neg_N: force threshold to stop motion (~Newtons, retraction direction)
        req_calibration: Disallow motion prior to homing
        """
        if req_calibration:
            if not self.motor.status['pos_calibrated']:
                self.logger.warning('Arm not calibrated')
                return

            if self.status['pos'] + x_m < self.soft_motion_limits[0]:  #Only clip motion when calibrated
                x_m = self.soft_motion_limits[0] - self.status['pos']
            if self.status['pos'] + x_m > self.soft_motion_limits[1]:
                x_m = self.soft_motion_limits[1] - self.status['pos']

        if stiffness is not None:
            stiffness = max(0, min(1.0, stiffness))
        else:
            stiffness = self.stiffness

        if v_m is not None:
            v_r = self.translate_to_motor_rad(min(abs(v_m), self.params['motion']['max']['vel_m']))
        else:
            v_r = self.vel_r

        if a_m is not None:
            a_r = self.translate_to_motor_rad(min(abs(a_m), self.params['motion']['max']['accel_m']))
        else:
            a_r = self.accel_r

        if contact_thresh_neg_N is not None:
            i_contact_neg = max(self.translate_force_to_motor_current(contact_thresh_neg_N),
                                self.params['contact_thresh_max_N'][0])
        else:
            i_contact_neg = self.i_contact_neg

        if contact_thresh_pos_N is not None:
            i_contact_pos = min(self.translate_force_to_motor_current(contact_thresh_pos_N),
                                self.params['contact_thresh_max_N'][1])
        else:
            i_contact_pos = self.i_contact_pos

        self.motor.set_command(mode=MODE_POS_TRAJ_INCR,
                               x_des=self.translate_to_motor_rad(x_m),
                               v_des=v_r,
                               a_des=a_r,
                               stiffness=stiffness,
                               i_feedforward=self.i_feedforward,
                               i_contact_pos=i_contact_pos,
                               i_contact_neg=i_contact_neg)


    # ######### Utility ##############################

    def motor_current_to_translate_force(self,i):
        return self.params['force_N_per_A']*(i-self.params['i_feedforward'])

    def translate_force_to_motor_current(self,f):
        return (f/self.params['force_N_per_A'])+self.params['i_feedforward']

    def motor_rad_to_translate(self,ang): #input in rad, output m
        return self.motor_rad_2_arm_m*ang

    def translate_to_motor_rad(self, arm_m):
        return arm_m/self.motor_rad_2_arm_m


    def __wait_for_contact(self, timeout=5.0):
        ts=time.time()
        while (time.time()-ts<timeout):
            self.pull_status()
            if self.motor.status['in_guarded_event']:
                return True
            time.sleep(0.01)
        return False

    def home(self,single_stop=True,measuring=False):
        """
        Home to hardstops
        """
        if not self.motor.hw_valid:
            self.logger.warning('Not able to home arm. Hardware not present')
            return
        print('Homing Arm...')
        self.motor.enable_guarded_mode()
        self.motor.disable_sync_mode()
        self.motor.reset_pos_calibrated()
        self.push_command()

        success=False
        extension_m=None

        #In direction
        self.pull_status()
        self.move_by(x_m=-1, contact_thresh_pos_N=self.params['homing_force_N'][1], contact_thresh_neg_N=self.params['homing_force_N'][0], req_calibration=False)
        self.push_command()

        if self.__wait_for_contact(timeout=10.0):
            print('Retraction contact detected at motor position (rad)',self.motor.status['pos'])
            time.sleep(1.0)
            self.pull_status()
            x=self.translate_to_motor_rad(self.params['range_m'][0])
            print('Marking arm position to (m)', self.params['range_m'][0])
            self.motor.mark_position(x)
            self.push_command()

            #Out direction
            if not single_stop:
                # Out direction
                self.pull_status()
                self.move_by(x_m=1.0, contact_thresh_pos_N=self.params['homing_force_N'][1], contact_thresh_neg_N=self.params['homing_force_N'][0], req_calibration=False)
                self.push_command()
                if self.__wait_for_contact(timeout=10.0):
                    print('Extension contact detected at arm position (rad)', self.motor.status['pos'])
                    time.sleep(1.0)
                    self.pull_status()
                    if not measuring:
                        print('Current arm position (m): ', self.status['pos'])
                        print('Expected arm position (m): ', self.params['range_m'][1])
                        if abs(self.status['pos'] - self.params['range_m'][1]) < .010:  # Within 1 cm
                            success = True
                        else:
                            self.logger.warning('Arm homing failed. Out of range')
                            success = False
                    else:
                        extension_m=self.status['pos']
                        success=True
                else:
                    self.logger.warning('Arm homing failed. Failed to detect contact')
                    success = False
            else:
                success = True
        else:
            self.logger.warning('Arm homing failed. Failed to detect contact')
            success = False

        if success:
            self.motor.set_pos_calibrated()
            self.push_command()
            self.pull_status()
            self.move_to(x_m=0.1)
            self.push_command()
            time.sleep(2.0)
            print('Arm homing successful')

        #Restore default
        if not self.motor.gains['enable_guarded_mode']:
            self.motor.disable_guarded_mode()
        if self.motor.gains['enable_sync_mode']:
            self.motor.enable_sync_mode()
        self.push_command()

        if measuring:
            return extension_m


    def step_sentry(self,robot):
        self.motor.step_sentry(robot)