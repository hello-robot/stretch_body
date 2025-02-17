from __future__ import print_function
from stretch_body.dynamixel_hello_XL430 import DynamixelHelloXL430
from stretch_body.device import Device
from stretch_body.robot_params import RobotParams
from stretch_body.gripper_conversion import GripperConversion



class StretchGripper(DynamixelHelloXL430):
    """
    API to the Stretch Gripper
    The StretchGripper motion is non-linear w.r.t to motor motion due to its design
    As such, the position of the gripper is represented at as unit-less value, 'pct'
    The Pct ranges from approximately -100 (fully closed) to approximately +50 to +200 (fully open)
    The fully open value (self.pct_max_open) is dependent on mechanical design of the gripper
    which changes depending on the robot generation (RE1, RE2, SE3, etc)
    A Pct of zero is the fingertips just touching
    """
    def __init__(self, chain=None, usb=None, name='stretch_gripper'):
        DynamixelHelloXL430.__init__(self, name, chain, usb)
        self.status['pos_pct']= 0.0
        self.pct_max_open=self.world_rad_to_pct(self.ticks_to_world_rad(self.params['range_t'][1])) #May be a bit greater than 50 given non-linear calibration
        self.poses = {'zero': 0,
                      'open': self.pct_max_open,
                      'close': -100}
        self.status['gripper_conversion'] = {'aperture_m':0.0,
                                             'finger_rad':0.0,
                                             'finger_effort':0.0,
                                             'finger_vel':0.0}
        self.gripper_conversion = GripperConversion(self.params['gripper_conversion'])

    def startup(self, threaded=True):
        return DynamixelHelloXL430.startup(self, threaded=threaded)

    def home(self,move_to_zero=True):
        DynamixelHelloXL430.home(self,single_stop=True,move_to_zero=move_to_zero,delay_at_stop=2.25)

    def pretty_print(self):
        print('--- StretchGripper ----')
        print("Position (%)",self.status['pos_pct'])
        DynamixelHelloXL430.pretty_print(self)

    def pose(self,p,v_r=None, a_r=None):
        """
        p: Dictionary key to named pose (eg 'close')
        """
        self.move_to(self.poses[p],v_r,a_r)

    def move_to(self,pct, v_r=None, a_r=None):
        """
        pct: commanded absolute position (Pct).
        v_r: velocity for trapezoidal motion profile (rad/s).
        a_r: acceleration for trapezoidal motion profile (rad/s^2)
        """
        x_r=self.pct_to_world_rad(pct)
        DynamixelHelloXL430.move_to(self,x_des=x_r, v_des=v_r, a_des=a_r)


    def move_by(self,delta_pct,v_r=None,a_r=None):
        """
        delta_pct: commanded incremental motion (pct).
        v_r: velocity for trapezoidal motion profile (rad/s).
        a_r: acceleration for trapezoidal motion profile (rad/s^2)
        """
        self.pull_status() #Ensure up to date
        self.move_to(self.status['pos_pct']+delta_pct,v_r,a_r)

    ############### Utilities ###############

    def pull_status(self,data=None):
        DynamixelHelloXL430.pull_status(self,data)
        self.status['pos_pct']=self.world_rad_to_pct(self.status['pos'])
        self.status['gripper_conversion']=self.gripper_conversion.get_status(self.status)

    def pct_to_world_rad(self,pct):
        pct_to_tick = -1 * ((self.params['zero_t'] - self.params['range_t'][0]) / 100.0)
        t=((-1*pct)-100)*pct_to_tick
        r = DynamixelHelloXL430.ticks_to_world_rad(self, t)
        return r

    def world_rad_to_pct(self,r):
        pct_to_tick = -1 * ((self.params['zero_t'] - self.params['range_t'][0]) / 100.0)
        t = DynamixelHelloXL430.world_rad_to_ticks(self,r)
        pct = -1*((t / pct_to_tick)+100)
        return pct

    def step_sentry(self, robot):
        """This sentry attempts to prevent the gripper servo from overheating during a prolonged grasp
        When the servo is stalled and exerting an effort above a threshold it will command a 'back off'
        position (slightly opening the grasp). This reduces the PID steady state error and lowers the
        commanded current. The gripper's spring design allows it to retain its grasp despite the backoff.
        """
        DynamixelHelloXL430.step_sentry(self, robot)
        if self.hw_valid and self.robot_params['robot_sentry']['stretch_gripper_overload'] and not self.is_homing:
            if self.status['stall_overload']:
                if self.in_vel_mode:
                    self.enable_pos()
                if self.status['effort'] < 0: #Only backoff in open direction
                    self.logger.debug('Backoff at stall overload')
                    self.move_by(self.params['stall_backoff'])

class StretchGripper3(StretchGripper):
    """
        Wrapper for version 3 (for DW3)
    """
    def __init__(self, chain=None, usb=None):
        StretchGripper.__init__(self, chain, usb,'stretch_gripper')
