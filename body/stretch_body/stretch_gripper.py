from stretch_body.dynamixel_hello_XL430 import DynamixelHelloXL430
import time
from stretch_body.hello_utils import *
import logging

class StretchGripper(DynamixelHelloXL430):
    """
    API to the Stretch RE1 Gripper
    The StretchGripper motion is non-linear w.r.t to motor motion due to its design
    As such, the position of the gripper is represented at as unit-less value, 'pct'
    The Pct ranges from approximately -100 (fully closed) to approximately +50 (fully open)
    A Pct of zero is the fingertips just touching
    """
    def __init__(self, chain=None):
        DynamixelHelloXL430.__init__(self,'stretch_gripper',chain)
        self.logger = logging.getLogger('robot.stretch_gripper')
        self.status['pos_pct']= 0.0
        self.poses = {'zero':0,'open': 50, 'close': -100}

    def home(self,move_to_zero=True):
        DynamixelHelloXL430.home(self,single_stop=True,move_to_zero=move_to_zero,delay_at_stop=3.0)

    def pretty_print(self):
        print '--- StretchGripper ----'
        print "Position (%)",self.status['pos_pct']
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

    def step_sentry(self):
        """
        This sentry attempts to prevent the gripper servo from overheating during a prolonged grasp
        When the servo is stalled and exerting an effort obove a threashold it will command a
        'back off' position (slightly open the grasp)
        This reduces the PID steady state error and lowers the commanded current
        The spring design of the gripper allows it retain its grasp despite the backoff
        """
        if self.status['stall_overload']:
            if self.status['effort'] < 0: #Only backoff in open direction
                self.logger.info('Backoff at stall overload')
                self.move_by(self.params['stall_backoff'])