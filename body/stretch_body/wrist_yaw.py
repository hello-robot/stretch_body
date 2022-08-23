from __future__ import print_function
from stretch_body.dynamixel_hello_XL430 import DynamixelHelloXL430
from stretch_body.hello_utils import *
import time


class WristYaw(DynamixelHelloXL430):
    """
    API to the Stretch wrist yaw joint
    """
    def __init__(self, chain=None):
        DynamixelHelloXL430.__init__(self, 'wrist_yaw', chain)
        self.poses = {'side': deg_to_rad(90.0),
                      'forward': deg_to_rad(0.0),
                      'stow': deg_to_rad(180.0)}

    def startup(self, threaded=True):
        return DynamixelHelloXL430.startup(self, threaded=threaded)

    def home(self):
        """
        Home to hardstops
        """
        DynamixelHelloXL430.home(self,single_stop=True)
        time.sleep(2.0) #extra time to get back

    def pose(self,p,v_r=None,a_r=None):
        """
        p: Dictionary key to named pose (eg 'forward')
        v_r: velocityfor trapezoidal motion profile (rad/s).
        a_r: acceleration for trapezoidal motion profile (rad/s^2)
        """
        self.move_to(self.poses[p],v_r,a_r)

    def step_sentry(self, robot):
        """This sentry attempts to prevent the wrist yaw servo from overheating
        if it is pushing against an object for too long. It works by backing off
        the commanded position from the current position, so as to lower the steady
        state error of the PID controller.
        """
        DynamixelHelloXL430.step_sentry(self, robot)
        if self.hw_valid and self.robot_params['robot_sentry']['wrist_yaw_overload']:
            if self.status['stall_overload']:
                if self.status['effort']>0:
                    self.move_by(self.params['stall_backoff'])
                    self.logger.debug('Backoff at stall overload')
                else:
                    self.move_by(-1*self.params['stall_backoff'])
                    self.logger.debug('Backoff at stall overload')
