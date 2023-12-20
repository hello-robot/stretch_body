from stretch_body.dynamixel_hello_XL430 import DynamixelHelloXL430
import stretch_body.hello_utils as hu


class WristRoll(DynamixelHelloXL430):
    """
    API to the Stretch RE1 wrist roll joint
    """
    def __init__(self, chain=None, usb=None, name='wrist_roll'):
        DynamixelHelloXL430.__init__(self, name, chain, usb)
        self.poses = {'cw_90': hu.deg_to_rad(90.0), 'forward': hu.deg_to_rad(0.0), 'ccw_90': hu.deg_to_rad(-90.0)}
        self.pos_current_ctrl_on_startup = False

    def enable_float_mode(self):
        """
        Command a small restorative force to center joint around 0
        """
        #Put joint in float around zero on exit
        if self.hw_valid and self.params['float_on_stop']:
            self.enable_pos_current_ctrl(current_limit=self.params['current_float_A'])
            self.move_to(0.0)

    def stop(self,close_port=True):
        if self.hw_valid and self.params['float_on_stop']:
            self.enable_float_mode()
        else:
            DynamixelHelloXL430.stop(close_port)

    def home(self):
        """
        Home to hardstops
        """
        pass

    def pose(self,p,v_r=None,a_r=None):
        """
        p: Dictionary key to named pose (eg 'forward')
        v_r: velocityfor trapezoidal motion profile (rad/s).
        a_r: acceleration for trapezoidal motion profile (rad/s^2)
        """
        self.move_to(self.poses[p],v_r,a_r)

    def step_sentry(self,robot):
        """
        This sentry attempts to prevent the wrist servo from overheating
        if it is pushing against an object for too long
        It works by backing off the commanded position from the current position
        so as to lower the steady state error of the PID controller
        """
        DynamixelHelloXL430.step_sentry(self,robot)
        if self.robot_params['robot_sentry']['wrist_roll_overload']:
            if self.status['stall_overload']:
                if self.status['effort']>0:
                    self.move_by(self.params['stall_backoff'])
                    self.logger.debug('Backoff at stall overload')
                else:
                    self.move_by(-1*self.params['stall_backoff'])
                    self.logger.debug('Backoff at stall overload')
