from stretch_body.dynamixel_hello_XL430 import DynamixelHelloXL430
import stretch_body.hello_utils as hu


class WristPitch(DynamixelHelloXL430):
    """
    API to the Stretch RE1 wrist pitch joint
    """
    def __init__(self, chain=None, usb=None, name='wrist_pitch'):
        DynamixelHelloXL430.__init__(self, name, chain, usb)
        self.poses = {'up': hu.deg_to_rad(56.0), 'forward': hu.deg_to_rad(0.0), 'down': hu.deg_to_rad(-90.0)}

    def startup(self, threaded=True):
        r= DynamixelHelloXL430.startup(self, threaded=threaded)
        if r:
            self.enable_pos_current_ctrl() #Default to current limited position control
        return r

    def stop(self):
        if self.hw_valid and self.params['float_on_stop']:
            self.enable_float_mode()
        else:
            DynamixelHelloXL430.stop(self)

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

    def enable_float_mode(self):
        """
        Command a constant current to the pitch joint that roughly gravity compensates
        the joint when it is straight out.
        When the joint is straight down it will float back to straight up
        Useful for human-interaction
        """
        i=self.motor.get_current()
        self.disable_torque()
        self.motor.enable_current()
        self.enable_torque()
        self.motor.set_goal_current(i)#self.current_to_ticks(self.params['current_float_A']))
        self.set_motion_params(force=True)

    def step_sentry(self,robot):
        """
        This sentry attempts to prevent the wrist servo from overheating
        if it is pushing against an object for too long
        It works by backing off the commanded position from the current position
        so as to lower the steady state error of the PID controller
        """
        DynamixelHelloXL430.step_sentry(self,robot)
        if self.robot_params['robot_sentry']['wrist_pitch_overload']:
            if self.status['stall_overload']:
                if self.status['effort']>0:
                    self.move_by(self.params['stall_backoff'])
                    self.logger.debug('Backoff at stall overload')
                else:
                    self.move_by(-1*self.params['stall_backoff'])
                    self.logger.debug('Backoff at stall overload')
