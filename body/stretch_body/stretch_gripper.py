from __future__ import print_function
from stretch_body.dynamixel_hello_XL430 import DynamixelHelloXL430

class StretchGripper(DynamixelHelloXL430):
    """
    API to the Stretch Gripper.
    
    The StretchGripper motion is non-linear w.r.t to motor motion due to its design.
    As such, the position of the gripper is represented at as unit-less value, 'pct'.
    The Pct ranges from approximately -100 (fully closed) to approximately +50 (fully open).
    A Pct of zero is the fingertips just touching.
    """
    def __init__(self, chain=None, usb=None):
        DynamixelHelloXL430.__init__(self, 'stretch_gripper', chain, usb)
        self.status['pos_pct']= 0.0
        self.pct_max_open=self.world_rad_to_pct(self.ticks_to_world_rad(self.params['range_t'][1])) #May be a bit greater than 50 given non-linear calibration
        self.poses = {'zero': 0,
                      'open': self.pct_max_open,
                      'close': -100}

    def startup(self, threaded=True):
        """Startup the Stretch Gripper.

        Parameters
        ----------
        threaded : bool, optional
            If True start the gripper in a separate thread, if False start the gripper in the current thread, by default True

        Returns
        -------
        bool:
            True if the servo started successfully, False otherwise.
        """
        return DynamixelHelloXL430.startup(self, threaded=threaded)

    def home(self,move_to_zero=True):
        """Home the gripper to its default position.

        Parameters
        ----------
        move_to_zero : bool, optional.
            Move the gripper to its zero position if True, if False perform homing without moving ti its zero position, by default True.
        """
        DynamixelHelloXL430.home(self,single_stop=True,move_to_zero=move_to_zero,delay_at_stop=3.0)

    def pretty_print(self):
        """Print the status of the gripper.
        """
        print('--- StretchGripper ----')
        print("Position (%)",self.status['pos_pct'])
        DynamixelHelloXL430.pretty_print(self)

    def pose(self,p,v_r=None, a_r=None):
        """
        p: Dictionary key to named pose (eg 'close').
        """
        self.move_to(self.poses[p],v_r,a_r)

    def move_to(self,pct, v_r=None, a_r=None):
        """Move the gripper to a specified position.

        Parameters
        ----------
        pct : float.
            The commanded absolute position in percentage (0.0 to 100.0).
        
        v_r : float or None, optional
            The desired velocity for for trapezoidal motion profile (rad/s), 
            if None the velocity is nos specified, by default None
        
        a_r : float or None, optional
            The desired acceleration acceleration for trapezoidal motion profile (rad/s^2), 
            if None the acceleration is not specified, by default None
        """
        x_r=self.pct_to_world_rad(pct)
        DynamixelHelloXL430.move_to(self,x_des=x_r, v_des=v_r, a_des=a_r)


    def move_by(self,delta_pct,v_r=None,a_r=None):
        """Move the gripper by a specified percentage.

        Parameters
        ----------
        delta_pct : float
            Commanded incremental motion (pct)
        v_r : float, optional
            Velocity for trapezoidal motion profile (rad/s), by default None
        a_r : float, optional
            Acceleration for trapezoidal motion profile (rad/s^2), by default None
        """
        self.pull_status() #Ensure up to date
        self.move_to(self.status['pos_pct']+delta_pct,v_r,a_r)

    ############### Utilities ###############

    def pull_status(self,data=None):
        """Pull the status of the gripper including its position.

        Parameters
        ----------
        data : dict or None, optional.
            Data to be pulled, if None only the default data will be updated, by default None
        """
        DynamixelHelloXL430.pull_status(self,data)
        self.status['pos_pct']=self.world_rad_to_pct(self.status['pos'])

    def pct_to_world_rad(self,pct):
        """Convert from percentage to world radians.

        Parameters
        ----------
        pct : float.
            The value in percentage from 0 to 100.

        Returns
        -------
        float:
            The value in world radians.
        """
        pct_to_tick = -1 * ((self.params['zero_t'] - self.params['range_t'][0]) / 100.0)
        t=((-1*pct)-100)*pct_to_tick
        r = DynamixelHelloXL430.ticks_to_world_rad(self, t)
        return r

    def world_rad_to_pct(self,r):
        """Convert from world radians to percentage.

        Parameters
        ----------
        r : float.
            The value in world radians.

        Returns
        -------
        float:
            The valie in percentage from 0 to 100.
        """
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
