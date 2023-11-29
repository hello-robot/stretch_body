from __future__ import print_function
from stretch_body.dynamixel_hello_XL430 import DynamixelHelloXL430

# Note: Copied from stretch_ros/hello_helpers
class GripperConversion:
    def __init__(self,finger_length_m=0.171,open_aperture_m=.09,closed_aperture_m=0.0,open_robotis=70.0,closed_robotis=0.0):
        # robotis position values (gripper.py)
        #      0 is very close to closed (fingers almost or barely touching)
        #     50 is maximally open
        #   -100 is maximally closed (maximum force applied to the object - might be risky for a large object)
        #
        # aperture is 12.5 cm wide when open (0.125 m, 125 mm)
        #
        # finger angle
        #   0.0 just barely closed
        #   fully opened is

        # from stretch_gripper.xacro
        # scale_finger_length = 0.9
        # scale_finger_length * 0.19011
        # = 0.171099
        self.finger_length_m = finger_length_m

        self.open_aperture_m = open_aperture_m #0.125
        self.closed_aperture_m = closed_aperture_m

        self.open_robotis = open_robotis
        self.closed_robotis = closed_robotis

        self.robotis_to_aperture_slope = ((self.open_aperture_m - self.closed_aperture_m) / (self.open_robotis - self.closed_robotis))

    def robotis_to_aperture(self, robotis_in):
        # linear model
        aperture_m = (self.robotis_to_aperture_slope * (robotis_in - self.closed_robotis)) + self.closed_aperture_m
        return aperture_m

    def aperture_to_robotis(self, aperture_m):
        # linear model
        robotis_out = ((aperture_m - self.closed_aperture_m) / self.robotis_to_aperture_slope) + self.closed_robotis
        return robotis_out

    def aperture_to_finger_rad(self, aperture_m):
        # arc length / radius = ang_rad
        finger_rad = (aperture_m/2.0)/self.finger_length_m
        return finger_rad

    def finger_rad_to_aperture(self, finger_rad):
        aperture_m = 2.0 * (finger_rad * self.finger_length_m)
        return aperture_m

    def finger_to_robotis(self, finger_ang_rad):
        aperture_m = self.finger_rad_to_aperture(finger_ang_rad)
        robotis_out = self.aperture_to_robotis(aperture_m)
        return robotis_out

    def robotis_to_finger(self, robotis_pct):
        aperture_m = self.robotis_to_aperture(robotis_pct)
        finger_rad = self.aperture_to_finger_rad(aperture_m)
        return finger_rad

    def status_to_all(self, gripper_status):
        aperture_m = self.robotis_to_aperture(gripper_status['pos_pct'])
        finger_rad = self.aperture_to_finger_rad(aperture_m)
        finger_effort = gripper_status['effort']
        finger_vel = (self.robotis_to_aperture_slope * gripper_status['vel'])/2.0
        return aperture_m, finger_rad, finger_effort, finger_vel

class StretchGripper(DynamixelHelloXL430):
    """
    API to the Stretch Gripper
    The StretchGripper motion is non-linear w.r.t to motor motion due to its design
    As such, the position of the gripper is represented at as unit-less value, 'pct'
    The Pct ranges from approximately -100 (fully closed) to approximately +50 (fully open)
    A Pct of zero is the fingertips just touching
    """
    def __init__(self, chain=None, usb=None):
        DynamixelHelloXL430.__init__(self, 'stretch_gripper', chain, usb)
        self.status['pos_pct']= 0.0
        self.pct_max_open=self.world_rad_to_pct(self.ticks_to_world_rad(self.params['range_t'][1])) #May be a bit greater than 50 given non-linear calibration
        self.poses = {'zero': 0,
                      'open': self.pct_max_open,
                      'close': -100}
        self.gripper_conversion = GripperConversion()

    def startup(self, threaded=True):
        return DynamixelHelloXL430.startup(self, threaded=threaded)

    def home(self,move_to_zero=True):
        DynamixelHelloXL430.home(self,single_stop=True,move_to_zero=move_to_zero,delay_at_stop=3.0)

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
