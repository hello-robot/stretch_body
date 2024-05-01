from __future__ import print_function
from stretch_body.dynamixel_hello_XL430 import DynamixelHelloXL430
from stretch_body.hello_utils import *
from stretch_body.dynamixel_X_chain import DynamixelXChain
from stretch_body.robot_params import RobotParams

class Head(DynamixelXChain):
    """
    API to the Stretch RE1 Head
    """
    def __init__(self, usb=None):
        if usb is None:
            usb = RobotParams.get_params()[1]['head']['usb_name']
        DynamixelXChain.__init__(self, usb=usb, name='head')
        self.joints = ['head_pan', 'head_tilt']
        for j in self.joints:
            self.add_motor(DynamixelHelloXL430(name=j, chain=self))
        self.poses = {'ahead': [0, 0], 'back': [deg_to_rad(-180), deg_to_rad(0)],
                      'tool': [deg_to_rad(-90), deg_to_rad(-45)],
                      'wheels': [deg_to_rad(0), deg_to_rad(-90)], 'left': [deg_to_rad(90), deg_to_rad(0)],
                      'up': [deg_to_rad(0), deg_to_rad(30)]}

    def startup(self, threaded=True):
        return DynamixelXChain.startup(self, threaded=threaded)

    def get_joint(self, joint_name):
        """Retrieves joint by name.

        Parameters
        ----------
        joint_name : str
            valid joints defined in ``joints``

        Returns
        -------
        DynamixelHelloXL430 or None
            Motor object on valid joint name, else None
        """
        return self.get_motor(joint_name)

    def move_to(self, joint, x_r, v_r=None, a_r=None):
        """
        joint: Name of the joint to move ('head_pan' or 'head_tilt')
        x_r: commanded absolute position (radians).
        v_r: velocity for trapezoidal motion profile (rad/s).
        a_r: acceleration for trapezoidal motion profile (rad/s^2)
        """
        self.motors[joint].move_to(x_r,v_r,a_r)

    def move_by(self, joint,  x_r, v_r=None, a_r=None):
        """
        joint: Name of the joint to move ('head_pan' or 'head_tilt')
        x_r: commanded incremental motion (radians).
        v_r: velocity for trapezoidal motion profile (rad/s).
        a_r: acceleration for trapezoidal motion profile (rad/s^2)
        """
        self.motors[joint].move_by(x_r,v_r,a_r)

    def set_velocity(self, joint, v_r, a_r=None):
        """
        joint: name of joint (string)
        v_r: commanded velocity (rad/s).
        a_r: acceleration motion profile (rad/s^2)
        """
        self.motors[joint].set_velocity(v_r, a_r)

    def home(self):
        if self.motors['head_pan'].params['req_calibration']:
            with self.pt_lock:
                self.motors['head_pan'].home(single_stop=True)

        if self.motors['head_tilt'].params['req_calibration']:
            with self.pt_lock:
                self.motors['head_tilt'].home(single_stop=True)

        self.move_to('head_pan', deg_to_rad(0))
        self.move_to('head_tilt', deg_to_rad(0))


    def pose(self, p, v_r=[None, None], a_r=[None, None]):
        """
        p: Dictionary key to named pose (eg 'ahead')
        v_r: list, velocities for trapezoidal motion profile (rad/s).
        a_r: list, accelerations for trapezoidal motion profile (rad/s^2)
        """
        self.move_to('head_pan', self.poses[p][0], v_r[0], a_r[0])
        self.move_to('head_tilt', self.poses[p][1], v_r[0], a_r[1])

