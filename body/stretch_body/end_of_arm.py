from __future__ import print_function
from stretch_body.dynamixel_X_chain import DynamixelXChain
import importlib

class EndOfArm(DynamixelXChain):
    """
    The EndOfArm class allows for an extensible serial chain of Dynamixel X series devices
    It allows the specific type of device to be declared at runtime via the Yaml parameters
    In this way, a user can add their own custom Dynamixel based tools to the robot end-of-arm by
    simply deriving it from DynamixelHelloXL430 and declaring the class name / Python module name
    in the User YAML file
    """
    def __init__(self, name='end_of_arm'):
        DynamixelXChain.__init__(self, usb='/dev/hello-dynamixel-wrist', name=name)
        self.joints = self.params.get('devices', {}).keys()
        for j in self.joints:
            module_name = self.params['devices'][j]['py_module_name']
            class_name = self.params['devices'][j]['py_class_name']
            dynamixel_device = getattr(importlib.import_module(module_name), class_name)(chain=self)
            self.add_motor(dynamixel_device)

    def startup(self, threaded=True):
        return DynamixelXChain.startup(self, threaded=threaded)

    def get_joint(self, joint_name):
        """Retrieves joint by name.

        Parameters
        ----------
        joint_name : str
            valid joints defined as defined in params['devices']

        Returns
        -------
        DynamixelHelloXL430 or None
            Motor object on valid joint name, else None
        """
        return self.get_motor(joint_name)

    def move_to(self, joint,x_r, v_r=None, a_r=None):
        """
        joint: name of joint (string)
        x_r: commanded absolute position (radians).
        v_r: velocity for trapezoidal motion profile (rad/s).
        a_r: acceleration for trapezoidal motion profile (rad/s^2)
        """
        with  self.pt_lock:
            self.motors[joint].move_to(x_r, v_r, a_r)

    def move_by(self, joint, x_r, v_r=None, a_r=None):
        """
        joint: name of joint (string)
        x_r: commanded incremental motion (radians).
        v_r: velocity for trapezoidal motion profile (rad/s).
        a_r: acceleration for trapezoidal motion profile (rad/s^2)
        """
        with self.pt_lock:
            self.motors[joint].move_by(x_r, v_r, a_r)

    def pose(self,joint, p,v_r=None, a_r=None):
        """
                joint: name of joint (string)
                p: named pose of joint
                v_r: velocity for trapezoidal motion profile (rad/s).
                a_r: acceleration for trapezoidal motion profile (rad/s^2)
                """
        with self.pt_lock:
            self.motors[joint].pose(p, v_r, a_r)

    def stow(self):
        print('--------- Stowing Wrist Yaw ----')
        self.move_to('wrist_yaw', self.params['stow']['wrist_yaw'])

    def home(self, joint=None):
        """
        Home to hardstops
        """
        if joint is None:
            for j in self.joints:
                print('--------- Homing %s ----'%j)
                self.motors[j].home()
        else:
            with self.pt_lock:
                self.motors[joint].home()

    def is_tool_present(self,class_name):
        """
        Return true if the given tool type is present (eg. StretchGripper)
        Allows for conditional logic when switching end-of-arm tools
        """
        for j in self.joints:
            if class_name == self.params['devices'][j]['py_class_name']:
                return True
        return False



