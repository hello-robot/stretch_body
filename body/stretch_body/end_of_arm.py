from __future__ import print_function
from stretch_body.dynamixel_X_chain import DynamixelXChain
import importlib
from stretch_body.robot_params import RobotParams
from stretch_body.device import Device


class EndOfArm(DynamixelXChain):
    """
    The EndOfArm class allows for an extensible serial chain of Dynamixel X series devices
    It allows the specific type of device to be declared at runtime via the Yaml parameters
    In this way, a user can add their own custom Dynamixel based tools to the robot end-of-arm by
    simply deriving it from DynamixelHelloXL430 and declaring the class name / Python module name
    in the User YAML file
    """
    def __init__(self, name='end_of_arm', usb=None):
        if usb is None:
            usb = RobotParams.get_params()[1]['end_of_arm']['usb_name']
        DynamixelXChain.__init__(self, usb=usb, name=name)
        self.joints = self.params.get('devices', {}).keys()
        for j in self.joints:
            module_name = self.params['devices'][j]['py_module_name']
            class_name = self.params['devices'][j]['py_class_name']
            dynamixel_device = getattr(importlib.import_module(module_name), class_name)(chain=self)
            self.add_motor(dynamixel_device)
        self.urdf_map={} #Override

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
    
    def set_velocity(self, joint, v_r, a_r=None):
        """
        joint: name of joint (string)
        v_r: commanded velocity (rad/s).
        a_r: acceleration motion profile (rad/s^2)
        """
        with self.pt_lock:
            self.motors[joint].set_velocity(v_r, a_r)

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
        pass #Override by specific tool

    def pre_stow(self,robot=None):
        pass #Override by specific tool

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

    def get_joint_configuration(self,brake_joints={}):
        """
        Construct a dictionary of tools current pose (for robot_collision_mgmt)
        Keys match joint names in URDF
        Specific tools should define urdf_map
        """
        ret = {}
        for j in self.urdf_map:
            jn = self.urdf_map[j]
            motor = self.get_joint(jn)
            dx = 0.0
            try:
                if brake_joints[j]:
                    dx = self.params['collision_mgmt']['k_brake_distance'][jn] * motor.get_braking_distance()
            except KeyError:
                dx=0
            ret[j] = motor.status['pos'] + dx

        gripper_joint = None
        for j in self.joints:
            if 'gripper' in j:
                gripper_joint = j

        if gripper_joint:
            dx = 0
            if brake_joints:
                for j in brake_joints:
                    if 'gripper' in j:
                        dx = self.params['collision_mgmt']['k_brake_distance'][j]
            finger_angle = self.get_joint(gripper_joint).status['gripper_conversion']['finger_rad'] + dx
            ret['joint_gripper_finger_left'] = finger_angle/2
            ret['joint_gripper_finger_right'] = finger_angle/2

        return ret
    
    def pre_stow(self,robot=None):
        pass



