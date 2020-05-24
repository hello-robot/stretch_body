from stretch_body.dynamixel_X_chain import DynamixelXChain
import importlib

class EndOfArm(DynamixelXChain):
    """
    The EndOfArm class allows for an extensible serial chain of Dynamixel X series devices
    It allows the specifc type of device to be declared at runtime via the Yaml parameters
    In this way, a user can add their own custom Dynamixel based tools to the robot end-of-arm by
    simply deriving it from DynamixelHelloXL430 and declaring the class name / Python module name
    in the User YAML file
    """
    def __init__(self):
        DynamixelXChain.__init__(self,'/dev/hello-dynamixel-wrist')
        self.name='end_of_arm'
        self.params=self.robot_params[self.name]
        self.joints=self.params['devices'].keys()
        for j in self.joints:
            module_name=self.params['devices'][j]['py_module_name']
            class_name=self.params['devices'][j]['py_class_name']
            dynamixel_device=getattr(importlib.import_module(module_name), class_name)(self)
            self.add_motor(dynamixel_device)

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

    def home(self, joint):
        """
        Home to hardstops
        """
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



