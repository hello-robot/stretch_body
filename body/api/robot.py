from stretch_body.common import revision_name
from stretch_body.r1.robot import Robot as RobotR1
from stretch_body.r2.robot import Robot as RobotR2

#API wrapper to arm class

#Todo: build in notion of command complete / idle / fail, and threading of read/write to device
#Todo: expose splined trajectory
#NOTE: Assumes that parameters, tuning, configuration are set in YAML before runtime, and user will not want to modify
class Robot():
    """
    API to the Stretch Arm
    """
    def __init__(self):
        if revision_name=='r1':
            self.dev=RobotR1()
            self.joints={'arm':self.dev.arm,'lift':self.dev.lift,'head_tilt':self.dev.head.motors['head_tilt'], 'head_pan':self.dev.head.motors['head_pan']}
        if revision_name=='r2':
            self.dev=RobotR2()
            self.joints = ['arm', 'lift']
        for j in self.dev.end_of_arm.joints:
            self.joints[j] = self.dev.end_of_arm.get_motor(j)

    def _startup(self):
        self.dev.startup()
    def _stop(self):
        self.dev.stop()
    def _push_command(self):
        self.dev.push_command()
    def _pull_status(self):
        self.dev.pull_status()

    def get_status(self):
        return self.dev.get_status()
    def home(self):
        self.dev.home()
    def stow(self):
        self.dev.stow()

    def set_param(self,param,val):
        #eg, robot.arm.hello-motor-arm.kPd=0.1
        pass
    def get_param(self):
        pass

    def base_translate_by(self, x_m, y_m, v_m=None, a_m=None):
        pass

    def base_rotate_by(self, x_r, v_r=None, a_r=None):
        pass

    def base_set_velocity(self, v_m, w_r, a=None):
        pass

    def set_velocity(self,j,v_m, a_m=None):
        if j in self.joints:
            return self.joints[j].set_velocity(v_m,a_m)
        return False

    def move_by(self,j,x_m, v_m=None, a_m=None):
        if j in self.joints:
            self.joints[j].move_to(j,x_m,v_m,a_m)
        return False

    def move_to(self,j,x_m, v_m=None, a_m=None):
        if j in self.joints:
            self.joints[j].move_to(x_m,v_m,a_m)
        return False
