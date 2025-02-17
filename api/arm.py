from stretch_body.common import revision_name
from stretch_body.r1.arm import Arm as ArmR1
from stretch_body.r2.arm import Arm as ArmR2

#API wrapper to arm class

#Todo: build in notion of command complete / idle / fail, and threading of read/write to device
#Todo: expose splined trajectory
class Arm():
    """
    API to the Stretch Arm
    """
    def __init__(self):
        if revision_name=='r1':
            self.dev=ArmR1()
        if revision_name=='r2':
            self.dev=ArmR2()


    def set_velocity(self,v_m, a_m=None):
        self.dev.set_velocity(v_m,a_m)

    def move_by(self,x_m, v_m=None, a_m=None):
        self.dev.move_to(x_m,v_m,a_m)

    def move_to(self,x_m, v_m=None, a_m=None):
        self.dev.move_to(x_m,v_m,a_m)
    def home(self):
        self.dev.home()


    def _startup(self):
        self.dev.startup()
    def _stop(self):
        self.dev.stop()
    def _push_command(self):
        self.dev.push_command()
    def _pull_status(self):
        self.dev.pull_status()