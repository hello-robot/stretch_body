from stretch_body.common import revision_name
import stretch_body.r1.arm
import stretch_body.r2.arm

#API wrapper to arm class

#Todo: build in notion of command complete / idle / fail, and threading of read/write to device
#Todo: expose splined trajectory


revision_name='r1'

def ArmFactory():
    if revision_name == 'r1':
        return stretch_body.r1.arm.Arm()
    if revision_name == 'r2':
        return  stretch_body.r2.arm.Arm()

class Arm(ArmFactory()):
    """
    API to the Stretch Arm
    """
    def __init__(self):
        super().init()

    def set_velocity(self,v_m, a_m=None):
        super().set_velocity(v_m,a_m)

    def move_by(self,x_m, v_m=None, a_m=None):
        super().move_to(x_m,v_m,a_m)

    def move_to(self,x_m, v_m=None, a_m=None):
        super().move_to(x_m,v_m,a_m)
    def home(self):
        super().home()

    def _startup(self):
        super().startup()
    def _stop(self):
        super().stop()
    def _push_command(self):
        super().push_command()
    def _pull_status(self):
        super().pull_status()