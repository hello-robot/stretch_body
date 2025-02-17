from stretch_body.common import revision_name
from stretch_body.r1.arm import Arm as ArmR1
from stretch_body.r2.arm import Arm as ArmR2

#API wrapper to arm class

#Todo: build in notion of command complete / idle / fail, and threading of read/write to device
#Todo: expose splined trajectory


class Arm(ArmBase):
    """
    API to the Stretch Stepper Board
    """

    def __init__(self, usb, name=None):
        ArmBase.__init__(self, usb, name)
        self.supported_revisions = {'r1': (ArmR1,),
                                    'r2': (ArmR2,)}

    def expand_protocol_methods(self, protocol_class):
        for attr_name, attr_value in protocol_class.__dict__.items():
            if callable(attr_value) and not attr_name.startswith("__"):
                setattr(self, attr_name, attr_value.__get__(self, Stepper))

    def startup(self, threaded=False):
        """
        First determine which protocol version the uC firmware is running.
        Based on that version, populates Stepper class with the supported specific Stepper_protocol_P* class methods.
        """
        StepperBase.startup(self, threaded=threaded)
        if self.hw_valid:
            if self.board_info['protocol_version'] in self.supported_protocols:
                protocol_classes = self.supported_protocols[self.board_info['protocol_version']]
                for p in protocol_classes[::-1]:
                    self.expand_protocol_methods(p)

class Arm():
    """
    API to the Stretch Arm
    """
    def __init__(self):
        if revision_name=='r1':
            self.__class__=ArmR1
        if revision_name=='r2':
            self.__class__=ArmR2

    def set_velocity(self,v_m, a_m=None):
        self.set_velocity(v_m,a_m)

    def move_by(self,x_m, v_m=None, a_m=None):
        self.move_to(x_m,v_m,a_m)

    def move_to(self,x_m, v_m=None, a_m=None):
        self.move_to(x_m,v_m,a_m)
    def home(self):
        self.home()


    def _startup(self):
        self.startup()
    def _stop(self):
        self.stop()
    def _push_command(self):
        self.push_command()
    def _pull_status(self):
        self.pull_status()