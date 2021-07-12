from stretch_body.dynamixel_hello_XL430 import DynamixelHelloXL430
from stretch_body.hello_utils import *

class WristPitch(DynamixelHelloXL430):
    def __init__(self, chain=None):
        DynamixelHelloXL430.__init__(self, 'wrist_pitch', chain)
        self.poses = {'tool_up': deg_to_rad(45),
                      'tool_down': deg_to_rad(-45)}

    def pose(self,p,v_r=None,a_r=None):
        self.move_to(self.poses[p],v_r,a_r)

