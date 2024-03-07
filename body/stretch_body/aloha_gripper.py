from stretch_body.dynamixel_hello_XL430 import DynamixelHelloXL430
from stretch_body.hello_utils import *

# Aloha Gripper params: https://gist.github.com/hello-fazil/728c0742eb3fb00667fe5d9d1a2b641d

class AlohaGripper(DynamixelHelloXL430):
    def __init__(self, chain=None):
        DynamixelHelloXL430.__init__(self, 'aloha_gripper', chain)
        self.poses = {'open': 1.3,
                      'close': 0}

    def pose(self,p,v_r=None,a_r=None):
        self.move_to(self.poses[p],v_r,a_r)