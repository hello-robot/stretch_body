from stretch_body.dynamixel_hello_XL430 import DynamixelHelloXL430
from stretch_body.hello_utils import *

# Aloha Gripper params: https://gist.github.com/hello-fazil/728c0742eb3fb00667fe5d9d1a2b641d

class AlohaGripper(DynamixelHelloXL430):
    def __init__(self, chain=None):
        DynamixelHelloXL430.__init__(self, 'stretch_gripper', chain)
        self.poses = {'open': 50,
                      'close': -100}
        self.status['pos_pct'] = 0.0

    def world_rad_to_pct(self, r):
        # Map physical motor radians (0 to 1.3) to standard pct units (-100 to 50)
        normalized = r / 1.3
        return (normalized * 150.0) - 100.0

    def pct_to_world_rad(self, pct):
        # Map standard pct units (-100 to 50) to physical motor radians (0 to 1.3)
        normalized = (pct + 100.0) / 150.0
        return normalized * 1.3

    def pose(self, p, v_r=None, a_r=None):
        self.move_to(self.poses[p], v_r, a_r)
    
    def move_to(self, pct, v_r=None, a_r=None):
        rad = self.pct_to_world_rad(pct)
        DynamixelHelloXL430.move_to(self, x_des=rad, v_des=v_r, a_des=a_r)

    def move_by(self, delta_pct, v_r=None, a_r=None):
        self.pull_status()
        self.move_to(self.status['pos_pct'] + delta_pct, v_r, a_r)

    def pull_status(self, data=None):
        DynamixelHelloXL430.pull_status(self, data)
        self.status['pos_pct'] = self.world_rad_to_pct(self.status['pos'])
    
    def home(self, move_to_zero=True):
        DynamixelHelloXL430.home(self, single_stop=True, move_to_zero=move_to_zero, delay_at_stop=2.25)

    def stow(self):
        self.move_to(-25)