from __future__ import print_function
from stretch_body.prismatic_joint import PrismaticJoint
from stretch_body.device import Device
from stretch_body.trajectories import PrismaticTrajectory

import time
import math


class Arm(PrismaticJoint):
    """
    API to the Stretch Arm
    """
    def __init__(self):
        PrismaticJoint.__init__(self, 'arm')

    # ######### Utilties ##############################

    def motor_rad_to_translate_m(self,ang): #input in rad, output m
        return (self.params['chain_pitch']*self.params['chain_sprocket_teeth']/self.params['gr_spur']/(math.pi*2))*ang

    def translate_m_to_motor_rad(self, x):
        return x/(self.params['chain_pitch']*self.params['chain_sprocket_teeth']/self.params['gr_spur']/(math.pi*2))


    def home(self, end_pos=0.1,to_positive_stop=False, measuring=False):
        return PrismaticJoint.home(self,end_pos=end_pos,to_positive_stop=to_positive_stop,measuring=measuring)