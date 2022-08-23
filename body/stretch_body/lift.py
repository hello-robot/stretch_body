from __future__ import print_function
from stretch_body.prismatic_joint import PrismaticJoint
from stretch_body.device import Device
from stretch_body.trajectories import PrismaticTrajectory

import time
import math


class Lift(PrismaticJoint):
    """
    API to the Stretch Lift
    """
    def __init__(self):
        PrismaticJoint.__init__(self, 'lift')

    # ######### Utilties ##############################

    def motor_rad_to_translate_m(self,ang): #input in rad
        d=self.params['pinion_t']*self.params['belt_pitch_m']/math.pi
        lift_m = (math.degrees(ang)/180.0)*math.pi*(d/2)
        return lift_m

    def translate_m_to_motor_rad(self, x):
        d = self.params['pinion_t'] * self.params['belt_pitch_m'] / math.pi
        ang = 180*x/((d/2)*math.pi)
        return math.radians(ang)

    def home(self, end_pos=0.6,to_positive_stop=True, measuring=False):
        return PrismaticJoint.home(self,end_pos=end_pos,to_positive_stop=to_positive_stop,measuring=measuring)


