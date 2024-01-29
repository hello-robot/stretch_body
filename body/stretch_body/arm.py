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
    def __init__(self,usb=None):
        PrismaticJoint.__init__(self, name='arm',usb=usb)

    # ######### Utilties ##############################

    def motor_rad_to_translate_m(self,ang): #input in rad, output m
        return (self.params['chain_pitch']*self.params['chain_sprocket_teeth']/self.params['gr_spur']/(math.pi*2))*ang

    def translate_m_to_motor_rad(self, x):
        return x/(self.params['chain_pitch']*self.params['chain_sprocket_teeth']/self.params['gr_spur']/(math.pi*2))


    def home(self, end_pos=0.1,to_positive_stop=False, measuring=False,do_pull_status=True):
        if self.params['use_adv_homing']:
            success,log=PrismaticJoint.home(self,end_pos=0.0,to_positive_stop=True,measuring=True,do_pull_status=do_pull_status)
            effort_max_1=max(log['dir_1']['effort_pct'])
            effort_min_1 = min(log['dir_1']['effort_pct'])
            effort_max_2 = max(log['dir_2']['effort_pct'])
            effort_min_2 = min(log['dir_2']['effort_pct'])

            print('Out',log['dir_1']['effort_pct'])
            print(' ')
            print('In', log['dir_2']['effort_pct'])
            print(' ')
            print('Out effort %f to %f / In effort %f to %f'%(effort_min_1,effort_max_1,effort_min_2,effort_max_2))


        else:
            success,log= PrismaticJoint.home(self,end_pos=end_pos,to_positive_stop=to_positive_stop,measuring=measuring,do_pull_status=do_pull_status)
        return success