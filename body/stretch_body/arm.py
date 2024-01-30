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
            print('---- Homing arm ---')
            print('--Finding extension limit--')
            success_pos,log_pos=self.home_single_ended(end_pos=0.4,to_positive_stop=True,measuring=False,do_pull_status=do_pull_status)

            if not success_pos:
                print('Warning: Arm is having difficulty with extension homing. Contact Hello Robot support.')
                return False

            x_pos=log_pos['x_contact']
            print('--Finding retraction limit--')
            success_neg,log_neg = self.home_single_ended(end_pos=0.1, to_positive_stop=False, measuring=True,do_pull_status=do_pull_status)
            x_neg=log_neg['x_contact']

            if not success_neg:
                print('Warning: Arm is having difficulty with retraction homing. Contact Hello Robot support.')
                return False

            print('--- Results ---')
            measured_rom=abs(x_pos-x_neg)
            print('Arm homed from %f to %f (m) for a range of %f'%(x_pos,x_neg,measured_rom))

            if (x_neg>0.02 or measured_rom<0.48):
                print('Warning: Arm is having difficulty with range of motion. Contact Hello Robot support.')
                return False

            self.params['range_m'][0] = x_neg
            self.write_configuration_param_to_YAML('%s.range_m' % self.name, self.params['range_m'])
            print('Arm successfully homed')
            return True

            # effort_max_pos = max(log_pos['data']['effort_pct'])
            # effort_min_pos = min(log_pos['data']['effort_pct'])
            # effort_max_neg = max(log_neg['data']['effort_pct'])
            # effort_min_neg = min(log_neg['data']['effort_pct'])
            # print('Extension effort min: %f max: %f '%(effort_min_pos,effort_max_pos))
            # print('Retraction effort min: %f max: %f ' % (effort_min_neg, effort_max_neg))


        else:
            return PrismaticJoint.home(self,end_pos=end_pos,to_positive_stop=to_positive_stop,measuring=measuring)
