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
        """Convert motor angle in radians to translation in meters

        Parameters
        ----------
        ang : float.
            The input angle is in radians.

        Returns
        -------
        float:
            The corresponding translation in meters.
        
        Notes
        -------
        This function calculates the translation in meters from an input angle
        in radians, based on the provided parameters.
        """
        return (self.params['chain_pitch']*self.params['chain_sprocket_teeth']/self.params['gr_spur']/(math.pi*2))*ang

    def translate_m_to_motor_rad(self, x):
        """Convert translation in meters to motor angle in radians

        Parameters
        ----------
        x : float.
            The input translation in meters.

        Returns
        -------
        float:
            The corresponding motor angle in radians.

        Notes
        -------
        This function calculates the motor angle in radians from an input
        translation in meters, based on the provided parameters.
        """
        return x/(self.params['chain_pitch']*self.params['chain_sprocket_teeth']/self.params['gr_spur']/(math.pi*2))


    def home(self, end_pos=0.1,to_positive_stop=False, measuring=False):
        return PrismaticJoint.home(self,end_pos=end_pos,to_positive_stop=to_positive_stop,measuring=measuring)