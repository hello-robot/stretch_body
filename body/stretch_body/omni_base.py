#!/usr/bin/env python3

from __future__ import print_function
from math import *
from stretch_body.stepper import *
from stretch_body.device import Device
from stretch_body.trajectories import DiffDriveTrajectory
from stretch_body.hello_utils import *
import time
import logging
import numpy

class OmniBase(Device):
    """
    API to the Stretch Mobile Base
    """
    def __init__(self):
        Device.__init__(self, 'omnibase')
        self.wheels=[Stepper(usb='/dev/hello-omni-1', name='hello-omni-1'),
                     Stepper(usb='/dev/hello-omni-2', name='hello-omni-2'),
                     Stepper(usb='/dev/hello-omni-3', name='hello-omni-3')]
        self.status = {'timestamp_pc':0}
        self.thread_rate_hz = 5.0
        self.first_step=True
        # wheel_circumference_m = self.params['wheel_diameter_m'] * pi
        # self.meters_per_motor_rad = (wheel_circumference_m / (2.0 * pi)) / self.params['gr']
        # self.wheel_separation_m = self.params['wheel_separation_m']

        # Default controller params
        self.stiffness=1.0
        self.vel_mr=self.translate_to_motor_rad(self.params['motion']['default']['vel_m'])
        self.accel_mr=self.translate_to_motor_rad(self.params['motion']['default']['accel_m'])
        self.fast_motion_allowed = True
    # ###########  Device Methods #############

    def startup(self, threaded=True):
        #Startup steppers first so that status is populated before this Device thread begins (if threaded==true)
        success = self.wheels[0].startup(threaded=False) and self.wheels[1].startup(threaded=False) and self.wheels[2].startup(threaded=False)
        if success:
            Device.startup(self, threaded=threaded)
            self.__update_status()
        return success

    def stop(self):
        Device.stop(self)
        for w in self.wheels:
            w.stop()

    def pretty_print(self):
        print('----------Base------')
        print('Timestamp PC (s):', self.status['timestamp_pc'])
        print('-----Omni1-----')
        self.wheels[0].pretty_print()
        print('-----Omni2-----')
        self.wheels[1].pretty_print()
        print('-----Omni3-----')
        self.wheels[2].pretty_print()

    # ###################################################
    def enable_freewheel_mode(self):
        """
        Force motors into freewheel
        """
        for w in self.wheels:
            w.enable_freewheel()

    def enable_pos_incr_mode(self):
        """
                Force motors into incremental position mode
        """
        for w in self.wheels:
            w.enable_pos_traj_incr()

    def set_omni_velocity(self,ax,ay,w):
        '''
        ax: linear velocity m/s
        at: linear velocity m/s
        w: rotation rad/s
        '''
        if self.params['use_vel_traj']:
            ctrl_mode =Stepper.MODE_VEL_TRAJ
        else:
            ctrl_mode =Stepper.MODE_VEL_PID
        u=self.v_base_to_motor_rad(ax,ay,w)
        self.wheels[0].set_command(mode=ctrl_mode, v_des=u[0])
        self.wheels[1].set_command(mode=ctrl_mode, v_des=u[1])
        self.wheels[2].set_command(mode=ctrl_mode, v_des=u[2])




    # ###################################################
    def push_command(self):
        for w in self.wheels:
            w.push_command()


    def pull_status(self):
        """
        Computes base odometery based on stepper positions / velocities
        """
        for w in self.wheels:
            w.pull_status()
        self.__update_status()

    async def pull_status_async(self):
        """
        Computes base odometery based on stepper positions / velocities
        """
        await self.wheels[0].pull_status_async()
        await self.wheels[1].pull_status_async()
        await self.wheels[2].pull_status_async()
        self.__update_status()

    def __update_status(self):
        self.status['timestamp_pc'] = time.time()


    # ########### Kinematic Conversions ####################

    # def pos_base_to_motor_rad(self,dw,dx,dy):
    #     r = self.params['wheel_diameter_m'] / 2
    #     d = self.params['base_radius_m']
    #     u1 = self.params['gr'] * (-d * dw + 1.0 * dx + 0                     ) / r
    #     u2 = self.params['gr'] * (-d * dw - 0.5 * dx - sin(math.pi / 3) * dy) / r
    #     u3 = self.params['gr'] * (-d * dw - 0.5 * dx + sin(math.pi / 3) * dy) / r
    #     print('IN', wz, vx, vy)
    #     print('U', u1, u2, u3)
    #     return [u1, u2, u3]

    def v_base_to_motor_rad(self,ax, ay, w):
        #https: // www.youtube.com / watch?v = ULQLD6VvXio
        r=self.params['wheel_diameter_m']/2
        d=self.params['base_radius_m']

        u1=self.params['gr']*(0.58*ax  -0.33*ay + 0.33*w)/r
        u2=self.params['gr']*(-0.58*ax -0.33*ay + 0.33*w)/r
        u3=self.params['gr']*(0        +0.67*ay + 0.33*w)/r
        print('IN',ax,ay,w)
        print('Out',u1,u2,u3)
        return [u1,u2,u3]

    def v_base_to_motor_rad2(self,wz,vx,vy):
        # https://modernrobotics.northwestern.edu/nu-gm-book-resource/13-2-omnidirectional-wheeled-mobile-robots-part-1-of-2/
        #Kinematics and Control A Three-wheeled Mobile Robot with Omni-directional Wheels
        r=self.params['wheel_diameter_m']/2
        d=self.params['base_radius_m']

        u1=self.params['gr']*(-d*wz + vx  )/r
        u2=self.params['gr']*(-d*wz -0.5*vx - sin(math.pi/3)*vy)/r
        u3=self.params['gr']*(-d*wz -0.5*vx +sin(math.pi/3)*vy)/r
        print('IN',wz,vx,vy)
        print('U',u1,u2,u3)
        return [u1,u2,u3]

    def translate_to_motor_rad(self,x_m):
        circ=self.params['wheel_diameter_m']*math.pi
        return deg_to_rad(360*self.params['gr']*x_m/circ)

    def motor_rad_to_translate(self,x_r):
        circ = self.params['wheel_diameter_m'] * math.pi
        return rad_to_deg(x_r)*circ/360/self.params['gr']

    def rotate_to_motor_rad(self,x_r):
        r = self.params['wheel_separation_m'] / 2.0
        c = r * x_r #distance wheel travels (m)
        return self.translate_to_motor_rad(c)

    def motor_rad_to_rotate(self, x_r):
        c = self.motor_rad_to_translate(x_r)
        r = self.params['wheel_separation_m'] / 2.0
        ang_rad = c /r
        return ang_rad

    def translation_to_rotation(self,x_m):
        x_mr=self.translate_to_motor_rad(x_m)
        x_r=self.motor_rad_to_rotate(x_mr)
        return x_r

    def rotation_to_translation(self,x_r):
        x_mr=self.rotate_to_motor_rad(x_r)
        x_m=self.motor_rad_to_translate(x_mr)
        return x_m



if __name__ == "__main__":
    b=OmniBase()
    b.startup()
    b.set_omni_velocity(3.0,0.0,0.0)
    b.push_command()
    time.sleep(5.0)
    # for i in range(1000):
    #     b.pull_status()
    #     b.pretty_print()
    #     time.sleep(.02)
    b.stop()