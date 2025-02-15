#!/usr/bin/env python3

from __future__ import print_function

import math
from math import *
from stretch_body.stepper import *
from stretch_body.device import Device
from stretch_body.hello_utils import *
from stretch_body.pimu import Pimu
import time
import logging
import numpy
import numpy as np
from stretch_body.hello_utils import rad_to_deg


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

        # Default controller params
        self.stiffness=1.0
        # self.vel_mr=self.translate_to_motor_rad(self.params['motion']['default']['vel_xy_m'])
        # self.accel_mr=self.translate_to_motor_rad(self.params['motion']['default']['accel_xy_m'])
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

    # def set_omni_velocity2(self,ax,ay,w,a_des=None):
    #     '''
    #     ax: linear velocity m/s
    #     at: linear velocity m/s
    #     w: rotation rad/s
    #     '''
    #     if self.params['use_vel_traj']:
    #         ctrl_mode =Stepper.MODE_VEL_TRAJ
    #     else:
    #         ctrl_mode =Stepper.MODE_VEL_PID
    #     u=self.v_base_to_motor_rad(ax,ay,w)
    #     self.wheels[0].set_command(mode=ctrl_mode, v_des=u[0],a_des=a_des)
    #     self.wheels[1].set_command(mode=ctrl_mode, v_des=u[1],a_des=a_des)
    #     self.wheels[2].set_command(mode=ctrl_mode, v_des=u[2],a_des=a_des)

    def set_omni_velocity(self,dir,v_des,a_des=None):#xy_des,a_w_des):
        '''
        dir: x,y,w for direction
        v_des: m/s or rad/s
        a_des: m/s^2 or rad/s^2
        '''
        if self.params['use_vel_traj']:
            ctrl_mode =Stepper.MODE_VEL_TRAJ
        else:
            ctrl_mode =Stepper.MODE_VEL_PID

        if dir =='x' or dir =='y':
            if a_des is not None:
                a_des = min(abs(a_des), self.params['motion']['max']['accel_xy_m'])
            else:
                a_des =  self.params['motion']['default']['accel_xy_m']
        else:
            if a_des is not None:
                a_des = min(abs(a_des), self.params['motion']['max']['accel_w_r'])
            else:
                a_des =  self.params['motion']['default']['accel_w_r']


        ax = self.v_base_to_motor_rad(a_des, 0, 0)
        ay = self.v_base_to_motor_rad(0, a_des, 0)
        aw = self.v_base_to_motor_rad(0, 0, a_des)

        #Workaround accel to zero bug when switching from X to Y dir
        if dir=='x':
            a0=abs(ax[0])
            a1 = abs(ax[1])
            a2 = abs(ax[2])
            [u0, u1, u2] = self.v_base_to_motor_rad(-v_des, 0, 0)
        if dir=='y':
            a0=abs(ay[0])
            a1 = abs(ay[1])
            a2 = abs(ax[2])
            [u0, u1, u2] = self.v_base_to_motor_rad(0, -1*v_des, 0) #Flip so +Y is forward in direction of arm
        if dir=='w':
            a0=abs(aw[0])
            a1 = abs(aw[1])
            a2 = abs(aw[2])
            [u0, u1, u2] = self.v_base_to_motor_rad(0, 0, v_des)

        self.wheels[0].set_command(mode=ctrl_mode, v_des=u0,a_des=a0)
        self.wheels[1].set_command(mode=ctrl_mode, v_des=u1,a_des=a1)
        self.wheels[2].set_command(mode=ctrl_mode, v_des=u2,a_des=a2)

    def rotate_by(self, w_r, v_r=None, a_r=None):
        """
        Incremental translation of the base
        w_r: rotation (rad)
        v_r: rotational velocity max (v_r)
        a_r: rotational acceleration (rad/s^2)
        """

        if v_r is not None:
            v_r = min(abs(v_r), self.params['motion']['max']['vel_w_r'])
        else:
            v_r =  self.params['motion']['default']['vel_w_r']

        if a_r is not None:
            a_r = min(abs(a_r), self.params['motion']['max']['accel_w_r'])
        else:
            a_r =  self.params['motion']['default']['accel_w_r']


        [x0, x1, x2] = self.v_base_to_motor_rad(0, 0, w_r)
        [u0, u1, u2] = self.v_base_to_motor_rad(0, 0, v_r)
        [a0, a1, a2] = self.v_base_to_motor_rad(0, 0, a_r)

        # print('X',[x0, x1, x2])
        # print('V',[u0, u1, u2])
        # print('A',[a0, a1, a2])

        self.wheels[0].set_command(mode=Stepper.MODE_POS_TRAJ_INCR,x_des=x0, v_des=abs(u0), a_des=abs(a0))
        self.wheels[1].set_command(mode=Stepper.MODE_POS_TRAJ_INCR,x_des=x1, v_des=abs(u1), a_des=abs(a1))
        #if (abs(x2)>.0001): #Hack for now to fix drift on back wheel
        self.wheels[2].set_command(mode=Stepper.MODE_POS_TRAJ_INCR,x_des=x2, v_des=abs(u2), a_des=abs(a2))


    def translate_by(self, x_m, y_m, v_m=None, a_m=None): #, stiffness=None, contact_thresh_N=None,contact_thresh=None):
        """
        Incremental translation of the base
        x_m, y_m: desired motion (m)
        v_m: velocity for trapezoidal motion profile (m/s) in direction of translation
        a_m: acceleration for trapezoidal motion profile (m/s^2) in direction of translation
        """
        y_m=-1*y_m #Flip so positive is in direction of arm
        if v_m is not None:
            v_m = min(abs(v_m), self.params['motion']['max']['vel_xy_m'])
        else:
            v_m =  self.params['motion']['default']['vel_xy_m']

        if a_m is not None:
            a_m = min(abs(a_m), self.params['motion']['max']['accel_xy_m'])
        else:
            a_m =  self.params['motion']['default']['accel_xy_m']

        theta=math.atan2(y_m,x_m)#Angle headed
        v_x = v_m*math.cos(theta)
        v_y = v_m*math.sin(theta)
        a_x = a_m*math.cos(theta)
        a_y = a_m*math.sin(theta)

        [x0, x1, x2] = self.v_base_to_motor_rad(x_m, y_m, 0)
        [u0, u1, u2] = self.v_base_to_motor_rad(v_x, v_y, 0)
        [a0, a1, a2] = self.v_base_to_motor_rad(a_x, a_y, 0)

        # print('X',[x0, x1, x2])
        # print('V',[u0, u1, u2])
        # print('A',[a0, a1, a2])
        #print('M0',x0,u0,a0)
        #print('M1', x1, u1, a1)
        #Hack to avoid drift, fix in firmware

        self.wheels[0].set_command(mode=Stepper.MODE_POS_TRAJ_INCR,x_des=x0, v_des=abs(u0), a_des=abs(a0))
        self.wheels[1].set_command(mode=Stepper.MODE_POS_TRAJ_INCR,x_des=x1, v_des=abs(u1), a_des=abs(a1))
        if (abs(x2)>.0001): #Hack for now to fix drift on back wheel
            self.wheels[2].set_command(mode=Stepper.MODE_POS_TRAJ_INCR,x_des=x2, v_des=abs(u2), a_des=abs(a2))

    #Hack for drift, temp
    def enable_hold_mode(self):
        self.wheels[0].set_command(mode=Stepper.MODE_HOLD)
        self.wheels[1].set_command(mode=Stepper.MODE_HOLD)
        self.wheels[2].set_command(mode=Stepper.MODE_HOLD)

    def enable_freewheel_mode(self):
        self.wheels[0].set_command(mode=Stepper.MODE_FREEWHEEL)
        self.wheels[1].set_command(mode=Stepper.MODE_FREEWHEEL)
        self.wheels[2].set_command(mode=Stepper.MODE_FREEWHEEL)

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
        # update the robot's velocity estimates
        base_vel=self.v_motor_rad_to_base([self.wheels[0].status['vel'],self.wheels[1].status['vel'],self.wheels[2].status['vel']])
        self.status['x_vel'] = base_vel[0]
        self.status['y_vel'] = base_vel[1]
        self.status['theta_vel'] = base_vel[2]
        self.status['timestamp_pc'] = time.time()


    # ########### Kinematic Conversions ####################

    def v_motor_rad_to_base (self,wheel_velocities):
        """
        NOTE: Copilot code not yet tested for accuracy/ calibration


        Convert the wheel velocities of a 3-wheel omnidirectional robot to the base velocities.

        Parameters:
        wheel_velocities (list or numpy array): Velocities of the three wheels [V1, V2, V3]
        Returns:
        numpy array: Base velocities [Vx, Vy, Vtheta]
        """
        r = self.params['wheel_diameter_m'] / 2
        d = self.params['base_radius_m']

        # Coefficients for the transformation matrix
        a = r / (3 * d)
        b = np.sqrt(3) * a

        # Transformation matrix from wheel velocities to base velocities
        T_inv = np.array([
            [-0.5, -0.5, -0.5],
            [b, -b, 0],
            [a, a, a]
        ])
        # Convert wheel velocities to base velocities
        base_velocities = np.matmul(T_inv, np.array(wheel_velocities)/self.params['gr'])
        return base_velocities

    # def pos_base_to_motor_rad(self,dw,dx,dy):
    #     r = self.params['wheel_diameter_m'] / 2
    #     d = self.params['base_radius_m']
    #     u1 = self.params['gr'] * (-d * dw + 1.0 * dx + 0                     ) / r
    #     u2 = self.params['gr'] * (-d * dw - 0.5 * dx - sin(math.pi / 3) * dy) / r
    #     u3 = self.params['gr'] * (-d * dw - 0.5 * dx + sin(math.pi / 3) * dy) / r
    #     print('IN', wz, vx, vy)
    #     print('U', u1, u2, u3)
    #     return [u1, u2, u3]



    def v_base_to_motor_rad4(self,ax, ay, w):
        wheel_speeds=self.convert_velocity_to_wheel_speeds(ax,ay,w,r=self.params['wheel_diameter_m']/2,l=self.params['base_radius_m'],gear_ratios=[self.params['gr'],self.params['gr'],self.params['gr']])

        return wheel_speeds
    def v_base_to_motor_rad2(self,ax, ay, w):
        #https://www.youtube.com/watch?v=ULQLD6VvXio
        r=self.params['wheel_diameter_m']/2
        d=self.params['base_radius_m']

        u1=self.params['gr']*(0.58*ax  -0.33*ay + 0.33*w)/r
        u2=self.params['gr']*(-0.58*ax -0.33*ay + 0.33*w)/r
        u3=self.params['gr']*(0            +0.67*ay + 0.33*w)/r
        # print('IN',ax,ay,w)
        # print('Out',u1,u2,u3)
        return [u1,u2,u3]

    def v_base_to_motor_rad(self,vx,vy,wz):
        # https://modernrobotics.northwestern.edu/nu-gm-book-resource/13-2-omnidirectional-wheeled-mobile-robots-part-1-of-2/
        #Kinematics and Control A Three-wheeled Mobile Robot with Omni-directional Wheels
        r=self.params['wheel_diameter_m']/2
        d=self.params['base_radius_m']

        u1=self.params['gr']*(-d*wz + 1.0*vx + 0 *vy)/r
        u2=self.params['gr']*(-d*wz -0.5*vx - sin(math.pi/3)*vy)/r
        u3=self.params['gr']*(-d*wz -0.5*vx +sin(math.pi/3)*vy)/r

        #Flip sign so Y+ is base forward in direction of arm, X+ is right
        return [-1*u2,-1*u3,-1*u1]

    # def translate_to_motor_rad(self,x_m):
    #     circ=self.params['wheel_diameter_m']*math.pi
    #     return deg_to_rad(360*self.params['gr']*x_m/circ)
    #
    # def motor_rad_to_translate(self,x_r):
    #     circ = self.params['wheel_diameter_m'] * math.pi
    #     return rad_to_deg(x_r)*circ/360/self.params['gr']
    #
    # def rotate_to_motor_rad(self,x_r):
    #     r = self.params['wheel_separation_m'] / 2.0
    #     c = r * x_r #distance wheel travels (m)
    #     return self.translate_to_motor_rad(c)
    #
    # def motor_rad_to_rotate(self, x_r):
    #     c = self.motor_rad_to_translate(x_r)
    #     r = self.params['wheel_separation_m'] / 2.0
    #     ang_rad = c /r
    #     return ang_rad
    #
    # def translation_to_rotation(self,x_m):
    #     x_mr=self.translate_to_motor_rad(x_m)
    #     x_r=self.motor_rad_to_rotate(x_mr)
    #     return x_r
    #
    # def rotation_to_translation(self,x_r):
    #     x_mr=self.rotate_to_motor_rad(x_r)
    #     x_m=self.motor_rad_to_translate(x_mr)
    #     return x_m



if __name__ == "__main__":
    b=OmniBase()
    b.startup()
    p=Pimu()
    p.startup()

    b.set_omni_velocity(dir='x', v_des=-0.1, a_des=0.25)
    #b.set_omni_velocity(3.0,0.0,0.0)
    b.push_command()
    p.trigger_motor_sync()
    p.push_command()

    time.sleep(6.0)
    # for i in range(1000):
    #     b.pull_status()
    #     b.pretty_print()
    #     time.sleep(.02)
    b.stop()
    p.stop()