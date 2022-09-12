#! /usr/bin/env python

from stretch_body.robot_collision import *
import math
from stretch_body.hello_utils import *

# #############################################
class CollisionArmCamera(RobotCollisionModel):
    """
    NOTE: Experimental. You may want to turn this off in the params (enable=0)
    RE1/ RE2 camera can clip the arm when lift is all the way up
    and the camera is looking parallel to the ground.
    """

    def __init__(self, collision_manager):
        RobotCollisionModel.__init__(self, collision_manager,'collision_arm_camera')
        self.state={'pan_in_danger_zone':{'ts':None,'near':.05,'duration':1.0, 'triggered':0}}

    def pan_in_danger_zone(self,x_pan):
        return x_pan > -.80 and x_pan < .80

    def tilt_in_danger_zone(self,x_tilt):
        return x_tilt < -.80
    def lift_near_danger_zone(self,x_lift):
        return x_lift >0.9 #Enough distance to deccel
    def lift_in_danger_zone(self,x_lift):
        return x_lift>1.015

    def prevent_lift_raise_into_camera(self,status):
        x_lift=status['lift']['pos']
        x_pan = status['head']['head_pan']['pos']
        x_tilt =  status['head']['head_tilt']['pos']

        if not self.lift_near_danger_zone(x_lift):
            return [None,None]
        #print('Lift %f Pan %f Tilt %f'%(x_lift,x_pan,x_tilt))
        if self.pan_in_danger_zone(x_pan) and self.tilt_in_danger_zone(x_tilt):
            #print('Camera in danger zone')
            return [None,1.02]
        return [None,None]

    def prevent_pan_camera_into_arm(self,status):
        x_lift = status['lift']['pos']
        x_pan = status['head']['head_pan']['pos']
        x_tilt = status['head']['head_tilt']['pos']
        #print('Lift %f Pan %f Tilt %f' % (x_lift, x_pan, x_tilt))
        if not self.lift_in_danger_zone(x_lift) or not self.tilt_in_danger_zone(x_tilt):
            return [None,None]
        if x_pan<0:
            return [None,-.80]
        else:
            return [.80,None]


    def prevent_tilt_camera_into_arm(self,status):
        x_lift = status['lift']['pos']
        x_pan = status['head']['head_pan']['pos']
        x_tilt = status['head']['head_tilt']['pos']
        #print('Lift %f Pan %f Tilt %f' % (x_lift, x_pan, x_tilt))
        if not self.lift_in_danger_zone(x_lift) or not self.pan_in_danger_zone(x_pan):
            return [None,None]
        return [-0.75,None]

    def step(self, status):
        limits={'lift': [None, None],'head_pan': [None, None],'head_tilt': [None, None]}
        limits['lift']=self.prevent_lift_raise_into_camera(status)
        limits['head_pan']= self.prevent_pan_camera_into_arm(status)
        limits['head_tilt'] = self.prevent_tilt_camera_into_arm(status)
        #print('Limits',limits)
        #print(limits['head_pan'],status['head']['head_pan']['pos'])
        return limits


# #############################################
class CollisionStretchGripper(RobotCollisionModel):
    """
    NOTE: Experimental. You may want to turn this off in the params (enable=0)
    Manage collisions of the standard Stretch Gripper tool with the base
    
    Side Of (direction of drive)
    ^
    |
    |
    ---> Forward of (direction of reach)
    """

    def __init__(self, collision_manager):
        RobotCollisionModel.__init__(self, collision_manager, 'collision_stretch_gripper')

    # ###
    def dist_fingertips_forward_of_yaw(self,x_yaw):
        r_gripper_tips=.24
        return r_gripper_tips* math.cos(x_yaw)

    def dist_fingertips_side_of_yaw(self,x_yaw):
        r_gripper_tips=.24
        return r_gripper_tips* math.sin(x_yaw)

    def dist_fingertips_forward_of_base(self,x_arm,x_yaw):
        return self.dist_yaw_forward_of_base(x_arm)+self.dist_fingertips_forward_of_yaw(x_yaw)
    
    def dist_fingertips_side_of_base(self,x_yaw):
        return self.dist_yaw_side_of_base()+self.dist_fingertips_side_of_yaw(x_yaw)
    
    def prevent_fingertips_lower_into_base(self,x_lift,x_arm,x_yaw):
        #Return lift limits
        d_fwd=self.dist_fingertips_forward_of_base(x_arm,x_yaw)
        d_side=self.dist_fingertips_side_of_base(x_yaw)
        if x_lift<0.1: #on floor
            return [None,None]
        if d_fwd>0.1:
            return [None,None]
        if d_side>0.1:
            return [0.18, None]
        return [0.22,None]

    def prevent_fingtips_yaw_into_base(self,x_yaw,x_lift):
        # Return arm limits
        if x_lift>.085 or x_yaw<1.57:
            return [None,None]
        d=self.dist_fingertips_forward_of_yaw(x_yaw)
        #print('arm limit',.06-d)
        return[0.06-d,None]
    # ###
    
    def dist_puller_forward_of_yaw(self,x_yaw):
        r_puller=.08
        return r_puller * math.sin(x_yaw)

    def dist_yaw_forward_of_base(self,x_arm):
        return x_arm-.01

    def dist_yaw_side_of_base(self):
        return -.065
    def dist_puller_past_base(self,x_arm,x_yaw):
        return self.dist_yaw_forward_of_base(x_arm) + self.dist_puller_forward_of_yaw(x_yaw)

    def dist_palm_past_base(self,x_arm):
        return x_arm-.04



    def prevent_palm_retract_into_base(self,x_lift):
        # Return arm limits
        if x_lift>.085:
            return [None,None]
        return [.075,None]

    def prevent_palm_lower_into_base(self,x_arm):
        # Return lift limits
        if x_arm<.06:
            return [.085,None]
        return [None,None]

    def prevent_puller_lower_into_base(self,x_arm,x_yaw):
        # Return lift limits
        if self.dist_puller_past_base(x_arm,x_yaw)<0.02:
            return [.075,None]
        return [None,None]

    def prevent_puller_retract_into_base(self,x_lift,x_yaw):
        # Return arm limits
        d=self.dist_puller_forward_of_yaw(x_yaw)
        if x_lift>.085 or d>0.01:
            return [None,None]
        return [.05-d,None]

    

    def step(self,status):

        x_arm=status['arm']['pos']
        #print('Xarm', x_arm)
        x_lift = status['lift']['pos']
        x_yaw=status['end_of_arm']['wrist_yaw']['pos']
        w = {'lift': [None, None], 'arm': [None, None], 'wrist_yaw': [None,None]}
        w['lift']=self.limit(self.prevent_fingertips_lower_into_base(x_lift,x_arm,x_yaw),self.limit(self.prevent_palm_lower_into_base(x_arm),self.prevent_puller_lower_into_base(x_arm,x_yaw)))
        w['arm']=self.limit(self.prevent_fingtips_yaw_into_base(x_yaw,x_lift),self.limit(self.prevent_puller_retract_into_base(x_lift,x_yaw),self.prevent_palm_retract_into_base(x_lift)))
        return w

