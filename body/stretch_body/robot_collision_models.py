#! /usr/bin/env python

from stretch_body.robot_collision import *
import math
from stretch_body.hello_utils import *

#! /usr/bin/env python

from stretch_body.robot_collision import *
from stretch_body.hello_utils import *
import math


class CollisionStretchDexWristToSelf(RobotCollisionModel):
    """
    NOTE: Experimental. You may want to turn this off in the params (enable=0)
    Manage collisions of the standard Stretch Gripper tool with the
    ground and the base
    """

    def __init__(self, collision_manager):
        RobotCollisionModel.__init__(self, collision_manager, 'collision_stretch_dex_wrist_to_self')

    def step(self, status):
        x_roll=status['end_of_arm']['wrist_roll']['pos']
        x_pitch = status['end_of_arm']['wrist_pitch']['pos']
        x_yaw = status['end_of_arm']['wrist_yaw']['pos']

        if x_pitch>self.params['pitch_up_thresh']:
            if x_roll<=deg_to_rad(-135) or x_roll>=deg_to_rad(135): #Servo body up
                workspace='limit_pitch_up_palm_up'
            if (x_roll<=deg_to_rad(135) and x_roll>=deg_to_rad(45)) or (x_roll>=deg_to_rad(-135) and x_roll<=deg_to_rad(-45)): #Servo body sideqays
                workspace='limit_pitch_up_palm_side'
            if x_roll<=deg_to_rad(45) and x_roll>=deg_to_rad(-45): #Servo body down
                workspace='limit_pitch_up_palm_down'
            w={ 'wrist_yaw':self.params[workspace]['yaw'],'wrist_pitch':self.params[workspace]['pitch'],'wrist_roll':[None, None]}
            return w
        else:
            return  {'wrist_yaw':[None,None],'wrist_pitch':[None,None],'wrist_roll':[None, None]}

class CollisionStretchDexWristToBase(RobotCollisionModel):
    """
    NOTE: Experimental. You may want to turn this off in the params (enable=0)
    Manage collisions of the standard Stretch Gripper tool with the
    ground and the base
    """

    def __init__(self, collision_manager):
        RobotCollisionModel.__init__(self, collision_manager, 'collision_stretch_dex_wrist_to_base')
        self.fk=EndOfArmForwardKinematics()

    def step(self, status):
        cfg = {
            'joint_wrist_yaw': status['end_of_arm']['wrist_yaw']['pos'],
            'joint_wrist_pitch': status['end_of_arm']['wrist_pitch']['pos'],
            'joint_wrist_roll': status['end_of_arm']['wrist_roll']['pos'],
            'joint_gripper_finger_left':0.0
        }
        pose = self.fk.tool_fk(cfg, 'link_gripper_fingertip_left')
        forward = pose[0][3]  # Forward, pos is towards wheels
        height = pose[1][3]  # Height , pos is up, pad out
        extension = pose[2][3]  # Extension, pos is reach
        if 0:
            print('------')
            print('Forward', forward)
            print('Height' ,height)
            print('Extension', extension)
            print('Lift',status['lift']['pos'])
            print('Arm', status['arm']['pos'])
            print("Wrist pitch",rad_to_deg(status['end_of_arm']['wrist_pitch']['pos']))


        wrist_yaw_fwd = deg_to_rad(90.0) #tool pointing to the front
        tool_clear_of_base = (status['end_of_arm']['wrist_yaw']['pos'] < wrist_yaw_fwd) and (status['arm']['pos'] > self.params['arm_clear_base'])
        tool_to_floor=max(0, status['lift']['pos']+self.params['base_to_floor_offset']+height)


        if tool_clear_of_base:
            lift_lower_limit = min(max(0,status['lift']['pos']-tool_to_floor), status['lift']['pos'])  # Only low it to reduce the ROM
        else:
            lift_lower_limit = min(max(self.params['palm_height'], -1*height), status['lift']['pos'])  # Only low it to reduce the ROM

        pitch_lower_limit = None
        if status['lift']['pos']>self.params['palm_height'] and status['lift']['pos']<self.params['lift_up']:
            if status['arm']['pos']<self.params['arm_clear_base']:
                q = -1*math.atan2(status['lift']['pos'],self.params['arm_clear_base'])  # How far pitch can fold back before hitting base
                pitch_lower_limit= min(status['end_of_arm']['wrist_pitch']['pos'],q)  # Only allow pitch to fold back further as lift raises, when lowering limit lift descent

        return {'lift': [lift_lower_limit, None], 'arm': [None, None], 'wrist_yaw': [None, None], 'wrist_pitch': [pitch_lower_limit, None], 'wrist_roll': [None, None]}





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

