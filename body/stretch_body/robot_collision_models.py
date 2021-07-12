#! /usr/bin/env python

from stretch_body.robot_collision import *
import math
from stretch_body.hello_utils import *

# #############################################
class CollisionArmCamera(RobotCollisionModel):
    """
    NOTE: Experimental. You may want to turn this off in the params (enable=0)
    RE1 camera can clip the arm when lift is all the way up
    and the camera is looking parallel to the ground.
    """

    def __init__(self, collision_manager):
        RobotCollisionModel.__init__(self, collision_manager,'collision_arm_camera')
        self.curr_workspace='no_limits'
        self.workspaces ={
            'no_limits':{
                'lift': [None, None],
                'head_pan': [None, None],
                'head_tilt': [None, None]},
            'lift_up_limit_head_pan_pos':{
                'lift':[None, None],
                'head_pan':[self.params['head_pan_avoid_clip_pos'],None],
                'head_tilt':[None,None]},
            'lift_up_limit_head_pan_neg': {
                'lift': [None, None],
                'head_pan': [None, self.params['head_pan_avoid_clip_neg']],
                'head_tilt': [None, None]},
            'lift_up_limit_lift':{
                'lift':[None, self.params['lift_near_clip']],
                'head_pan': [None, None],
                'head_tilt': [None,None]},
            'lift_up_limit_head_tilt': {
                'lift': [None, None],
                'head_pan': [None, None],
                'head_tilt': [self.params['head_tilt_avoid_clip'], None]}
        }

    def step(self, status):
        lift_approach_clip= status['lift']['pos'] > self.params['lift_approach_clip']
        head_tilt_in_clip_range = status['head']['head_tilt']['pos'] < self.params['head_tilt_avoid_clip']
        head_pan_in_clip_range = status['head']['head_pan']['pos']<self.params['head_pan_avoid_clip_pos'] and status['head']['head_pan']['pos']>self.params['head_pan_avoid_clip_neg']

        if  not lift_approach_clip:
            self.curr_workspace = 'no_limits'
            return self.workspaces[self.curr_workspace]

        if self.curr_workspace == 'no_limits' and lift_approach_clip: #Lift is entering clip zone
            #Case 1: Pan and tilt are in clip zone, limit lift range
            #Case 2: Tilt is in clip zone but pan is not (on pos side), keep pan on pos
            #Case 3: Tilt is in clip zone but pan is not (on neg side), keep pan on neg
            #Case 4: Just limit tilt while lift is in the clip zone
            if  head_pan_in_clip_range and head_tilt_in_clip_range:
                self.curr_workspace = 'lift_up_limit_lift'
                return self.workspaces[self.curr_workspace]
            elif not head_pan_in_clip_range and head_tilt_in_clip_range:
                if status['head']['head_pan']['pos']>=self.params['head_pan_avoid_clip_pos']:
                    self.curr_workspace = 'lift_up_limit_head_pan_pos'
                else:
                    self.curr_workspace = 'lift_up_limit_head_pan_neg'
            else:
                self.curr_workspace = 'lift_up_limit_head_tilt'
        return self.workspaces[self.curr_workspace]

# #############################################
class CollisionStretchGripper(RobotCollisionModel):
    """
    NOTE: Experimental. You may want to turn this off in the params (enable=0)
    Manage collisions of the standard Stretch Gripper tool with the
    ground and the base
    """

    def __init__(self, collision_manager):
        RobotCollisionModel.__init__(self, collision_manager, 'collision_stretch_gripper')

    def step(self, status):
        wrist_yaw_limit = [None, None]
        arm_retract_limit=None
        lift_lower_limit=None


        # With arm retracted, current distal position of feature relative to right edge of base (negative towards mast)
        dist_palm_past = -1 * self.params['arm_palm_beyond_base']
        dist_gripper_past = dist_palm_past * 0.5 + self.params['r_gripper_tips'] * math.cos(status['end_of_arm']['wrist_yaw']['pos'])
        dist_puller_past = dist_palm_past * 0.5 + self.params['r_puller'] * math.sin(status['end_of_arm']['wrist_yaw']['pos'])
        dist_padding = 1.1

        #dist_gripper_past_forward = self.params['r_gripper_tips'] * math.sin(status['end_of_arm']['wrist_yaw']['pos']) - 0.15

        dist_past = min(min(dist_palm_past, dist_gripper_past), dist_puller_past) * dist_padding
        arm_reach_required = -1 * dist_past
        tool_over_base = status['arm']['pos']<=arm_reach_required and status['lift']['pos']>self.params['lift_palm_above_base']
        r_eff = (self.params['r_gripper_tips'] + self.params['arm_palm_beyond_base'] * 0.5) * dist_padding  # effective swept circle of gripper
        q = math.acos(min(1.0,max(-1.0,-1 * status['arm']['pos'] / r_eff)))  # How far tool can stow before hitting base
        q = max(status['end_of_arm']['wrist_yaw']['pos'],q)  # Only allow wrist to fold back further as arm extends, when retracting limit arm retraction
        #Set lower lift limit if tool over the base
        if tool_over_base:
            if dist_gripper_past>0:
                lift_lower_limit=min(status['lift']['pos'],self.params['lift_palm_above_base']) #Only allow limit moving downward
            else:
                lift_lower_limit=min(status['lift']['pos'],self.params['lift_fingertip_above_base'])
            if dist_gripper_past<0 and status['lift']['pos']<self.params['lift_fingertip_above_base']:
                arm_retract_limit = min(status['arm']['pos'], arm_reach_required)
                wrist_yaw_limit = [None, q]
        else:
            # Compute the allowable wrist yaw range give base clearance/arm extension
            if status['arm']['pos'] < r_eff:
                arm_retract_limit = min(status['arm']['pos'], arm_reach_required)  # Only allow for increasing ability to retract from current position
                wrist_yaw_limit = [None, q]
            else:
                arm_retract_limit=self.params['arm_palm_beyond_base']

        w={ 'lift':[lift_lower_limit, None], 'arm': [arm_retract_limit,None], 'wrist_yaw':wrist_yaw_limit}

        return w

