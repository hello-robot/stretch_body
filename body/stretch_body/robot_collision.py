#! /usr/bin/env python

from stretch_body.device import Device
import importlib
import urdfpy
import numpy as np
import os
import time

class EndOfArmForwardKinematics():
    # Compute the FK for a tool link wrt to the fixed end_of_arm frame (link_arm_l0)
    def __init__(self):
        urdf_file = os.path.join(os.environ['HELLO_FLEET_PATH'], os.environ['HELLO_FLEET_ID'],'exported_urdf/stretch.urdf')
        self.robot_model = urdfpy.URDF.load(urdf_file)

    def tool_fk(self,cfg,link):
        # returns the 4x4 transform from <link> to link_arm_l0
        #cfg: dictionary of joint positions of tool (including wrist yaw). Eg: {'joint_wrist_yaw': 0.1, 'joint_gripper_finger_right': 0.1}
        #link: name of link that is after wrist_yaw in the kinematic chain.Eg 'link_gripper_fingertip_right'
        #For reference, link_arm_l0:
        # Origin Center of first cuff
        # X: parallel to ground, points towards wrist
        # Y: Parallel to graviy, points up
        # Z:  Parallel to arm extension, points towards reach

        joint_cfg = self.robot_model._process_cfg(cfg)
        link_set = set()
        link_set.add(self.robot_model._link_map[link])

        # This is a modified version of link_fk of urdfpy
        # That stops FK at the 'link_wrist_yaw' of Stretch
        for lnk in self.robot_model._reverse_topo:
            if lnk not in link_set:
                continue
            pose = np.eye(4, dtype=np.float64)
            path = self.robot_model._paths_to_base[lnk]
            for i in range(len(path) - 1):
                child = path[i]
                parent = path[i + 1]
                joint = self.robot_model._G.get_edge_data(child, parent)['joint']
                cfg = None
                if joint in joint_cfg:
                    cfg = joint_cfg[joint]
                pose = joint.get_child_pose(cfg).dot(pose)
                if parent.name == 'link_arm_l0':
                    return pose
        return None

class RobotCollisionModel(Device):
    """
    The RobotCollisionManager  provides simple self-collision avoidance
    It works by setting the acceptible joint range for each joint based
    on the current kinematic state of the robot.

    A custom collision class can be instantiated by declaring the class name / Python module name
    in the User YAML file
    """
    def __init__(self,name):
        Device.__init__(self)
        self.name=name
        print name
        self.param = self.robot_params[name]



class BaseToolCollision(RobotCollisionModel):
        def __init__(self):
            RobotCollisionModel.__init__(self, 'base_tool_collision')
            self.fk = EndOfArmForwardKinematics()
            self.ts_last=time.time()
        def step(self,status):
            cfg={'joint_wrist_yaw':status['end_of_arm']['wrist_yaw']['pos']}
            #cfg = {'joint_wrist_yaw': time.time()}
            pose=self.fk.tool_fk(cfg,'link_gripper_fingertip_right')
            dt=time.time()-self.ts_last
            self.ts_last=time.time()
            #tooltip=[pose[0][3],pose[1][3],pose[2][3]]
            #print dt
            #print tooltip
            #print '------------'
            x=pose[0][3]
            y=pose[1][3]
            z=pose[2][3]
            print 'Extension', z
            print 'Height',y
            print 'Forward',x


class LiftRangeOfMotion(RobotCollisionModel):
    def __init__(self,robot):
        RobotCollisionModel.__init__(self, 'base_tool_collision')
        self.fk = EndOfArmForwardKinematics()
        self.robot=robot

    def step(self):
        cfg = {'joint_wrist_yaw': status['end_of_arm']['wrist_yaw']['pos']}
        pose = self.fk.tool_fk(cfg, 'link_gripper_fingertip_right')
        x = pose[0][3] #Forward
        y = pose[1][3] #Height
        z = pose[2][3] #Extension'
        status=self.robot.get_status()
        #Limit lower range when arm is retracted and tool down
        x_min=0
        if status['arm']['pos']<0.25: #m
            x_min=max(.075,-1.25*y)
            print 'Setting range down to ',x_min
            self.robot.lift.set_motion_limits(x_min,None)
        else:
            self.robot.lift.set_motion_limits(None, None)

        #print('Extension', z)
        #print('Height', y)
        #print('Forward', x)

class RobotCollisionManager(Device):
    """
    The RobotCollisionManager manages a set of collision models, as defined in YAML.
    It is called periodically by the Robot thread.
    Each model computes the acceptible joint range given the current kinematic state of the robot.
    The RobotCollisionManager then sets the joint limits for each joint to the most restrive set of ranges
    given all models.
    """
    def __init__(self,robot):
        Device.__init__(self)
        urdf_file = os.path.join(os.environ['HELLO_FLEET_PATH'], os.environ['HELLO_FLEET_ID'],'exported_urdf/stretch.urdf')
        self.robot_model = urdfpy.URDF.load(urdf_file)
        self.robot=robot
        self.param=self.robot_params['robot_collision_manager']
        self.model_names = self.param['models'].keys()
        self.models=[]
        self.robot.lift.collision_manager=self
        for m in self.model_names:
            module_name = self.param['models'][m]['py_module_name']
            class_name = self.param['models'][m]['py_class_name']
            print module_name,class_name
            self.models.append(getattr(importlib.import_module(module_name), class_name)())


    def startup(self):
        pass

    def check(self,joint_name,x_pos):
        status = self.robot.get_status()
        pos_map={'arm:': status['arm']['pos'],'lift': status['lift']['pos'], 'wrist_yaw': status['wrist_yaw']['pos']}
        pos_map[joint_name]=x_pos
        pos_map['arm']= pos_map['arm']*0.25
        cfg = {
            'joint_lift': pos_map['lift'],
            'joint_arm_l0': pos_map['arm'],
            'joint_arm_l1': pos_map['arm'],
            'joint_arm_l2': pos_map['arm'],
            'joint_arm_l3': pos_map['arm'],
            'joint_wrist_yaw': pos_map['wrist_yaw']}
        if
        fk=self.robot_mode.link_fk(cfg=cfg,link='link_gripper_fingertip_right',use_names=True)
        x = fk[0][3]
        y = fk[1][3]
        z = fk[2][3]


    def step(self):
        status=self.robot.get_status()
        for m in self.models:
            m.step(status)


class JointRestriction:
    def __init__(self, joint_name, x_min, x_max):
        self.joint_name = joint_name
        self.safe_range = [x_min, x_max]
        self.constraints = []

    def add_constraint(self, c):
        # A constraint is a list of 'joint conditions, as dictionaries
        # Each dictionary has, for example, the form {'joint_name':'arm','x_min':0.1,'x_max':0.5}
        # A joint condition is met if the joint position is outside the range x_min <--> x_max
        # If x_min or x_max is None then there is no limit on the range of motion in that direction
        # The constraint is 'met' then if all of the joint conditions are met
        self.constraints.append(c)

    def is_out_of_range(self, joint_condition, x_curr):
        # The joint_condition is a dictionary like {'joint_name':'arm','x_min':0.1,'x_max':0.5}
        # Return True if the current joint pose is outside
        # of the acceptible joint range
        x_min = joint_condition['x_min']
        x_max = joint_condition['x_max']
        if x_min is None and x_max is None:
            return False
        elif x_min is None and x_curr < x_max:
            return False
        elif x_max is None and x_curr > x_min:
            return False
        else:
            return (x_curr < x_min or x_curr > x_max)

    def evaluate_constraint(self, constraint, pose):
        # Returns True if all the the joint conditions are out of range (eg AND)
        for joint_condition in constraint:
            x_curr = pose[joint_condition['joint_name']]
            if not self.is_out_of_range(joint_condition, x_curr):
                return False
        return True

    def get_safe_range(self, pose):
        # Pose is a dictionary with each current joint position of the robot
        # Evaluate each constraint
        # If any evaluate are met, return the resctrited range of motion for the joint
        # Otherwise return None
        for c in self.constraints:
            if self.evaluate_constraint(c, pose):
                return self.safe_range
        return [None, None]
