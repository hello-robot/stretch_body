from stretch_body.end_of_arm import *
import threading
import time

# ############# STRETCH RE1 / RE2 #######################
class ToolNone(EndOfArm):
    """
    Just a WristYaw for RE1 / Stretch 2
    """
    def __init__(self):
        EndOfArm.__init__(self, name='tool_none')

        #This maps from the name of a joint in the URDF to the name of the joint in Stretch Body
        #It is used by CollisionMgmt 
        self.urdf_map={'joint_wrist_yaw':'wrist_yaw'} #Not mapping fingers for collision mgmt yet
    def stow(self):
        # Fold in wrist and gripper
        print('--------- Stowing Wrist Yaw ----')
        self.move_to('wrist_yaw', self.params['stow']['wrist_yaw'])

class ToolStretchGripper(EndOfArm):
    """
    Wrist Yaw + standard Stretch Gripper for RE1 / Stretch 2
    """
    def __init__(self):
        EndOfArm.__init__(self, name='tool_stretch_gripper')

        #This maps from the name of a joint in the URDF to the name of the joint in Stretch Body
        #It is used by CollisionMgmt
        self.urdf_map={'joint_wrist_yaw':'wrist_yaw'} #Not mapping fingers for collision mgmt yet

    def stow(self):
        # Fold in wrist and gripper
        print('--------- Stowing Wrist Yaw ----')
        self.move_to('wrist_yaw', self.params['stow']['wrist_yaw'])
        print('--------- Stowing Gripper ----')
        self.move_to('stretch_gripper', self.params['stow']['stretch_gripper'])

    # def get_joint_configuration(self,braked=False):
    #     """
    #     Construct a dictionary of tools current pose (for robot_collision_mgmt)
    #     Keys match joint names in URDF
    #     """
    #     g=self.get_motor('stretch_gripper')
    #     _, gripper_finger_rad, _, _ = g.gripper_conversion.status_to_all(g.status)
    #     return {
    #         'joint_gripper_finger_left': gripper_finger_rad,
    #         'joint_gripper_finger_right': gripper_finger_rad}

class ToolStretchDexWrist(EndOfArm):
    """
    DexWrist2 (brought over from toolshare) + straight gripper for RE1 / Stretch 2
    """
    def __init__(self, name='tool_stretch_dex_wrist'):
        EndOfArm.__init__(self, name)

        #This maps from the name of a joint in the URDF to the name of the joint in Stretch Body
        #It is used by CollisionMgmt
        self.urdf_map={
            'joint_wrist_yaw':'wrist_yaw',
            'joint_wrist_pitch': 'wrist_pitch',
            'joint_wrist_roll':'wrist_roll' #Not mapping fingers for collision mgmt yet
        }
    def stow(self):
        # Fold in wrist and gripper
        print('--------- Stowing ToolStretchDexWrist ----')
        self.move_to('wrist_pitch', self.params['stow']['wrist_pitch'])
        self.move_to('wrist_roll', self.params['stow']['wrist_roll'])
        self.move_to('wrist_yaw', self.params['stow']['wrist_yaw'])
        self.move_to('stretch_gripper', self.params['stow']['stretch_gripper'])

    def home(self):
        self.motors['stretch_gripper'].home()
        self.motors['wrist_pitch'].move_to(0)
        self.motors['wrist_roll'].move_to(0)
        self.motors['wrist_yaw'].home()

    def pre_stow(self,robot=None):
        if robot:
            robot.end_of_arm.move_to('wrist_pitch', robot.end_of_arm.params['stow']['wrist_pitch'])
        else:
            self.move_to('wrist_pitch', self.params['stow']['wrist_pitch'])

# ##########################################################3#
class EOA_Wrist_DW3_Tool_NIL(EndOfArm):
    """
    Wrist Yaw / Pitch / Roll only for version 3 of DexWrist
    """
    def __init__(self, name='eoa_wrist_dw3_tool_nil'):
        EndOfArm.__init__(self, name)

        #This maps from the name of a joint in the URDF to the name of the joint in Stretch Body
        #It is used by CollisionMgmt.
        self.urdf_map={
            'joint_wrist_yaw':'wrist_yaw',
            'joint_wrist_pitch': 'wrist_pitch',
            'joint_wrist_roll':'wrist_roll'}
    def stow(self):
        # Fold in wrist and gripper
        print('--------- Stowing %s ----'%self.name)
        self.move_to('wrist_pitch', self.params['stow']['wrist_pitch'])
        self.move_to('wrist_roll', self.params['stow']['wrist_roll'])
        self.move_to('wrist_yaw', self.params['stow']['wrist_yaw'])

    def home(self):
        self.motors['wrist_pitch'].move_to(self.params['stow']['wrist_pitch'])
        self.motors['wrist_roll'].move_to(self.params['stow']['wrist_roll'])
        self.motors['wrist_yaw'].home()

    def pre_stow(self,robot=None):
        if robot:
            robot.end_of_arm.move_to('wrist_pitch', robot.end_of_arm.params['stow']['wrist_pitch'])
        else:
            self.move_to('wrist_pitch', self.params['stow']['wrist_pitch'])

class EOA_Wrist_DW3_Tool_SG3(EndOfArm):
    """
    Wrist Yaw / Pitch / Roll + Stretch Gripper 3
    """
    def __init__(self, name='eoa_wrist_dw3_tool_sg3'):
        EndOfArm.__init__(self, name)

        #This maps from the name of a joint in the URDF to the name of the joint in Stretch Body
        #It is used by CollisionMgmt
        self.urdf_map={
            'joint_wrist_yaw':'wrist_yaw',
            'joint_wrist_pitch': 'wrist_pitch',
            'joint_wrist_roll':'wrist_roll' #Not mapping fingers for collision mgmt yet
        }

    def stow(self):
        # Fold in wrist and gripper
        print('--------- Stowing %s ----'%self.name)
        self.move_to('wrist_pitch', self.params['stow']['wrist_pitch'])
        self.move_to('wrist_roll', self.params['stow']['wrist_roll'])
        self.move_to('wrist_yaw', self.params['stow']['wrist_yaw'])
        self.move_to('stretch_gripper', self.params['stow']['stretch_gripper'])

    def home(self):
        self.motors['wrist_pitch'].move_to(self.params['stow']['wrist_pitch'])
        self.motors['wrist_roll'].move_to(self.params['stow']['wrist_roll'])
        self.motors['wrist_yaw'].home()
        self.motors['stretch_gripper'].home()
    
    def pre_stow(self,robot=None):
        if robot:
            robot.end_of_arm.move_to('wrist_pitch', robot.end_of_arm.params['stow']['wrist_pitch'])
        else:
            self.move_to('wrist_pitch', self.params['stow']['wrist_pitch'])

    def step_sentry(self, robot):
        super().step_sentry(robot)
        if robot.collision.running:
            wrist_p = self.get_joint('wrist_pitch').status['pos']
            wrist_y = self.get_joint('wrist_yaw').status['pos']
            if wrist_p > -0.2 and wrist_y < -0.2:
                # print("In Special Stop")
                self.get_joint('wrist_yaw').forced_collision_stop_override = {'pos': False, 'neg':True}
                
            else:
                # print("Out Special Stop")
                self.get_joint('wrist_yaw').forced_collision_stop_override = {'pos': False, 'neg':False}

class EOA_Wrist_DW3_Tool_Tablet_12in(EndOfArm):
    """
    Wrist Yaw / Pitch / Roll + 12in Tablet
    """
    def __init__(self, name='eoa_wrist_dw3_tool_tablet_12in'):
        EndOfArm.__init__(self, name)
        self.portrait_orientation = self.params['portrait_orientation']
        self.lock_roll_joint = self.params['lock_wrist_roll']
        #This maps from the name of a joint in the URDF to the name of the joint in Stretch Body
        #It is used by CollisionMgmt
        self.urdf_map={
            'joint_wrist_yaw':'wrist_yaw',
            'joint_wrist_pitch': 'wrist_pitch',
            'joint_wrist_roll':'wrist_roll' #Not mapping fingers for collision mgmt yet
        }

    def move_by(self, joint, x_r, v_r=None, a_r=None, enable_wrist_roll = False):
        if joint=='wrist_roll':
            if not self.lock_roll_joint:
                return EndOfArm.move_by(self, joint, x_r, v_r, a_r)
            elif self.lock_roll_joint and enable_wrist_roll:
                return EndOfArm.move_by(self, joint, x_r, v_r, a_r)
            else:
                return None
        return EndOfArm.move_by(self, joint, x_r, v_r, a_r)
    
    def move_to(self, joint, x_r, v_r=None, a_r=None, enable_wrist_roll = False):
        if joint=='wrist_roll':
            if not self.lock_roll_joint:
                return EndOfArm.move_to(self, joint, x_r, v_r, a_r)
            elif self.lock_roll_joint and enable_wrist_roll:
                return EndOfArm.move_to(self, joint, x_r, v_r, a_r)
            else:
                return None
        return EndOfArm.move_to(self, joint, x_r, v_r, a_r)

    def set_velocity(self, joint, v_r, a_r=None, enable_wrist_roll = False):
        if joint=='wrist_roll':
            if not self.lock_roll_joint:
                return EndOfArm.set_velocity(self, joint, v_r, a_r)
            elif self.lock_roll_joint and enable_wrist_roll:
                return EndOfArm.set_velocity(self, joint, v_r, a_r)
            else:
                return None
        return EndOfArm.set_velocity(self, joint, v_r, a_r)
    
    def pre_stow(self,robot=None):
        if robot:
            if robot.lift.status['pos'] > 0.9:
                robot.lift.move_by(-0.2)
            robot.push_command()
            time.sleep(0.25)
            while robot.lift.motor.status['is_moving']:
                time.sleep(0.1)

        if not (2 > self.motors['wrist_yaw'].status['pos'] > 0):
            self.move_to('wrist_pitch',-1.57)
    
    def switch_to_portrait_mode(self):
        self.portrait_orientation = True
        self.reset_tablet_orientation()
    
    def switch_to_landscape_mode(self):
        self.portrait_orientation = False
        self.reset_tablet_orientation()
    
    def reset_tablet_orientation(self):
        if self.portrait_orientation:
            self.motors['wrist_roll'].move_to(1.57)
        else:
            self.motors['wrist_roll'].move_to(self.params['stow']['wrist_roll'])

    def stow(self):
        # Fold Arm, Wrist yaw turns left making the tabled screen face forward.
        print('--------- Stowing %s ----'%self.name)
        self.reset_tablet_orientation()
        self.move_to('wrist_yaw', self.params['stow']['wrist_yaw'])
        time.sleep(1)
        self.move_to('wrist_pitch', self.params['stow']['wrist_pitch'])
        time.sleep(1)

    def home(self):
        # Tablet should face completely downwards during homing 
        self.motors['wrist_pitch'].move_to(-1.57)
        while self.motors['wrist_pitch'].motor.is_moving():
            time.sleep(0.2)
        self.reset_tablet_orientation()
        self.motors['wrist_yaw'].home()
        self.motors['wrist_pitch'].move_to(self.params['stow']['wrist_pitch'])
        self.motors['wrist_yaw'].move_to(1.57)
        time.sleep(2)
    
    def step_sentry(self, robot):
        super().step_sentry(robot)
        if robot.collision.running:
            wrist_p = self.get_joint('wrist_pitch').status['pos']
            wrist_y = self.get_joint('wrist_yaw').status['pos']
            # TODO: Add more special conditions around this part
            if wrist_p > -0.2 and wrist_y < 0.18:
                # print("In Special Stop")
                self.get_joint('wrist_yaw').forced_collision_stop_override = {'pos': False, 'neg':True}
                self.get_joint('wrist_pitch').forced_collision_stop_override = {'pos': True, 'neg':False}
                
            else:
                # print("Out Special Stop")
                self.get_joint('wrist_yaw').forced_collision_stop_override = {'pos': False, 'neg':False}
                self.get_joint('wrist_pitch').forced_collision_stop_override = {'pos': False, 'neg':False}
