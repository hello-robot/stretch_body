from stretch_body.end_of_arm import *

# ############# STRETCH RE1 / RE2 #######################
class ToolNone(EndOfArm):
    """
    Just a WristYaw for RE1 / Stretch 2
    """
    def __init__(self):
        EndOfArm.__init__(self, name='tool_none')

class ToolStretchGripper(EndOfArm):
    """
    Wrist Yaw + standard Stretch Gripper for RE1 / Stretch 2
    """
    def __init__(self):
        EndOfArm.__init__(self, name='tool_stretch_gripper')

    def stow(self):
        # Fold in wrist and gripper
        print('--------- Stowing Wrist Yaw ----')
        self.move_to('wrist_yaw', self.params['stow']['wrist_yaw'])
        print('--------- Stowing Gripper ----')
        self.move_to('stretch_gripper', self.params['stow']['stretch_gripper'])

    def get_joint_configuration(self,braked=False):
        """
        Construct a dictionary of tools current pose (for robot_collision_mgmt)
        Keys match joint names in URDF
        """
        g=self.get_motor('stretch_gripper')
        _, gripper_finger_rad, _, _ = g.gripper_conversion.status_to_all(g.status)
        return {
            'joint_gripper_finger_left': gripper_finger_rad,
            'joint_gripper_finger_right': gripper_finger_rad}

class ToolStretchDexWrist(EndOfArm):
    def __init__(self, name='tool_stretch_dex_wrist'):
        EndOfArm.__init__(self, name)

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

# ##########################################################3#
class EOA_Wrist_DW3_Tool_NIL(EndOfArmV2):
    """
    Wrist Yaw / Pitch / Roll only for version 3 of DexWrist
    """
    def __init__(self, name='eoa_wrist_dw3_tool_nil'):
        EndOfArmV2.__init__(self, name)

    def stow(self):
        # Fold in wrist and gripper
        print('--------- Stowing EOA_Wrist_DW3_Tool_NIL ----')
        self.move_to('wrist_pitch', self.params['stow']['wrist_pitch'])
        self.move_to('wrist_roll', self.params['stow']['wrist_roll'])
        self.move_to('wrist_yaw', self.params['stow']['wrist_yaw'])

    def home(self):
        self.motors['wrist_pitch'].move_to(self.params['stow']['wrist_pitch'])
        self.motors['wrist_roll'].move_to(self.params['stow']['wrist_roll'])
        self.motors['wrist_yaw'].home()

class EOA_Wrist_DW3_Tool_SG3(EndOfArmV2):
    """
    Wrist Yaw / Pitch / Roll + Stretch Gripper 3
    """
    def __init__(self, name='eoa_wrist_dw3_tool_sg3'):
        EndOfArmV2.__init__(self, name)

    def stow(self):
        # Fold in wrist and gripper
        print('--------- Stowing EOA_Wrist_DW3_Tool_SG3 ----')
        self.move_to('wrist_pitch', self.params['stow']['wrist_pitch'])
        self.move_to('wrist_roll', self.params['stow']['wrist_roll'])
        self.move_to('wrist_yaw', self.params['stow']['wrist_yaw'])
        self.move_to('stretch_gripper', self.params['stow']['stretch_gripper'])
    def home(self):
        self.motors['wrist_pitch'].move_to(self.params['stow']['wrist_pitch'])
        self.motors['wrist_roll'].move_to(self.params['stow']['wrist_roll'])
        self.motors['wrist_yaw'].home()
        self.motors['stretch_gripper'].home()
