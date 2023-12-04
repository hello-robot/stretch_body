from stretch_body.end_of_arm import EndOfArm


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


class ToolDW3_NoGripper(EndOfArm):
    """
    Wrist Yaw / Pitch / Roll only for version 3 of DexWrist
    """
    def __init__(self, name='tool_dw3_no_gripper'):
        EndOfArm.__init__(self, name)

    def stow(self):
        # Fold in wrist and gripper
        print('--------- Stowing ToolDW3NoGripper ----')
        self.move_to('wrist_pitch', self.params['stow']['wrist_pitch'])
        self.move_to('wrist_roll', self.params['stow']['wrist_roll'])
        self.move_to('wrist_yaw', self.params['stow']['wrist_yaw'])

    def home(self):
        self.motors['wrist_pitch'].move_to(self.params['stow']['wrist_pitch'])
        self.motors['wrist_roll'].move_to(self.params['stow']['wrist_roll'])
        self.motors['wrist_yaw'].home()

class ToolDW3_Custom(ToolDW3_NoGripper):
    """
    Wrist Yaw / Pitch / Roll only for version 3 of DexWrist
    """
    def __init__(self, name='tool_dw3_custom'):
        ToolDW3_NoGripper.__init__(self, name)

class ToolDW3_SG3(EndOfArm):
    """
    Wrist Yaw / Pitch / Roll and Gripper for version 3 of DexWrist and version 3 of Stretch Gripper
    """
    def __init__(self, name='tool_dw3_sg3'):
        EndOfArm.__init__(self, name)

    def stow(self):
        # Fold in wrist and gripper
        print('--------- Stowing Tool_DW3_SG3 ----')
        self.move_to('wrist_pitch', self.params['stow']['wrist_pitch'])
        self.move_to('wrist_roll', self.params['stow']['wrist_roll'])
        self.move_to('wrist_yaw', self.params['stow']['wrist_yaw'])
        self.move_to('stretch_gripper', self.params['stow']['stretch_gripper'])

    def home(self):
        self.motors['stretch_gripper'].home()
        self.motors['wrist_pitch'].move_to(self.params['stow']['wrist_pitch'])
        self.motors['wrist_roll'].move_to(self.params['stow']['wrist_roll'])
        self.motors['wrist_yaw'].home()