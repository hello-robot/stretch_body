from stretch_body.end_of_arm import EndOfArm

class ToolNone(EndOfArm):
    def __init__(self):
        EndOfArm.__init__(self, name='tool_none')

class ToolStretchGripper(EndOfArm):
    def __init__(self):
        EndOfArm.__init__(self, name='tool_stretch_gripper')

    def stow(self):
        # Fold in wrist and gripper
        print('--------- Stowing Wrist Yaw ----')
        self.move_to('wrist_yaw', self.params['stow']['wrist_yaw'])
        print('--------- Stowing Gripper ----')
        self.move_to('stretch_gripper', self.params['stow']['stretch_gripper'])
