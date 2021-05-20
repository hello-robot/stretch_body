from stretch_body.end_of_arm import EndOfArm

class ToolNone(EndOfArm):
    def __init__(self, name='tool_none'):
        EndOfArm.__init__(self)
        self.joints = [] #Overwrite joints define/loaded by 'end_of_arm'
        self.motors = {}
        self.add_joints(self.robot_params[self.name])
        self.overwrite_params(self.params,self.robot_params[self.name])

class ToolStretchGripper(EndOfArm):
    def __init__(self, name='tool_stretch_gripper'):
        EndOfArm.__init__(self)
        self.joints = []
        self.motors = {}
        self.add_joints(self.robot_params[self.name])
        self.overwrite_params(self.params,self.robot_params[self.name])

    def stow(self):
        # Fold in wrist and gripper
        print('--------- Stowing Wrist Yaw ----')
        self.move_to('wrist_yaw', self.params['stow']['wrist_yaw'])
        print('--------- Stowing Gripper ----')
        self.move_to('stretch_gripper', self.params['stow']['stretch_gripper'])
