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
