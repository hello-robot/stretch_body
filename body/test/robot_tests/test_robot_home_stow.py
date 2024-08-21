#!/usr/bin/env python3
import stretch_body.robot as robot
import time
import unittest


class Test_BRI_robot_home_stow(unittest.TestCase):
    """
    Test to Verify Robot's homing and stowing behaviour
    """

    @classmethod
    def setUpClass(self):
        self.robot = robot.Robot()
        self.robot.startup()

    @classmethod
    def tearDownClass(self):
        self.robot.stop()

    def check_joint_poses(self, poses_dict, check_name):
        self.robot.pull_status()

        arm_pose = self.robot.status['arm']['pos']
        self.assertAlmostEqual(arm_pose, poses_dict['arm'], places=2, msg='%s Value "arm" Different' % check_name)

        lift_pose = self.robot.status['lift']['pos']
        self.assertAlmostEqual(lift_pose, poses_dict['lift'], places=2, msg='%s Value "lift" Different' % check_name)

        head_pan_pose = self.robot.status['head']['head_pan']['pos']
        self.assertAlmostEqual(head_pan_pose, poses_dict['head_pan'], places=1,
                               msg='%s Value "head_pan" Different' % check_name)

        head_tilt_pose = self.robot.status['head']['head_tilt']['pos']
        self.assertAlmostEqual(head_tilt_pose, poses_dict['head_tilt'], places=1,
                               msg='%s Value "head_tilt" Different' % check_name)

        if self.robot.params['tool'] == 'tool_stretch_dex_wrist':
            wrist_yaw_pose = self.robot.status['end_of_arm']['wrist_yaw']['pos']
            self.assertAlmostEqual(wrist_yaw_pose, poses_dict['wrist_yaw'], places=1,
                                msg='%s Value "wrist_yaw" Different' % check_name)
        
            wrist_roll_pose = self.robot.status['end_of_arm']['wrist_roll']['pos']
            self.assertAlmostEqual(wrist_roll_pose, poses_dict['wrist_roll'], places=1,
                                   msg='%s Value "wrist_roll" Different' % check_name)
            wrist_pitch_pose = self.robot.status['end_of_arm']['wrist_pitch']['pos']
            self.assertAlmostEqual(wrist_pitch_pose, poses_dict['wrist_pitch'], places=1,
                                   msg='%s Value "wrist_pitch" Different' % check_name)
    
            gripper_pose = self.robot.status['end_of_arm']['stretch_gripper']['pos']
            self.assertAlmostEqual(gripper_pose, poses_dict['stretch_gripper'], places=1,
                                msg='%s Value "stretch_gripper" Different' % check_name)
            
        if self.robot.params['tool'] == 'eoa_wrist_dw3_tool_sg3':
            wrist_yaw_pose = self.robot.status['end_of_arm']['wrist_yaw']['pos']
            self.assertAlmostEqual(wrist_yaw_pose, poses_dict['wrist_yaw'], places=1,
                                msg='%s Value "wrist_yaw" Different' % check_name)
            
            wrist_roll_pose = self.robot.status['end_of_arm']['wrist_roll']['pos']
            self.assertAlmostEqual(wrist_roll_pose, poses_dict['wrist_roll'], places=1,
                                   msg='%s Value "wrist_roll" Different' % check_name)
            wrist_pitch_pose = self.robot.status['end_of_arm']['wrist_pitch']['pos']
            self.assertAlmostEqual(wrist_pitch_pose, poses_dict['wrist_pitch'], places=1,
                                   msg='%s Value "wrist_pitch" Different' % check_name)
            
            gripper_pose = self.robot.status['end_of_arm']['stretch_gripper']['pos']
            self.assertAlmostEqual(gripper_pose, poses_dict['stretch_gripper'], places=1,
                                msg='%s Value "stretch_gripper" Different' % check_name)
            
        if self.robot.params['tool'] == 'tool_stretch_gripper':
            wrist_yaw_pose = self.robot.status['end_of_arm']['wrist_yaw']['pos']
            self.assertAlmostEqual(wrist_yaw_pose, poses_dict['wrist_yaw'], places=1,
                                msg='%s Value "wrist_yaw" Different' % check_name)
            
            gripper_pose = self.robot.status['end_of_arm']['stretch_gripper']['pos']
            self.assertAlmostEqual(gripper_pose, poses_dict['stretch_gripper'], places=1,
                                msg='%s Value "stretch_gripper" Different' % check_name)

    def test_robot_home(self):
        """
        Manually verify robot homing and assert joint poses are ~equal to homing poses.
        """
        self.robot.home()
        time.sleep(1.5)

        home_poses = {'head_pan': 0,
                      'head_tilt': 0,
                      'arm': 0.1,
                      'lift': 0.6}

        if self.robot.params['tool'] == 'tool_stretch_dex_wrist':
            home_poses['wrist_yaw'] = 0
            home_poses['wrist_roll'] = 0
            home_poses['wrist_pitch'] = 0
            home_poses['stretch_gripper'] = 0

        if self.robot.params['tool'] == 'eoa_wrist_dw3_tool_sg3':
            home_poses['wrist_yaw'] = 0
            home_poses['wrist_roll'] = self.robot.params['stow']['wrist_roll']
            home_poses['wrist_pitch'] = self.robot.params['stow']['wrist_pitch']
            home_poses['stretch_gripper'] = 0

        if self.robot.params['tool'] == 'tool_stretch_gripper':
            home_poses['wrist_yaw'] = 0
            home_poses['stretch_gripper'] = 0

        if self.robot.params['tool'] == 'eoa_wrist_dw3_tool_nil':
            home_poses['wrist_yaw'] = 0
            home_poses['wrist_roll'] = self.robot.params['stow']['wrist_roll']
            home_poses['wrist_pitch'] = self.robot.params['stow']['wrist_pitch']

        self.check_joint_poses(home_poses, 'Homing')

    def test_robot_stow(self):
        """
        Manually verify robot stowing and assert joint poses are ~equal to stowing poses.
        """
        print('Stowing robot.')
        self.robot.stow()
        time.sleep(1.5)

        stow_poses = {}
        stow_poses['arm'] = self.robot.get_stow_pos('arm')
        stow_poses['lift'] = self.robot.get_stow_pos('lift')
        stow_poses['head_pan'] = self.robot.get_stow_pos('head_pan')
        stow_poses['head_tilt'] = self.robot.get_stow_pos('head_tilt')


        if self.robot.params['tool'] == 'tool_stretch_dex_wrist':
            stow_poses['wrist_yaw'] = self.robot.end_of_arm.params['stow']['wrist_yaw']
            stow_poses['wrist_roll'] = self.robot.end_of_arm.params['stow']['wrist_roll']
            stow_poses['wrist_pitch'] = self.robot.end_of_arm.params['stow']['wrist_pitch']
            stow_poses['stretch_gripper'] = self.robot.end_of_arm.params['stow']['stretch_gripper']


        if self.robot.params['tool'] == 'eoa_wrist_dw3_tool_sg3':
            stow_poses['wrist_yaw'] = self.robot.end_of_arm.params['stow']['wrist_yaw']
            stow_poses['wrist_roll'] = self.robot.end_of_arm.params['stow']['wrist_roll']
            stow_poses['wrist_pitch'] = self.robot.end_of_arm.params['stow']['wrist_pitch']
            stow_poses['stretch_gripper'] = self.robot.end_of_arm.params['stow']['stretch_gripper']
            

        if self.robot.params['tool'] == 'tool_stretch_gripper':
            stow_poses['wrist_yaw'] = self.robot.end_of_arm.params['stow']['wrist_yaw']
            stow_poses['stretch_gripper'] = self.robot.end_of_arm.params['stow']['stretch_gripper']

        if self.robot.params['tool'] == 'eoa_wrist_dw3_tool_nil':
            stow_poses['wrist_yaw'] = self.robot.end_of_arm.params['stow']['wrist_yaw']
            stow_poses['wrist_roll'] = self.robot.end_of_arm.params['stow']['wrist_roll']
            stow_poses['wrist_pitch'] = self.robot.end_of_arm.params['stow']['wrist_pitch']

        self.check_joint_poses(stow_poses, 'Stowing')


test_suite = unittest.TestSuite()
test_suite.addTest(Test_BRI_robot_home_stow('test_robot_home'))
test_suite.addTest(Test_BRI_robot_home_stow('test_robot_stow'))