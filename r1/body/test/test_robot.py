# Logging level must be set before importing any stretch_body class
import stretch_body.robot_params
stretch_body.robot_params.RobotParams.set_logging_level("DEBUG")

import unittest
import stretch_body.robot

import time
import math


class TestRobot(unittest.TestCase):
    @unittest.skip(reason='TODO: cleanup')
    def test_home_and_stow(self):
        """Test robot homes and stows correctly.

        Note: this test assumes the stretch_gripper end-effector is attached.
        """
        print('test_home_and_stow')
        r = stretch_body.robot.Robot()
        self.assertTrue(r.startup())

        # r.home()
        # r.pull_status()
        # time.sleep(1) # wrist_yaw yields 3.4 (stowed position) unless sleep here
        # self.assertAlmostEqual(r.status['lift']['pos'], 0.58, places=1)
        # self.assertAlmostEqual(r.status['arm']['pos'], 0.1, places=3)
        # self.assertAlmostEqual(r.status['head']['head_tilt']['pos'], 0.0, places=1)
        # self.assertAlmostEqual(r.status['head']['head_pan']['pos'], 0.0, places=1)
        # self.assertAlmostEqual(r.status['end_of_arm']['wrist_yaw']['pos'], 0.0, places=1)
        # self.assertAlmostEqual(r.status['end_of_arm']['stretch_gripper']['pos'], 0.0, places=1)

        r.stow()
        r.pull_status()
        self.assertAlmostEqual(r.status['lift']['pos'], r.get_stow_pos('lift'), places=1)
        self.assertAlmostEqual(r.status['arm']['pos'], r.get_stow_pos('arm'), places=3)
        self.assertAlmostEqual(r.status['head']['head_tilt']['pos'], r.get_stow_pos('head_tilt'), places=1)
        self.assertAlmostEqual(r.status['head']['head_pan']['pos'], r.get_stow_pos('head_pan'), places=1)
        self.assertAlmostEqual(r.status['end_of_arm']['wrist_yaw']['pos'], r.get_stow_pos('wrist_yaw'), places=1)

        r.stop()

    @unittest.skip(reason='TODO: Running this test will cause the other two to fail due to busy serial ports')
    def test_endofarmtool_loaded(self):
        """Verify end_of_arm tool loaded correctly in robot.
        """
        print('test_endofarmtool_loaded')
        r = stretch_body.robot.Robot()
        self.assertEqual(r.end_of_arm.name, r.params['tool'])
        r.stop()

    @unittest.skip(reason='TODO: cleanup')
    def test_endofarmtool_custom_stowing(self):
        """Verify custom stowing for non-endofarm devices from tool works.
        """
        print('test_endofarmtool_custom_stowing')
        r = stretch_body.robot.Robot()
        self.assertTrue(r.startup())
        if not r.is_homed():
            self.fail("test requires robot to be homed")

        # Start with regular stowing
        r.stow()
        r.pull_status()
        self.assertAlmostEqual(r.status['lift']['pos'], 0.2, places=3)
        self.assertAlmostEqual(r.status['arm']['pos'], 0.0, places=3)

        # Add custom stowing for lift and arm
        r.robot_params[r.params['tool']]['stow']['lift'] = 0.25
        r.robot_params[r.params['tool']]['stow']['arm'] = 0.05
        r.stow()
        r.pull_status()
        self.assertAlmostEqual(r.status['lift']['pos'], 0.25, places=3)
        self.assertAlmostEqual(r.status['arm']['pos'], 0.05, places=3)

        # Remove custom stowing
        r.robot_params[r.params['tool']]['stow'].pop('lift', None)
        r.robot_params[r.params['tool']]['stow'].pop('arm', None)
        r.stop()

    @unittest.skip(reason='TODO: cleanup')
    def test_soft_limits_not_overwritten(self):
        """Verify that limits set via the set_soft_limits API are upper bounds on
        limits that can be set by collision models in the `RobotCollision.step`
        function. The `Robot` class manages threaded execution of `RobotCollision`.
        """
        print('test_soft_limits_not_overwritten')
        r = stretch_body.robot.Robot()
        r.params['use_collision_manager'] = True
        self.assertTrue(r.startup())
        if not r.is_homed():
            self.fail("test requires robot to be homed")

        upper_limit = 0.1
        bad_goal = upper_limit + 0.1
        r.arm.set_soft_motion_limit_min(0.0)
        r.arm.set_soft_motion_limit_max(upper_limit)
        r.push_command()
        time.sleep(1.0)
        r.arm.move_to(bad_goal)
        r.push_command()
        time.sleep(2.0)
        r.pull_status()
        time.sleep(1.0)
        self.assertNotAlmostEqual(r.status['arm']['pos'], bad_goal, places=3)
        r.stop()

    @unittest.skip(reason='TODO: cleanup')
    def test_dynamixel_runstop(self):
        """Test end_of_arm respects runstop from pimu
        """
        print('test_dynamixel_runstop')
        r = stretch_body.robot.Robot()
        self.assertTrue(r.startup())
        if not r.is_homed():
            self.fail("test requires robot to be homed")

        # Move robot to starting position
        r.end_of_arm.move_to('wrist_yaw', 0.0)
        time.sleep(4.0)

        # Begin moving to stow position
        r.end_of_arm.move_to('wrist_yaw', math.pi)
        time.sleep(1.0)

        # Interrupt moving to stow position
        r.pimu.runstop_event_trigger()
        r.push_command()
        time.sleep(0.1)
        r.pimu.runstop_event_reset()
        r.push_command()
        time.sleep(0.1)

        # Verify not at stow position
        r.pull_status()
        self.assertNotAlmostEqual(r.status['end_of_arm']['wrist_yaw']['pos'], math.pi, places=2)
        r.stop()

    @unittest.skip(reason='TODO: cleanup')
    def test_waypoint_trajectory(self):
        print('test_waypoint_trajectory')
        r = stretch_body.robot.Robot()
        r.startup()

        # Move robot to starting position
        r.end_of_arm.move_to('wrist_yaw', 3.14)
        r.end_of_arm.move_to('stretch_gripper', 0)
        r.head.move_to('head_tilt', 0)
        r.head.move_to('head_pan', 0)
        r.arm.move_to(0)
        r.lift.move_to(0.2)
        time.sleep(10.0)

        r.end_of_arm.get_joint('stretch_gripper').trajectory.add(0, 0)
        r.end_of_arm.get_joint('stretch_gripper').trajectory.add(6, 50)
        r.end_of_arm.get_joint('stretch_gripper').trajectory.add(12, 0)
        r.end_of_arm.get_joint('wrist_yaw').trajectory.add(0, 3.14)
        r.end_of_arm.get_joint('wrist_yaw').trajectory.add(6, 2.14)
        r.end_of_arm.get_joint('wrist_yaw').trajectory.add(12, 3.14)
        r.head.get_joint('head_tilt').trajectory.add(0, 0)
        r.head.get_joint('head_tilt').trajectory.add(6, -1.0)
        r.head.get_joint('head_tilt').trajectory.add(12, 0)
        r.head.get_joint('head_pan').trajectory.add(0, 0)
        r.head.get_joint('head_pan').trajectory.add(6, -1.0)
        r.head.get_joint('head_pan').trajectory.add(12, 0)
        r.arm.trajectory.add(0, 0)
        r.arm.trajectory.add(6, 0.1)
        r.arm.trajectory.add(12, 0.0)
        r.base.trajectory.add(0, 0, 0, 0)
        r.base.trajectory.add(6, 0.05, 0, 0)
        r.base.trajectory.add(12, 0, 0, 0)
        r.follow_trajectory()
        time.sleep(13)

        r.stop()

    def test_waypoint_trajectory_multidof_base(self):
        print('test_waypoint_trajectory_multidof_base')
        r = stretch_body.robot.Robot()
        r.startup()

        # Move robot to starting position
        r.end_of_arm.move_to('wrist_yaw', 3.14)
        r.end_of_arm.move_to('stretch_gripper', 0)
        r.head.move_to('head_tilt', 0)
        r.head.move_to('head_pan', 0)
        r.arm.move_to(0)
        r.lift.move_to(0.2)
        time.sleep(10.0)

        r.end_of_arm.get_joint('stretch_gripper').trajectory.add(0, 0)
        r.end_of_arm.get_joint('stretch_gripper').trajectory.add(6, 50)
        r.end_of_arm.get_joint('stretch_gripper').trajectory.add(24, 0)
        r.end_of_arm.get_joint('wrist_yaw').trajectory.add(0, 3.14)
        r.end_of_arm.get_joint('wrist_yaw').trajectory.add(6, 2.14)
        r.end_of_arm.get_joint('wrist_yaw').trajectory.add(24, 3.14)
        r.head.get_joint('head_tilt').trajectory.add(0, 0)
        r.head.get_joint('head_tilt').trajectory.add(6, -1.0)
        r.head.get_joint('head_tilt').trajectory.add(12, 0)
        r.head.get_joint('head_tilt').trajectory.add(6, 0.2)
        r.head.get_joint('head_tilt').trajectory.add(24, 0)
        r.head.get_joint('head_pan').trajectory.add(0, 0)
        r.head.get_joint('head_pan').trajectory.add(6, -1.0)
        r.head.get_joint('head_pan').trajectory.add(12, 0)
        r.head.get_joint('head_pan').trajectory.add(18, 0.2)
        r.head.get_joint('head_pan').trajectory.add(24, 0)
        r.arm.trajectory.add(0, 0, 0)
        r.arm.trajectory.add(6, 0.1, 0)
        r.arm.trajectory.add(12, 0.0, 0)
        r.arm.trajectory.add(18, 0.15, 0)
        r.arm.trajectory.add(24, 0.0, 0)
        r.base.trajectory.add(0, 0, 0, 0, 0, 0, 0, 0)
        r.base.trajectory.add(6, 0.4, 0, 0, 0, 0, 0, 0)
        r.base.trajectory.add(12, 0.4, 0, math.pi, 0, 0, 0, 0)
        r.base.trajectory.add(18, 0.0, 0, math.pi, 0, 0, 0, 0)
        r.base.trajectory.add(24, 0.0, 0, 0, 0, 0, 0, 0)
        r.follow_trajectory()
        time.sleep(26)

        r.stop()
