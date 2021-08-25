# Logging level must be set before importing any stretch_body class
import stretch_body.robot_params
stretch_body.robot_params.RobotParams.set_logging_level("DEBUG")
import stretch_body.hello_utils as hu
import unittest
import stretch_body.robot

import time




class TestCollisionGripperBase(unittest.TestCase):



    def test_move_gripper_to_base(self):
        r = stretch_body.robot.Robot()
        r.params['use_collision_manager']=True

        r.startup()
        #Start with lift at known safe position near head
        r.lift.move_to(0.25)
        r.push_command()
        time.sleep(1.0)
        r.lift.motor.wait_until_at_setpoint()

        #Now move gripper into collision position
        r.end_of_arm.move_to('wrist_yaw',x_r=3.3)
        time.sleep(2.0)

        print('###################################')
        print('Ready for collision test...')
        print('Monitor for accidental collision between the gripper and the base')
        print('Entering the jog loop now....')
        print('----------')
        print('Hit enter to jog the lift down 2mm')
        print('Enter x to exit jog loop if collision is imminent')
        print('Enter q to exit jog loop is collision is not possible')
        while(True):
            x=raw_input()
            if x=='q':
                success=True
                break
            if x=='x':
                success=False
                break
            r.lift.move_by(-.002)
            r.push_command()

        r.lift.move_to(0.22)
        r.push_command()
        time.sleep(1.0)
        r.lift.motor.wait_until_at_setpoint()
        self.assertTrue(success)

        #Check that if extend gripper and arm a bit the lift can go down
        r.end_of_arm.move_to('wrist_yaw', x_r=0.0)
        r.arm.move_to(0.1)
        r.push_command()
        time.sleep(2.0)
        r.lift.move_to(0.005)
        r.push_command()
        time.sleep(3.0)
        self.assertAlmostEqual(r.status['lift']['pos'],.005, places=1)

        #Check that arm retaction wont collide wrist
        print('###################################')
        print('Ready for collision test...')
        print('Monitor for accidental collision between the wrist and the base')
        print('Entering the jog loop now....')
        print('----------')
        print('Hit enter to jog the arm in 2mm')
        print('Enter x to exit jog loop if collision is imminent')
        print('Enter q to exit jog loop is collision is not possible')
        while (True):
            x = raw_input()
            if x == 'q':
                success = True
                break
            if x == 'x':
                success = False
                break
            r.arm.move_by(-.002)
            r.push_command()
        self.assertTrue(success)

        # Check that cant rotate gripper into base
        r.end_of_arm.move_to('wrist_yaw', x_r=1.27)
        print('###################################')
        print('Ready for collision test...')
        print('Monitor for accidental collision between the gripper and the base')
        print('Entering the jog loop now....')
        print('----------')
        print('Hit enter to jog the wrist in 1 deg')
        print('Enter x to exit jog loop if collision is imminent')
        print('Enter q to exit jog loop is collision is not possible')
        while (True):
            x = raw_input()
            if x == 'q':
                success = True
                break
            if x == 'x':
                success = False
                break
            r.end_of_arm.move_by('wrist_yaw', x_r=hu.deg_to_rad(1.0))
            r.arm.move_by(0.0)
            r.push_command()
        self.assertTrue(success)

        r.stop()

class TestCollisionArmCamera(unittest.TestCase):

    def test_move_arm_to_camera(self):
        r = stretch_body.robot.Robot()
        r.params['use_collision_manager']=True

        r.startup()
        #Start with lift at known safe position near head
        r.lift.move_to(1.0)
        r.push_command()
        time.sleep(1.0)
        r.lift.motor.wait_until_at_setpoint()

        #Now move camera into collision position
        r.head.move_to('head_pan',x_r=0)
        r.head.move_to('head_tilt', x_r=-1.575)

        print('###################################')
        print('Ready for collision test...')
        print('Monitor for accidental collision between the camera and the arm')
        print('Entering the jog loop now....')
        print('----------')
        print('Hit enter to jog the lift up 2mm')
        print('Enter x to exit jog loop if collision is imminent')
        print('Enter q to exit jog loop is collision is not possible')
        while(True):
            x=raw_input()
            if x=='q':
                success=True
                break
            if x=='x':
                success=False
                break
            r.lift.move_by(.002)
            r.push_command()
        r.lift.move_to(0.9)
        r.push_command()
        time.sleep(1.0)
        r.lift.motor.wait_until_at_setpoint()
        self.assertTrue(success)

        #Now move camera out of way and bring arm up
        r.head.move_to('head_pan', x_r=hu.deg_to_rad(-60.0))
        time.sleep(1.0)
        r.lift.move_to(1.07)
        r.push_command()
        time.sleep(1.0)
        r.lift.motor.wait_until_at_setpoint()
        self.assertAlmostEqual(r.status['lift']['pos'], 1.07, places=1)

        #Now try to pan camera into arm
        print('###################################')
        print('Ready for collision test...')
        print('Monitor for accidental collision between the camera and the arm')
        print('Entering the jog loop now....')
        print('----------')
        print('Hit enter to jog the pan the camera by 2 deg')
        print('Enter x to exit jog loop if collision is imminent')
        print('Enter q to exit jog loop is collision is not possible')
        while (True):
            x = raw_input()
            if x == 'q':
                success = True
                break
            if x == 'x':
                success = False
                break
            r.head.move_by('head_pan', x_r=hu.deg_to_rad(2.0))
        self.assertTrue(success)


        # Now try to tilt camera down then pan
        r.head.move_to('head_tilt', x_r=-0.75)
        time.sleep(1.0)
        self.assertAlmostEqual(r.status['head']['head_tilt']['pos'], -0.75, places=1)

        r.head.move_to('head_pan',x_r=0.75)
        time.sleep(2.0)
        self.assertAlmostEqual(r.status['head']['head_pan']['pos'], 0.75, places=1)

        r.head.move_to('head_pan', x_r=-0.75)
        time.sleep(2.0)
        self.assertAlmostEqual(r.status['head']['head_pan']['pos'], -0.75, places=1)

        r.head.move_to('head_pan', x_r=0.0)
        time.sleep(2.0)
        self.assertAlmostEqual(r.status['head']['head_pan']['pos'], -0.0, places=1)

        #Now try to tilt the camera up
        print('###################################')
        print('Ready for collision test...')
        print('Monitor for accidental collision between the camera and the arm')
        print('Entering the jog loop now....')
        print('----------')
        print('Hit enter to jog the tilt the camera by 2 deg')
        print('Enter x to exit jog loop if collision is imminent')
        print('Enter q to exit jog loop is collision is not possible')
        while (True):
            x = raw_input()
            if x == 'q':
                success = True
                break
            if x == 'x':
                success = False
                break
            r.head.move_by('head_tilt', x_r=hu.deg_to_rad(-2.0))
        self.assertTrue(success)
        self.assertAlmostEqual(r.status['head']['head_pan']['pos'], -0.0, places=1)


        #Now restart from a position where the arm is already up
        r.head.move_to('head_tilt', x_r=-0.75)
        r.head.move_to('head_pan', x_r=hu.deg_to_rad(-60.0))

        r.stop()
        r = stretch_body.robot.Robot()
        r.params['use_collision_manager'] = True
        r.startup()

        r.lift.move_to(1.09)
        r.push_command()
        time.sleep(1.0)
        r.lift.motor.wait_until_at_setpoint()
        self.assertAlmostEqual(r.status['lift']['pos'], 1.09, places=1)

        r.lift.move_to(1.07)
        r.push_command()
        time.sleep(1.0)
        r.lift.motor.wait_until_at_setpoint()
        self.assertAlmostEqual(r.status['lift']['pos'], 1.09, places=1)

        #Now try to pan camera into arm
        print('###################################')
        print('Ready for collision test...')
        print('Monitor for accidental collision between the camera and the arm')
        print('Entering the jog loop now....')
        print('----------')
        print('Hit enter to jog the pan the camera by 2 deg')
        print('Enter x to exit jog loop if collision is imminent')
        print('Enter q to exit jog loop is collision is not possible')
        while (True):
            x = raw_input()
            if x == 'q':
                success = True
                break
            if x == 'x':
                success = False
                break
            r.head.move_by('head_pan', x_r=hu.deg_to_rad(2.0))
        self.assertTrue(success)


        #Finally move back to safe place
        r.lift.move_to(0.75)
        r.push_command()
        time.sleep(1.0)
        r.lift.motor.wait_until_at_setpoint()
        self.assertAlmostEqual(r.status['lift']['pos'], 0.75, places=1)

        r.stop()
        self.assertTrue(success)
        #TODO lift drops when hits limit
        """
        - safety event but not guarded event triggered
        - before we triggered sync as soon as pulse raised (avoid race?)
        - before 200ms pulse to RS
        """
