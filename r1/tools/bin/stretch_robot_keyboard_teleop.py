#!/usr/bin/env python3
from __future__ import print_function
import sys, tty, termios
import stretch_body.robot as hello_robot
from stretch_body.hello_utils import *
import argparse
import sys
print_stretch_re_use()


parser=argparse.ArgumentParser(description='Control the robot base, lift, arm, head, and tool from the keyboard')
args=parser.parse_args()


robot=hello_robot.Robot()
if not robot.startup():
    sys.exit(1)

small_move_m=.01
large_move_m=0.1
small_rotate_rad=deg_to_rad(1.0)
large_rotate_rad=deg_to_rad(10.0)

small_lift_move_m=.01
large_lift_move_m=.075
small_arm_move_m=.010
large_arm_move_m=.075

gg=0.5

def get_keystroke():

    fd=sys.stdin.fileno()
    old_settings=termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch=sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd,termios.TCSADRAIN,old_settings)
    return ch

def menu():
    print('--------------')
    print('Mobile Base')
    print('i / , / j / l : small forward / back / left / right')
    print('I / < / J / L : large forward / back / left / right')
    print('')
    print('Arm')
    print('w / x / a / d : small up / down / in / out')
    print('W / X / A / D : large up / down / in / out')
    print('')
    print('Head')
    print('f / h / t / b : ahead / back / tool / wheels')
    print('')
    print('Tool')
    print('8 / 2 / 0 / 5 : forward / stow / open / close')
    print('')
    print('Mechaduinos')
    print('e / p  : effort / pose ')
    print('')
    print('System')
    print('? / q : help / quit')
    print('--------------')

try:
    menu()
    while True:
        if True:

            c=get_keystroke()

            # mobile base commands
            if c == 'i':
                robot.base.translate_by(small_move_m)
            if c == ',':
                robot.base.translate_by(-1*small_move_m)
            if c == 'j':
                robot.base.rotate_by(small_rotate_rad)
            if c == 'l':
                robot.base.rotate_by(-1*small_rotate_rad)
            if c == 'I':
                robot.base.translate_by(large_move_m)
            if c == '<':
                robot.base.translate_by(-1*large_move_m)
            if c == 'J':
                robot.base.rotate_by(large_rotate_rad)
            if c == 'L':
                robot.base.rotate_by(-1*large_rotate_rad)

            # lift commands
            if c == 'w':
                robot.lift.move_by(small_lift_move_m)
            if c == 'x':
                robot.lift.move_by(-1*small_lift_move_m)
            if c == 'W':
                robot.lift.move_by(large_lift_move_m)
            if c == 'X':
                robot.lift.move_by(-1*large_lift_move_m)

            # arm commands

            if c == 'a':
                robot.arm.move_by(-1*small_arm_move_m)
            if c == 'd':
                robot.arm.move_by(small_arm_move_m)
            if c == 'A':
                robot.arm.move_by(-1 * large_arm_move_m)
            if c == 'D':
                robot.arm.move_by(large_arm_move_m)

            # head commands
            if c == 'f':
                robot.head.pose('ahead')
            if c == 'h':
                robot.head.pose('back')
            if c == 't':
                robot.head.pose('tool')
            if c == 'b':
                robot.head.pose('wheels')

            # tool commands
            if c == '2':
                robot.end_of_arm.pose('wrist_yaw','stow')
            if c == '8':
                robot.end_of_arm.pose('wrist_yaw','forward')
            if c == '0':
                robot.end_of_arm.pose('stretch_gripper','open')
            if c == '5':
                robot.end_of_arm.pose('stretch_gripper','close')
                
            # Mechaduino commands
            if c == 'e':
                print('Mobile Base Effort: ', robot.base.left_wheel.status['effort'],robot.base.right_wheel.status['effort'])
                print('Arm Force: ', robot.status['arm']['force'])
            if c == 'p':
                print('Mobile Base Pose: ', robot.base.status['pos'])
                print('Arm Position: ', robot.status['arm']['pos'])

            robot.push_command()

            
            # System commands
            if c=='Q' or c=='q':
                print('Quiting')
                robot.stop()
                sys.exit()
            if c=='?':
                menu()
                
except (ThreadServiceExit,KeyboardInterrupt, SystemExit):
    pass
