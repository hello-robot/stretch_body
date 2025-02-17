#!/usr/bin/env python3
from stretch_body.hello_utils import *
import sys
import stretch_body.wrist_yaw as wrist_yaw
import stretch_body.wrist_pitch as wrist_pitch
import stretch_body.wrist_roll as wrist_roll
import argparse
print_stretch_re_use()

parser=argparse.ArgumentParser(description='Jog the dexterous wrist joints from the keyboard')
group = parser.add_mutually_exclusive_group(required=True)
group.add_argument("--yaw", help="Jog yaw joint",action="store_true")
group.add_argument("--pitch", help="Jog pitch joint",action="store_true")
group.add_argument("--roll", help="Jog roll joint",action="store_true")

args, _ = parser.parse_known_args()



w=None

if args.yaw:
    print("Controlling Yaw Joint")
    w=wrist_yaw.WristYaw()
    w.startup()
    poses = {'zero': 0, 'X': deg_to_rad(90), 'Y': deg_to_rad(-45)}
elif args.pitch:
    print("Controlling Pitch Joint")
    w=wrist_pitch.WristPitch()
    w.startup()
    poses = {'zero': 0, 'X': deg_to_rad(55), 'Y': deg_to_rad(-90)}
elif args.roll:
    print("Controlling Roll Joint")
    w = wrist_roll.WristRoll()
    w.startup()
    poses = {'zero': 0, 'X': deg_to_rad(90), 'Y': deg_to_rad(-90)}

if w is not None:
    v_des=w.params['motion']['default']['vel']
    a_des=w.params['motion']['default']['accel']

    def menu_top():
        print('------ MENU -------')
        print('m: menu')
        print('a: increment 15 deg')
        print('b: decrement 15 deg')
        print('p: position (ticks)')
        print('x: pose X')
        print('y: pose Y')
        print('z: pose zero')
        print('1: speed slow')
        print('2: speed default')
        print('3: speed fast')
        print('4: speed max')
        print('-------------------')

    def step_interaction():
        global v_des, a_des
        menu_top()
        x=sys.stdin.readline()
        if len(x)>1:
            if x[0]=='m':
                menu_top()
            if x[0]=='p':
                p = w.ticks_to_world_rad(float(x[1:]))
                w.move_to(p,v_des,a_des)

            if x[0] == 'a':
                w.move_by(deg_to_rad(15), v_des, a_des)

            if x[0] == 'b':
                w.move_by(deg_to_rad(-15), v_des, a_des)

            if x[0] == '1':
                v_des = w.params['motion']['slow']['vel']
                a_des = w.params['motion']['slow']['accel']

            if x[0] == '2':
                v_des = w.params['motion']['default']['vel']
                a_des = w.params['motion']['default']['accel']

            if x[0] == '3':
                v_des = w.params['motion']['fast']['vel']
                a_des = w.params['motion']['fast']['accel']

            if x[0] == '4':
                v_des = w.params['motion']['max']['vel']
                a_des = w.params['motion']['max']['accel']

            if x[0] == 'x':
                w.move_to(poses['X'], v_des, a_des)
            if x[0] == 'y':
                w.move_to(poses['Y'], v_des, a_des)
            if x[0] == 'z':
                w.move_to(poses['zero'], v_des, a_des)
        else:
            w.pretty_print()

    try:
        while True:
            try:
                step_interaction()
            except (ValueError):
                print('Bad input...')
            w.pull_status()
    except (ThreadServiceExit, KeyboardInterrupt):
        pass
w.stop()



