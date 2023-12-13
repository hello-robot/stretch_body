#!/usr/bin/env python3
from __future__ import print_function
import stretch_body.robot_params
stretch_body.robot_params.RobotParams.set_logging_level("DEBUG")
from stretch_body.hello_utils import *
import sys
import stretch_body.wrist_pitch as wrist_pitch
import argparse
print_stretch_re_use()

parser=argparse.ArgumentParser(description='Jog the wrist_pitch joint from the keyboard')
args=parser.parse_args()

poses = {'zero':0, 'up':deg_to_rad(90), 'down': deg_to_rad(-45)}
w=wrist_pitch.WristPitch()
if not w.startup(threaded=False):
    exit()

v_des=w.params['motion']['default']['vel']
a_des=w.params['motion']['default']['accel']

def menu_top():
    print('------ MENU -------')
    print('m: menu')
    print('a: increment 15 deg')
    print('b: decrement 15 deg')
    print('p: position (ticks)')
    print('z: zero')
    print('l: up')
    print('r: down')
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

        if x[0] == 'l':
            w.move_to(poses['up'], v_des, a_des)
        if x[0] == 'r':
            w.move_to(poses['down'], v_des, a_des)
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
    w.stop()

