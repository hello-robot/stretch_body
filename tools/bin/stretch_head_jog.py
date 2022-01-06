#!/usr/bin/env python
from __future__ import print_function
import stretch_body.robot_params
stretch_body.robot_params.RobotParams.set_logging_level("DEBUG")
import stretch_body.head as head
from stretch_body.hello_utils import *
import sys
import argparse
print_stretch_re_use()

parser=argparse.ArgumentParser(description='Jog the head from the keyboard')
args=parser.parse_args()

h=head.Head()
if not h.startup(threaded=False):
    exit()

v_des=[h.motors['head_pan'].params['motion']['default']['vel'], h.motors['head_tilt'].params['motion']['default']['vel']]
a_des=[h.motors['head_pan'].params['motion']['default']['accel'], h.motors['head_tilt'].params['motion']['default']['accel']]

def menu_top():
    print('------ MENU -------')
    print('m: menu')
    print('a: increment pan 10 deg')
    print('b: decrement pan 10 deg')
    print('c: increment tilt 10 deg')
    print('d: decrement tilt 10 deg')
    print('e: ahead')
    print('f: back')
    print('g: tool')
    print('h: wheels')
    print('i: left')
    print('j: up')
    print('p: pan go to pos ticks')
    print('t: tilt go to pos ticks')
    print('x: home')
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
            p = float(x[1:])
            p = h.motors['head_pan'].ticks_to_world_rad(p)
            h.move_to('head_pan', p, v_des[0], a_des[0])

        if x[0]=='x':
            h.home()

        if x[0]=='t':
            t = float(x[1:])
            t= h.motors['head_tilt'].ticks_to_world_rad(t)
            h.move_to('head_tilt', t, v_des[1], a_des[1])

        if x[0] == 'a':
            h.move_by('head_pan',deg_to_rad(10), v_des[0], a_des[0])
        if x[0] == 'b':
            h.move_by('head_pan', deg_to_rad(-10), v_des[0], a_des[0])
        if x[0] == 'c':
            h.move_by('head_tilt', deg_to_rad(10), v_des[1], a_des[1])
        if x[0] == 'd':
            h.move_by('head_tilt', deg_to_rad(-10), v_des[1], a_des[1])

        if x[0] == '1':
            v_des = [h.motors['head_pan'].params['motion']['slow']['vel'], h.motors['head_tilt'].params['motion']['slow']['vel']]
            a_des = [h.motors['head_pan'].params['motion']['slow']['accel'], h.motors['head_tilt'].params['motion']['slow']['accel']]

        if x[0] == '2':
            v_des = [h.motors['head_pan'].params['motion']['default']['vel'], h.motors['head_tilt'].params['motion']['default']['vel']]
            a_des = [h.motors['head_pan'].params['motion']['default']['accel'], h.motors['head_tilt'].params['motion']['default']['accel']]

        if x[0] == '3':
            v_des = [h.motors['head_pan'].params['motion']['fast']['vel'], h.motors['head_tilt'].params['motion']['fast']['vel']]
            a_des = [h.motors['head_pan'].params['motion']['fast']['accel'], h.motors['head_tilt'].params['motion']['fast']['accel']]

        if x[0] == '4':
            v_des = [h.motors['head_pan'].params['motion']['max']['vel'], h.motors['head_tilt'].params['motion']['max']['vel']]
            a_des = [h.motors['head_pan'].params['motion']['max']['accel'], h.motors['head_tilt'].params['motion']['max']['accel']]

        if x[0] == 'e':
            h.pose('ahead', v_des, a_des)
        if x[0] == 'f':
            h.pose('back', v_des, a_des)
        if x[0] == 'g':
            h.pose('tool', v_des, a_des)
        if x[0] == 'h':
            h.pose('wheels', v_des, a_des)
        if x[0] == 'i':
            h.pose('left', v_des, a_des)
        if x[0] == 'j':
            h.pose('up', v_des, a_des)
    else:
        h.pretty_print()



try:
    while True:
        try:
            step_interaction()
        except (ValueError):
            print('Bad input...')
        h.pull_status()
except (ThreadServiceExit, KeyboardInterrupt):
    h.stop()

