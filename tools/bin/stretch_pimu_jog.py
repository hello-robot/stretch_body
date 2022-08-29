#!/usr/bin/env python
from __future__ import print_function
import sys
from stretch_body.pimu import Pimu
import stretch_body.hello_utils as hu
hu.print_stretch_re_use()

p=Pimu()
if not p.startup():
    exit()

import argparse
parser=argparse.ArgumentParser(description='Comnmand and query the Pimu (Power+IMU) board from the keyboard')
args=parser.parse_args()

def menu():
    print('------ MENU -------')
    print('m: menu')
    print('i: reset imu')
    print('f: toggle fan')
    print('b: toggle buzzer')
    print('p: beep')
    print('t: trigger motor sync')
    print('r: reset board')
    print('x: reset runstop event')
    print('o: trigger runstop event')
    print('y: reset cliff event')
    print('-------------------')

def step_interaction():
    menu()
    x=sys.stdin.readline()
    p.pull_status()
    if len(x)>1:
        if x[0]=='m':
            menu()
        if x[0]=='x':
            print('Resetting Runstop Event')
            p.runstop_event_reset()
        if x[0]=='o':
            print('Triggering Runstop Event')
            p.runstop_event_trigger()
        if x[0]=='y':
            print('Resetting Cliff Event')
            p.cliff_event_reset()
        if x[0]=='r':
            print('Resetting Board!!!')
            p.board_reset()
        if x[0]=='i':
            p.imu_reset()
        if x[0]=='f':
            if p.status['fan_on']:
                p.set_fan_off()
            else:
                p.set_fan_on()
        if x[0] == 'p':
            p.trigger_beep()
        if x[0]=='b':
            if p.status['buzzer_on']:
                p.set_buzzer_off()
            else:
                p.set_buzzer_on()
        if x[0] == 't':
            p.trigger_motor_sync()
        p.push_command()
    else:
        p.pretty_print()

try:
    while True:
        try:
            step_interaction()
        except (ValueError):
            print('Bad input...')
except (KeyboardInterrupt, SystemExit):
    p.stop()
