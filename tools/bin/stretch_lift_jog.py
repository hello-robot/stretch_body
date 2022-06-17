#!/usr/bin/env python
from __future__ import print_function
import sys, tty, termios
import time
import stretch_body.lift as lift
import argparse
import stretch_body.hello_utils as hu
hu.print_stretch_re_use()

parser=argparse.ArgumentParser(description='Jog the lift motion from the keyboard')
args=parser.parse_args()

small_move_m=.01
large_move_m=0.3



l=lift.Lift()
if not l.startup(threaded=False):
    exit()
l.motor.disable_sync_mode()
l.push_command()

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
    print('m: menu')
    print('u / d : small up down')
    print('U / D : large up down')
    print('f: stiffness float')
    print('s: stiffness soft')
    print('h: stiffness hard')
    print('1: rate slow')
    print('2: rate default')
    print('3: rate fast')
    print('4: rate max')
    print('q: quit')
    print('')
    print('Input?')

rate='default'
req_calibration=False
stiffness=1.0
try:
    menu()
    while True:
        c=get_keystroke()
        l.pull_status()
        l.pretty_print()

        if c == 'f':
            stiffness=0.0
            l.move_by(x_m=0, v_m=l.params['motion'][rate]['vel_m'], a_m=l.params['motion'][rate]['accel_m'],
                      stiffness=stiffness, req_calibration=req_calibration)
        if c == 's':
            stiffness=0.3
            l.move_by(x_m=0, v_m=l.params['motion'][rate]['vel_m'], a_m=l.params['motion'][rate]['accel_m'],
                      stiffness=stiffness, req_calibration=req_calibration)
        if c == 'h':
            stiffness=1.0
            l.move_by(x_m=0, v_m=l.params['motion'][rate]['vel_m'], a_m=l.params['motion'][rate]['accel_m'],
                      stiffness=stiffness, req_calibration=req_calibration)

        if c=='1':
            rate='slow'
        if c == '2':
            rate = 'default'
        if c == '3':
            rate = 'fast'
        if c == '4':
            rate = 'max'
        if c=='m':
            menu()
        if c=="Q" or c=='q':
            break
        if c == 'u':
            l.move_by(x_m= small_move_m, v_m=l.params['motion'][rate]['vel_m'], a_m=l.params['motion'][rate]['accel_m'],stiffness=stiffness, req_calibration=req_calibration)
        if c == 'd':
            l.move_by(x_m=-1*small_move_m, v_m=l.params['motion'][rate]['vel_m'], a_m=l.params['motion'][rate]['accel_m'],stiffness=stiffness, req_calibration=req_calibration)
        if c == 'U':
            l.move_by(x_m=large_move_m, v_m=l.params['motion'][rate]['vel_m'], a_m=l.params['motion'][rate]['accel_m'],stiffness=stiffness, req_calibration=req_calibration)
        if c == 'D':
            l.move_by(x_m=-1*large_move_m, v_m=l.params['motion'][rate]['vel_m'], a_m=l.params['motion'][rate]['accel_m'],stiffness=stiffness, req_calibration=req_calibration)
        l.push_command()
        time.sleep(0.1)
except (KeyboardInterrupt, SystemExit):
    pass
l.stop()
