#!/usr/bin/env python
from __future__ import print_function
import sys, tty, termios
import time
from stretch_body.hello_utils import *
import stretch_body.base as base
import stretch_body.pimu as pimu
import argparse

print_stretch_re_use()

parser=argparse.ArgumentParser(description='Jog the base motion from the keyboard')
args=parser.parse_args()

small_rotate_rad=deg_to_rad(1.0)
large_rotate_rad=deg_to_rad(10.0)

b=base.Base()
if not b.startup():
    exit()
large_move_m=0.1
small_move_m=large_move_m/8

v_r_slow = b.translation_to_rotation(b.params['motion']['slow']['vel_m'])
v_r_def = b.translation_to_rotation(b.params['motion']['default']['vel_m'])
v_r_fast=b.translation_to_rotation(b.params['motion']['fast']['vel_m'])
v_r_max = b.translation_to_rotation(b.params['motion']['max']['vel_m'])
a_r_slow = b.translation_to_rotation(b.params['motion']['slow']['accel_m'])
a_r_def = b.translation_to_rotation(b.params['motion']['default']['accel_m'])
a_r_fast=b.translation_to_rotation(b.params['motion']['fast']['accel_m'])
a_r_max = b.translation_to_rotation(b.params['motion']['max']['accel_m'])
v_r={'fast':v_r_fast,'default':v_r_def,'slow':v_r_slow,'max':v_r_max}
a_r={'fast':a_r_fast,'default':a_r_def,'slow':a_r_slow,'max':a_r_max}

p=pimu.Pimu()
p.startup()

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
    print('')
    print('1: rate slow')
    print('2: rate default')
    print('3: rate fast')
    print('4: rate max')
    print('w: CW/CCW 90 deg')
    print('x: forward-> back 0.5m')
    print('y: spin at 22.5deg/s')
    print('')
    print('f / b / l / r : small forward / back / left / right')
    print('F / B / L / R : large forward / back / left / right')
    print('o: freewheel')
    print('p: pretty print')
    print('q: quit')
    print('')
    print('Input?')

rate ='default'

try:
    menu()
    while True:
        if True:

            c=get_keystroke()

            #Read current motor positions when in sync mode
            #p.trigger_motor_sync()
            #time.sleep(0.1)
            b.pull_status()
            #print('################################'
            #b.pretty_print()

            if c=='p':
                b.pretty_print()
            if c == '1':
                rate = 'slow'
            if c == '2':
                rate = 'default'
            if c == '3':
                rate = 'fast'
            if c == '4':
                rate = 'max'
            if c =='x':
                    print('Enter pause before starting motion (s)[10]')
                    try:
                        x=float(input())
                    except ValueError:
                        x=10.0
                    time.sleep(x)
                    b.translate_by(x_m=0.5, v_m=b.params['motion'][rate]['vel_m'], a_m=b.params['motion'][rate]['accel_m'])
                    b.push_command()
                    p.trigger_motor_sync()
                    time.sleep(4.0)
                    b.translate_by(x_m=-0.5, v_m=b.params['motion'][rate]['vel_m'], a_m=b.params['motion'][rate]['accel_m'])
                    b.push_command()
                    p.trigger_motor_sync()
                    time.sleep(4.0)
            if c =='w':
                print('Enter pause before starting motion (s)[10]')
                try:
                    x = float(input())
                except ValueError:
                    x = 10.0
                time.sleep(x)
                b.rotate_by(x_r=deg_to_rad(90.0), v_r=v_r[rate], a_r=a_r[rate])
                b.push_command()
                p.trigger_motor_sync()
                time.sleep(4.0)
                b.rotate_by(x_r=deg_to_rad(-90.0), v_r=v_r[rate], a_r=a_r[rate])
                b.push_command()
                p.trigger_motor_sync()
                time.sleep(4.0)
            if c =='y':
                print('Enter pause before starting motion (s)[10]')
                try:
                    x = float(input())
                except ValueError:
                    x = 10.0
                time.sleep(x)
                b.set_rotational_velocity(v_r=deg_to_rad(90.0)/4.0)
                b.push_command()
                p.trigger_motor_sync()
                ts=time.time()
                print('Starting 30s turn at 90deg/sec')
                tsl=time.time()
                while time.time()-ts<30.0:
                    b.pull_status()
                    print('DT:',time.time()-ts,'Vel (deg/s)', rad_to_deg(b.status['theta_vel']))
                    time.sleep(0.5)
                b.set_rotational_velocity(v_r=0)
                b.push_command()
                p.trigger_motor_sync()
                menu()
            if c == 'f':
                b.translate_by(small_move_m, v_m=b.params['motion'][rate]['vel_m'], a_m=b.params['motion'][rate]['accel_m'])
            if c == 'b':
                b.translate_by(-1*small_move_m, v_m=b.params['motion'][rate]['vel_m'], a_m=b.params['motion'][rate]['accel_m'])
            if c == 'l':
                b.rotate_by(small_rotate_rad, v_r=v_r[rate], a_r=a_r[rate])
            if c == 'r':
                b.rotate_by(-1*small_rotate_rad, v_r=v_r[rate], a_r=a_r[rate])
            if c == 'F':
                b.translate_by(large_move_m, v_m=b.params['motion'][rate]['vel_m'], a_m=b.params['motion'][rate]['accel_m'])
            if c == 'B':
                b.translate_by(-1*large_move_m, v_m=b.params['motion'][rate]['vel_m'], a_m=b.params['motion'][rate]['accel_m'])
            if c == 'L':
                b.rotate_by(large_rotate_rad, v_r=v_r[rate], a_r=a_r[rate])
            if c == 'R':
                b.rotate_by(-1*large_rotate_rad, v_r=v_r[rate], a_r=a_r[rate])
            if c == 'o':
                b.enable_freewheel_mode()
            if c=='m':
                menu()
            if c=="Q" or c=='q':
                break

            b.push_command()
            p.trigger_motor_sync()
            time.sleep(0.1)
except (KeyboardInterrupt, SystemExit):
    pass
b.stop()
