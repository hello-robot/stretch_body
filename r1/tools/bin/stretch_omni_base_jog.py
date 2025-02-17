#!/usr/bin/env python3
from __future__ import print_function
import sys, tty, termios
import time
from stretch_body.hello_utils import *
import stretch_body.omni_base as base
import stretch_body.pimu as pimu
import argparse

print_stretch_re_use()

parser=argparse.ArgumentParser(description='Jog the base motion from the keyboard')
args=parser.parse_args()



b=base.OmniBase()

# pKd_d=100.0
# pKi_d=0.0#.01
# pKi_limit=50.0
# pKp_d=24.0 #12.0
#
# for i in range(3):
#     b.wheels[i].gains['pKd_d']=pKd_d
#     b.wheels[i].gains['pKi_d']=pKi_d
#     b.wheels[i].gains['pKi_limit']=pKi_limit
#     b.wheels[i].gains['pKp_d']=pKp_d


if not b.startup(threaded=False):
    exit()

large_move_m=0.1
small_move_m=large_move_m/8
small_rotate_rad=deg_to_rad(1.0)
large_rotate_rad=deg_to_rad(10.0)

# v_r_slow = b.translation_to_rotation(b.params['motion']['slow']['vel_xy_m'])
# v_r_def = b.translation_to_rotation(b.params['motion']['default']['vel_xy_m'])
# v_r_fast=b.translation_to_rotation(b.params['motion']['fast']['vel_xy_m'])
# v_r_max = b.translation_to_rotation(b.params['motion']['max']['vel_xy_m'])
# a_r_slow = b.translation_to_rotation(b.params['motion']['slow']['accel_xy_m'])
# a_r_def = b.translation_to_rotation(b.params['motion']['default']['accel_xy_m'])
# a_r_fast=b.translation_to_rotation(b.params['motion']['fast']['accel_xy_m'])
# a_r_max = b.translation_to_rotation(b.params['motion']['max']['accel_xy_m'])
# v_r={'fast':v_r_fast,'default':v_r_def,'slow':v_r_slow,'max':v_r_max}
# a_r={'fast':a_r_fast,'default':a_r_def,'slow':a_r_slow,'max':a_r_max}

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
    print('o: freewheel')
    print('h: hold')
    print('')
    print('f / b / l / r : small forward / back / left / right')
    print('F / B / L / R : large forward / back / left / right')
    print('u / v : small rotate CW / CCW')
    print('U / V : large rotate CW / CCW')
    print('')
    print('')
    print('w: cycle CW/CCW 90 deg')
    print('x: cycle forward-> back 0.5m')
    print('y: cycle right-> left 0.5m')
    print('s: cycle square of 0.5m')
    print('z: spin at 90deg/s')
    print('')

    print('p: pretty print')
    print('q: quit')
    print('')
    print('Input?')

rate ='default'

try:
    while True:
        if True:
            menu()
            print('---------')
            c=get_keystroke()

            #Read current motor positions when in sync mode
            #p.trigger_motor_sync()
            #time.sleep(0.1)
            b.pull_status()

            # ################################################################################################
            if c=='p':
                b.pretty_print()
            if c == 'o':
                b.enable_freewheel_mode()
            if c == 'h':
                b.enable_hold_mode()
            if c == 'm':
                menu()
            if c == "Q" or c == 'q':
                break
            # ################################################################################################
            if c == 'r':
                b.translate_by(x_m=small_move_m, y_m=0, v_m=b.params['motion'][rate]['vel_xy_m'],
                               a_m=b.params['motion'][rate]['accel_xy_m'])
            if c == 'l':
                b.translate_by(x_m=-1 * small_move_m, y_m=0, v_m=b.params['motion'][rate]['vel_xy_m'],
                               a_m=b.params['motion'][rate]['accel_xy_m'])
            if c == 'R':
                b.translate_by(x_m=large_move_m, y_m=0, v_m=b.params['motion'][rate]['vel_xy_m'],
                               a_m=b.params['motion'][rate]['accel_xy_m'])
            if c == 'L':
                b.translate_by(x_m=-1 * large_move_m, y_m=0, v_m=b.params['motion'][rate]['vel_xy_m'],
                               a_m=b.params['motion'][rate]['accel_xy_m'])
            if c == 'b':
                b.translate_by(x_m=0, y_m=-1 * small_move_m, v_m=b.params['motion'][rate]['vel_xy_m'],
                               a_m=b.params['motion'][rate]['accel_xy_m'])
            if c == 'f':
                b.translate_by(x_m=0, y_m=small_move_m, v_m=b.params['motion'][rate]['vel_xy_m'],
                               a_m=b.params['motion'][rate]['accel_xy_m'])
            if c == 'B':
                b.translate_by(x_m=0, y_m=-1 * large_move_m, v_m=b.params['motion'][rate]['vel_xy_m'],
                               a_m=b.params['motion'][rate]['accel_xy_m'])
            if c == 'F':
                b.translate_by(x_m=0, y_m=large_move_m, v_m=b.params['motion'][rate]['vel_xy_m'],
                               a_m=b.params['motion'][rate]['accel_xy_m'])

            if c == 'u':
                b.rotate_by(w_r=small_rotate_rad, v_r=b.params['motion'][rate]['vel_w_r'],
                               a_r=b.params['motion'][rate]['accel_w_r'])
            if c == 'v':
                b.rotate_by(w_r=-1*small_rotate_rad, v_r=b.params['motion'][rate]['vel_w_r'],
                               a_r=b.params['motion'][rate]['accel_w_r'])

            if c == 'U':
                b.rotate_by(w_r=large_rotate_rad, v_r=b.params['motion'][rate]['vel_w_r'],
                               a_r=b.params['motion'][rate]['accel_w_r'])
            if c == 'V':
                b.rotate_by(w_r=-1*large_rotate_rad, v_r=b.params['motion'][rate]['vel_w_r'],
                               a_r=b.params['motion'][rate]['accel_w_r'])
            # ################################################################################################
            if c == '1':
                rate = 'slow'
            if c == '2':
                rate = 'default'
            if c == '3':
                rate = 'fast'
            if c == '4':
                rate = 'max'

            if c=='z':
                print('Enter num revolutions [1]')
                try:
                    x = int(input())
                except ValueError:
                    x = 1
                ts=time.time()
                b.set_omni_velocity('w',v_des=deg_to_rad(90),a_des=b.params['motion'][rate]['accel_w_r'])
                b.push_command()
                p.trigger_motor_sync()
                while time.time()-ts<(x*4+0.5):
                    time.sleep(0.1)
                b.set_omni_velocity('w',v_des=0,a_des=b.params['motion'][rate]['accel_w_r'])
                b.push_command()
                p.trigger_motor_sync()

            if c =='s':
                    print('Enter num cycles [10]')

                    try:
                        x=int(input())
                    except ValueError:
                        x=10
                    qq=[[0.5, 0], [0,0.5], [-0.5,0],[0,-0.5]]
                    for i in range(x):
                        for q in qq:
                            b.translate_by(q[0], y_m=q[1],v_m=b.params['motion'][rate]['vel_xy_m'], a_m=b.params['motion'][rate]['accel_xy_m'])
                            b.push_command()
                            p.trigger_motor_sync()
                            time.sleep(2.5)
            if c =='x':
                    print('Enter num cycles [10]')
                    try:
                        x=int(input())
                    except ValueError:
                        x=10
                    for i in range(x):
                        #time.sleep(x)
                        b.translate_by(x_m=0.5, y_m=0.0,v_m=b.params['motion'][rate]['vel_xy_m'], a_m=b.params['motion'][rate]['accel_xy_m'])
                        b.push_command()
                        p.trigger_motor_sync()
                        for k in range(40):
                            b.pull_status()
                            print('Vx',b.status['x_vel'])
                            print('Vy', b.status['y_vel'])
                            print('Vw', b.status['theta_vel'])
                            time.sleep(0.1)
                        #time.sleep(4.0)
                        b.translate_by(x_m=-0.5,y_m=0.0,v_m=b.params['motion'][rate]['vel_xy_m'], a_m=b.params['motion'][rate]['accel_xy_m'])
                        b.push_command()
                        p.trigger_motor_sync()
                        for k in range(40):
                            b.pull_status()
                            print('Vx', b.status['x_vel'])
                            print('Vy', b.status['y_vel'])
                            print('Vw', b.status['theta_vel'])
                            time.sleep(0.1)
            if c =='y':
                    print('Enter num cycles [10]')
                    try:
                        x=int(input())
                    except ValueError:
                        x=10
                    for i in range(x):
                        #time.sleep(x)
                        b.translate_by(x_m=0.0, y_m=0.5,v_m=b.params['motion'][rate]['vel_xy_m'], a_m=b.params['motion'][rate]['accel_xy_m'])
                        b.push_command()
                        p.trigger_motor_sync()
                        time.sleep(4.0)
                        b.translate_by(x_m=0.0,y_m=-0.5,v_m=b.params['motion'][rate]['vel_xy_m'], a_m=b.params['motion'][rate]['accel_xy_m'])
                        b.push_command()
                        p.trigger_motor_sync()
                        time.sleep(4.0)
            if c =='w':
                print('Enter num cycles [10]')
                try:
                    x = int(input())
                except ValueError:
                    x = 10
                for i in range(x):
                    b.rotate_by(w_r=deg_to_rad(90.0), v_r=b.params['motion'][rate]['vel_w_r'],a_r=b.params['motion'][rate]['accel_w_r'])
                    b.push_command()
                    p.trigger_motor_sync()
                    time.sleep(4.0)
                    b.rotate_by(w_r=deg_to_rad(-90.0), v_r=b.params['motion'][rate]['vel_w_r'],a_r=b.params['motion'][rate]['accel_w_r'])
                    b.push_command()
                    p.trigger_motor_sync()
                    time.sleep(4.0)

            b.push_command()
            p.trigger_motor_sync()
            time.sleep(0.1)
except (KeyboardInterrupt, SystemExit):
    pass
b.stop()
