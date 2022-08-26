#!/usr/bin/env python
from __future__ import print_function
import time
import stretch_body.pimu as pimu
from stretch_body.hello_utils import *
import argparse
print_stretch_re_use()
# ###################################


parser=argparse.ArgumentParser(description='Visualize Pimu (Power+IMU) board data with an oscilloscope')
parser.add_argument("--cliff", help="Scope base cliff sensors",action="store_true")
parser.add_argument("--at_cliff", help="Scope base at_cliff signal",action="store_true")
parser.add_argument("--voltage", help="Scope bus voltage (V)",action="store_true")
parser.add_argument("--current", help="Scope bus current (A)",action="store_true")
parser.add_argument("--temp", help="Scope base internal temperature (C)",action="store_true")
parser.add_argument("--ax", help="Scope base accelerometer AX",action="store_true")
parser.add_argument("--ay", help="Scope base accelerometer AY",action="store_true")
parser.add_argument("--az", help="Scope base accelerometer AZ",action="store_true")
parser.add_argument("--mx", help="Scope base magnetometer MX",action="store_true")
parser.add_argument("--my", help="Scope base magnetometer MY",action="store_true")
parser.add_argument("--mz", help="Scope base magnetometer MZ",action="store_true")
parser.add_argument("--gx", help="Scope base gyro GX",action="store_true")
parser.add_argument("--gy", help="Scope base gyro GY",action="store_true")
parser.add_argument("--gz", help="Scope base gyro GZ",action="store_true")
parser.add_argument("--roll", help="Scope base imu Roll",action="store_true")
parser.add_argument("--pitch", help="Scope base imu Pitch",action="store_true")
parser.add_argument("--heading", help="Scope base imu Heading",action="store_true")
parser.add_argument("--bump", help="Scope base imu bump level",action="store_true")
args = parser.parse_args()


p=pimu.Pimu()
if not p.startup():
    exit()


# Cliff
if args.cliff:
    import stretch_body.scope as scope
    s=scope.Scope4(yrange=[-75,75],title='Cliff')
    try:
        while True:
            p.pull_status()
            s.step_display(p.status['cliff_range'][0],p.status['cliff_range'][1],p.status['cliff_range'][2],p.status['cliff_range'][3])
            print(p.status['cliff_range'])
            time.sleep(0.02)
    except (SystemExit, KeyboardInterrupt):
        p.stop()

if args.at_cliff:
    import stretch_body.scope as scope
    s=scope.Scope4(yrange=[-.25,1.25],title='At Cliff')
    try:
        while True:
            p.pull_status()
            s.step_display(p.status['at_cliff'][0],p.status['at_cliff'][1],p.status['at_cliff'][2],p.status['at_cliff'][3])
            print(p.status['at_cliff'])
            time.sleep(0.02)
    except (SystemExit, KeyboardInterrupt):
        p.stop()

# Voltage
if args.voltage:
    import stretch_body.scope as scope
    s=scope.Scope(yrange=[9,15],title='Voltage (V)')
    try:
        while True:
            p.pull_status()
            s.step_display(p.status['voltage'])
            print(p.status['voltage'])
            time.sleep(0.02)
    except (SystemExit, KeyboardInterrupt):
        p.stop()

# Temp
if args.temp:
    import stretch_body.scope as scope
    s=scope.Scope(yrange=[0,70],title='Temp (C)')
    try:
        while True:
            p.pull_status()
            s.step_display(p.status['temp'])
            print(p.status['temp'])
            time.sleep(0.02)
    except (SystemExit, KeyboardInterrupt):
        p.stop()

# Current
if args.current:
    import stretch_body.scope as scope
    s=scope.Scope(yrange=[0,5],title='Current (A)')
    try:
        while True:
            p.pull_status()
            s.step_display(p.status['current'])
            print(p.status['current'])
            time.sleep(0.02)
    except (SystemExit, KeyboardInterrupt):
        p.stop()

    sp = scope.Scope(yrange=[-60,60], title='Pitch')
    sh = scope.Scope(yrange=[0,360], title='Heading')


#Mag
if args.bump:
    import stretch_body.scope as scope
    s = scope.Scope(yrange=[-1,15],title='Bump')
    itr=0
    try:
        while True:
            p.pull_status()
            time.sleep(0.02)
            itr=itr+1
            if itr==50:
                itr=0
                p.trigger_beep()
                p.push_command()
            print('Bump',p.status['imu']['bump'])
            s.step_display(p.status['imu']['bump'])
    except (SystemExit, KeyboardInterrupt):
        p.stop()

#Mag
if args.mx:
    import stretch_body.scope as scope
    s = scope.Scope(yrange=[-20,20],title='MX')
    try:
        while True:
            p.pull_status()
            time.sleep(0.02)
            print('MX',p.status['imu']['mx'])
            s.step_display(p.status['imu']['mx'])
    except (SystemExit, KeyboardInterrupt):
        p.stop()

if args.my:
    import stretch_body.scope as scope
    s = scope.Scope(yrange=[-20,20],title='MY')
    try:
        while True:
            p.pull_status()
            time.sleep(0.02)
            print('MY',p.status['imu']['my'])
            s.step_display(p.status['imu']['my'])
    except (SystemExit, KeyboardInterrupt):
        p.stop()

if args.mz:
    import stretch_body.scope as scope
    s = scope.Scope(yrange=[-20,20],title='MZ')
    try:
        while True:
            p.pull_status()
            time.sleep(0.02)
            print('MZ',p.status['imu']['mz'])
            s.step_display(p.status['imu']['mz'])
    except (SystemExit, KeyboardInterrupt):
        p.stop()

#Accel
if args.ax:
    import stretch_body.scope as scope
    s = scope.Scope(yrange=[-2,2], title='AX')
    try:
        while True:
            p.pull_status()
            time.sleep(0.02)
            print('AX',p.status['imu']['ax'])
            s.step_display(p.status['imu']['ax'])
    except (SystemExit, KeyboardInterrupt):
        p.stop()

if args.ay:
    import stretch_body.scope as scope
    s = scope.Scope(yrange=[-2,2], title='AY')
    try:
        while True:
            p.pull_status()
            time.sleep(0.02)
            print('AY',p.status['imu']['ay'])
            s.step_display(p.status['imu']['ay'])
    except (SystemExit, KeyboardInterrupt):
        p.stop()

if args.az:
    import stretch_body.scope as scope
    s = scope.Scope(yrange=[-11,-8], title='AZ')
    try:
        while True:
            p.pull_status()
            time.sleep(0.02)
            print('AZ',p.status['imu']['az'])
            s.step_display(p.status['imu']['az'])
    except (SystemExit, KeyboardInterrupt):
        p.stop()


if args.gx:
    import stretch_body.scope as scope
    s = scope.Scope(yrange=[-0.5,0.5], title='GX')
    try:
        while True:
            p.pull_status()
            time.sleep(0.02)
            print('GX',p.status['imu']['gx'])
            s.step_display(p.status['imu']['gx'])
    except (SystemExit, KeyboardInterrupt):
        p.stop()

if args.gy:
    import stretch_body.scope as scope
    s = scope.Scope(yrange=[-0.5,0.5], title='GY')
    try:
        while True:
            p.pull_status()
            time.sleep(0.02)
            print('GY',p.status['imu']['gy'])
            s.step_display(p.status['imu']['gy'])
    except (SystemExit, KeyboardInterrupt):
        p.stop()

if args.gz:
    import stretch_body.scope as scope
    s = scope.Scope(yrange=[-0.5,0.5], title='GZ')
    try:
        while True:
            p.pull_status()
            time.sleep(0.02)
            print('GZ',p.status['imu']['gz'])
            s.step_display(p.status['imu']['gz'])
    except (SystemExit, KeyboardInterrupt):
        p.stop()

# IMU
if args.roll:
    import stretch_body.scope as scope
    s = scope.Scope(title='Roll',yrange=[-20,20])
    try:
        while True:
            p.pull_status()
            time.sleep(0.02)
            print('Roll',rad_to_deg(p.status['imu']['roll']))
            s.step_display(rad_to_deg(p.status['imu']['roll']))
    except (SystemExit, KeyboardInterrupt):
        p.stop()

if args.pitch:
    import stretch_body.scope as scope
    s = scope.Scope(yrange=[-20,20], title='Pitch')
    try:
        while True:
            p.pull_status()
            time.sleep(0.02)
            print('Pitch',rad_to_deg(p.status['imu']['pitch']))
            s.step_display(rad_to_deg(p.status['imu']['pitch']))
    except (SystemExit, KeyboardInterrupt):
        p.stop()

if args.heading:
    import stretch_body.scope as scope
    s = scope.Scope(yrange=[-10,370], title='Heading')
    try:
        while True:
            p.pull_status()
            time.sleep(0.02)
            print('Heading',rad_to_deg(p.status['imu']['heading']))
            s.step_display(rad_to_deg(p.status['imu']['heading']))
    except (SystemExit, KeyboardInterrupt):
        p.stop()
