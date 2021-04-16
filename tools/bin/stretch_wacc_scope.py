#!/usr/bin/env python
from __future__ import print_function
import time
import stretch_body.wacc as wacc
from stretch_body.hello_utils import *
import argparse
print_stretch_re_use()
# ###################################


parser=argparse.ArgumentParser(description='Visualize Wacc (Wrist+Accel) board data with an oscilloscope')
parser.add_argument("--ax", help="Scope  accelerometer AX",action="store_true")
parser.add_argument("--ay", help="Scope  accelerometer AY",action="store_true")
parser.add_argument("--az", help="Scope  accelerometer AZ",action="store_true")
parser.add_argument("--a0", help="Scope  analog-in-0",action="store_true")
parser.add_argument("--d0", help="Scope  digital-in-0",action="store_true")
parser.add_argument("--d1", help="Scope  digital-in-1",action="store_true")
parser.add_argument("--tap", help="Scope single tap",action="store_true")

args = parser.parse_args()


p=wacc.Wacc()
if not p.startup():
    exit()


#Accel
if args.ax:
    import stretch_body.scope as scope
    s = scope.Scope(yrange=[8,11], title='AX')
    try:
        while True:
            p.pull_status()
            time.sleep(0.02)
            print('AX',p.status['ax'])
            s.step_display(p.status['ax'])
    except (SystemExit, KeyboardInterrupt):
        p.stop()

if args.ay:
    import stretch_body.scope as scope
    s = scope.Scope(yrange=[-2,2], title='AY')
    try:
        while True:
            p.pull_status()
            time.sleep(0.02)
            print('AY',p.status['ay'])
            s.step_display(p.status['ay'])
    except (SystemExit, KeyboardInterrupt):
        p.stop()

if args.az:
    import stretch_body.scope as scope
    s = scope.Scope(yrange=[-2,2], title='AZ')
    try:
        while True:
            p.pull_status()
            time.sleep(0.02)
            print('AZ',p.status['az'])
            s.step_display(p.status['az'])
    except (SystemExit, KeyboardInterrupt):
        p.stop()



if args.a0:
    import stretch_body.scope as scope
    s = scope.Scope( title='A0')
    try:
        while True:
            p.pull_status()
            time.sleep(0.02)
            print('A0',p.status['a0'])
            s.step_display(p.status['a0'])
    except (SystemExit, KeyboardInterrupt):
        p.stop()

if args.d0:
    import stretch_body.scope as scope
    s = scope.Scope( title='D0')
    try:
        while True:
            p.pull_status()
            time.sleep(0.02)
            print('D0',p.status['d0'])
            s.step_display(p.status['d0'])
    except (SystemExit, KeyboardInterrupt):
        p.stop()

if args.d1:
    import stretch_body.scope as scope
    s = scope.Scope( title='D1')
    try:
        while True:
            p.pull_status()
            time.sleep(0.02)
            print('D1',p.status['d1'])
            s.step_display(p.status['d1'])
    except (SystemExit, KeyboardInterrupt):
        p.stop()