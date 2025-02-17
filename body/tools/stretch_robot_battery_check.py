#!/usr/bin/env python3
from __future__ import print_function
import stretch_body.robot as robot
from colorama import Fore, Back, Style
import argparse
import sys
import stretch_body.hello_utils as hu
hu.print_stretch_re_use()

parser=argparse.ArgumentParser(description='Print battery state to console')
args=parser.parse_args()


def val_in_range(val_name, val,vmin, vmax):
    p=val <=vmax and val>=vmin
    if p:
        print(Fore.GREEN +'[Pass] ' + val_name + ' with ' + str(val))
    else:
        print(Fore.RED +'[Fail] ' + val_name + ' with ' +str(val)+ ' out of range ' +str(vmin) + ' to ' + str(vmax))

# #####################################################

r=robot.Robot()
if not r.startup():
    sys.exit(1)
r.pimu.pull_status()
val_in_range('Voltage',r.pimu.status['voltage'], vmin=r.pimu.config['low_voltage_alert'], vmax=14.0)
val_in_range('Current',r.pimu.status['current'], vmin=0.1, vmax=r.pimu.config['high_current_alert'])
# val_in_range('CPU Temp',r.pimu.status['cpu_temp'], vmin=15, vmax=80)
print(Style.RESET_ALL)
r.stop()
