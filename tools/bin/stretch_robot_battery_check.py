#!/usr/bin/env python
from __future__ import print_function
import stretch_body.pimu as pimu
from colorama import Fore, Back, Style
import argparse
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

p=pimu.Pimu(verbose=False)
if not p.startup():
    exit()
p.pull_status()
val_in_range('Voltage',p.status['voltage'], vmin=p.config['low_voltage_alert'], vmax=14.0)
val_in_range('Current',p.status['current'], vmin=0.1, vmax=p.config['high_current_alert'])
val_in_range('CPU Temp',p.status['cpu_temp'], vmin=15, vmax=80)
print(Style.RESET_ALL)
p.stop()
