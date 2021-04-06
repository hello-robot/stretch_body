#!/usr/bin/env python
from __future__ import print_function
import stretch_body.pimu as pimu
import stretch_body.head as head
import stretch_body.end_of_arm as end_of_arm
import stretch_body.wacc as wacc
import stretch_body.stepper as stepper
import os, fnmatch
import subprocess
from colorama import Fore, Back, Style
import argparse
import stretch_body.hello_utils as hu
hu.print_stretch_re_use()


parser=argparse.ArgumentParser(description='Check that all robot hardware is present and reporting sane values')
args=parser.parse_args()

# #####################################################
def val_in_range(val_name, val,vmin, vmax):
    p=val <=vmax and val>=vmin
    if p:
        print(Fore.GREEN +'[Pass] ' + val_name + ' = ' + str(val))
    else:
        print(Fore.RED +'[Fail] ' + val_name + ' = ' +str(val)+ ' out of range ' +str(vmin) + ' to ' + str(vmax))

def val_is_not(val_name, val,vnot):
    if val is not vnot:
        print(Fore.GREEN +'[Pass] ' + val_name + ' = ' + str(val))
    else:
        print(Fore.RED +'[Fail] ' + val_name + ' = ' +str(val))

# #####################################################
print(Style.RESET_ALL)
print('---- Checking Devices ----')
robot_devices={'hello-wacc':0, 'hello-motor-left-wheel':0,'hello-pimu':0, 'hello-lrf':0,'hello-dynamixel-head':0,'hello-dynamixel-wrist':0,'hello-motor-arm':0,'hello-motor-right-wheel':0,
               'hello-motor-lift':0,'hello-respeaker':0}

listOfFiles = os.listdir('/dev')
pattern = "hello*"
for entry in listOfFiles:
    if fnmatch.fnmatch(entry, pattern):
            robot_devices[entry]=1
for k in robot_devices.keys():
    if robot_devices[k]:
        print(Fore.GREEN +'[Pass] : '+k)
    else:
        print(Fore.RED +'[Fail] : '+ k)
# #####################################################
print(Style.RESET_ALL)
if robot_devices['hello-pimu']:
    print('---- Checking Pimu ----')
    p=pimu.Pimu(verbose=False)
    p.startup()
    p.pull_status()
    val_in_range('Voltage',p.status['voltage'], vmin=p.config['low_voltage_alert'], vmax=14.5)
    val_in_range('Current',p.status['current'], vmin=0.5, vmax=p.config['high_current_alert'])
    val_in_range('Temperature',p.status['temp'], vmin=10, vmax=40)
    val_in_range('Cliff-0',p.status['cliff_range'][0], vmin=p.config['cliff_thresh'], vmax=20)
    val_in_range('Cliff-1',p.status['cliff_range'][1], vmin=p.config['cliff_thresh'], vmax=20)
    val_in_range('Cliff-2',p.status['cliff_range'][2], vmin=p.config['cliff_thresh'], vmax=20)
    val_in_range('Cliff-3',p.status['cliff_range'][3], vmin=p.config['cliff_thresh'], vmax=20)
    val_in_range('IMU AZ',p.status['imu']['az'], vmin=-10.1, vmax=-9.5)
    val_in_range('IMU Pitch', hu.rad_to_deg(p.status['imu']['pitch']), vmin=-12, vmax=12)
    val_in_range('IMU Roll', hu.rad_to_deg(p.status['imu']['roll']), vmin=-12, vmax=12)
    print(Style.RESET_ALL)
    p.stop()
# #####################################################
print(Style.RESET_ALL)
if robot_devices['hello-dynamixel-wrist']:
    print('---- Checking EndOfArm ----')
    w = end_of_arm.EndOfArm()
    try:
        #w.startup()
        for mk in w.motors.keys():
            if w.motors[mk].do_ping(verbose=True):
                print(Fore.GREEN +'[Pass] Ping of: '+mk)
            else:
                print(Fore.RED + '[Fail] Ping of: ' + mk)
            if w.motors[mk].motor.is_calibrated():
                print(Fore.GREEN +'[Pass] Calibrated: '+mk)
            else:
                print(Fore.RED + '[Fail] Not Calibrated: ' + mk)
            print(Style.RESET_ALL)
        # w.stop()
    except IOError:
        print(Fore.RED + '[Fail] Startup of EndOfArm')
# #####################################################
print(Style.RESET_ALL)
if robot_devices['hello-dynamixel-head']:
    print('---- Checking Head ----')
    h = head.Head()
    #h.startup()
    for mk in h.motors.keys():
        if h.motors[mk].do_ping(verbose=True):
            print(Fore.GREEN +'[Pass] Ping of: '+mk)
        else:
            print(Fore.RED + '[Fail] Ping of: ' + mk)
        print(Style.RESET_ALL)
    #h.stop()
# #####################################################
print(Style.RESET_ALL)
if robot_devices['hello-wacc']:
    print('---- Checking Wacc ----')
    w=wacc.Wacc(verbose=False)
    w.startup()
    w.pull_status()
    val_in_range('AX',w.status['ax'], vmin=8.0, vmax=11.0)
    print(Style.RESET_ALL)
    w.stop()

# #####################################################
print(Style.RESET_ALL)
if robot_devices['hello-motor-left-wheel']:
    print('---- Checking hello-motor-left-wheel ----')
    m = stepper.Stepper('/dev/hello-motor-left-wheel',verbose=False)
    m.startup()
    m.pull_status()
    val_is_not('Position',m.status['pos'], vnot=0)
    print(Style.RESET_ALL)
    m.stop()
# #####################################################
print(Style.RESET_ALL)
if robot_devices['hello-motor-right-wheel']:
    print('---- Checking hello-motor-right-wheel ----')
    m = stepper.Stepper('/dev/hello-motor-right-wheel',verbose=False)
    m.startup()
    m.pull_status()
    val_is_not('Position',m.status['pos'], vnot=0)
    print(Style.RESET_ALL)
    m.stop()
# #####################################################
print(Style.RESET_ALL)
if robot_devices['hello-motor-arm']:
    print('---- Checking hello-motor-arm ----')
    m = stepper.Stepper('/dev/hello-motor-arm',verbose=False)
    m.startup()
    m.pull_status()
    val_is_not('Position',m.status['pos'], vnot=0)
    val_is_not('Position Calibrated', m.status['pos_calibrated'], vnot=False)
    print(Style.RESET_ALL)
    m.stop()
# #####################################################
print(Style.RESET_ALL)
if robot_devices['hello-motor-lift']:
    print('---- Checking hello-motor-lift ----')
    m = stepper.Stepper('/dev/hello-motor-lift',verbose=False)
    m.startup()
    m.pull_status()
    val_is_not('Position',m.status['pos'], vnot=0)
    val_is_not('Position Calibrated', m.status['pos_calibrated'], vnot=False)
    print(Style.RESET_ALL)
    m.stop()
# #####################################################
print(Style.RESET_ALL)
print ('---- Checking for Intel D435i ----')
cmd = "lsusb -d 8086:0b3a"
returned_value = subprocess.call(cmd,shell=True)  # returns the exit code in unix
if returned_value==0:
    print(Fore.GREEN + '[Pass] : Device found ')
else:
    print(Fore.RED + '[Fail] : No device found')
