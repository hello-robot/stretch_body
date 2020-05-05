#!/usr/bin/env python
import os
import subprocess

print 'Versions'
print "-------------------------------------------------------------------------"
import stretch_body
import stretch_body.base as base
import stretch_body.arm as arm
import stretch_body.lift as lift
import stretch_body.pimu as pimu
import stretch_body.wacc as wacc
p=pimu.Pimu()
p.startup()
w=wacc.Wacc()
w.startup()
b=base.Base()
b.startup()
a=arm.Arm()
a.startup()
l=lift.Lift()
l.startup()
print('Version: stretch_body: %s'%stretch_body.__version__)
print('---------- Boards ----------')
print('Pimu:            %s'%p.board_info['board_version'])
print('Wacc:            %s'%w.board_info['board_version'])
print('Left wheel:      %s' %b.left_wheel.board_info['board_version'])
print('Right wheel:     %s' %b.left_wheel.board_info['board_version'])
print('Lift stepper:    %s' %l.motor.board_info['board_version'])
print('Arm stepper:     %s' %a.motor.board_info['board_version'])
print('---------- Firmware ----------')
print('Pimu:            %s'%p.board_info['firmware_version'])
print('Wacc:            %s'%w.board_info['firmware_version'])
print('Left wheel:      %s'%b.left_wheel.board_info['firmware_version'])
print('Right wheel:     %s'%b.left_wheel.board_info['firmware_version'])
print('Lift:            %s'%l.motor.board_info['firmware_version'])
print('Arm:             %s'%a.motor.board_info['firmware_version'])
p.stop()
w.stop()
b.stop()
l.stop()
a.stop()
print ""

print "USB Root Hubs"
print "-------------------------------------------------------------------------"
subprocess.call(['lsusb', '-s' ':1'])
print ""

print "USB Bus 1"
print "-------------------------------------------------------------------------"
subprocess.call(['lsusb', '-s','1:'])
print ""

print "USB Bus 2"
print "-------------------------------------------------------------------------"
subprocess.call(['lsusb','-s', '2:'])
print ""

print "USB Bus 3"
print "-------------------------------------------------------------------------"
subprocess.call(['lsusb', '-s', '3:'])
print ""

print "USB Bus 4"
print "-------------------------------------------------------------------------"
subprocess.call(['lsusb', '-s', '4:'])
print ""


print "Hello Robot Devices"
print "-------------------------------------------------------------------------"
#subprocess.call(['ls', '-l', '/dev/hello*'])
os.system('ls -l /dev/hello*')
print ""

print " Hard Drive"
print "-------------------------------------------------------------------------"
subprocess.call(['df','-h', '--type=ext4'])
print "---------"
subprocess.call(['sudo','nvme','list'])
print "---------"
subprocess.call(['sudo', 'nvme', 'smart-log', '/dev/nvme0n1'])
print "---------"
print ""

print "Network"
print "-------------------------------------------------------------------------"
subprocess.call(['ifconfig'])
print ""


print 'D435i Camera'
print "-------------------------------------------------------------------------"
subprocess.call(['lsusb','-d', '8086:0b3a'])
print ""

print 'Temperatures'
print "-------------------------------------------------------------------------"
subprocess.call(['sensors'])
print ""

print 'Reboot History'
print "-------------------------------------------------------------------------"
subprocess.call(['last','reboot'])
print ""

