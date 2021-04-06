#!/usr/bin/env python
from __future__ import print_function
import os
import subprocess
import stretch_body.hello_utils as hu
hu.print_stretch_re_use()

print('Versions')
print("-------------------------------------------------------------------------")
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
print('Right wheel:     %s' %b.right_wheel.board_info['board_version'])
print('Lift stepper:    %s' %l.motor.board_info['board_version'])
print('Arm stepper:     %s' %a.motor.board_info['board_version'])
print('---------- Firmware ----------')
print('Pimu:            %s'%p.board_info['firmware_version'])
print('Wacc:            %s'%w.board_info['firmware_version'])
print('Left wheel:      %s'%b.left_wheel.board_info['firmware_version'])
print('Right wheel:     %s'%b.right_wheel.board_info['firmware_version'])
print('Lift:            %s'%l.motor.board_info['firmware_version'])
print('Arm:             %s'%a.motor.board_info['firmware_version'])
p.stop()
w.stop()
b.stop()
l.stop()
a.stop()
print("")


subprocess.call(['echo', 'USB Root Hubs'])
subprocess.call(['echo', "-------------------------------------------------------------------------"])
subprocess.call(['lsusb', '-s' ':1'])
subprocess.call(['echo'," "])


subprocess.call(['echo', "USB Bus 1"])
subprocess.call(['echo', "-------------------------------------------------------------------------"])
subprocess.call(['lsusb', '-s','1:'])
subprocess.call(['echo'," "])

subprocess.call(['echo', "USB Bus 2"])
subprocess.call(['echo', "-------------------------------------------------------------------------"])
subprocess.call(['lsusb','-s', '2:'])
subprocess.call(['echo'," "])

subprocess.call(['echo', "USB Bus 3"])
subprocess.call(['echo', "-------------------------------------------------------------------------"])
subprocess.call(['lsusb', '-s', '3:'])
subprocess.call(['echo'," "])

subprocess.call(['echo', "USB Bus 4"])
subprocess.call(['echo', "-------------------------------------------------------------------------"])
subprocess.call(['lsusb', '-s', '4:'])
subprocess.call(['echo'," "])


subprocess.call(['echo', "Hello Robot Devices"])
subprocess.call(['echo', "-------------------------------------------------------------------------"])
#subprocess.call(['ls', '-l', '/dev/hello*'])
os.system('ls -l /dev/hello*')
subprocess.call(['echo'," "])

subprocess.call(['echo', " Hard Drive"])
subprocess.call(['echo', "-------------------------------------------------------------------------"])
subprocess.call(['df','-h', '--type=ext4'])
subprocess.call(['echo',"---------"])
subprocess.call(['sudo','nvme','list'])
subprocess.call(['echo',"---------"])
subprocess.call(['sudo', 'nvme', 'smart-log', '/dev/nvme0n1'])
subprocess.call(['echo',"---------"])
subprocess.call(['echo'," "])
subprocess.call(['echo'," "])

subprocess.call(['echo', "Network"])
subprocess.call(['echo', "-------------------------------------------------------------------------"])
subprocess.call(['ifconfig'])
subprocess.call(['echo'," "])

subprocess.call(['echo', "D435i Camera"])
subprocess.call(['echo', "-------------------------------------------------------------------------"])
subprocess.call(['lsusb','-d', '8086:0b3a'])
subprocess.call(['echo'," "])

subprocess.call(['echo', "Temperatures"])
subprocess.call(['echo', "-------------------------------------------------------------------------"])
subprocess.call(['sensors'])
subprocess.call(['echo'," "])

subprocess.call(['echo', "Reboot History"])
subprocess.call(['echo', "-------------------------------------------------------------------------"])
subprocess.call(['last','reboot'])
subprocess.call(['echo'," "])

