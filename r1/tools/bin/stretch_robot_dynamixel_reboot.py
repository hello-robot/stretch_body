#!/usr/bin/env python3
from __future__ import print_function
import argparse
import stretch_body.hello_utils as hu
import stretch_body.robot

hu.print_stretch_re_use()


parser=argparse.ArgumentParser(description='Reboot all Dynamixel servos on robot')
args=parser.parse_args()

r=stretch_body.robot.Robot()
r.startup()

for j in ['head_pan','head_tilt']:
    if r.head.motors[j].motor.do_ping(verbose=False):
        print('Rebooting: %s'%j)
        r.head.motors[j].motor.do_reboot()

for j in r.end_of_arm.joints:
    if r.end_of_arm.get_motor(j).motor.do_ping(verbose=False):
        print('Rebooting: %s' % j)
        r.end_of_arm.get_motor(j).motor.do_reboot()
r.stop()

print('')
print('Dynamixel servo reboot complete. You will need to re-home servos now.')
