#!/usr/bin/env python
from __future__ import print_function
from stretch_body.dynamixel_XL430 import *
import argparse
import stretch_body.hello_utils as hu
hu.print_stretch_re_use()

parser=argparse.ArgumentParser(description='Reboot all Dynamixel servos on robot')
args=parser.parse_args()

print('---- Rebooting Head ---- ')
for id in range(15):
    m = DynamixelXL430(id, '/dev/hello-dynamixel-head')
    m.startup()
    if (m.do_ping(verbose=False)):
        m.do_reboot()
    else:
        m.stop()

print('---- Rebooting Wrist ---- ')
for id in range(15):
    m = DynamixelXL430(id, '/dev/hello-dynamixel-wrist')
    m.startup()
    if (m.do_ping(verbose=False)):
        m.do_reboot()
    else:
        m.stop()