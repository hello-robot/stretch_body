#!/usr/bin/env python
from __future__ import print_function
import stretch_body.stretch_gripper as gripper
import time
import argparse
import stretch_body.hello_utils as hu
hu.print_stretch_re_use()

parser=argparse.ArgumentParser(description='Calibrate the gripper position by closing until motion stops')
args=parser.parse_args()

g=gripper.StretchGripper()
if not g.startup(threaded=False):
    exit()
g.home()
time.sleep(3.0)
g.stop()