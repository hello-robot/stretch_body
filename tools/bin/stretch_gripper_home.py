#!/usr/bin/env python

import stretch_body.stretch_gripper as gripper
import time
import argparse
import stretch_body.hello_utils as hu
hu.print_stretch_re_use()

parser=argparse.ArgumentParser(description='Calibrate the gripper position by closing until motion stops')
args=parser.parse_args()

g=gripper.StretchGripper()
g.startup()
g.home()
time.sleep(3.0)
g.stop()