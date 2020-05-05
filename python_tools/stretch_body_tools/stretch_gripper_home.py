#!/usr/bin/env python

import stretch_body.stretch_gripper as gripper
import time
import argparse

parser=argparse.ArgumentParser(description='Calibrate the gripper position by closing until motion stops')
args=parser.parse_args()

g=gripper.StretchGripper()
g.startup()
g.home()
time.sleep(3.0)
g.stop()