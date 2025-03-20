#!/usr/bin/env python3
from __future__ import print_function
import stretch_body.stretch_gripper as gripper
from stretch_body.robot_params import RobotParams
import time
import argparse
import stretch_body.hello_utils as hu
hu.print_stretch_re_use()

config_params = RobotParams().get_params()[1]
py_class = config_params[config_params['robot']['tool']]['devices']['stretch_gripper']['py_class_name']
parser=argparse.ArgumentParser(description='Calibrate the gripper position by closing until motion stops')
args=parser.parse_args()
GripperClass = getattr(gripper, py_class)
g = GripperClass()
if not g.startup(threaded=False):
    exit()
g.home()
time.sleep(3.0)
g.stop()