#!/usr/bin/env python3
from __future__ import print_function
import stretch_body.robot_params
#stretch_body.robot_params.RobotParams.set_logging_level("DEBUG")
from stretch_body.robot import Robot
from stretch_body.hello_utils import *
import argparse
print_stretch_re_use()
import time

parser=argparse.ArgumentParser(description='Run the Robot Monitor and print to console')
args=parser.parse_args()

r=Robot()

print('Starting Robot Monitor. Ctrl-C to exit')
r.startup()


try:
    while True:
        time.sleep(1.0)
except (KeyboardInterrupt, SystemExit,ThreadServiceExit):
    pass
r.stop()
