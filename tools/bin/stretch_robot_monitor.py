#!/usr/bin/env python
from __future__ import print_function
from stretch_body.robot import Robot
from stretch_body.hello_utils import *
import argparse
print_stretch_re_use()

parser=argparse.ArgumentParser(description='Run the Robot Monitor and print to console')
args=parser.parse_args()

r=Robot()
r.params['log_to_console']=1
print('Starting Robot Monitor. Ctrl-C to exit')
r.startup()


try:
    while True:
        time.sleep(1.0)
except (KeyboardInterrupt, SystemExit,ThreadServiceExit):
    pass
r.stop()
