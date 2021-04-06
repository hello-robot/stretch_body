#!/usr/bin/env python
from __future__ import print_function
import stretch_body.robot as robot
import argparse
import stretch_body.hello_utils as hu
hu.print_stretch_re_use()

parser=argparse.ArgumentParser(description='Calibrate position for all robot joints')
args=parser.parse_args()

r=robot.Robot()
r.startup()
r.home()
r.stop()
