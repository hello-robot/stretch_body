#!/usr/bin/env python
from __future__ import print_function
import stretch_body.wrist_yaw as wrist_yaw
import argparse
import stretch_body.hello_utils as hu
hu.print_stretch_re_use()

parser=argparse.ArgumentParser(description='Calibrate the wrist_yaw position by moving to both hardstops')
args=parser.parse_args()

g=wrist_yaw.WristYaw()
if not g.startup():
    exit()
g.home()
g.stop()
