#!/usr/bin/env python

import stretch_body.wrist_yaw as wrist_yaw
import argparse
parser=argparse.ArgumentParser(description='Calibrate the wrist_yaw position by moving to both hardstops')
args=parser.parse_args()

g=wrist_yaw.WristYaw()
g.startup()
g.home()
g.stop()
