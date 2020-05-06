#!/usr/bin/env python
import stretch_body.robot as robot

import argparse
parser=argparse.ArgumentParser(description='Calibrate position for all robot joints')
args=parser.parse_args()

r=robot.Robot()
r.startup()
r.home()
r.stop()
