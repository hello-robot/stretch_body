#!/usr/bin/env python3
from __future__ import print_function
import stretch_body.robot as rb
import stretch_body.hello_utils as hu

import argparse
parser=argparse.ArgumentParser(description='Move robot to stow position')
args=parser.parse_args()


hu.print_stretch_re_use()

robot = rb.Robot()
robot.startup()
if not robot.pimu.status['runstop_event']:
    robot.stow()
else:
    robot.logger.warning('Cannot stow while run-stopped')
robot.stop()
