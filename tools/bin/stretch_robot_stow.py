#!/usr/bin/env python
from __future__ import print_function
import stretch_body.robot as rb
import stretch_body.hello_utils as hu
hu.print_stretch_re_use()

robot = rb.Robot()
robot.startup()
robot.stow()
robot.stop()
