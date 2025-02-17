#!/usr/bin/env python3
from __future__ import print_function
import stretch_body.arm as arm
import stretch_body.hello_utils as hu
hu.print_stretch_re_use()

import argparse
parser=argparse.ArgumentParser(description='Calibrate arm position by movint to hardstop')
args=parser.parse_args()

a=arm.Arm()
if not a.startup(threaded=False):
    exit()
a.home()
a.stop()

