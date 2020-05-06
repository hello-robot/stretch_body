#!/usr/bin/env python

import stretch_body.arm as arm

import argparse
parser=argparse.ArgumentParser(description='Calibrate arm position by moving to one/both hardstops')
parser.add_argument("--double", help="Home to both hardstops",action="store_true")
args=parser.parse_args()

a=arm.Arm()
a.startup()
a.home(single_stop= not args.double)
a.stop()

