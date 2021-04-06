#!/usr/bin/env python
from __future__ import print_function
import stretch_body.lift as lift
import argparse
import stretch_body.hello_utils as hu
hu.print_stretch_re_use()

parser=argparse.ArgumentParser(description='Calibrate the lift position by moving to the upper hardstop')
args=parser.parse_args()

l=lift.Lift()
if not l.startup():
    exit()
l.home()
l.stop()

