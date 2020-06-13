#!/usr/bin/env python
import stretch_body.lift as lift
import argparse
import stretch_body.hello_utils as hu
hu.print_stretch_re_use()

parser=argparse.ArgumentParser(description='Calibrate the lift position by moving to the upper hardstop')
args=parser.parse_args()

l=lift.Lift()
l.startup()
l.home()
l.stop()

