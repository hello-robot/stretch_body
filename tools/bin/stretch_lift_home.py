#!/usr/bin/env python

import stretch_body.lift as lift

import argparse
parser=argparse.ArgumentParser(description='Calibrate the lift position by moving to the upper hardstop')
args=parser.parse_args()

l=lift.Lift()
l.startup()
l.home()
l.stop()

