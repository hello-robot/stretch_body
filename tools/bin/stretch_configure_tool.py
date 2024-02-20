#!/usr/bin/env python3

import stretch_body.hello_utils as hu
hu.print_stretch_re_use()

import argparse
import stretch_body.robot as robot

parser=argparse.ArgumentParser(description='Check that all robot hardware is present and reporting sane values')
parser.add_argument('-v', "--verbose", help="Prints more information", action="store_true")
args=parser.parse_args()

