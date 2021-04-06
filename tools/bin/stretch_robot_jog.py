#!/usr/bin/env python
from __future__ import print_function
import sys
from stretch_body.robot import Robot
from stretch_body.hello_utils import *
import argparse
print_stretch_re_use()

parser=argparse.ArgumentParser(description='Print all robot data to console')
args=parser.parse_args()

r=Robot()
r.startup()


def menu():
    print('------ MENU -------')
    print('m: menu')


def step_interaction():
    menu()
    x=sys.stdin.readline()
    if len(x)>1:
        if x[0]=='m':
            menu()

    else:
        r.pretty_print()

try:
    while True:
        try:
            step_interaction()
        except (ValueError):
            print('Bad input...')
except (KeyboardInterrupt, SystemExit,ThreadServiceExit):
    r.stop()
