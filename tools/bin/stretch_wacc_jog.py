#!/usr/bin/env python3
from __future__ import print_function
import sys
from stretch_body.wacc import Wacc
import argparse
import stretch_body.hello_utils as hu
hu.print_stretch_re_use()

parser=argparse.ArgumentParser(description='Comnmand and query the Wacc (Wrist Accelerometer) board from the keyboard')
args=parser.parse_args()

w=Wacc()
if not w.startup():
    exit()



def menu():
    print('------ MENU -------')
    print('m: menu')
    print('r: reset board')
    print('a: set D2 on')
    print('b: set D2 off')
    print('c: set D3 on')
    print('d: set D3 off')
    print('-------------------')

def step_interaction():
    menu()
    x=sys.stdin.readline()
    if len(x)>1:
        if x[0]=='m':
            menu()
        if x[0] == 'a':
            w.set_D2(1)
        if x[0] == 'b':
            w.set_D2(0)
        if x[0] == 'c':
            w.set_D3(1)
        if x[0] == 'd':
            w.set_D3(0)
        if x[0]=='r':
            print('Resetting Board. Exiting...')
            w.board_reset()
            w.push_command()
            exit()
        w.push_command()
    else:
        w.pull_status()
        w.pretty_print()

try:
    while True:
        try:
            step_interaction()
        except (ValueError):
            print('Bad input...')
except (KeyboardInterrupt, SystemExit):
    w.stop()
