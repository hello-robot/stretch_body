#!/usr/bin/env python3
from __future__ import print_function
from rplidar import *
import argparse
import stretch_body.hello_utils as hu
from stretch_body.device import Device
import time

hu.print_stretch_re_use()

parser=argparse.ArgumentParser(description='Tool to control the RP-Lidar')
parser.add_argument("--motor_on", help="Turn motor on",action="store_true")
parser.add_argument("--motor_off", help="Turn motor off",action="store_true")
parser.add_argument("--info", help="Device info",action="store_true")
parser.add_argument("--health", help="Get device health",action="store_true")
parser.add_argument("--reset", help="Reset device",action="store_true")
parser.add_argument("--range", help="Print range reading",action="store_true")
parser.add_argument("--speed", help="Measure the sampling speed",action="store_true")
args=parser.parse_args()


try:
    lidar_dev = Device('lidar')
    lidar_usb = lidar_dev.params['usb_name']
    lidar = RPLidar(lidar_usb,baudrate=lidar_dev.params['baud'])
except RPLidarException:
    print ('RPLidar not present')
    exit()
prev_motor_running=lidar.motor_running

if args.info:
    print(lidar.get_info())

if args.motor_on:
    lidar.start_motor()
    prev_motor_running=True

if args.motor_off:
    lidar.stop_motor()
    prev_motor_running=False

if args.health:
    print(lidar.get_health())

if args.reset:
    print(lidar.reset())

if args.range:
    for i, scan in enumerate(lidar.iter_scans()):
        if i > 0:
            break
        print('%d: Got %d measurments' % (i, len(scan)))
        print('Scan',scan)




if args.speed:
    old_t = None
    data = []
    try:
        print('Press Ctrl+C to stop')
        for i, scan in enumerate(lidar.iter_scans(scan_type='express')):
            now = time.time()
            if old_t is None:
                old_t = now
                continue
            delta = now - old_t
            print('Points-per-sec: %.2f Rate: %.2f Hz, Scan len %d' % (len(scan)/delta, 1/delta, len(scan)))
            data.append(delta)
            old_t = now
    except KeyboardInterrupt:
        print('Stopping. Computing mean...')
        # lidar.stop()
        # lidar.disconnect()
        delta = sum(data)/len(data)
        print('Mean: %.2f Hz, %.2f RPM' % (1/delta, 60/delta))

if not args.motor_on: #Turn off motor by default
    lidar.stop_motor()
