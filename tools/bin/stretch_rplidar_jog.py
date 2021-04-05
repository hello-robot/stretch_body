#!/usr/bin/env python
from __future__ import print_function
from rplidar import *
import pickle
import argparse
import stretch_body.hello_utils as hu
hu.print_stretch_re_use()

parser=argparse.ArgumentParser(description='Tool to control the RP-Lidar')
parser.add_argument("--motor_on", help="Turn motor on",action="store_true")
parser.add_argument("--motor_off", help="Turn motor off",action="store_true")
parser.add_argument("--info", help="Device info",action="store_true")
parser.add_argument("--health", help="Get device health",action="store_true")
parser.add_argument("--reset", help="Reset device",action="store_true")
parser.add_argument("--read_measurements", nargs="?", type=str, dest="read_measurements", const="",
                    help="Print range reading and save as pickle to given filepath")
args, _ = parser.parse_known_args()

try:
    lidar = RPLidar('/dev/hello-lrf')
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
    health = lidar.get_health()
    if health[0] == 'Good':
        print("Lidar's health is Good")
    else:
        print("Lidar's health is {0}, with error code {1}".format(*health))

if args.reset:
    print(lidar.reset())

if args.read_measurements is not None:
    scans = []
    print('Ctrl-C to stop reading scans')
    try:
        for scan in lidar.iter_scans():
            scans.append(scan)
            print("Got {0} measurements in scan".format(len(scan)))
    except:
        scans.pop(0)
        try:
            with open(args.read_measurements, 'wb') as file:
                print('Saving {0} scans to {1}'.format(len(scans), args.read_measurements))
                pickle.dump(scans, file, protocol=pickle.HIGHEST_PROTOCOL)
        except:
            print("Unable to save scans to file '{0}'.".format(args.read_measurements))

if not args.motor_on: #Turn off motor by default
    lidar.stop_motor()
