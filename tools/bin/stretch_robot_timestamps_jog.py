#!/usr/bin/env python
from __future__ import print_function
import stretch_body.robot
import stretch_body.scope
import random
from stretch_body.hello_utils import *

import argparse
import stretch_body.hello_utils as hu
hu.print_stretch_re_use()

parser=argparse.ArgumentParser(description='Tool to check the Stretch timestamps')
parser.add_argument("--sensor_stats", help="Histogram sensor timestamps relative to the HW sync",action="store_true")
parser.add_argument("--sensor_delta", help="Display timestamp deltas from the HW sync",action="store_true")
parser.add_argument("--display", help="Print timestamps to screen",action="store_true")
args=parser.parse_args()

r=stretch_body.robot.Robot()
r.startup()

if args.sensor_delta:
  try:
      while True:
          s=r.get_status()
          hw_sync=s['timestamps']['hw_sync']
          p0 = (s['timestamps']['pimu_imu']-hw_sync).to_usecs()
          t0=  (s['timestamps']['left_wheel_enc']-hw_sync).to_usecs()
          t1 = (s['timestamps']['right_wheel_enc'] - hw_sync).to_usecs()
          t2 = (s['timestamps']['lift_enc'] - hw_sync).to_usecs()
          t3 = (s['timestamps']['arm_enc'] - hw_sync).to_usecs()
          w0 = (s['timestamps']['wacc_acc'] - hw_sync).to_usecs()
          print('---------------------------')
          print('DT Pimu IMU            :'+str(p0))
          print('DT Left Wheel Encoder  :' + str(t0))
          print('DT Right Wheel Encoder :' + str(t1))
          print('DT Lift Encoder        :' + str(t2))
          print('DT Arm Encoder         :' + str(t3))
          print('DT Wacc Accel          :' + str(w0))
          time.sleep(0.25)
  except (ThreadServiceExit):
    r.stop()

if args.display:
    try:
        while True:
            r.timestamp_manager.pretty_print()
            time.sleep(0.25)
    except (KeyboardInterrupt, SystemExit, ThreadServiceExit):
        r.stop()


if args.sensor_stats:
  print('Starting sensor timestamp analysis...')
  print('Sync mode enabled: '+str(r.timestamp_manager.param['sync_mode_enabled']))
  print('Time align status: ' + str(r.timestamp_manager.param['time_align_status']))
  print('Use skew compensation: ' + str(r.pimu.clock_manager.params['use_skew_compensation']))
  print('---------------------------')
  ts_hist=[[],[],[],[],[], []]
  for i in range(100):
      print('I '+str(i)+' of 100')
      s=r.get_status()
      hw_sync=s['timestamps']['hw_sync']
      p0 = (s['timestamps']['pimu_imu']-hw_sync).to_usecs()
      t0=  (s['timestamps']['left_wheel_enc']-hw_sync).to_usecs()
      t1 = (s['timestamps']['right_wheel_enc'] - hw_sync).to_usecs()
      t2 = (s['timestamps']['lift_enc'] - hw_sync).to_usecs()
      t3 = (s['timestamps']['arm_enc'] - hw_sync).to_usecs()
      w0 = (s['timestamps']['wacc_acc'] - hw_sync).to_usecs()

      ts_hist[0].append(t0)
      ts_hist[1].append(t1)
      ts_hist[2].append(t2)
      ts_hist[3].append(t3)
      ts_hist[4].append(p0)
      ts_hist[5].append(w0)
      print(r.status['timestamps'])
      time.sleep(random.random()*0.1) #Randomize timing so get non biased distribution
  r.stop()
  import matplotlib.pyplot as plt
  import numpy as np
  fig, axs = plt.subplots(1, 6, sharey=True, tight_layout=True)
  fig.suptitle('Distribution of sensor timestamps from sync line event')

  axs[0].hist(x=ts_hist[0], bins='auto', color='#0504aa', alpha=0.7, rwidth=0.85)
  axs[1].hist(x=ts_hist[1], bins='auto', color='#0504aa', alpha=0.7, rwidth=0.85)
  axs[2].hist(x=ts_hist[2], bins='auto', color='#0504aa', alpha=0.7, rwidth=0.85)
  axs[3].hist(x=ts_hist[3], bins='auto', color='#0504aa', alpha=0.7, rwidth=0.85)
  axs[4].hist(x=ts_hist[4], bins='auto', color='#0504aa', alpha=0.7, rwidth=0.85)
  axs[5].hist(x=ts_hist[5], bins='auto', color='#0504aa', alpha=0.7, rwidth=0.85)

  axs[0].set_title('Left Wheel Encoder')
  axs[1].set_title('Right Wheel Encoder')
  axs[2].set_title('Lift Wheel Encoder')
  axs[3].set_title('Arm Encoder')
  axs[4].set_title('Pimu IMU ')
  axs[5].set_title('Wacc Acc')
  plt.show()

