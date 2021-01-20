#!/usr/bin/env python

import stretch_body.scope
import stretch_body.head
from stretch_body.hello_utils import *

import sys
import time
import argparse

parser=argparse.ArgumentParser(description='Test out trajectories on the head joints from a GUI.')
parser.add_argument("--head_pan", '-s', help="Test out head_pan instead of head_tilt in GUI mode", action="store_true")
parser.add_argument("--text", '-t', help="Use text options instead of GUI", action="store_true")
parser.add_argument("--preloaded_traj", '-p', help="Load one of three predefined trajectories", choices=['1', '2', '3'], default='1')
parser.add_argument("--velocity_ctrl", '-v', help="Use velocity control to follow trajectory", action="store_true")
args, _ = parser.parse_known_args()

if get_display() is None:
    print('Display server not available. Using text options.')
    args.text = True

h = stretch_body.head.Head()
h.startup()
h.pull_status()
htilt_yrange = (h.get_joint('head_tilt').ticks_to_world_rad(h.get_joint('head_tilt').params['range_t'][0]),
                h.get_joint('head_tilt').ticks_to_world_rad(h.get_joint('head_tilt').params['range_t'][1]))
htilt_vrange = (-3.0, 3.0)
hpan_yrange = (h.get_joint('head_pan').ticks_to_world_rad(h.get_joint('head_pan').params['range_t'][0]),
               h.get_joint('head_pan').ticks_to_world_rad(h.get_joint('head_pan').params['range_t'][1]))
hpan_vrange = (-3.0, 3.0)

if args.preloaded_traj == '1':
    tilt_vtime = [0.0, 3.0, 7.0, 10.0]
    tilt_vpos = [h.status['head_tilt']['pos'], deg_to_rad(-90.0), deg_to_rad(20.0), deg_to_rad(0.0)]
    tilt_vvel = [h.status['head_tilt']['vel'], deg_to_rad(0.0), deg_to_rad(0.0), deg_to_rad(0.0)]
    pan_vtime = [0.0, 4.0, 8.0, 10.0]
    pan_vpos = [h.status['head_pan']['pos'], deg_to_rad(-180.0), deg_to_rad(60.0), deg_to_rad(0.0)]
    pan_vvel = [h.status['head_pan']['vel'], deg_to_rad(0.0), deg_to_rad(0.0), deg_to_rad(0.0)]
elif args.preloaded_traj == '2':
    tilt_vtime = [0.0, 3.0, 6.0]
    tilt_vpos = [h.status['head_tilt']['pos'], deg_to_rad(-50.0), deg_to_rad(0.0)]
    tilt_vvel = [h.status['head_tilt']['vel'], deg_to_rad(0.0), deg_to_rad(0.0)]
    pan_vtime = [0.0, 3.0, 6.0]
    pan_vpos = [h.status['head_pan']['pos'], deg_to_rad(-100.0), deg_to_rad(0.0)]
    pan_vvel = [h.status['head_pan']['vel'], deg_to_rad(0.0), deg_to_rad(0.0)]
elif args.preloaded_traj == '3':
    tilt_vtime = [0.0, 30.0, 60.0]
    tilt_vpos = [h.status['head_tilt']['pos'], deg_to_rad(-90.0), deg_to_rad(0.0)]
    tilt_vvel = [h.status['head_tilt']['vel'], deg_to_rad(0.0), deg_to_rad(10.0)]
    pan_vtime = [0.0, 30.0, 60.0]
    pan_vpos = [h.status['head_pan']['pos'], deg_to_rad(-180.0), deg_to_rad(0.0)]
    pan_vvel = [h.status['head_pan']['vel'], deg_to_rad(0.0), deg_to_rad(10.0)]

if not args.text:
    def start_trajectory(times, positions, velocities):
        if args.head_pan:
            for waypoint in zip(times, positions, velocities):
                h.get_joint('head_pan').trajectory.add_waypoint(t_s=waypoint[0], x_r=waypoint[1], v_r=waypoint[2])
            h.get_joint('head_pan').start_trajectory(position_ctrl=not args.velocity_ctrl, threaded=False)
        else:
            for waypoint in zip(times, positions, velocities):
                h.get_joint('head_tilt').trajectory.add_waypoint(t_s=waypoint[0], x_r=waypoint[1], v_r=waypoint[2])
            h.get_joint('head_tilt').start_trajectory(position_ctrl=not args.velocity_ctrl, threaded=False)

    def sense_trajectory():
        h.pull_status()
        if args.head_pan:
            if h.status['head_pan']['trajectory_active']:
                h.get_joint('head_pan').push_trajectory()
                return (h.get_joint('head_pan').traj_curr_time - h.get_joint('head_pan').traj_start_time, h.status['head_pan']['pos'])
        else:
            if h.status['head_tilt']['trajectory_active']:
                h.get_joint('head_tilt').push_trajectory()
                return (h.get_joint('head_tilt').traj_curr_time - h.get_joint('head_tilt').traj_start_time, h.status['head_tilt']['pos'])

    def update_trajectory(times, positions, velocities):
        if args.head_pan:
            h.get_joint('head_pan').trajectory.clear_waypoints()
            for waypoint in zip(times, positions, velocities):
                h.get_joint('head_pan').trajectory.add_waypoint(t_s=waypoint[0], x_r=waypoint[1], v_r=waypoint[2])
        else:
            h.get_joint('head_tilt').trajectory.clear_waypoints()
            for waypoint in zip(times, positions, velocities):
                h.get_joint('head_tilt').trajectory.add_waypoint(t_s=waypoint[0], x_r=waypoint[1], v_r=waypoint[2])

    def stop_trajectory():
        if args.head_pan:
            h.get_joint('head_pan').stop_trajectory()
            h.get_joint('head_pan').trajectory.clear_waypoints()
            time.sleep(0.25)
            h.pull_status()
            return h.status['head_pan']['pos']
        else:
            h.get_joint('head_tilt').stop_trajectory()
            h.get_joint('head_tilt').trajectory.clear_waypoints()
            time.sleep(0.25)
            h.pull_status()
            return h.status['head_tilt']['pos']

    if args.head_pan:
        s = stretch_body.scope.TrajectoryScope(pan_vtime, pan_vpos, pan_vvel,
                yrange=hpan_yrange, vrange=hpan_vrange, sense_frequency=100,
                title="Head Pan Trajectory",
                ylabel="Head Pan Joint Range (rad)")
        s.start(start_trajectory, sense_trajectory, update_trajectory, stop_trajectory)
    else:
        s = stretch_body.scope.TrajectoryScope(tilt_vtime, tilt_vpos, tilt_vvel,
                yrange=htilt_yrange, vrange=htilt_vrange, sense_frequency=100,
                title="Head Tilt Trajectory",
                ylabel="Head Tilt Joint Range (rad)")
        s.start(start_trajectory, sense_trajectory, update_trajectory, stop_trajectory)
else:
    def menu_top():
        print('------ MENU -------')
        print('m: menu')
        print('a: start trajectory')
        print('s: stop trajectory')
        print('d: print trajectory status')
        print('q: quit')
        print('1: default trajectory')
        print('2: short trajectory')
        print('3: long trajectory')
        print('-------------------')

    def step_interaction():
        menu_top()
        x=sys.stdin.readline()
        if len(x)>1:
            if x[0]=='m':
                pass
            if x[0]=='a':
                print("\nExecuting trajectory:\nHead_Tilt: {0}\nHead_Pan: {1}\n".format(
                    h.get_joint('head_tilt').trajectory, h.get_joint('head_pan').trajectory))
                h.get_joint('head_tilt').start_trajectory(position_ctrl=not args.velocity_ctrl, watchdog_timeout=0)
                h.get_joint('head_pan').start_trajectory(position_ctrl=not args.velocity_ctrl, watchdog_timeout=0)
            if x[0]=='s':
                h.get_joint('head_tilt').stop_trajectory()
                h.get_joint('head_pan').stop_trajectory()
            if x[0]=='d':
                h.pull_status()
                if h.status['head_tilt']['trajectory_active'] or h.status['head_pan']['trajectory_active']:
                    remaining = max(h.get_joint('head_tilt').duration_remaining(), h.get_joint('head_pan').duration_remaining())
                    tilt_error = h.get_joint('head_tilt').traj_curr_goal.position - h.status['head_tilt']['pos']
                    pan_error = h.get_joint('head_pan').traj_curr_goal.position - h.status['head_pan']['pos']
                    print("\nDuration Remaining: {0}\nHead_Tilt Tracking Error: {1}\nHead_Pan Tracking Error: {1}\n".format(
                        remaining, tilt_error, pan_error))
                else:
                    print("\nTrajectory inactive\n")
            if x[0]=='q':
                h.get_joint('head_tilt').stop_trajectory()
                h.get_joint('head_pan').stop_trajectory()
                h.stop()
                exit()
            if x[0]=='1':
                h.pull_status()
                h.get_joint('head_tilt').traj_start_time = time.time() - 0.07
                h.get_joint('head_pan').traj_start_time = time.time() - 0.07
                tilt_vtime = [0.0, 3.0, 7.0, 10.0]
                tilt_vpos = [h.status['head_tilt']['pos'], deg_to_rad(-90.0), deg_to_rad(20.0), deg_to_rad(0.0)]
                tilt_vvel = [h.status['head_tilt']['vel'], deg_to_rad(0.0), deg_to_rad(0.0), deg_to_rad(0.0)]
                pan_vtime = [0.0, 4.0, 8.0, 10.0]
                pan_vpos = [h.status['head_pan']['pos'], deg_to_rad(-180.0), deg_to_rad(60.0), deg_to_rad(0.0)]
                pan_vvel = [h.status['head_pan']['vel'], deg_to_rad(0.0), deg_to_rad(0.0), deg_to_rad(0.0)]
                h.get_joint('head_tilt').trajectory.clear_waypoints()
                h.get_joint('head_pan').trajectory.clear_waypoints()
                for waypoint in zip(tilt_vtime, tilt_vpos, tilt_vvel):
                    h.get_joint('head_tilt').trajectory.add_waypoint(t_s=waypoint[0], x_r=waypoint[1], v_r=waypoint[2])
                for waypoint in zip(pan_vtime, pan_vpos, pan_vvel):
                    h.get_joint('head_pan').trajectory.add_waypoint(t_s=waypoint[0], x_r=waypoint[1], v_r=waypoint[2])
                print("\nLoading trajectory:\nHead_Tilt: {0}\nHead_Pan: {1}\n".format(h.get_joint('head_tilt').trajectory, h.get_joint('head_pan').trajectory))
            if x[0]=='2':
                h.pull_status()
                h.get_joint('head_tilt').traj_start_time = time.time() - 0.07
                h.get_joint('head_pan').traj_start_time = time.time() - 0.07
                tilt_vtime = [0.0, 3.0, 6.0]
                tilt_vpos = [h.status['head_tilt']['pos'], deg_to_rad(-50.0), deg_to_rad(0.0)]
                tilt_vvel = [h.status['head_tilt']['vel'], deg_to_rad(0.0), deg_to_rad(0.0)]
                pan_vtime = [0.0, 3.0, 6.0]
                pan_vpos = [h.status['head_pan']['pos'], deg_to_rad(-100.0), deg_to_rad(0.0)]
                pan_vvel = [h.status['head_pan']['vel'], deg_to_rad(0.0), deg_to_rad(0.0)]
                h.get_joint('head_tilt').trajectory.clear_waypoints()
                h.get_joint('head_pan').trajectory.clear_waypoints()
                for waypoint in zip(tilt_vtime, tilt_vpos, tilt_vvel):
                    h.get_joint('head_tilt').trajectory.add_waypoint(t_s=waypoint[0], x_r=waypoint[1], v_r=waypoint[2])
                for waypoint in zip(pan_vtime, pan_vpos, pan_vvel):
                    h.get_joint('head_pan').trajectory.add_waypoint(t_s=waypoint[0], x_r=waypoint[1], v_r=waypoint[2])
                print("\nLoading trajectory:\nHead_Tilt: {0}\nHead_Pan: {1}\n".format(h.get_joint('head_tilt').trajectory, h.get_joint('head_pan').trajectory))
            if x[0]=='3':
                h.pull_status()
                h.get_joint('head_tilt').traj_start_time = time.time() - 0.07
                h.get_joint('head_pan').traj_start_time = time.time() - 0.07
                tilt_vtime = [0.0, 30.0, 60.0]
                tilt_vpos = [h.status['head_tilt']['pos'], deg_to_rad(-90.0), deg_to_rad(0.0)]
                tilt_vvel = [h.status['head_tilt']['vel'], deg_to_rad(0.0), deg_to_rad(10.0)]
                pan_vtime = [0.0, 30.0, 60.0]
                pan_vpos = [h.status['head_pan']['pos'], deg_to_rad(-180.0), deg_to_rad(0.0)]
                pan_vvel = [h.status['head_pan']['vel'], deg_to_rad(0.0), deg_to_rad(10.0)]
                h.get_joint('head_tilt').trajectory.clear_waypoints()
                h.get_joint('head_pan').trajectory.clear_waypoints()
                for waypoint in zip(tilt_vtime, tilt_vpos, tilt_vvel):
                    h.get_joint('head_tilt').trajectory.add_waypoint(t_s=waypoint[0], x_r=waypoint[1], v_r=waypoint[2])
                for waypoint in zip(pan_vtime, pan_vpos, pan_vvel):
                    h.get_joint('head_pan').trajectory.add_waypoint(t_s=waypoint[0], x_r=waypoint[1], v_r=waypoint[2])
                print("\nLoading trajectory:\nHead_Tilt: {0}\nHead_Pan: {1}\n".format(h.get_joint('head_tilt').trajectory, h.get_joint('head_pan').trajectory))
        else:
            h.pull_status()
            if h.status['head_tilt']['trajectory_active'] or h.status['head_pan']['trajectory_active']:
                remaining = max(h.get_joint('head_tilt').duration_remaining(), h.get_joint('head_pan').duration_remaining())
                tilt_error = h.get_joint('head_tilt').traj_curr_goal.position - h.status['head_tilt']['pos']
                pan_error = h.get_joint('head_pan').traj_curr_goal.position - h.status['head_pan']['pos']
                print("\nDuration Remaining: {0}\nHead_Tilt Tracking Error: {1}\nHead_Pan Tracking Error: {1}\n".format(
                    remaining, tilt_error, pan_error))
            else:
                print("\nTrajectory inactive\n")

    try:
        for waypoint in zip(tilt_vtime, tilt_vpos, tilt_vvel):
            h.get_joint('head_tilt').trajectory.add_waypoint(t_s=waypoint[0], x_r=waypoint[1], v_r=waypoint[2])
        for waypoint in zip(pan_vtime, pan_vpos, pan_vvel):
            h.get_joint('head_pan').trajectory.add_waypoint(t_s=waypoint[0], x_r=waypoint[1], v_r=waypoint[2])
        while True:
            try:
                step_interaction()
            except (ValueError):
                print('Bad input...')
    except (ThreadServiceExit, KeyboardInterrupt):
        h.get_joint('head_tilt').stop_trajectory()
        h.get_joint('head_pan').stop_trajectory()
        h.stop()
