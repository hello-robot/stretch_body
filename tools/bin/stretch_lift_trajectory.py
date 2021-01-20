#!/usr/bin/env python

import stretch_body.scope
import stretch_body.lift
from stretch_body.hello_utils import *

import sys
import time
import argparse

parser=argparse.ArgumentParser(description='Test out trajectories on the lift joint from a GUI.')
parser.add_argument("--text", '-t', help="Use text options instead of GUI", action="store_true")
parser.add_argument("--preloaded_traj", '-p', help="Load one of three predefined trajectories", choices=['1', '2', '3'], default='1')
args, _ = parser.parse_known_args()

if get_display() is None:
    print('Display server not available. Using text options.')
    args.text = True

l = stretch_body.lift.Lift()
l.startup()
l.pull_status()
l_yrange = tuple(l.params['range_m'])
l_vrange = (-1 * l.params['motion']['trajectory_max']['vel_m'],
	        l.params['motion']['trajectory_max']['vel_m'])

if args.preloaded_traj == '1':
    vtime = [0.0, 15.0, 30.0]
    vpos = [l.status['pos'], 0.9, 0.2]
    vvel = [l.status['vel'], 0.0, 0.0]
elif args.preloaded_traj == '2':
    vtime = [0.0, 3.0, 6.0, 9.0]
    vpos = [l.status['pos'], 0.3, 0.4, 0.2]
    vvel = [l.status['vel'], 0.0,  0.0, 0.0]
elif args.preloaded_traj == '3':
    vtime = [0.0, 30.0, 60.0]
    vpos = [l.status['pos'], 0.9, 0.2]
    vvel = [l.status['vel'], 0.0, -0.08]

if not args.text:
    def start_trajectory(times, positions, velocities):
        for waypoint in zip(times, positions, velocities):
            l.trajectory.add_waypoint(t_s=waypoint[0], x_m=waypoint[1], v_m=waypoint[2])
        l.start_trajectory(threaded=False)

    def sense_trajectory():
        l.pull_status()
        if l.status['motor']['trajectory_active']:
            l.push_trajectory()
            if l.traj_start_time is not None and l.traj_curr_time is not None:
                return (l.traj_curr_time - l.traj_start_time, l.status['pos'])

    def update_trajectory(times, positions, velocities):
        l.trajectory.clear_waypoints()
        for waypoint in zip(times, positions, velocities):
            l.trajectory.add_waypoint(t_s=waypoint[0], x_m=waypoint[1], v_m=waypoint[2])

    def stop_trajectory():
        l.stop_trajectory()
        l.trajectory.clear_waypoints()
        time.sleep(0.25)
        l.pull_status()
        return l.status['pos']

    s = stretch_body.scope.TrajectoryScope(vtime, vpos, vvel,
            yrange=l_yrange, vrange=l_vrange, sense_frequency=100,
            title="Lift Trajectory",
            ylabel="Lift Joint Range (m)")
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
                print("\nExecuting trajectory: {0}\n".format(l.trajectory))
                l.start_trajectory()
            if x[0]=='s':
                l.stop_trajectory()
            if x[0]=='d':
                l.pull_status()
                if l.status['motor']['trajectory_active'] and l.traj_start_time is not None and l.traj_curr_time is not None:
                    t = l.traj_curr_time - l.traj_start_time
                    err = l.trajectory.evaluate_at(t).position - l.status['pos']
                    print("\nDuration Remaining: {0}\nTracking Error: {1}\n".format(l.duration_remaining(), err))
                else:
                    print("\nTrajectory inactive\n")
            if x[0]=='q':
                l.stop_trajectory()
                l.stop()
                exit()
            if x[0] == '1':
                l.pull_status()
                if l.status['motor']['trajectory_active']:
                    print("\nCannot change trajectory while active\n")
                else:
                    vtime = [0.0, 15.0, 30.0]
                    vpos = [l.status['pos'], 0.9, 0.2]
                    vvel = [l.status['vel'], 0.0, 0.0]
                    l.trajectory.clear_waypoints()
                    for waypoint in zip(vtime, vpos, vvel):
                        l.trajectory.add_waypoint(t_s=waypoint[0], x_m=waypoint[1], v_m=waypoint[2])
                    print("\nLoading trajectory: {0}\n".format(l.trajectory))
            if x[0]=='2':
                l.pull_status()
                if l.status['motor']['trajectory_active']:
                    print("\nCannot change trajectory while active\n")
                else:
                    vtime = [0.0, 3.0, 6.0, 9.0]
                    vpos = [l.status['pos'], 0.3, 0.4, 0.2]
                    vvel = [l.status['vel'], 0.0,  0.0, 0.0]
                    l.trajectory.clear_waypoints()
                    for waypoint in zip(vtime, vpos, vvel):
                        l.trajectory.add_waypoint(t_s=waypoint[0], x_m=waypoint[1], v_m=waypoint[2])
                    print("\nLoading trajectory: {0}\n".format(l.trajectory))
            if x[0] == '3':
                l.pull_status()
                if l.status['motor']['trajectory_active']:
                    print("\nCannot change trajectory while active\n")
                else:
                    vtime = [0.0, 30.0, 60.0]
                    vpos = [l.status['pos'], 0.9, 0.2]
                    vvel = [l.status['vel'], 0.0, -0.08]
                    l.trajectory.clear_waypoints()
                    for waypoint in zip(vtime, vpos, vvel):
                        l.trajectory.add_waypoint(t_s=waypoint[0], x_m=waypoint[1], v_m=waypoint[2])
                    print("\nLoading trajectory: {0}\n".format(l.trajectory))
        else:
            l.pull_status()
            if l.status['motor']['trajectory_active'] and l.traj_start_time is not None and l.traj_curr_time is not None:
                t = l.traj_curr_time - l.traj_start_time
                err = l.trajectory.evaluate_at(t).position - l.status['pos']
                print("\nDuration Remaining: {0}\nTracking Error: {1}\n".format(l.duration_remaining(), err))
            else:
                print("\nTrajectory inactive\n")

    try:
        for waypoint in zip(vtime, vpos, vvel):
            l.trajectory.add_waypoint(t_s=waypoint[0], x_m=waypoint[1], v_m=waypoint[2])
        while True:
            try:
                step_interaction()
            except (ValueError):
                print('Bad input...')
    except (ThreadServiceExit, KeyboardInterrupt):
        l.stop_trajectory()
        l.stop()
