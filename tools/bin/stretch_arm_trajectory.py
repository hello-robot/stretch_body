#!/usr/bin/env python

import stretch_body.scope
import stretch_body.arm
from stretch_body.hello_utils import *

import sys
import time
import argparse

parser=argparse.ArgumentParser(description='Test out trajectories on the arm joint from a GUI.')
parser.add_argument("--text", '-t', help="Use text options instead of GUI", action="store_true")
parser.add_argument("--preloaded_traj", '-p', help="Load one of three predefined trajectories", choices=['1', '2', '3'], default='1')
args, _ = parser.parse_known_args()

if get_display() is None:
    print('Display server not available. Using text options.')
    args.text = True

a = stretch_body.arm.Arm()
a.startup()
a.pull_status()
a_yrange = tuple(a.params['range_m'])
a_vrange = (-1 * a.params['motion']['trajectory_max']['vel_m'],
	        a.params['motion']['trajectory_max']['vel_m'])

if args.preloaded_traj == '1':
    vtime = [0.0, 10.0, 20.0]
    vpos = [a.status['pos'], 0.5, 0.0]
    vvel = [a.status['vel'], 0.0, 0.0]
elif args.preloaded_traj == '2':
    vtime = [0.0, 3.0, 6.0, 9.0]
    vpos = [a.status['pos'], 0.10, 0.3, 0.15]
    vvel = [a.status['vel'], 0.0,  0.0, 0.0]
elif args.preloaded_traj == '3':
    vtime = [0.0, 30.0, 60.0]
    vpos = [a.status['pos'], 0.5, 0.0]
    vvel = [a.status['vel'], 0.0, -0.06]

if not args.text:
    def start_trajectory(times, positions, velocities):
        for waypoint in zip(times, positions, velocities):
            a.trajectory.add_waypoint(t_s=waypoint[0], x_m=waypoint[1], v_m=waypoint[2])
        a.start_trajectory(threaded=False)

    def sense_trajectory():
        a.pull_status()
        if a.status['motor']['trajectory_active']:
            a.push_trajectory()
            if a.traj_start_time is not None and a.traj_curr_time is not None:
                return (a.traj_curr_time - a.traj_start_time, a.status['pos'])

    def update_trajectory(times, positions, velocities):
        a.trajectory.clear_waypoints()
        for waypoint in zip(times, positions, velocities):
            a.trajectory.add_waypoint(t_s=waypoint[0], x_m=waypoint[1], v_m=waypoint[2])

    def stop_trajectory():
        a.stop_trajectory()
        a.trajectory.clear_waypoints()
        time.sleep(0.25)
        a.pull_status()
        return a.status['pos']

    s = stretch_body.scope.TrajectoryScope(vtime, vpos, vvel,
            yrange=a_yrange, vrange=a_vrange, sense_frequency=100,
            title="Arm Trajectory",
            ylabel="Arm Joint Range (m)")
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
                print("\nExecuting trajectory: {0}\n".format(w.trajectory))
                w.start_trajectory(position_follow_mode=not args.velocity_ctrl)
            if x[0]=='s':
                w.stop_trajectory()
            if x[0]=='d':
                w.pull_status()
                if w.status['trajectory_active']:
                    print("\nDuration Remaining: {0}\nTracking Error: {1}\n".format(w.duration_remaining(), w.traj_curr_goal.position - w.status['pos']))
                else:
                    print("\nTrajectory inactive\n")
            if x[0]=='q':
                w.stop_trajectory()
                w.stop()
                exit()
            if x[0] == '1':
                w.pull_status()
                w.traj_start_time = time.time() - 0.07
                vtime = [0.0, 5.0, 10.0]
                vpos = [w.status['pos'], deg_to_rad(-45), deg_to_rad(90.0)]
                vvel = [w.status['vel'], deg_to_rad(0.0), deg_to_rad(10.0)]
                w.trajectory.clear_waypoints()
                for waypoint in zip(vtime, vpos, vvel):
                    w.trajectory.add_waypoint(t_s=waypoint[0], x_r=waypoint[1], v_r=waypoint[2])
                print("\nLoading trajectory: {0}\n".format(w.trajectory))
            if x[0]=='2':
                w.pull_status()
                w.traj_start_time = time.time() - 0.07
                vtime = [0.0, 3.0, 6.0, 9.0]
                vpos = [w.status['pos'], deg_to_rad(40.0), deg_to_rad(-40.0), deg_to_rad(90.0)]
                vvel = [w.status['vel'], deg_to_rad(90.0), deg_to_rad(-90.0), deg_to_rad(0.0)]
                w.trajectory.clear_waypoints()
                for waypoint in zip(vtime, vpos, vvel):
                    w.trajectory.add_waypoint(t_s=waypoint[0], x_r=waypoint[1], v_r=waypoint[2])
                print("\nLoading trajectory: {0}\n".format(w.trajectory))
            if x[0] == '3':
                w.pull_status()
                w.traj_start_time = time.time() - 0.07
                vtime = [0.0, 30.0, 60.0]
                vpos = [w.status['pos'], deg_to_rad(0.0), deg_to_rad(90.0)]
                vvel = [w.status['vel'], deg_to_rad(0.0), deg_to_rad(10.0)]
                w.trajectory.clear_waypoints()
                for waypoint in zip(vtime, vpos, vvel):
                    w.trajectory.add_waypoint(t_s=waypoint[0], x_r=waypoint[1], v_r=waypoint[2])
                print("\nLoading trajectory: {0}\n".format(w.trajectory))
        else:
            w.pull_status()
            if w.status['trajectory_active']:
                print("\nDuration Remaining: {0}\nTracking Error: {1}\n".format(w.duration_remaining(), w.traj_curr_goal.position - w.status['pos']))
            else:
                print("\nTrajectory inactive\n")

    try:
        for waypoint in zip(vtime, vpos, vvel):
            w.trajectory.add_waypoint(t_s=waypoint[0], x_r=waypoint[1], v_r=waypoint[2])
        while True:
            try:
                step_interaction()
            except (ValueError):
                print('Bad input...')
    except (ThreadServiceExit, KeyboardInterrupt):
        w.stop_trajectory()
        w.stop()
