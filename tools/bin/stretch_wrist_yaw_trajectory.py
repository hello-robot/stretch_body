#!/usr/bin/env python

import stretch_body.scope
import stretch_body.wrist_yaw
from stretch_body.hello_utils import *

import sys
import time
import argparse

parser=argparse.ArgumentParser(description='Test out trajectories on the wrist_yaw joint from a GUI.')
parser.add_argument("--text", '-t', help="Use text options instead of GUI", action="store_true")
parser.add_argument("--preloaded_traj", '-p', help="Load one of three predefined trajectories", choices=['1', '2', '3'], default='1')
parser.add_argument("--velocity_ctrl", '-v', help="Use velocity control to follow trajectory", action="store_true")
args, _ = parser.parse_known_args()

if get_display() is None:
    print('Display server not available. Using text options.')
    args.text = True

w = stretch_body.wrist_yaw.WristYaw()
w.startup()
w.pull_status()
w_yrange = (w.ticks_to_world_rad(w.params['range_t'][0]),
            w.ticks_to_world_rad(w.params['range_t'][1]))
w_vrange = (-3.0, 3.0)

if args.preloaded_traj == '1':
    vtime = [0.0, 5.0, 10.0]
    vpos = [w.status['pos'], deg_to_rad(-45), deg_to_rad(90.0)]
    vvel = [w.status['vel'], deg_to_rad(0.0), deg_to_rad(10.0)]
elif args.preloaded_traj == '2':
    vtime = [0.0, 3.0, 6.0, 9.0]
    vpos = [w.status['pos'], deg_to_rad(40.0), deg_to_rad(-40.0), deg_to_rad(90.0)]
    vvel = [w.status['vel'], deg_to_rad(90.0), deg_to_rad(-90.0), deg_to_rad(0.0)]
elif args.preloaded_traj == '3':
    vtime = [0.0, 30.0, 60.0]
    vpos = [w.status['pos'], deg_to_rad(0.0), deg_to_rad(90.0)]
    vvel = [w.status['vel'], deg_to_rad(0.0), deg_to_rad(10.0)]

if not args.text:
    def start_trajectory(times, positions, velocities):
        for waypoint in zip(times, positions, velocities):
            w.trajectory.add_waypoint(t_s=waypoint[0], x_r=waypoint[1], v_r=waypoint[2])
        w.start_trajectory(position_ctrl=not args.velocity_ctrl, threaded=False)

    def sense_trajectory():
        w.pull_status()
        if w.status['trajectory_active']:
            w.push_trajectory()
            return (w.traj_curr_time - w.traj_start_time, w.status['pos'])

    def update_trajectory(times, positions, velocities):
        w.trajectory.clear_waypoints()
        for waypoint in zip(times, positions, velocities):
            w.trajectory.add_waypoint(t_s=waypoint[0], x_r=waypoint[1], v_r=waypoint[2])

    def stop_trajectory():
        w.stop_trajectory()
        w.trajectory.clear_waypoints()
        time.sleep(0.25)
        w.pull_status()
        return w.status['pos']

    s = stretch_body.scope.TrajectoryScope(vtime, vpos, vvel,
            yrange=w_yrange, vrange=w_vrange, sense_frequency=100,
            title="Wrist Yaw Trajectory",
            ylabel="Wrist Yaw Joint Range (rad)")
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
                w.trajectory.delete_waypoint(0)
                w.pull_status()
                w.trajectory.add_waypoint(t_s=0.0, x_r=w.status['pos'], v_r=w.status['vel'])
                print("\nExecuting trajectory: {0}\n".format(w.trajectory))
                w.start_trajectory(position_ctrl=not args.velocity_ctrl)
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
