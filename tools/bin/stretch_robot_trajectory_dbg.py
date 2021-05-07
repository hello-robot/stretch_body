#!/usr/bin/env python3
from __future__ import print_function
import stretch_body.scope
import stretch_body.robot
from stretch_body.hello_utils import *

import sys
import time
import argparse

parser=argparse.ArgumentParser(description='Test out trajectories on the arm joint from a GUI.')
parser.add_argument("--text", '-t', help="Use text options instead of GUI", action="store_true")
parser.add_argument("--preloaded_traj", '-p', help="Load one of three predefined trajectories", choices=['1', '2', '3'], default='1')
args, _ = parser.parse_known_args()

#Start robot before get_display() to avoid resource starving of robot by the display during creation
r = stretch_body.robot.Robot()
r.startup()
time.sleep(0.5)

if get_display() is None:
    print('Display server not available. Using text options.')
    args.text = True


a_yrange = tuple(r.arm.params['range_m'])
a_vrange = (-1 * r.arm.params['motion']['trajectory_max']['vel_m'],
	        r.arm.params['motion']['trajectory_max']['vel_m'])


status = r.get_status()
if args.preloaded_traj == '1':
    vtime = [0.0, 10.0, 20.0]
    vpos = [status['arm']['pos'], 0.5, 0.0]
    vvel = [status['arm']['vel'], 0.0, 0.0]
elif args.preloaded_traj == '2':
    vtime = [0.0, 3.0, 6.0, 9.0]
    vpos = [status['arm']['pos'], 0.10, 0.3, 0.15]
    vvel = [status['arm']['vel'], 0.0,  0.0, 0.0]
elif args.preloaded_traj == '3':
    vtime = [0.0, 30.0, 60.0]
    vpos = [status['arm']['pos'], 0.5, 0.0]
    vvel = [status['arm']['vel'], 0.0, -0.06]

if not args.text:
    def start_trajectory(times, positions, velocities):
        for waypoint in zip(times, positions, velocities):
            r.arm.trajectory.add_waypoint(t_s=waypoint[0], x_m=waypoint[1], v_m=waypoint[2])
        r.start_trajectory()

    def sense_trajectory():
        status = r.get_status()
        if status['arm']['motor']['trajectory_active']:
            if r.arm.traj_start_time is not None and r.arm.traj_curr_time is not None:
                return (r.arm.traj_curr_time - r.arm.traj_start_time,status['arm']['pos'])

    def update_trajectory(times, positions, velocities):
        r.arm.trajectory.clear_waypoints()
        for waypoint in zip(times, positions, velocities):
            r.arm.trajectory.add_waypoint(t_s=waypoint[0], x_m=waypoint[1], v_m=waypoint[2])

    def stop_trajectory():
        r.stop_trajectory()
        r.arm.trajectory.clear_waypoints()
        time.sleep(0.25)
        return r.get_status()['arm']['pos']

    s = stretch_body.scope.TrajectoryScope(vtime, vpos, vvel,
            yrange=a_yrange, vrange=a_vrange, sense_frequency=100,
            title="Arm Trajectory",
            ylabel="Arm Joint Range (m)")
    s.start(start_trajectory, sense_trajectory, update_trajectory, stop_trajectory)
    r.stop_trajectory()
    r.stop()

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
                r.arm.trajectory.delete_waypoint(0)
                r.arm.trajectory.add_waypoint(t_s=0.0, x_m=r.arm.status['pos'], v_m=r.arm.status['vel'])
                print("\nExecuting trajectory: {0}\n".format(r.arm.trajectory))
                r.start_trajectory()
            if x[0]=='s':
                r.stop_trajectory()
            if x[0]=='d':
                if r.arm.status['motor']['trajectory_active'] and r.arm.traj_start_time is not None and r.arm.traj_curr_time is not None:
                    t = r.arm.traj_curr_time - r.arm.traj_start_time
                    err = r.arm.trajectory.evaluate_at(t).position - r.arm.status['pos']
                    print("\nDuration Remaining: {0}\nTracking Error: {1}\n".format(r.arm.duration_remaining(), err))
                else:
                    print("\nTrajectory inactive\n")
            if x[0]=='q':
                r.stop_trajectory()
                r.stop()
                exit()
            if x[0] == '1':
                if r.arm.status['motor']['trajectory_active']:
                    print("\nCannot change trajectory while active\n")
                else:
                    vtime = [0.0, 10.0, 20.0]
                    vpos = [r.arm.status['pos'], 0.5, 0.0]
                    vvel = [r.arm.status['vel'], 0.0, 0.0]
                    r.arm.trajectory.clear_waypoints()
                    for waypoint in zip(vtime, vpos, vvel):
                        r.arm.trajectory.add_waypoint(t_s=waypoint[0], x_m=waypoint[1], v_m=waypoint[2])
                    print("\nLoading trajectory: {0}\n".format(r.arm.trajectory))
            if x[0]=='2':
                if r.arm.status['motor']['trajectory_active']:
                    print("\nCannot change trajectory while active\n")
                else:
                    vtime = [0.0, 3.0, 6.0, 9.0]
                    vpos = [r.arm.status['pos'], 0.10, 0.3, 0.15]
                    vvel = [r.arm.status['vel'], 0.0,  0.0, 0.0]
                    r.arm.trajectory.clear_waypoints()
                    for waypoint in zip(vtime, vpos, vvel):
                        r.arm.trajectory.add_waypoint(t_s=waypoint[0], x_m=waypoint[1], v_m=waypoint[2])
                    print("\nLoading trajectory: {0}\n".format(r.arm.trajectory))
            if x[0] == '3':
                if r.arm.status['motor']['trajectory_active']:
                    print("\nCannot change trajectory while active\n")
                else:
                    vtime = [0.0, 30.0, 60.0]
                    vpos = [r.arm.status['pos'], 0.5, 0.0]
                    vvel = [r.arm.status['vel'], 0.0, -0.06]
                    r.arm.trajectory.clear_waypoints()
                    for waypoint in zip(vtime, vpos, vvel):
                        r.arm.trajectory.add_waypoint(t_s=waypoint[0], x_m=waypoint[1], v_m=waypoint[2])
                    print("\nLoading trajectory: {0}\n".format(r.arm.trajectory))
        else:
            if r.arm.status['motor']['trajectory_active'] and r.arm.traj_start_time is not None and r.arm.traj_curr_time is not None:
                t = r.arm.traj_curr_time - r.arm.traj_start_time
                err = r.arm.trajectory.evaluate_at(t).position - r.arm.status['pos']
                print("\nDuration Remaining: {0}\nTracking Error: {1}\n".format(r.arm.duration_remaining(), err))
            else:
                print("\nTrajectory inactive\n")


    try:
        for waypoint in zip(vtime, vpos, vvel):
            r.arm.trajectory.add_waypoint(t_s=waypoint[0], x_m=waypoint[1], v_m=waypoint[2])
        while True:
            try:
                step_interaction()
            except (ValueError):
                print('Bad input...')
    except (ThreadServiceExit, KeyboardInterrupt):
        r.stop_trajectory()
        r.stop()
