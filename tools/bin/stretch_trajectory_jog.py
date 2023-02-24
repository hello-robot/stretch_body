#!/usr/bin/env python3
from __future__ import print_function
import stretch_body.scope
from stretch_body.hello_utils import *

import sys
import time
import argparse

parser=argparse.ArgumentParser(description='Test out splined trajectories on the various joint from a GUI or text menu.')
parser.add_argument("--text", '-t', help="Use text options instead of GUI", action="store_true")
parser.add_argument("--preloaded_traj", '-p', help="Load one of three predefined trajectories", choices=['1', '2', '3'], default='1')

group = parser.add_mutually_exclusive_group(required=True)
group.add_argument("--head_pan", help="Test trajectories on the head_pan joint", action="store_true")
group.add_argument("--head_tilt", help="Test trajectories on the head_tilt joint", action="store_true")
group.add_argument("--wrist_yaw", help="Test trajectories on the wrist_yaw joint", action="store_true")
group.add_argument("--gripper", help="Test trajectories on the stretch_gripper joint", action="store_true")
group.add_argument("--arm", help="Test trajectories on the arm joint", action="store_true")
group.add_argument("--lift", help="Test trajectories on the lift joint", action="store_true")
group.add_argument("--base_translate", help="Test translational trajectories on diff-drive base", action="store_true")
group.add_argument("--base_rotate", help="Test rotational trajectories on diff-drive base", action="store_true")
group.add_argument("--full_body", help="Test trajectories on all joints at once", action="store_true")
args, _ = parser.parse_known_args()

if get_display() is None:
    print('Display server not available. Using text options.')
    args.text = True

def add_waypoints(times,positions,velocities):
    for waypoint in zip(times, positions, velocities):
        j.trajectory.add(waypoint[0], waypoint[1], waypoint[2])

# ################### WRIST YAW ###################################
if args.wrist_yaw:
    import stretch_body.wrist_yaw
    j = stretch_body.wrist_yaw.WristYaw()
    add_waypoint_cb = add_waypoints
    j.startup(threaded=True)
    j_yrange = (j.ticks_to_world_rad(j.params['range_t'][0]),
                j.ticks_to_world_rad(j.params['range_t'][1]))
    #j_vrange = (-3.0, 3.0)
    j_vrange = (-1 * j.params['motion']['trajectory_max']['vel_r'], j.params['motion']['trajectory_max']['vel_r'])
    key_pos='pos'
    j_title='Wrist Yaw Trajectory'
    j_label="Wrist Yaw Joint Range (rad)"
    j_req_calibration=True
    if args.preloaded_traj == '1':
        vtime = [0.0, 5.0, 10.0]
        vpos = [j.status['pos'], deg_to_rad(-45), deg_to_rad(90.0)]
        vvel = [j.status['vel'], deg_to_rad(0.0), deg_to_rad(10.0)]
    elif args.preloaded_traj == '2':
        vtime = [0.0, 5.0, 10.0, 15.0]
        vpos = [j.status['pos'], deg_to_rad(40.0), deg_to_rad(-30.0), deg_to_rad(90.0)]
        vvel = [j.status['vel'], deg_to_rad(90.0), deg_to_rad(-40.0), deg_to_rad(0.0)]
    elif args.preloaded_traj == '3':
        vtime = [0.0, 30.0, 60.0]
        vpos = [j.status['pos'], deg_to_rad(0.0), deg_to_rad(90.0)]
        vvel = [j.status['vel'], deg_to_rad(0.0), deg_to_rad(10.0)]

# ################### HEAD PAN ###################################
if args.head_pan:
    import stretch_body.dynamixel_hello_XL430
    j = stretch_body.dynamixel_hello_XL430.DynamixelHelloXL430('head_pan')
    add_waypoint_cb = add_waypoints
    j.startup(threaded=True)
    j_yrange = (j.ticks_to_world_rad(j.params['range_t'][0]),
                j.ticks_to_world_rad(j.params['range_t'][1]))
    #j_vrange = (-3.0, 3.0)
    j_vrange = (-1 * j.params['motion']['trajectory_max']['vel_r'], j.params['motion']['trajectory_max']['vel_r'])
    key_pos='pos'
    j_title='Head Pan Trajectory'
    j_label="Head Pan Joint Range (rad)"
    j_req_calibration = False

    if args.preloaded_traj == '1':
        vtime = [0.0, 4.0, 8.0, 10.0]
        vpos = [j.status['pos'], deg_to_rad(-180.0), deg_to_rad(60.0), deg_to_rad(0.0)]
        vvel = [j.status['vel'], deg_to_rad(0.0), deg_to_rad(0.0), deg_to_rad(0.0)]
    elif args.preloaded_traj == '2':
        vtime = [0.0, 3.0, 6.0]
        vpos = [j.status['pos'], deg_to_rad(-100.0), deg_to_rad(0.0)]
        vvel = [j.status['vel'], deg_to_rad(0.0), deg_to_rad(0.0)]
    elif args.preloaded_traj == '3':
        vtime = [0.0, 30.0, 60.0]
        vpos = [j.status['pos'], deg_to_rad(-180.0), deg_to_rad(0.0)]
        vvel = [j.status['vel'], deg_to_rad(0.0), deg_to_rad(10.0)]
# ################### GRIPPER ###################################
if args.gripper:
    import stretch_body.stretch_gripper
    j = stretch_body.stretch_gripper.StretchGripper()
    add_waypoint_cb = add_waypoints
    j.startup(threaded=True)
    j_yrange = (j.pct_to_world_rad(j.poses['close']),j.pct_to_world_rad(j.poses['open']))
    #j_vrange = (-3.0, 3.0)
    j_vrange = (-1 * j.params['motion']['trajectory_max']['vel_r'], j.params['motion']['trajectory_max']['vel_r'])
    key_pos='pos'
    j_title='Stretch Gripper Trajectory'
    j_label="Stretch Gripper Joint Range (Rad)"
    j_req_calibration = True

    if args.preloaded_traj == '1':
        vtime = [0.0, 4.0, 8.0, 10.0]
        vpos = [j.status['pos'], j.pct_to_world_rad(-80), j.pct_to_world_rad(40), j.pct_to_world_rad(0)]
        vvel = [j.status['vel'], deg_to_rad(0.0), deg_to_rad(0.0), deg_to_rad(0.0)]
    elif args.preloaded_traj == '2':
        vtime = [0.0, 3.0, 6.0]
        vpos = [j.status['pos'], j.pct_to_world_rad(50.0), j.pct_to_world_rad(-30)]
        vvel = [j.status['vel'], deg_to_rad(0.0), deg_to_rad(0.0), deg_to_rad(0.0)]
    elif args.preloaded_traj == '3':
        vtime = [0.0, 30.0, 60.0]
        vpos = [j.status['pos'], j.pct_to_world_rad(30.0), j.pct_to_world_rad(30)]
        vvel = [j.status['vel'], deg_to_rad(0.0), deg_to_rad(10.0)]

# ################### HEAD TILT ###################################
if args.head_tilt:
    import stretch_body.dynamixel_hello_XL430
    j = stretch_body.dynamixel_hello_XL430.DynamixelHelloXL430('head_tilt')
    add_waypoint_cb = add_waypoints
    j.startup(threaded=True)
    j_yrange = (j.ticks_to_world_rad(j.params['range_t'][0]),
                j.ticks_to_world_rad(j.params['range_t'][1]))
    j_vrange = (-1 * j.params['motion']['trajectory_max']['vel_r'], j.params['motion']['trajectory_max']['vel_r'])
    #j_vrange = (-3.0, 3.0)
    key_pos='pos'
    j_title='Head Tilt Trajectory'
    j_label="Head Tilt Joint Range (rad)"
    j_req_calibration = False

    if args.preloaded_traj == '1':
        vtime = [0.0, 3.0, 7.0, 10.0]
        vpos = [j.status['pos'], deg_to_rad(-90.0), deg_to_rad(20.0), deg_to_rad(0.0)]
        vvel = [j.status['vel'], deg_to_rad(0.0), deg_to_rad(0.0), deg_to_rad(0.0)]
    elif args.preloaded_traj == '2':
        vtime = [0.0, 3.0, 6.0]
        vpos = [j.status['pos'], deg_to_rad(-50.0), deg_to_rad(0.0)]
        vvel = [j.status['vel'], deg_to_rad(0.0), deg_to_rad(0.0)]
    elif args.preloaded_traj == '3':
        vtime = [0.0, 30.0, 60.0]
        vpos = [j.status['pos'], deg_to_rad(-90.0), deg_to_rad(0.0)]
        vvel = [j.status['vel'], deg_to_rad(0.0), deg_to_rad(10.0)]

# ################### ARM ###################################
if args.arm:
    import stretch_body.arm
    j = stretch_body.arm.Arm()
    add_waypoint_cb = add_waypoints
    j.startup(threaded=True)
    j.motor.disable_sync_mode()
    #j.motor.disable_guarded_mode()
    j.push_command()
    j_yrange = (j.params['range_m'][0], j.params['range_m'][1])
    j_vrange = (-1 * j.params['motion']['trajectory_max']['vel_m'],j.params['motion']['trajectory_max']['vel_m'])
    key_pos='pos'
    j_title='Stretch Arm Trajectory'
    j_label="Stretch Arm Joint Range (Rad)"
    j_req_calibration = True

    if args.preloaded_traj == '1':
        vtime = [0.0, 10.0, 20.0]
        vpos = [j.status['pos'], 0.45, 0.0]
        vvel = [j.status['vel'], 0.0, 0.0]
    elif args.preloaded_traj == '2':
        vtime = [0.0, 3.0, 6.0, 9.0]
        vpos = [j.status['pos'], 0.10, 0.3, 0.15]
        vvel = [j.status['vel'], 0.0, 0.0, 0.0]
    elif args.preloaded_traj == '3':
        vtime = [0.0, 30.0, 60.0]
        vpos = [j.status['pos'], 0.45, 0.0]
        vvel = [j.status['vel'], 0.0, -0.06]
# ################### Lift ###################################
if args.lift:
    import stretch_body.lift
    j = stretch_body.lift.Lift()
    add_waypoint_cb = add_waypoints
    j.startup(threaded=True)
    j.motor.disable_sync_mode()
    #j.motor.disable_guarded_mode()
    j.push_command()
    j_yrange = (j.params['range_m'][0], j.params['range_m'][1])
    j_vrange = (-1 * j.params['motion']['trajectory_max']['vel_m'],j.params['motion']['trajectory_max']['vel_m'])
    key_pos='pos'
    j_title='Stretch Lift Trajectory'
    j_label="Stretch Lift Joint Range (Rad)"
    j_req_calibration = True

    if args.preloaded_traj == '1':
        vtime = [0.0, 15.0, 30.0]
        vpos = [j.status['pos'], 0.9, 0.2]
        vvel = [j.status['vel'], 0.0, 0.0]
    elif args.preloaded_traj == '2':
        vtime = [0.0, 3.0, 6.0, 9.0]
        vpos = [j.status['pos'], 0.3, 0.4, 0.2]
        vvel = [j.status['vel'], 0.0, 0.0, 0.0]
    elif args.preloaded_traj == '3':
        vtime = [0.0, 30.0, 60.0]
        vpos = [j.status['pos'], 0.9, 0.2]
        vvel = [j.status['vel'], 0.0, -0.08]

# ################### Base ###################################
if args.base_translate or args.base_rotate:
    import stretch_body.base
    j = stretch_body.base.Base()
    j.startup(threaded=True)
    j.right_wheel.disable_sync_mode()
    j.left_wheel.disable_sync_mode()
    j.push_command()
    if args.base_translate:
        j_yrange = (-0.5, 0.5) #meters
        j_vrange = (-1 * j.params['motion']['trajectory_max']['vel_r'],j.params['motion']['trajectory_max']['accel_r'])
        j_title = 'Stretch Base Translate Trajectory'
        j_label = "Stretch Base Translate Range (Rad)"
        def add_translate_waypoints(times, positions, velocities):
            for waypoint in zip(times, positions, velocities):
                j.trajectory.add(time=waypoint[0], x=waypoint[1], y=0, theta=0, translational_vel=waypoint[2],rotational_vel=0)
        add_waypoint_cb = add_translate_waypoints
        key_pos = 'x'
        if args.preloaded_traj == '1':
            vtime = [0.0, 3.0, 6.0]
            vpos = [j.status['x'], 0.1, 0.0]
            vvel=  [j.status['x_vel'], 0.0, 0.0]
        if args.preloaded_traj == '2':
            vtime = [0.0, 3.0, 6.0,9.0,12.0]
            vpos = [j.status['x'], 0.1, 0.15,0.2,0.0]
            vvel=  [j.status['x_vel'], 0.2, -0.1,-0.05,0.0]
        if args.preloaded_traj == '3':
            vtime = [0.0, 10.0, 15.0, 20.0]
            vpos = [j.status['x'], 0.1, 0.4,0.0]
            vvel=  [j.status['x_vel'], 0.2, -0.1,0.0]

    if args.base_rotate:
        j_yrange = (deg_to_rad(-180),deg_to_rad(180))  # radians
        j_vrange = (-1 * j.params['motion']['trajectory_max']['vel_r'], j.params['motion']['trajectory_max']['accel_r'])
        j_title = 'Stretch Base Rotate Trajectory'
        j_label = "Stretch Base Rotate Range (Rad)"


        def add_rotate_waypoints(times, positions, velocities):
            for waypoint in zip(times, positions, velocities):
                j.trajectory.add(time=waypoint[0], x=0, y=0, theta=waypoint[1], translational_vel=0,rotational_vel=waypoint[2])

        add_waypoint_cb = add_rotate_waypoints
        key_pos = 'theta'
        if args.preloaded_traj == '1':
            vtime = [0.0, 3.0, 6.0]
            vpos = [j.status['theta'], deg_to_rad(90.0), 0.0]
            vvel = [j.status['theta_vel'], 0.0, 0.0]
        if args.preloaded_traj == '2':
            vtime = [0.0, 3.0, 6.0, 9.0, 12.0]
            vpos = [j.status['theta'], deg_to_rad(90.0),deg_to_rad(-45.0),deg_to_rad(45.0), 0.0]
            vvel = [j.status['theta_vel'], deg_to_rad(10.0),deg_to_rad(-5.0),deg_to_rad(-10.0), 0.0]
        if args.preloaded_traj == '3':
            vtime = [0.0, 10.0, 15.0, 20.0]
            vpos = [j.status['theta'], deg_to_rad(45.0),deg_to_rad(-10.0), 0.0]
            vvel = [j.status['theta_vel'], deg_to_rad(5.0),deg_to_rad(-5.0), 0.0]


# ################### Full Body ###################################
if args.full_body:
    import stretch_body.robot

    r = stretch_body.robot.Robot()
    r.startup()

    print('move all joints to initial positions')
    r.arm.move_to(0.0)
    r.lift.move_to(0.2)
    r.push_command()
    r.head.pose('ahead')
    r.end_of_arm.motors['wrist_yaw'].pose('side')
    time.sleep(4.0)

    r.lift.trajectory.add(t_s=0.0, x_m=0.2, v_m=0.0)
    r.lift.trajectory.add(t_s=10.0, x_m=0.9, v_m=0.0)
    r.lift.trajectory.add(t_s=20.0, x_m=0.2, v_m=0.0)
    r.arm.trajectory.add(t_s=0.0, x_m=0.0, v_m=0.0)
    r.arm.trajectory.add(t_s=10.0, x_m=0.5, v_m=0.0)
    r.arm.trajectory.add(t_s=20.0, x_m=0.0, v_m=0.0)

    r.base.trajectory.add(time=0.0, x=0,y=0, theta=deg_to_rad(0.0), translational_vel=0.0, rotational_vel=0.0 )
    r.base.trajectory.add(time=10.0, x=0,y=0, theta=deg_to_rad(180.0), translational_vel=0.0, rotational_vel=0.0)
    r.base.trajectory.add(time=20.0,  x=0,y=0,theta=deg_to_rad(0.0), translational_vel=0.0, rotational_vel=0.0)


    r.end_of_arm.motors['wrist_yaw'].trajectory.add(t_s=0.0, x_r=deg_to_rad(90.0), v_r=0.0)
    r.end_of_arm.motors['wrist_yaw'].trajectory.add(t_s=10.0, x_r=deg_to_rad(-45.0), v_r=0.0)
    r.end_of_arm.motors['wrist_yaw'].trajectory.add(t_s=20.0, x_r=deg_to_rad(90.0), v_r=0.0)
    #r.end_of_arm.motors['wrist_yaw'].trajectory.add_waypoint(t_s=40.0, x_r=deg_to_rad(180.0), v_r=0.0)

    r.head.get_joint('head_tilt').trajectory.add(t_s=0.0, x_r=deg_to_rad(0.0), v_r=0.0)
    r.head.get_joint('head_tilt').trajectory.add(t_s=10.0, x_r=deg_to_rad(-90.0), v_r=0.0)
    r.head.get_joint('head_tilt').trajectory.add(t_s=20.0, x_r=deg_to_rad(0.0), v_r=0.0)

    r.head.get_joint('head_pan').trajectory.add(t_s=0.0, x_r=deg_to_rad(0.0), v_r=0.0)
    r.head.get_joint('head_pan').trajectory.add(t_s=10.0, x_r=deg_to_rad(-180.0), v_r=0.0)
    r.head.get_joint('head_pan').trajectory.add(t_s=20.0, x_r=deg_to_rad(0.0), v_r=0.0)

    print('start follow_trajectory')
    print('Remove all cables from base. Ensure robot is clear to make full body motion.')
    print('Hit enter when ready.')
    input()
    r.follow_trajectory()
    ts = time.time()
    while r.is_trajectory_active():
        #dt = (time.time() - ts) / 20.0  # 0-1
        print('Time remaining: %f'%(20.0-(time.time()-ts)))
        # r.base.set_translate_velocity(v_m=dt*0.5) #Uncomment this for base motion (put up on blocks first!)
        #r.push_command()
        time.sleep(0.1)

    r.stop_trajectory()
    r.stop()
    print('done')
    exit(0)

# ##########################################################################

if not args.text:

    def start_trajectory(times, positions, velocities):
        add_waypoint_cb(times,positions,velocities)
        if not args.base_translate and not args.base_rotate:
            j.follow_trajectory(req_calibration=j_req_calibration, move_to_start_point=False)
        else:
            j.follow_trajectory()

    def sense_trajectory():
        #j.pretty_print()
        if j.is_trajectory_active():
            if args.base_rotate:
                if j.status['theta']>deg_to_rad(180.0): #Handle rollover in odometry
                    return (j.get_trajectory_ts(), j.status['theta']-deg_to_rad(360.0))
            return (j.get_trajectory_ts(), j.status[key_pos])

    def update_trajectory(times, positions, velocities):
        j.trajectory.clear()
        for waypoint in zip(times, positions, velocities):
            j.trajectory.add(waypoint[0], waypoint[1], waypoint[2])

    def stop_trajectory():
        j.stop_trajectory()
        j.trajectory.clear()
        time.sleep(0.25)
        return j.status[key_pos]

    s = stretch_body.scope.TrajectoryScope(vtime, vpos, vvel,
            yrange=j_yrange, vrange=j_vrange, sense_frequency=15,
            title=j_title,
            ylabel=j_label)
    s.start(start_trajectory, sense_trajectory, update_trajectory, stop_trajectory)
else:

    def menu_top():
        print('------ MENU -------')
        print('m: menu')
        print('a: start trajectory')
        print('s: stop trajectory')
        print('q: quit')
        print('-------------------')

    def step_interaction():
        menu_top()
        x=sys.stdin.readline()
        if len(x)>1:
            if x[0]=='m':
                pass
            if x[0]=='a':
                add_waypoint_cb(vtime,vpos,vvel)
                if not args.base_translate and not args.base_rotate:
                    j.follow_trajectory(req_calibration=j_req_calibration, move_to_start_point=True)
                else:
                    j.follow_trajectory()
                print("\nExecuting trajectory: {0}\n".format(j.trajectory))
            if x[0]=='s':
                j.stop_trajectory()
            if x[0]=='q':
                j.stop_trajectory()
                j.stop()
                exit()
        else:
            if j.is_trajectory_active():
                print("\nDuration Remaining: {0}.\nCurrent pos {1}\n".format(j.get_trajectory_time_remaining(), key_pos))
            else:
                print("\nTrajectory inactive\n")

    try:
        while True:
            try:
                step_interaction()
            except (ValueError):
                print('Bad input...')
    except (ThreadServiceExit, KeyboardInterrupt):
        j.stop_trajectory()
        j.stop()
