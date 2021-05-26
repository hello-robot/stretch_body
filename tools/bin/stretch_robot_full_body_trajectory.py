#!/usr/bin/env python3

from stretch_body.hello_utils import *
import stretch_body.robot
import time

r = stretch_body.robot.Robot()
r.startup()

print('move all joints to initial positions')
r.arm.move_to(0.0)
r.lift.move_to(0.2)
r.push_command()
r.head.home()
r.end_of_arm.motors['wrist_yaw'].pose('side')
time.sleep(4.0)

r.lift.trajectory.add_waypoint(t_s=0.0, x_m=0.2, v_m=0.0)
r.lift.trajectory.add_waypoint(t_s=10.0, x_m=0.9, v_m=0.0)
r.lift.trajectory.add_waypoint(t_s=20.0, x_m=0.2, v_m=0.0)


r.arm.trajectory.add_waypoint(t_s=0.0, x_m=0.0, v_m=0.0)
r.arm.trajectory.add_waypoint(t_s=10.0, x_m=0.5, v_m=0.0)
r.arm.trajectory.add_waypoint(t_s=20.0, x_m=0.0, v_m=0.0)

#r.base.trajectory.add_rotate_waypoint(t_s=0.0, x_r=deg_to_rad(0.0), v_r=0.0)
#r.base.trajectory.add_rotate_waypoint(t_s=10.0, x_r=deg_to_rad(180.0), v_r=0.0)
#r.base.trajectory.add_rotate_waypoint(t_s=20.0, x_r=deg_to_rad(0.0), v_r=0.0)
#r.base.trajectory.add_rotate_waypoint(t_s=30.0, x_r=deg_to_rad(-90.0), v_r=0.0)
#r.base.trajectory.add_rotate_waypoint(t_s=40.0, x_r=deg_to_rad(0.0), v_r=0.0)

r.end_of_arm.motors['wrist_yaw'].trajectory.add_waypoint(t_s=0.0, x_r=deg_to_rad(90.0), v_r=0.0)
r.end_of_arm.motors['wrist_yaw'].trajectory.add_waypoint(t_s=10.0, x_r=deg_to_rad(-45.0), v_r=0.0)
r.end_of_arm.motors['wrist_yaw'].trajectory.add_waypoint(t_s=20.0, x_r=deg_to_rad(90.0), v_r=0.0)
# r.end_of_arm.motors['wrist_yaw'].trajectory.add_waypoint(t_s=40.0, x_r=deg_to_rad(180.0), v_r=0.0)

r.head.get_joint('head_tilt').trajectory.add_waypoint(t_s=0.0, x_r=deg_to_rad(0.0), v_r=0.0)
r.head.get_joint('head_tilt').trajectory.add_waypoint(t_s=10.0, x_r=deg_to_rad(-90.0), v_r=0.0)
r.head.get_joint('head_tilt').trajectory.add_waypoint(t_s=20.0, x_r=deg_to_rad(0.0), v_r=0.0)

r.head.get_joint('head_pan').trajectory.add_waypoint(t_s=0.0, x_r=deg_to_rad(0.0), v_r=0.0)
r.head.get_joint('head_pan').trajectory.add_waypoint(t_s=10.0, x_r=deg_to_rad(-90.0), v_r=0.0)
r.head.get_joint('head_pan').trajectory.add_waypoint(t_s=20.0, x_r=deg_to_rad(0.0), v_r=0.0)

print('start trajectory')
r.start_trajectory()
ts=time.time()
while r.is_trajectory_executing():
    dt=(time.time()-ts)/22.0 #0-1
    #r.base.set_translate_velocity(v_m=dt*0.5) #Uncomment this for base motion (put up on blocks first!)
    r.push_command()
    time.sleep(0.1)

r.stop_trajectory()
r.stop()
print('done')

