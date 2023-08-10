#!/usr/bin/env python3
import stretch_body.robot_params
# stretch_body.robot_params.RobotParams.set_logging_level("DEBUG")
import numpy as np
import time
from stretch_body import dynamixel_hello_XL430
from stretch_body import robot
import time
import matplotlib.pyplot as plt
import drawnow
####### This is a experimental code and not an official unittest test code ######

class MultiDataMultiFramePlot:
    def __init__(self, y_data, title="", y_label=""):
        self.y_label = y_label
        self.title = title
        self.num_frames = 0
        self.data_sets = []
        self.y_data = y_data
    
    def add_data(self,x_data,label,lines=None):
        self.data_sets.append((x_data, label, lines))
        self.num_frames = self.num_frames + 1

    def plot(self):
        fig, axs = plt.subplots(self.num_frames, 1, figsize=(8, 6*self.num_frames))

        for frame_idx, ax in enumerate(axs):
            data = self.data_sets[frame_idx]
            ax.plot(self.y_data, data[0], label=data[1])
            lines = data[2]
            # ax.set_xlabel(data[1])
            ax.set_ylabel(self.y_label)
            if lines is not None:
                for l in lines:
                    y = l[0]
                    label = l[1]
                    ax.axhline(y,color=label)
            ax.set_title(f"{self.title}: {data[1]}")
            ax.legend()
            ax.grid(True)

        plt.tight_layout()
        plt.show()
        
def generate_sine_wave(amplitude, frequency, time, step, phase=0):
    time = np.arange(0, time, step)
    y_values = amplitude * np.sin((2 * np.pi * frequency * time)+ phase)

    return time, y_values

r = robot.Robot()
r.startup()
# r.home()


# joint_name = 'head_pan'
# motor = r.head.get_joint(joint_name)
joint_name = 'wrist_yaw' # wrist_yaw, stretch_gripper, wrist_pitch, wrist_roll
motor = r.end_of_arm.get_joint(joint_name)
# motor = dynamixel_hello_XL430.DynamixelHelloXL430("wrist_yaw")
# motor.startup()
motor.home()
total_time = 30
interval = 1/30 # s
freaquency = 0.1 #Hz

phase = 0
max_vel_ticks = motor.motor.get_vel_limit()
print(f"Vel Limit: {max_vel_ticks} ticks/s | {abs(motor.ticks_to_world_rad_per_sec(max_vel_ticks))} rad/s")
print(f"Vel gains P: {motor.motor.get_vel_P_gain()} | I: {motor.motor.get_vel_I_gain()}")
max_vel = abs(motor.ticks_to_world_rad_per_sec(max_vel_ticks))
# max_vel = 3
min_pos = motor.get_soft_motion_limits()[0]
max_pos = motor.get_soft_motion_limits()[1]
# motor.set_vel_dead_zone(0.1)
# deadzone = motor.vel_dead_zone

T, y_values = generate_sine_wave(max_vel,freaquency, total_time, interval, phase)
vel_track = []
pos_track = []
effort_track = []

start = time.time()
for x in y_values:
    motor.set_velocity(x)
    vel_track.append(motor.status['vel'])
    pos_track.append(motor.status['pos'])
    effort_track.append(motor.status['effort'])
    time.sleep(interval)
r.stop()
elapsed = time.time() - start
print(f"Time Elapsed: {elapsed} s")

plotter = MultiDataMultiFramePlot(y_data=T,y_label="Time",title=f"Velocity Control: {joint_name}")
plotter.add_data(y_values,"Target velocity [rad/s]")
plotter.add_data(vel_track,"velocity [rad/s]")
plotter.add_data(pos_track,"position [rad]",[(min_pos,'red'),
                                             (max_pos,'red'),
                                             (min_pos+motor.vel_brake_zone_thresh,'blue'), 
                                             (max_pos-motor.vel_brake_zone_thresh,'blue')])
plotter.add_data(effort_track, "Effort")
plotter.plot()

