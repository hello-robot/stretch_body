#!/usr/bin/env python3
import stretch_body.robot_params
# stretch_body.robot_params.RobotParams.set_logging_level("DEBUG")
import numpy as np
import time
from stretch_body import dynamixel_hello_XL430
from stretch_body import robot
import time
import matplotlib.pyplot as plt

####### This is a an experimental test tool and not an unittest test code ######

"""
This script visualizes the safety braking mechanism of the dynamixel joints (head_pan, head_tilt, wrist_yaw) that tests the following:
- dampened velocities are issued beyond the velocity braking zone
- zero velocity maintained at the joint limits
- Max velocities input near joint limits are braked smoothly and won't get into overoad error state.
This test script sends an sine wave of input velocties at 0.1 Hz and amplitude equal to the maximum velocity. 
"""

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

def plot_data(motor, time_values, y_values, vel_track, pos_track, effort_track):
    joint_name = motor.name
    min_pos = motor.get_soft_motion_limits()[0]
    max_pos = motor.get_soft_motion_limits()[1]
    plotter = MultiDataMultiFramePlot(y_data=time_values, y_label="Time",title=f"Velocity Control: {joint_name}")
    plotter.add_data(y_values,"Target velocity [rad/s]")
    plotter.add_data(vel_track,"velocity [rad/s]")
    plotter.add_data(pos_track,"position [rad]",[(min_pos,'red'),
                                                (max_pos,'red'),])
                                                # (min_pos+motor.vel_brake_zone_thresh,'blue'), 
                                                # (max_pos-motor.vel_brake_zone_thresh,'blue')])
    plotter.add_data(effort_track, "Effort")
    plotter.plot()

def run_test_on_motor(motor):
    motor.home()
    total_time = 20
    interval = 1/100 # s
    freaquency = .1 #Hz
    phase = np.pi/2

    max_vel_ticks = motor.motor.get_vel_limit()
    print(f"Vel Limit: {max_vel_ticks} ticks/s | {abs(motor.ticks_to_world_rad_per_sec(max_vel_ticks))} rad/s")
    print(f"Vel gains P: {motor.motor.get_vel_P_gain()} | I: {motor.motor.get_vel_I_gain()}")
    max_vel = abs(motor.ticks_to_world_rad_per_sec(max_vel_ticks))
    print(f"Vel Brakezone Thresh: {motor.vel_brake_zone_thresh} rad")

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
    elapsed = time.time() - start
    print(f"Time Elapsed: {elapsed} s")
    plot_data(motor, T, y_values, vel_track, pos_track, effort_track)
    return T, y_values, vel_track, pos_track, effort_track

def test_head_pan_joint():
    r = robot.Robot()
    r.startup()

    joint_name = 'head_pan'
    motor = r.head.get_joint(joint_name)
    T, input_velocities, vel_track, pos_track, effort_track = run_test_on_motor(motor)
    r.stop()

def test_head_tilt_joint():
    r = robot.Robot()
    r.startup()

    joint_name = 'head_tilt'
    motor = r.head.get_joint(joint_name)
    T, input_velocities, vel_track, pos_track, effort_track = run_test_on_motor(motor)
    r.stop()

def test_wrist_yaw_joint():
    r = robot.Robot()
    r.startup()
    joint_name = 'wrist_yaw'
    motor = r.end_of_arm.get_joint(joint_name)
    T, input_velocities, vel_track, pos_track, effort_track = run_test_on_motor(motor)
    r.stop()

def test_wrist_pitch_joint():
    r = robot.Robot()
    r.startup()

    joint_name = 'wrist_pitch'
    motor = r.end_of_arm.get_joint(joint_name)
    T, input_velocities, vel_track, pos_track, effort_track = run_test_on_motor(motor)
    r.stop()

def test_wrist_roll_joint():
    r = robot.Robot()
    r.startup()

    joint_name = 'wrist_roll'
    motor = r.end_of_arm.get_joint(joint_name)
    T, input_velocities, vel_track, pos_track, effort_track = run_test_on_motor(motor)
    r.stop()

if __name__=="__main__":
    test_wrist_yaw_joint()
    # test_wrist_pitch_joint()
    # test_wrist_roll_joint()
    # test_head_pan_joint()
    # test_head_tilt_joint()