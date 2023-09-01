#!/usr/bin/env python3
from __future__ import print_function
import stretch_body.xbox_controller as xc
from inputs import UnpluggedError
import stretch_body.robot as rb
from stretch_body.hello_utils import *
import os
import time
import argparse
import random
import math
import numpy as np

print_stretch_re_use()

def to_parabola_transform(x):
    if x<0:
        return  -1*(abs(x)**2)
    else:
        return x**2

def bound_value(value, lower_bound, upper_bound):
    if value < lower_bound:
        return lower_bound
    elif value > upper_bound:
        return upper_bound
    else:
        return value

def map_to_range(value, new_min, new_max):
    # Ensure value is between 0 and 1
    value = max(0, min(1, value))
    mapped_value = (value - 0) * (new_max - new_min) / (1 - 0) + new_min
    return mapped_value


class CommandBaseVelocity:
    def __init__(self, robot):
        self.base = robot.base
        self._prev_set_vel_ts = None
        self.max_linear_vel = self.base.params['motion']['max']['vel_m']
        self.max_rotation_vel = 1.90241 # rad/s
        self.safety_v_m = 0
        self.safety_w_r = 0
        self.scale_factor = 1
    
    def _step_feedback_check(self,v_m, w_r):
        if self._prev_set_vel_ts is None:
            return
        if self.base.status['timestamp_pc'] > self._prev_set_vel_ts:
            # modify vel value based on 
            # feedback from reading the status
            pass
        self.safety_v_m = v_m*self.scale_factor
        self.safety_w_r = w_r*self.scale_factor

    def _process_stick_to_vel(self, x, y):
        # do someting
        v_m = map_to_range(abs(y), 0, self.max_linear_vel)
        if y<0:
            v_m = -1*v_m
        x = -1*x
        w_r = map_to_range(abs(x), 0, self.max_rotation_vel)
        if x<0:
            w_r = -1*w_r
        self._step_feedback_check(v_m, w_r)
    
    def command_stick_to_velocity(self, x, y):
        x = to_parabola_transform(x)
        y = to_parabola_transform(y)
        self._process_stick_to_vel(x,y)
        self.base.set_velocity(self.safety_v_m, self.safety_w_r)
        self._prev_set_vel_ts = time.time()
        print(f"[CommandBaseVelocity]  X: {x} | Y: {y} || v_m: {self.safety_v_m} | w_r: {self.safety_w_r}")

class CommandLiftVelocity:
    def __init__(self, robot):
        self.motor = robot.lift
        self._prev_set_vel_ts = None
        self.max_linear_vel = self.motor.params['motion']['max']['vel_m']
        self.safety_v_m = 0
        self.scale_factor = 1.0
    
    def _step_feedback_check(self,v_m):
        if self._prev_set_vel_ts is None:
            return
        if self.motor.status['timestamp_pc'] > self._prev_set_vel_ts:
            # modify vel value based on 
            # feedback from reading the status
            pass
        self.safety_v_m = v_m*self.scale_factor

    def _process_stick_to_vel(self, x):
        # do someting
        v_m = map_to_range(abs(x), 0, self.max_linear_vel)
        if x<0:
            v_m = -1*v_m

        self._step_feedback_check(v_m)
    
    def command_stick_to_velocity(self, x):
        x = to_parabola_transform(x)
        self._process_stick_to_vel(x)
        self.motor.set_velocity(self.safety_v_m)
        self._prev_set_vel_ts = time.time()
        print(f"[CommandLiftVelocity]  X: {x} || v_m: {self.safety_v_m}")

class CommandArmVelocity:
    def __init__(self, robot):
        self.motor = robot.arm
        self._prev_set_vel_ts = None
        self.max_linear_vel = self.motor.params['motion']['default']['vel_m']
        self.safety_v_m = 0
        self.scale_factor = 1
    
    def _step_feedback_check(self,v_m):
        if self._prev_set_vel_ts is None:
            return
        if self.motor.status['timestamp_pc'] > self._prev_set_vel_ts:
            # modify vel value based on 
            # feedback from reading the status
            pass
        self.safety_v_m = v_m*self.scale_factor

    def _process_stick_to_vel(self, x):
        # do someting
        v_m = map_to_range(abs(x), 0, self.max_linear_vel)
        if x<0:
            v_m = -1*v_m

        self._step_feedback_check(v_m)
    
    def command_stick_to_velocity(self, x):
        x = to_parabola_transform(x)
        self._process_stick_to_vel(x)
        self.motor.set_velocity(self.safety_v_m)
        self._prev_set_vel_ts = time.time()
        print(f"[CommandArmVelocity]  X: {x} || v_m: {self.safety_v_m}")


class CommandWristYawVelocity:
    def __init__(self, robot):
        self.motor = robot.end_of_arm.get_joint('wrist_yaw')
        self._prev_set_vel_ts = None
        self.max_linear_vel = self.motor.params['motion']['max']['vel']
        self.safety_v = 0
        self.scale_factor = 1
    
    def _step_feedback_check(self,v):
        if self._prev_set_vel_ts is None:
            return
        if self.motor.status['timestamp_pc'] > self._prev_set_vel_ts:
            # modify vel value based on 
            # feedback from reading the status
            pass
        self.safety_v = v*self.scale_factor

    def _process_stick_to_vel(self, x):
        # do someting
        x = -1*x
        v = map_to_range(abs(x), 0, self.max_linear_vel)
        if x<0:
            v = -1*v

        self._step_feedback_check(v)
    
    def command_stick_to_velocity(self, x):
        x = to_parabola_transform(x)
        self._process_stick_to_vel(x)
        self.motor.set_velocity(self.safety_v)
        self._prev_set_vel_ts = time.time()
        print(f"[CommandWristYawVelocity]  X: {x} || v: {self.safety_v}")


class CommandWristPitchVelocity:
    def __init__(self, robot):
        self.motor = robot.end_of_arm.get_joint('wrist_pitch')
        self._prev_set_vel_ts = None
        self.max_linear_vel = self.motor.params['motion']['max']['vel']
        self.safety_v = 0
        self.scale_factor = 1

    def _step_feedback_check(self,v):
        if self._prev_set_vel_ts is None:
            return
        if self.motor.status['timestamp_pc'] > self._prev_set_vel_ts:
            # modify vel value based on 
            # feedback from reading the status
            pass
        self.safety_v = v*self.scale_factor

    def _process_stick_to_vel(self, x):
        # do someting
        x = -1*x
        v = map_to_range(abs(x), 0, self.max_linear_vel)
        if x<0:
            v = -1*v

        self._step_feedback_check(v)
    
    def command_stick_to_velocity(self, x):
        x = to_parabola_transform(x)
        self._process_stick_to_vel(x)
        self.motor.set_velocity(self.safety_v)
        self._prev_set_vel_ts = time.time()
        print(f"[CommandWristPitchVelocity]  X: {x} || v: {self.safety_v}")

class CommandHeadPanVelocity:
    def __init__(self, robot):
        self.motor = robot.head.get_joint('head_pan')
        self._prev_set_vel_ts = None
        self.max_linear_vel = self.motor.params['motion']['max']['vel']
        self.safety_v = 0
        self.scale_factor = 1
    
    def _step_feedback_check(self,v):
        if self._prev_set_vel_ts is None:
            return
        if self.motor.status['timestamp_pc'] > self._prev_set_vel_ts:
            # modify vel value based on 
            # feedback from reading the status
            pass
        self.safety_v = v*self.scale_factor

    def _process_stick_to_vel(self, x):
        # do someting
        x = -1*x
        v = map_to_range(abs(x), 0, self.max_linear_vel)
        if x<0:
            v = -1*v

        self._step_feedback_check(v)
    
    def command_stick_to_velocity(self, x):
        x = to_parabola_transform(x)
        self._process_stick_to_vel(x)
        self.motor.set_velocity(self.safety_v)
        self._prev_set_vel_ts = time.time()
        print(f"[CommandHeadPanVelocity]  X: {x} || v: {self.safety_v}")

class CommandHeadTiltVelocity:
    def __init__(self, robot):
        self.motor = robot.head.get_joint('head_tilt')
        self._prev_set_vel_ts = None
        self.max_linear_vel = self.motor.params['motion']['max']['vel']
        self.safety_v = 0
        self.scale_factor = 1
    
    def _step_feedback_check(self,v):
        if self._prev_set_vel_ts is None:
            return
        if self.motor.status['timestamp_pc'] > self._prev_set_vel_ts:
            # modify vel value based on 
            # feedback from reading the status
            pass
        self.safety_v = v*self.scale_factor

    def _process_stick_to_vel(self, x):
        # do someting
        x = -1*x
        v = map_to_range(abs(x), 0, self.max_linear_vel)
        if x<0:
            v = -1*v

        self._step_feedback_check(v)
    
    def command_stick_to_velocity(self, x):
        x = to_parabola_transform(x)
        self._process_stick_to_vel(x)
        self.motor.set_velocity(self.safety_v)
        self._prev_set_vel_ts = time.time()
        print(f"[CommandHeadTiltVelocity]  X: {x} || v: {self.safety_v}")

class CommandGripperPosition:
    def __init__(self, robot):
        self.motor = robot.end_of_arm.get_joint('stretch_gripper')
        self.gripper_rotate_pct = 10.0
        self.gripper_accel = self.motor.params['motion']['max']['accel']
        self.gripper_vel = self.motor.params['motion']['max']['vel']
    
    def open_gripper(self):
        self.motor.move_by(self.gripper_rotate_pct, self.gripper_vel, self.gripper_accel)
        
    def close_gripper(self):
        self.motor.move_by(-self.gripper_rotate_pct, self.gripper_vel, self.gripper_accel)

def do_double_beep(robot):
    robot.pimu.trigger_beep()
    robot.push_command()
    time.sleep(0.5)
    robot.pimu.trigger_beep()
    robot.push_command()
    time.sleep(0.5)

def main():
    xbox_controller = xc.XboxController()
    xbox_controller.start()
    robot = rb.Robot()
    try:
        if robot.startup():
            do_double_beep(robot)
        
        base_command = CommandBaseVelocity(robot)
        lift_command = CommandLiftVelocity(robot)
        arm_command = CommandArmVelocity(robot)
        wirst_yaw_command = CommandWristYawVelocity(robot)
        head_pan = CommandHeadPanVelocity(robot)
        head_tilt =  CommandHeadTiltVelocity(robot)
        gripper = CommandGripperPosition(robot)
        
        if 'wrist_pitch' in list(robot.end_of_arm.joints):
            wrist_pitch_command = CommandWristPitchVelocity(robot)
        sleep = 1/100
        while True:
            state = xbox_controller.get_state()
            if not robot.is_calibrated() and state['start_button_pressed']:
                robot.home()
                time.sleep(1)
            if robot.is_calibrated():
                if state['top_button_pressed']:
                    robot.stow()
                    time.sleep(1)
                else:
                    if state['right_shoulder_button_pressed']:
                        wirst_yaw_command.command_stick_to_velocity(state['right_stick_x'])
                        if 'wrist_pitch' in list(robot.end_of_arm.joints):
                            wrist_pitch_command.command_stick_to_velocity(state['right_stick_y'])
                        arm_command.command_stick_to_velocity(0)
                        lift_command.command_stick_to_velocity(0)
                    else:
                        arm_command.command_stick_to_velocity(state['right_stick_x'])
                        lift_command.command_stick_to_velocity(state['right_stick_y'])
                        wirst_yaw_command.command_stick_to_velocity(0)
                        if 'wrist_pitch' in list(robot.end_of_arm.joints):
                            wrist_pitch_command.command_stick_to_velocity(0)
                    if state['left_shoulder_button_pressed']:
                        head_pan.command_stick_to_velocity(state['left_stick_x'])
                        head_tilt.command_stick_to_velocity(state['left_stick_y'])
                        base_command.command_stick_to_velocity(0,0)
                    else:
                        base_command.command_stick_to_velocity(state['left_stick_x'],state['left_stick_y'])
                        head_pan.command_stick_to_velocity(0)
                        head_tilt.command_stick_to_velocity(0)
                    if state['bottom_button_pressed']:
                        gripper.open_gripper()
                    elif state['right_button_pressed']:
                        gripper.close_gripper()
                    robot.push_command()
            else:
                print('press the start button to calibrate the robot')
                
            time.sleep(sleep)
    except (ThreadServiceExit, KeyboardInterrupt, SystemExit, UnpluggedError):
        robot.stop()
        xbox_controller.stop()
        
    



if __name__ == "__main__":
    main()
