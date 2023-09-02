#!/usr/bin/env python3
from __future__ import print_function
import stretch_body.xbox_controller as xc
from stretch_body.device import Device
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


class CommandBase:
    def __init__(self, robot):
        self.base = robot.base
        self._prev_set_vel_ts = None
        self.max_linear_vel = self.base.params['motion']['max']['vel_m']
        self.max_rotation_vel = 1.90241 # rad/s
        self.safety_v_m = 0
        self.safety_w_r = 0
        self.scale_factor = 1
        self.precision_mode = False
    
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
    
    def command_stick_to_motion(self, x, y):
        # x = to_parabola_transform(x)
        # y = to_parabola_transform(y)
        self._process_stick_to_vel(x,y)
        self.base.set_velocity(self.safety_v_m, self.safety_w_r)
        self._prev_set_vel_ts = time.time()
        # print(f"[CommandBase]  X: {x} | Y: {y} || v_m: {self.safety_v_m} | w_r: {self.safety_w_r}")

class CommandLift:
    def __init__(self, robot):
        self.motor = robot.lift
        self.dead_zone = 0.0001
        self._prev_set_vel_ts = None
        self.max_linear_vel = self.motor.params['motion']['max']['vel_m']
        self.safety_v_m = 0
        self.scale_factor = 1.0
        self.precision_mode = False
        
        self.start_pos = None
        self.target_position = self.start_pos
    
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
    
    def command_stick_to_motion(self, x):
        if abs(x) < self.dead_zone:
            x = 0
        if not self.precision_mode:
            self.start_pos = None
            # x = to_parabola_transform(x)
            self._process_stick_to_vel(x)
            self.motor.set_velocity(self.safety_v_m)
            self._prev_set_vel_ts = time.time()
            # print(f"[CommandLift]  X: {x} || v_m: {self.safety_v_m}")
        else:
            if self.start_pos is None:
                self.start_pos = self.motor.status['pos']
                self.target_position = self.start_pos
                
                
            r = 1/100 # max precision vel
            xv = map_to_range(abs(x),0,r)
            if x<0:
                xv = -1*xv
            # if self.motor.status['timestamp_pc'] > self._prev_set_vel_ts:
            self.step_precision_move_by(xv)
                # self.step_precision_move_by(xv)
                
            
    def step_precision_move_by(self,xv):
        Kp_position = 2
        curr_v = self.motor.status['vel']
        e = xv - curr_v
        self.motor.move_by(e*Kp_position)
        self._prev_set_vel_ts = time.time()
        print(f"LIFT || start_pos: {self.start_pos} | xv: {xv} | curr_v: {curr_v}")
    
    def step_precision_move_by(self,xv):
        Kp_position =0.5
        # Read the current joint position
        current_position = self.motor.status['pos']

        # Calculate the time elapsed since the last iteration
        current_time = time.time()
        elapsed_time = current_time - self._prev_set_vel_ts 

        # Calculate the desired change in position to achieve the desired velocity
        desired_position_change = xv * elapsed_time
        
        # Update the target position based on the desired position change
        self.target_position = self.target_position + desired_position_change

        # Calculate the position error
        position_error = self.target_position - current_position

        # Calculate the control effort (position control)
        x_des = Kp_position * position_error

        # Set the control_effort as the position setpoint for the joint
        self.motor.move_to(self.start_pos + x_des)

        # Update the previous_time for the next iteration
        self._prev_set_vel_ts = time.time()
        print(f"start_pos: {self.start_pos} || current_position: {current_position} || target_pos: {self.target_position} || desired_position_change: {desired_position_change} || position_error: {position_error} || xv: {xv} || x_des: {x_des}|| v_curr: {self.motor.status['vel']}")

class CommandArm:
    def __init__(self, robot):
        self.motor = robot.arm
        self._prev_set_vel_ts = None
        self.max_linear_vel = self.motor.params['motion']['default']['vel_m']
        self.safety_v_m = 0
        self.scale_factor = 1
        self.precision_mode = False
        self.dead_zone = 0.001
    
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
    
    def command_stick_to_motion(self, x):
        if abs(x) < self.dead_zone:
            x = 0
        if not self.precision_mode:
            self.start_pos = None
            # x = to_parabola_transform(x)
            self._process_stick_to_vel(x)
            self.motor.set_velocity(self.safety_v_m)
            self._prev_set_vel_ts = time.time()
        # print(f"[CommandArm]  X: {x} || v_m: {self.safety_v_m}")
        else:
            if self.start_pos is None:
                self.start_pos = self.motor.status['pos']
                
            r = 10/100 # 10 cm/s
            xv = map_to_range(abs(x),0,r)
            if x<0:
                xv = -1*xv
            if self.motor.status['timestamp_pc'] > self._prev_set_vel_ts:
                self.step_precision_move_by(xv)
                
    def step_precision_move_by(self,xv):
        Kp_position = 0.1
        curr_v = self.motor.status['vel']
        e = xv - curr_v
        self.motor.move_by(e*Kp_position)
        self._prev_set_vel_ts = time.time()
        print(f"ARM || start_pos: {self.start_pos} | xv: {xv} | curr_v: {curr_v}")

class CommandWristYaw:
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
    
    def command_stick_to_motion(self, x):
        x = to_parabola_transform(x)
        self._process_stick_to_vel(x)
        self.motor.set_velocity(self.safety_v)
        self._prev_set_vel_ts = time.time()
        # print(f"[CommandWristYaw]  X: {x} || v: {self.safety_v}")


class CommandWristPitch:
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
    
    def command_stick_to_motion(self, x):
        x = to_parabola_transform(x)
        self._process_stick_to_vel(x)
        self.motor.set_velocity(self.safety_v)
        self._prev_set_vel_ts = time.time()
        # print(f"[CommandWristPitch]  X: {x} || v: {self.safety_v}")

class CommandHeadPan:
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
    
    def command_stick_to_motion(self, x):
        x = to_parabola_transform(x)
        self._process_stick_to_vel(x)
        self.motor.set_velocity(self.safety_v)
        self._prev_set_vel_ts = time.time()
        # print(f"[CommandHeadPan]  X: {x} || v: {self.safety_v}")

class CommandHeadTilt:
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
    
    def command_stick_to_motion(self, x):
        x = to_parabola_transform(x)
        self._process_stick_to_vel(x)
        self.motor.set_velocity(self.safety_v)
        self._prev_set_vel_ts = time.time()
        # print(f"[CommandHeadTilt]  X: {x} || v: {self.safety_v}")

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



class TeleopController:
    def __init__(self):
        self.xbox_controller = xc.XboxController()
        self.precision_mode = False
        self.robot = rb.Robot()
        self.controller_state = None
        self.end_of_arm_tool = self.robot.end_of_arm.name
        self.sleep = 1/100
        self.print_mode = False
            
        self.base_command = CommandBase(self.robot)
        self.lift_command = CommandLift(self.robot)
        self.arm_command = CommandArm(self.robot)
        self.wirst_yaw_command = CommandWristYaw(self.robot)
        self.head_pan_command = CommandHeadPan(self.robot)
        self.head_tilt_command =  CommandHeadTilt(self.robot)
        
        self.gripper = None
        self.wrist_pitch_command = None
        self.wrist_roll_command = None
        
        if not self.end_of_arm_tool == 'tool_stretch_gripper':
            self.gripper = CommandGripperPosition(self.robot)
        if self.end_of_arm_tool == 'tool_stretch_dex_wrist':
            self.wrist_pitch_command = CommandWristPitch(self.robot)
            
        print(f"Key mapped to End-Of-Arm Tool: {self.end_of_arm_tool}")
        
    def update_state(self):
        self.controller_state = self.xbox_controller.get_state()
        self.precision_mode = self.controller_state['right_trigger_pulled'] > 0.9
    
    def startup(self):
        self.xbox_controller.start()
        if self.robot.startup():
            self.do_double_beep()

    def do_double_beep(self):
        self.robot.pimu.trigger_beep()
        self.robot.push_command()
        time.sleep(0.5)
        self.robot.pimu.trigger_beep()
        self.robot.push_command()
        time.sleep(0.5)
    
    def command_joints(self):
        if self.controller_state['right_shoulder_button_pressed']:
            self.wirst_yaw_command.command_stick_to_motion(self.controller_state['right_stick_x'])
            if self.wrist_pitch_command:
                self.wrist_pitch_command.command_stick_to_motion(self.controller_state['right_stick_y'])
            self.arm_command.command_stick_to_motion(0)
            self.lift_command.command_stick_to_motion(0)
        else:
            self.arm_command.command_stick_to_motion(self.controller_state['right_stick_x'])
            self.lift_command.command_stick_to_motion(self.controller_state['right_stick_y'])
            self.wirst_yaw_command.command_stick_to_motion(0)
            if self.wrist_pitch_command:
                self.wrist_pitch_command.command_stick_to_motion(0)
        if self.controller_state['left_shoulder_button_pressed']:
            self.head_pan_command.command_stick_to_motion(self.controller_state['left_stick_x'])
            self.head_tilt_command.command_stick_to_motion(self.controller_state['left_stick_y'])
            self.base_command.command_stick_to_motion(0,0)
        else:
            self.base_command.command_stick_to_motion(self.controller_state['left_stick_x'],self.controller_state['left_stick_y'])
            self.head_pan_command.command_stick_to_motion(0)
            self.head_tilt_command.command_stick_to_motion(0)
        if self.gripper:
            if self.controller_state['right_button_pressed']:
                self.gripper.open_gripper()
            elif self.controller_state['bottom_button_pressed']:
                self.gripper.close_gripper()

    def step(self):
        if self.robot.is_calibrated():
            if self.controller_state['top_button_pressed']:
                self.robot.stow()
                time.sleep(1)
            else:
                self.arm_command.precision_mode = self.precision_mode
                self.lift_command.precision_mode = self.precision_mode
                self.base_command.precision_mode = self.precision_mode
                self.command_joints()
                self.robot.push_command()
        else:
            print('press the start button to calibrate the robot')
            
    def main(self):
        try:
            while True:
                self.update_state()
                if not self.robot.is_calibrated() and self.controller_state['start_button_pressed']:
                    self.robot.home()
                    time.sleep(1)
                self.step()
                time.sleep(self.sleep)
        except (ThreadServiceExit, KeyboardInterrupt, SystemExit, UnpluggedError):
            self.xbox_controller.stop()
            self.robot.stop()



if __name__ == "__main__":
   controller = TeleopController()
   controller.startup()
   controller.main()
