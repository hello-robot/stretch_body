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
        self.dead_zone = 0.0001
        self._prev_set_vel_ts = None
        self.max_linear_vel = self.base.params['motion']['max']['vel_m']
        self.max_rotation_vel = 1.90241 # rad/s
        self.normal_linear_vel = self.base.params['motion']['default']['vel_m']
        self.normal_rotation_vel = self.max_rotation_vel*0.4
        self.safety_v_m = 0
        self.safety_w_r = 0
        self.scale_factor_linear = 1
        self.scale_factor_rot = 1
        self.precision_mode = False
        self.fast_base_mode = False
        self.acc = self.base.params['motion']['max']['accel_m']

        # Precision mode params
        self.precision_max_linear_vel = 0.01 # m/s 
        self.precision_max_rot_vel = 0.04 # rad/s 
    
    def _safety_check(self,v_m, w_r):
        if self._prev_set_vel_ts is None:
            return
        if self.base.status['timestamp_pc'] > self._prev_set_vel_ts:
            # modify vel value based on 
            # feedback from reading the status
            pass
        self.safety_v_m = v_m*self.scale_factor_linear
        self.safety_w_r = w_r*self.scale_factor_rot

    def _process_stick_to_vel(self, x, y):
        # do someting
        max_linear_vel =  self.max_linear_vel if self.fast_base_mode else self.normal_linear_vel
        max_rotation_vel = self.max_rotation_vel if self.fast_base_mode else self.normal_rotation_vel
        v_m = map_to_range(abs(y), 0, max_linear_vel)
        if y<0:
            v_m = -1*v_m
        x = -1*x
        w_r = map_to_range(abs(x), 0, max_rotation_vel)
        if x<0:
            w_r = -1*w_r
        self._safety_check(v_m, w_r)
    
    def command_stick_to_motion(self, x, y):
        if abs(x) < self.dead_zone:
            x = 0
        if abs(y) < self.dead_zone:
            y = 0
        x = to_parabola_transform(x)
        # y = to_parabola_transform(y) 
        
        # Standard Mode
        if not self.precision_mode:
            self.start_pos = None
            self.start_theta = None
            self._process_stick_to_vel(x,y)
            self.base.set_velocity(self.safety_v_m, self.safety_w_r, a=self.acc)
            self._prev_set_vel_ts = time.time()
            # print(f"[CommandBase]  X: {x} | Y: {y} || v_m: {self.safety_v_m} | w_r: {self.safety_w_r}")
        else:
        # Precision Mode
            if self.start_pos is None:
                self.start_pos = [self.base.status['x'],self.base.status['y']]
                self.start_theta = self.base.status['theta']
                self.target_position = self.start_pos
                self.target_theta = self.start_theta  
            yv = map_to_range(abs(y),0,self.precision_max_linear_vel)
            xv = map_to_range(abs(x),0,self.precision_max_rot_vel)
            if x<0:
                xv = -1*xv
            if y<0:
                yv = -1*yv
            if abs(x)>abs(y):
                self.step_precision_rotate(xv)
            else:
                self.step_precision_translate(yv)
            # Update the previous_time for the next iteration
            self._prev_set_vel_ts = time.time()
            
    def step_precision_rotate(self, xv):
        # Calculate the time elapsed since the last iteration
        current_time = time.time()
        elapsed_time = current_time - self._prev_set_vel_ts 

        # Calculate the desired change in position to achieve the desired velocity
        desired_theta_change = xv * elapsed_time

        # Set the control_effort as the position setpoint for the joint
        if abs(xv)>=0:
            self.base.rotate_by(-1*desired_theta_change)

    def step_precision_translate(self, yv):
        
        # Calculate the time elapsed since the last iteration
        current_time = time.time()
        elapsed_time = current_time - self._prev_set_vel_ts 

        # Calculate the desired change in position to achieve the desired velocity
        desired_position_change = yv * elapsed_time

        # Set the control_effort as the position setpoint for the joint
        if abs(yv)>=0:
            self.base.translate_by(desired_position_change)
            
class CommandLift:
    def __init__(self, robot):
        self.motor = robot.lift
        self.dead_zone = 0.0001
        self._prev_set_vel_ts = None
        self.max_linear_vel = self.motor.params['motion']['max']['vel_m']
        self.safety_v_m = 0
        self.scale_factor = 1.0
        self.precision_mode = False
        self.acc = self.motor.params['motion']['fast']['accel_m']
        
        # Precision mode params
        self.start_pos = None
        self.target_position = self.start_pos
        self.precision_kp = 0.5
        self.precision_max_vel = 0.02 # m/s
        self.stopped_for_prec = False 
    
    def _safety_check(self,v_m):
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
        self._safety_check(v_m)
    
    def command_stick_to_motion(self, x):
        if abs(x) < self.dead_zone:
            x = 0
        # x = to_parabola_transform(x)
        
        # Standard Mode
        if not self.precision_mode:
            self.start_pos = None
            self._process_stick_to_vel(x)
            self.motor.set_velocity(self.safety_v_m, a_m=self.acc)
            self._prev_set_vel_ts = time.time()
            # print(f"[CommandLift]  X: {x} || v_m: {self.safety_v_m}")
        else:
        # Precision Mode
            if self.start_pos is None:
                self.start_pos = self.motor.status['pos']
                self.target_position = self.start_pos
                # TODO: Wait for the velocity to settle to zero
            
            r = self.precision_max_vel
            xv = map_to_range(abs(x),0,r)
            if x<0:
                xv = -1*xv
            # if self.motor.status['timestamp_pc'] > self._prev_set_vel_ts:
            self.step_precision_move(xv)
    
    def step_precision_move(self,xv):
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
        x_des = self.precision_kp * position_error

        # Set the control_effort as the position setpoint for the joint
        self.motor.move_to(self.start_pos + x_des)

        # Update the previous_time for the next iteration
        self._prev_set_vel_ts = time.time()
        # print(f"[LIFT]\nstart_pos: {self.start_pos}\n"
        #       f"current_position: {current_position}\n"
        #       f"target_pos: {self.target_position}\n"
        #       f"desired_position_change: {desired_position_change}\n"
        #       f"position_error: {position_error}\n"
        #       f"xv: {xv}\n"
        #       f"x_des: {x_des}\n"
        #       f"v_curr: {self.motor.status['vel']}\n")

class CommandArm:
    def __init__(self, robot):
        self.motor = robot.arm
        self.dead_zone = 0.0001
        self._prev_set_vel_ts = None
        self.max_linear_vel = self.motor.params['motion']['default']['vel_m']
        self.safety_v_m = 0
        self.scale_factor = 1.0
        self.precision_mode = False
        self.acc = self.motor.params['motion']['fast']['accel_m']
        
        # Precision mode params
        self.start_pos = None
        self.target_position = self.start_pos
        self.precision_kp = 0.6
        self.precision_max_vel = 0.02 # m/s 
    
    def _safety_check(self,v_m):
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
        self._safety_check(v_m)
    
    def command_stick_to_motion(self, x):
        if abs(x) < self.dead_zone:
            x = 0
        # x = to_parabola_transform(x)
        if not self.precision_mode:
        # Standard Mode
            self.start_pos = None
            self._process_stick_to_vel(x)
            self.motor.set_velocity(self.safety_v_m,a_m=self.acc)
            self._prev_set_vel_ts = time.time()
            # print(f"[CommandLift]  X: {x} || v_m: {self.safety_v_m}")
        else:
        # Precision Mode
            if self.start_pos is None:
                self.start_pos = self.motor.status['pos']
                self.target_position = self.start_pos
                # TODO: Wait for the velocity to settle to zero
                
            r = self.precision_max_vel
            xv = map_to_range(abs(x),0,r)
            if x<0:
                xv = -1*xv
            self.step_precision_move(xv)
    
    def step_precision_move(self,xv):
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
        x_des = self.precision_kp * position_error

        # Set the control_effort as the position setpoint for the joint
        if abs(xv)>=0:
            self.motor.move_to(self.start_pos + x_des)

        # Update the previous_time for the next iteration
        self._prev_set_vel_ts = time.time()
        # print(f"[ARM]\nstart_pos: {self.start_pos}\n"
        #       f"current_position: {current_position}\n"
        #       f"target_pos: {self.target_position}\n"
        #       f"desired_position_change: {desired_position_change}\n"
        #       f"position_error: {position_error}\n"
        #       f"xv: {xv}\n"
        #       f"x_des: {x_des}\n"
        #       f"v_curr: {self.motor.status['vel']}\n")

class CommandDxlJoint:
    def __init__(self, robot, name):
        if 'wrist' in name:
            self.motor = robot.end_of_arm.get_joint(name)
        if 'head' in name:
            self.motor = robot.head.get_joint(name)
        self.dead_zone = 0.001
        self._prev_set_vel_ts = None
        self.max_vel = self.motor.params['motion']['default']['vel']
        self.safety_v = 0
        self.scale_factor = 1
        self.precision_mode = False
        
        self.precision_scale_down = 0.05
    
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
        v = map_to_range(abs(x), 0, self.max_vel)
        if x<0:
            v = -1*v

        self._step_feedback_check(v)
    
    def command_stick_to_motion(self, x):
        if abs(x)<self.dead_zone:
            x = 0
        # x = to_parabola_transform(x)
        self._process_stick_to_vel(x)
        if self.precision_mode:
            self.safety_v = self.safety_v*self.precision_scale_down
        self.motor.set_velocity(self.safety_v)
        self._prev_set_vel_ts = time.time()

    def command_button_to_motion(self,direction):
        vel = self.max_vel
        if self.precision_mode:
            vel = vel*self.precision_scale_down
        if direction==1:
            self.motor.set_velocity(vel)
        elif direction==-1:
            self.motor.set_velocity(-1*vel)
            
class CommandGripperPosition:
    def __init__(self, robot):
        self.motor = robot.end_of_arm.get_joint('stretch_gripper')
        self.gripper_rotate_pct = 10.0
        self.gripper_accel = self.motor.params['motion']['max']['accel']
        self.gripper_vel = self.motor.params['motion']['max']['vel']
        self.precision_mode = False
    
    def open_gripper(self):
        self.motor.move_by(self.gripper_rotate_pct, self.gripper_vel, self.gripper_accel)
        
    def close_gripper(self):
        self.motor.move_by(-self.gripper_rotate_pct, self.gripper_vel, self.gripper_accel)

class TeleopController:
    def __init__(self):
        self.xbox_controller = xc.XboxController()
        self.precision_mode = False
        self.fast_base_mode = False
        self.robot = rb.Robot()
        self.controller_state = None
        self.end_of_arm_tool = self.robot.end_of_arm.name
        self.sleep = 1/100
        self.print_mode = False
            
        self.base_command = CommandBase(self.robot)
        self.lift_command = CommandLift(self.robot)
        self.arm_command = CommandArm(self.robot)
        self.wirst_yaw_command = CommandDxlJoint(self.robot,'wrist_yaw')
        self.wirst_yaw_command.max_vel = 1.2
        self.head_pan_command = CommandDxlJoint(self.robot,'head_pan')
        self.head_tilt_command =  CommandDxlJoint(self.robot,'head_tilt')
        
        self.gripper = None
        self.wrist_pitch_command = None
        self.wrist_roll_command = None
        
        if not self.end_of_arm_tool == 'tool_stretch_gripper' and not self.end_of_arm_tool=='tool_none':
            self.gripper = CommandGripperPosition(self.robot)
        if self.end_of_arm_tool == 'tool_stretch_dex_wrist':
            self.wrist_pitch_command = CommandDxlJoint(self.robot,'wrist_pitch')
            self.wrist_roll_command = CommandDxlJoint(self.robot,'wrist_roll')
            
        print(f"Key mapped to End-Of-Arm Tool: {self.end_of_arm_tool}")
        
    def update_state(self):
        self.controller_state = self.xbox_controller.get_state()
        self.precision_mode = self.controller_state['left_trigger_pulled'] > 0.9
        self.fast_base_mode = self.controller_state['right_trigger_pulled'] > 0.9
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

    def command_joints_A(self):
        # Standard Key Mapping
        
        if self.controller_state['right_shoulder_button_pressed']:
            self.wirst_yaw_command.command_button_to_motion(-1)
            
        elif self.controller_state['left_shoulder_button_pressed']:
            self.wirst_yaw_command.command_button_to_motion(1)
        else:
            self.wirst_yaw_command.command_stick_to_motion(0)
        
        if self.end_of_arm_tool == 'tool_stretch_dex_wrist':
            if self.controller_state['top_pad_pressed']:
                self.wrist_pitch_command.command_button_to_motion(1)
            elif self.controller_state['bottom_pad_pressed']:
                self.wrist_pitch_command.command_button_to_motion(-1)
            else:
                self.wrist_pitch_command.command_stick_to_motion(0)
            
            if self.controller_state['left_pad_pressed']:
                self.wrist_roll_command.command_button_to_motion(-1)
            elif self.controller_state['right_pad_pressed']:
                self.wrist_roll_command.command_button_to_motion(1)
            else:
                self.wrist_roll_command.command_stick_to_motion(0)
        else:
            if self.controller_state['top_pad_pressed']:
                self.head_tilt_command.command_button_to_motion(1)
            elif self.controller_state['bottom_pad_pressed']:
                self.head_tilt_command.command_button_to_motion(-1)
            else:
                self.head_tilt_command.command_stick_to_motion(0)
            
            if self.controller_state['left_pad_pressed']:
                self.head_pan_command.command_button_to_motion(1)
            elif self.controller_state['right_pad_pressed']:
                self.head_pan_command.command_stick_to_motion(-1)
            else:
                self.head_pan_command.command_stick_to_motion(0)
                
        self.arm_command.command_stick_to_motion(self.controller_state['right_stick_x'])
        self.lift_command.command_stick_to_motion(self.controller_state['right_stick_y'])
        self.base_command.command_stick_to_motion(self.controller_state['left_stick_x'],self.controller_state['left_stick_y'])

        if self.gripper:
            if self.controller_state['right_button_pressed']:
                self.gripper.open_gripper()
            elif self.controller_state['bottom_button_pressed']:
                self.gripper.close_gripper()

    def command_joints_B(self):
        # All analog key mapping
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
    
    def update_modes(self):
        self.arm_command.precision_mode = self.precision_mode
        self.lift_command.precision_mode = self.precision_mode
        self.base_command.precision_mode = self.precision_mode
        self.base_command.fast_base_mode = self.fast_base_mode
        self.wirst_yaw_command.precision_mode = self.precision_mode
        if self.gripper:
            self.gripper.precision_mode = self.precision_mode
        if self.end_of_arm_tool == 'tool_stretch_dex_wrist':
            self.wrist_pitch_command.precision_mode = self.precision_mode
            self.wrist_roll_command.precision_mode = self.precision_mode
        self.head_pan_command.precision_mode = self.precision_mode
        self.head_tilt_command.precision_mode = self.precision_mode
    
    
    def step(self):
        if self.robot.is_calibrated():
            if self.controller_state['top_button_pressed']:
                self.manage_stow()
                time.sleep(1)
            else:
                self.update_modes()
                self.command_joints_A()
                # self.command_joints_B()
                self.robot.push_command()
        else:
            print('press the start button to calibrate the robot')

    def manage_stow(self):
        if self.robot.is_calibrated():
            # Reset motion params as fast for xbox
            v = self.robot.end_of_arm.motors['wrist_yaw'].params['motion']['default']['vel']
            a = self.robot.end_of_arm.motors['wrist_yaw'].params['motion']['default']['accel']
            self.robot.end_of_arm.motors['wrist_yaw'].set_motion_params(v, a)
            self.robot.stow()

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
