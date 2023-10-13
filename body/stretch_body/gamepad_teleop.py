#!/usr/bin/env python3
from __future__ import print_function
import stretch_body.gamepad_controller as gc
from stretch_body.device import Device
from inputs import UnpluggedError
import stretch_body.robot as rb
from stretch_body.hello_utils import *
from stretch_body.robot_params import RobotParams
import os
import time
import threading
import sys
import click
import numpy as np
import subprocess


class CommandBase:
    def __init__(self):
        self.params = RobotParams().get_params()[1]['base']
        self.dead_zone = 0.0001
        self._prev_set_vel_ts = None
        self.max_linear_vel = self.params['motion']['max']['vel_m']
        self.max_rotation_vel = 1.90241 # rad/s
        self.normal_linear_vel = self.params['motion']['default']['vel_m']
        self.normal_rotation_vel = self.max_rotation_vel*0.4
        self.precision_mode = False
        self.fast_base_mode = False
        self.acc = self.params['motion']['max']['accel_m']

        # Precision mode params
        self.precision_max_linear_vel = 0.02 # m/s Very precise: 0.01
        self.precision_max_rot_vel = 0.08 # rad/s Very precise: 0.04
    
    def command_stick_to_motion(self, x, y, robot):
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
            v_m, w_r = self._process_stick_to_vel(x,y, robot)
            robot.base.set_velocity(v_m, w_r, a=self.acc)
            self._prev_set_vel_ts = time.time()
        else:
        # Precision Mode
            if self.start_pos is None:
                self.start_pos = [robot.base.status['x'],robot.base.status['y']]
                self.start_theta = robot.base.status['theta']
                self.target_position = self.start_pos
                self.target_theta = self.start_theta  
            yv = map_to_range(abs(y),0,self.precision_max_linear_vel)
            xv = map_to_range(abs(x),0,self.precision_max_rot_vel)
            if x<0:
                xv = -1*xv
            if y<0:
                yv = -1*yv
            if abs(x)>abs(y):
                self._step_precision_rotate(xv, robot)
            else:
                self._step_precision_translate(yv, robot)
            # Update the previous_time for the next iteration
            self._prev_set_vel_ts = time.time()
    
    def command_button_to_motion_rotate(self, direction, robot):
        v_m = 0
        w_r = direction*self.normal_rotation_vel
        if self.fast_base_mode and self.is_fastbase_safe(robot):
            w_r = direction*self.max_rotation_vel
        if self.precision_mode:
            w_r = direction*self.precision_max_rot_vel
        robot.base.set_velocity(v_m, w_r, a=self.acc)
        # Update the previous_time for the next iteration
        self._prev_set_vel_ts = time.time()

    def command_button_to_motion_translate(self, direction, robot):
        v_m = direction*self.normal_linear_vel
        w_r = 0
        if self.fast_base_mode and self.is_fastbase_safe(robot):
            v_m = direction*self.max_linear_vel
        if self.precision_mode:
            v_m = direction*self.precision_max_linear_vel
        robot.base.set_velocity(v_m, w_r, a=self.acc)
        # Update the previous_time for the next iteration
        self._prev_set_vel_ts = time.time()
    
    def stop_motion(self, robot):
        robot.base.set_velocity(0, 0, a=self.acc)

    def is_fastbase_safe(self, robot):
        arm = robot.arm.status['pos'] < (robot.get_stow_pos('arm') + 0.1) # check if arm pos under stow pose + 0.1 m
        lift = robot.lift.status['pos'] < (robot.get_stow_pos('lift') + 0.05) # check if lift pos is under stow pose + 0.05m
        return arm and lift
        
    def _safety_check(self,v_m, w_r):
        # Do some safety checks and modify vel
        return v_m, w_r

    def _process_stick_to_vel(self, x, y, robot):
        max_linear_vel = self.normal_linear_vel
        max_rotation_vel = self.normal_rotation_vel
        if self.fast_base_mode and self.is_fastbase_safe(robot):
            max_linear_vel =  self.max_linear_vel
            max_rotation_vel = self.max_rotation_vel
        v_m = map_to_range(abs(y), 0, max_linear_vel)
        if y<0:
            v_m = -1*v_m
        x = -1*x
        w_r = map_to_range(abs(x), 0, max_rotation_vel)
        if x<0:
            w_r = -1*w_r
        return self._safety_check(v_m, w_r)
            
    def _step_precision_rotate(self, xv, robot):
        # Calculate the time elapsed since the last iteration
        current_time = time.time()
        elapsed_time = current_time - self._prev_set_vel_ts 

        # Calculate the desired change in position to achieve the desired velocity
        desired_theta_change = xv * elapsed_time

        # Set the control_effort as the position setpoint for the joint
        if abs(xv)>=0:
            robot.base.rotate_by(-1*desired_theta_change)

    def _step_precision_translate(self, yv, robot):
        
        # Calculate the time elapsed since the last iteration
        current_time = time.time()
        elapsed_time = current_time - self._prev_set_vel_ts 

        # Calculate the desired change in position to achieve the desired velocity
        desired_position_change = yv * elapsed_time

        # Set the control_effort as the position setpoint for the joint
        if abs(yv)>=0:
            robot.base.translate_by(desired_position_change)
            
class CommandLift:
    def __init__(self):
        self.params = RobotParams().get_params()[1]['lift']
        self.dead_zone = 0.0001
        self._prev_set_vel_ts = None
        self.max_linear_vel = self.params['motion']['max']['vel_m']
        self.precision_mode = False
        self.acc = self.params['motion']['max']['accel_m']
        
        # Precision mode params
        self.start_pos = None
        self.target_position = self.start_pos
        self.precision_kp = 0.5 # Very Precise: 0.5
        self.precision_max_vel = 0.04 # m/s Very Precise: 0.02 m/s
        self.stopped_for_prec = False 
    
    def command_stick_to_motion(self, x, robot):
        if abs(x) < self.dead_zone:
            x = 0
        # x = to_parabola_transform(x)
        
        # Standard Mode
        if not self.precision_mode:
            self.start_pos = None
            v_m = self._process_stick_to_vel(x)
            robot.lift.set_velocity(v_m, a_m=self.acc)
            self._prev_set_vel_ts = time.time()
            # print(f"[CommandLift]  X: {x} || v_m: {self.safety_v_m}")
        else:
        # Precision Mode
            if self.start_pos is None:
                self.start_pos = robot.lift.status['pos']
                self.target_position = self.start_pos
                # TODO: Wait for the velocity to settle to zero
            
            r = self.precision_max_vel
            xv = map_to_range(abs(x),0,r)
            if x<0:
                xv = -1*xv
            self._step_precision_move(xv, robot)
    
    def command_button_to_motion(self, direction, robot):
        v_m = direction*self.max_linear_vel
        if self.precision_mode:
            v_m = direction*self.precision_max_vel
        robot.lift.set_velocity(v_m, a_m=self.params['motion']['default']['accel_m'])
    
    def stop_motion(self, robot):
        robot.lift.set_velocity(0, a_m=self.params['motion']['max']['accel_m'])


    def _safety_check(self,v_m):
        # Do some safety checks and modify vel
        return v_m

    def _process_stick_to_vel(self, x):
        v_m = map_to_range(abs(x), 0, self.max_linear_vel)
        if x<0:
            v_m = -1*v_m
        return self._safety_check(v_m)
    
    def _step_precision_move(self,xv, robot):
        # Read the current joint position
        current_position = robot.lift.status['pos']

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
        robot.lift.move_to(self.start_pos + x_des)

        # Update the previous_time for the next iteration
        self._prev_set_vel_ts = time.time()

class CommandArm:
    def __init__(self):
        self.params = RobotParams().get_params()[1]['arm']
        self.dead_zone = 0.0001
        self._prev_set_vel_ts = None
        self.max_linear_vel = self.params['motion']['default']['vel_m']
        self.precision_mode = False
        self.acc = self.params['motion']['max']['accel_m']
        
        # Precision mode params
        self.start_pos = None
        self.target_position = self.start_pos
        self.precision_kp = 0.6 # Very Precise: 0.6
        self.precision_max_vel = 0.04 # m/s  Very Precise: 0.02 m/s

    def command_stick_to_motion(self, x, robot):
        if abs(x) < self.dead_zone:
            x = 0
        # x = to_parabola_transform(x)
        if not self.precision_mode:
        # Standard Mode
            self.start_pos = None
            v_m = self._process_stick_to_vel(x)
            robot.arm.set_velocity(v_m,a_m=self.acc)
            self._prev_set_vel_ts = time.time()
        else:
        # Precision Mode
            if self.start_pos is None:
                self.start_pos = robot.arm.status['pos']
                self.target_position = self.start_pos
                # TODO: Wait for the velocity to settle to zero
                
            r = self.precision_max_vel
            xv = map_to_range(abs(x),0,r)
            if x<0:
                xv = -1*xv
            self._step_precision_move(xv, robot)

    def command_button_to_motion(self, direction, robot):
        v_m = direction*self.max_linear_vel
        if self.precision_mode:
            v_m = direction*self.precision_max_vel
        robot.arm.set_velocity(v_m, a_m=self.params['motion']['default']['accel_m'])

    def stop_motion(self, robot):
        robot.arm.set_velocity(0, a_m=self.params['motion']['max']['accel_m'])

    def _safety_check(self,v_m):
        # Do some safety checks and modify vel
        return v_m

    def _process_stick_to_vel(self, x):
        v_m = map_to_range(abs(x), 0, self.max_linear_vel)
        if x<0:
            v_m = -1*v_m
        return self._safety_check(v_m)
    
    def _step_precision_move(self,xv, robot):
        # Read the current joint position
        current_position = robot.arm.status['pos']

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
            robot.arm.move_to(self.start_pos + x_des)

        # Update the previous_time for the next iteration
        self._prev_set_vel_ts = time.time()

class CommandDxlJoint:
    def __init__(self, name, max_vel=None, acc_type=None):
        self.params = RobotParams().get_params()[1][name]
        self.name = name
        self.dead_zone = 0.001
        self._prev_set_vel_ts = None
        self.max_vel = max_vel if max_vel else self.params['motion']['default']['vel']
        self.precision_mode = False
        self.acc = None
        if acc_type:
            self.acc = self.params['motion'][acc_type]['accel']
        
        self.precision_scale_down = 0.05

    def command_stick_to_motion(self, x, robot):
        if 'wrist' in self.name:
            motor = robot.end_of_arm.get_joint(self.name)
        if 'head' in self.name:
            motor = robot.head.get_joint(self.name)
        if abs(x)<self.dead_zone:
            x = 0
        acc = self.acc
        if x==0:
            acc = self.params['motion']['max']['accel'] #Stop with Strong Acceleration
        # x = to_parabola_transform(x)
        v = self._process_stick_to_vel(x)
        if self.precision_mode:
            v = v*self.precision_scale_down
        motor.set_velocity(v, acc)
        self._prev_set_vel_ts = time.time()

    def command_button_to_motion(self,direction, robot):
        if 'wrist' in self.name:
            motor = robot.end_of_arm.get_joint(self.name)
        if 'head' in self.name:
            motor = robot.head.get_joint(self.name)
        vel = self.max_vel
        if self.precision_mode:
            vel = vel*self.precision_scale_down
        if direction==1:
            motor.set_velocity(vel, self.acc)
        elif direction==-1:
            motor.set_velocity(-1*vel, self.acc)
        self._prev_set_vel_ts = time.time()
    
    def stop_motion(self, robot):
        if 'wrist' in self.name:
            motor = robot.end_of_arm.get_joint(self.name)
        if 'head' in self.name:
            motor = robot.head.get_joint(self.name)
        
        motor.set_velocity(0,self.params['motion']['max']['accel'])

    def _safety_check(self,v):
        # Do some safety checks and modify vel
        return v
    
    def _process_stick_to_vel(self, x):
        x = -1*x
        v = map_to_range(abs(x), 0, self.max_vel)
        if x<0:
            v = -1*v

        return self._safety_check(v)
            
class CommandGripperPosition:
    def __init__(self):
        self.name = 'stretch_gripper'
        self.params = RobotParams().get_params()[1][self.name]
        self.gripper_rotate_pct = 10.0
        self.gripper_accel = self.params['motion']['max']['accel']
        self.gripper_vel = self.params['motion']['max']['vel']
        self.precision_mode = False
    
    def open_gripper(self, robot):
        robot.end_of_arm.get_joint(self.name).move_by(self.gripper_rotate_pct, self.gripper_vel, self.gripper_accel)
        
    def close_gripper(self, robot):
        robot.end_of_arm.get_joint(self.name).move_by(-self.gripper_rotate_pct, self.gripper_vel, self.gripper_accel)

class GamePadTeleop(Device):
    def __init__(self, robot_instance = True, print_dongle_status = True, lock=None):
        Device.__init__(self, 'stretch_gamepad')
        self.gamepad_controller = gc.GamePadController(print_dongle_status=print_dongle_status)
        self.precision_mode = False
        self.fast_base_mode = False
        self.robot = None
        self._needs_robot_startup = robot_instance
        if robot_instance:
            self.robot = rb.Robot()
        self.controller_state = self.gamepad_controller.gamepad_state
        self.end_of_arm_tool = RobotParams().get_params()[1]['robot']['tool']
        self.sleep = 1/50
        self.print_mode = False
        self._i = 0
        
        self.fn_button_command = self.params['function_cmd'] # command to execute on pressing X(left button) for N seconds
        self.fn_button_detect_span = self.params['press_time_span'] #s
        
        self._last_fn_btn_press = None
        self._last_shutdwon_btn_press = None
            
        self.base_command = CommandBase()
        self.lift_command = CommandLift()
        self.arm_command = CommandArm()
        self.wirst_yaw_command = CommandDxlJoint('wrist_yaw', max_vel=1.5, acc_type='slow')
        self.head_pan_command = CommandDxlJoint('head_pan')
        self.head_tilt_command =  CommandDxlJoint('head_tilt', acc_type='slow')
        
        self.gripper = None
        self.wrist_pitch_command = None
        self.wrist_roll_command = None
        self.is_gamepad_dongle = False
        
        if not self.end_of_arm_tool == 'tool_stretch_gripper' and not self.end_of_arm_tool=='tool_none':
            self.gripper = CommandGripperPosition()
        if self.end_of_arm_tool == 'tool_stretch_dex_wrist':
            self.wrist_pitch_command = CommandDxlJoint('wrist_pitch', max_vel=1, acc_type='slow')
            self.wrist_roll_command = CommandDxlJoint('wrist_roll')
            
        print(f"Key mapped to End-Of-Arm Tool: {self.end_of_arm_tool}")
        self.lock = lock
        if not self.lock:
            self.lock = threading.Lock()
    
    def update_gamepad_state(self, robot=None):
        # Should be added in a loop to asycournously update the keypress inputs and fn action exec
        if self.robot:
            robot = self.robot
        with self.lock:
            self.controller_state = self.gamepad_controller.gamepad_state
            self.is_gamepad_dongle = self.gamepad_controller.is_gamepad_dongle
            self.fn_button_Action(robot)
        
    def _update_state(self, state = None):
        with self.lock:
            if state is None:
                self.controller_state = self.gamepad_controller.gamepad_state
            else:
                self.controller_state = state
        self.is_gamepad_dongle = self.gamepad_controller.is_gamepad_dongle
        self.precision_mode = self.controller_state['left_trigger_pulled'] > 0.9
        self.fast_base_mode = self.controller_state['right_trigger_pulled'] > 0.9
        
    def startup(self, robot = None):
        if self.robot:
            robot = self.robot
        self.gamepad_controller.setDaemon(True)
        self.gamepad_controller.start()
        if self.robot:
            if self.robot.startup():
                self.do_double_beep()
        else:
            self.do_double_beep(robot)
    
    def do_single_beep(self, robot=None):
        if self.robot:
            robot = self.robot
        robot.pimu.trigger_beep()
        robot.push_command()
          
    def do_double_beep(self, robot = None):
        if self.robot:
            robot = self.robot
        robot.pimu.trigger_beep()
        robot.push_command()
        time.sleep(0.5)
        robot.pimu.trigger_beep()
        robot.push_command()
        time.sleep(0.5)
    
    def do_four_beep(self, robot = None):
        if self.robot:
            robot = self.robot
        robot.pimu.trigger_beep()
        robot.push_command()
        time.sleep(0.5)
        robot.pimu.trigger_beep()
        robot.push_command()
        time.sleep(0.5)
        robot.pimu.trigger_beep()
        robot.push_command()
        time.sleep(0.5)
        robot.pimu.trigger_beep()
        robot.push_command()
        time.sleep(0.5)

    def command_joints(self, robot=None):
        if self.robot:
            robot = self.robot
        # Standard Key Mapping
        
        dxl_zero_vel_set_division_factor = 3 
        # Note: Coninuously commanding velocities to Dxls with ROS2 drivers causes the service callback thread unresponsive because it looks like 
        # dxl commands create a bottleneck holding the execution thread. This is solved by using a loop division factor that skips dxl commands on
        # consecutive loops avoiding the above situation.
        
        # Wrist Yaw Control        
        if self.controller_state['right_shoulder_button_pressed']:
            self.wirst_yaw_command.command_button_to_motion(-1,robot)
            
        elif self.controller_state['left_shoulder_button_pressed']:
            self.wirst_yaw_command.command_button_to_motion(1,robot)
        else:
            if self._i % dxl_zero_vel_set_division_factor == 0:
                self.wirst_yaw_command.stop_motion(robot)
        
        if not self.end_of_arm_tool == 'tool_stretch_dex_wrist' or (self.end_of_arm_tool == 'tool_stretch_dex_wrist' and self.controller_state['right_stick_button_pressed']):
            # Head Control
            if self.controller_state['top_pad_pressed']:
                self.head_tilt_command.command_button_to_motion(1,robot)
            elif self.controller_state['bottom_pad_pressed']:
                self.head_tilt_command.command_button_to_motion(-1,robot)
            else:
                if self._i % dxl_zero_vel_set_division_factor == 0:
                    self.head_tilt_command.stop_motion(robot)
            
            if self.controller_state['left_pad_pressed']:
                self.head_pan_command.command_button_to_motion(1,robot)
            elif self.controller_state['right_pad_pressed']:
                self.head_pan_command.command_button_to_motion(-1,robot)
            else:
                if self._i % dxl_zero_vel_set_division_factor == 0:
                    self.head_pan_command.stop_motion(robot)
                        
        elif self.end_of_arm_tool == 'tool_stretch_dex_wrist' and not self.controller_state['right_stick_button_pressed']:
            # Dex Wrist Control
            if self.controller_state['top_pad_pressed']:
                self.wrist_pitch_command.command_button_to_motion(1,robot)
            elif self.controller_state['bottom_pad_pressed']:
                self.wrist_pitch_command.command_button_to_motion(-1,robot)
            else:
                if self._i % dxl_zero_vel_set_division_factor == 0:
                    self.wrist_pitch_command.stop_motion(robot)
            
            if self.controller_state['left_pad_pressed']:
                self.wrist_roll_command.command_button_to_motion(-1,robot)
            elif self.controller_state['right_pad_pressed']:
                self.wrist_roll_command.command_button_to_motion(1,robot)
            else:
                if self._i % dxl_zero_vel_set_division_factor == 0:
                    self.wrist_roll_command.stop_motion(robot)
                
        self.arm_command.command_stick_to_motion(self.controller_state['right_stick_x'],robot)
        self.lift_command.command_stick_to_motion(self.controller_state['right_stick_y'],robot)
        self.base_command.command_stick_to_motion(self.controller_state['left_stick_x'],self.controller_state['left_stick_y'],robot)

        if self.gripper:
            if self.controller_state['right_button_pressed']:
                self.gripper.open_gripper(robot)
            elif self.controller_state['bottom_button_pressed']:
                self.gripper.close_gripper(robot)
                    
    def _update_modes(self):
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
        
    def do_motion(self, state = None, robot = None):
        # Should be added in a loop to do motion
        if not robot:
            robot = self.robot
        self._i = self._i + 1 
        self._update_state(state)
        self._update_modes()
        with self.lock:
            if not robot.is_calibrated() and self.controller_state['start_button_pressed']:
                robot.home()
            if robot.is_calibrated():
                if self.controller_state['top_button_pressed']:
                    self._manage_stow(robot)
                else:
                    if self.gamepad_controller.is_gamepad_dongle:
                        self.command_joints(robot)
                        self.manage_shutdown(robot)
                    else:
                        self._safety_stop(robot)
            else:   
                if self._i % 100 == 0: 
                    print('press the start button to calibrate the robot')
            self.fn_button_Action(robot)
            
    def fn_button_Action(self, robot):      
        if self.params['enable_fn_button']:
            if self.controller_state['left_button_pressed']:
                if not self._last_fn_btn_press:
                    self._last_fn_btn_press = time.time()
                if time.time() - self._last_fn_btn_press >= self.fn_button_detect_span:
                    self._last_fn_btn_press = None
                    click.secho(f"Executing Function command: {self.fn_button_command}", fg="green", bold=True)
                    self.do_four_beep(robot)
                    self.execute_fn_cmd()
            else:
                self._last_fn_btn_press = None
    
    def execute_fn_cmd(self):
        if self.fn_button_command:
            execute_command_non_blocking(self.fn_button_command)
    
    def _safety_stop(self, robot):
        self.wirst_yaw_command.command_stick_to_motion(0, robot)
        self.arm_command.command_stick_to_motion(0, robot)
        self.lift_command.command_stick_to_motion(0, robot)
        self.head_pan_command.command_stick_to_motion(0, robot)
        self.head_tilt_command.command_stick_to_motion(0, robot)
        self.base_command.command_stick_to_motion(0,0, robot)
        if self.end_of_arm_tool == 'tool_stretch_dex_wrist':
            self.wrist_pitch_command.command_stick_to_motion(0, robot)
            self.wrist_roll_command.command_stick_to_motion(0, robot)

    def _manage_stow(self, robot):
        if robot.is_calibrated():
            # Reset motion params as fast for xbox
            v = robot.end_of_arm.motors['wrist_yaw'].params['motion']['default']['vel']
            a = robot.end_of_arm.motors['wrist_yaw'].params['motion']['default']['accel']
            robot.end_of_arm.motors['wrist_yaw'].set_motion_params(v, a)
            robot.stow()
            self.do_single_beep(robot)
    
    def stop(self):
        if self._needs_robot_startup:
            self.robot.stop()
        if not self.gamepad_controller.stop_thread:
            self.gamepad_controller.shutdown_flag.set()
            self.gamepad_controller.join(1)
    
    def manage_shutdown(self, robot):
        if self.controller_state['select_button_pressed']:
            if not self._last_shutdwon_btn_press:
                self._last_shutdwon_btn_press = time.time()
            if time.time() - self._last_shutdwon_btn_press >= 2:
                print("Shutting Down the Robot...")
                self._last_shutdwon_btn_press = None
                robot.pimu.trigger_beep()
                robot.stow()
                self.gamepad_controller.stop()
                robot.stop()
                time.sleep(1.0)
                os.system(
                    'paplay --device=alsa_output.pci-0000_00_1f.3.analog-stereo /usr/share/sounds/ubuntu/stereo/desktop-logout.ogg')
                os.system('sudo shutdown now')  # sudoers should be set up to not need a password
        else:
            self._last_fn_btn_press = None



    def mainloop(self):
        try:
            while True:
                self.do_motion()
                self.robot.push_command()
                time.sleep(self.sleep)
        except (ThreadServiceExit, KeyboardInterrupt, SystemExit, UnpluggedError):
            self.gamepad_controller.stop()
            self.robot.stop()


def execute_command_non_blocking(command):
    try:
        # Use subprocess.Popen to start the command in a separate process
        process = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    except Exception as e:
        print(f"An error occurred: {e}")
        
if __name__ == "__main__":
   gamepad_teleop = GamePadTeleop()
   gamepad_teleop.startup()
   gamepad_teleop.mainloop()
