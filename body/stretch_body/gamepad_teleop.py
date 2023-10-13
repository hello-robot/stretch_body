#!/usr/bin/env python3
from __future__ import print_function
import stretch_body.gamepad_controller as gc
from stretch_body.device import Device
from inputs import UnpluggedError
import stretch_body.robot as rb
from stretch_body.hello_utils import *
from stretch_body.robot_params import RobotParams
from stretch_body.gamepad_command_groups import *
import os
import time
import threading
import sys
import click
import numpy as np
import subprocess

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
        
    def _update_state(self, state = None):
        with self.lock:
            if state is None:
                self.controller_state = self.gamepad_controller.gamepad_state
            else:
                self.controller_state = state
        self.is_gamepad_dongle = self.gamepad_controller.is_gamepad_dongle
        
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

    def command_robot_joints(self, robot):
        """
        The gamepad stick/buttons states are mapped to robot motion in this method. 
        A user must modify this method to create cutom gamepad key mapping.
        The following buttons are pre-assigned to high priority robot functions and should not be used:
        
        START/start_button_pressed - Press to home robot if not calibrated
        BACK/select_button_pressed - Long press to trigger a shutdown
        MDDLE RING BUTTON/middle_led_ring_button_pressed - Controller hardware specific
        """
        
        if self.controller_state['top_button_pressed']:
            self.stow_robot(robot)

        # Set control modes flags
        self.precision_mode = self.controller_state['left_trigger_pulled'] > 0.9
        self.fast_base_mode = self.controller_state['right_trigger_pulled'] > 0.9

        dxl_zero_vel_set_division_factor = 3 
        # Note: Coninuously commanding velocities to chained Dxls above 15 Hz might cause thread blocking issues while used in multithreaded executors (E.g. ROS2). 
        # Using a division factor to downscale rate.

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
        
        self.manage_shutdown(robot) # Stows the robot and performs a PC shutdown when the Back/SELECT_BUTTON is long pressed for 2s. Comment to turn off
        self.manage_fn_button(robot) # Executes the command assigned to the function_cmd param when button X/LEFT_BUTTON is pressed for defined duration. Comment to turn off
                    
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
                if self.gamepad_controller.is_gamepad_dongle:
                    self.command_robot_joints(robot)
                else:
                    self._safety_stop(robot)
            else:   
                if self._i % 100 == 0: 
                    print('press the start button to calibrate the robot')
            
    def manage_fn_button(self, robot):    
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

    def stow_robot(self, robot):
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
