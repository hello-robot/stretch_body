#!/usr/bin/env python3
from __future__ import print_function
import stretch_body.gamepad_controller as gc
from stretch_body.device import Device
from inputs import UnpluggedError
import stretch_body.robot as rb
from stretch_body.hello_utils import *
from stretch_body.robot_params import RobotParams
from stretch_body.dynamixel_hello_XL430 import DynamixelCommError
from stretch_body import gamepad_joints
import os
import time
import threading
import sys
import click
import numpy as np
import subprocess
import threading

"""
The GamePadTeleop runs the Stretch's main gamepad controller that ships with 
the robot. The GamePadController is used to listen to the gamepad's inputs 
(button presses,analog stick, trigger) and convert them into robot motions
using the gamepad_joints library's motion command classes.

The default gamepad controller key mappings can be customized by overriding the
GamePadTeleop.command_robot_joints() method. 

Aditionally this class provides other robot function through the gamepad to be 
customized such as manage_shutdown(), manage_fn_button() and setting precision_mode.
"""

class GamePadTeleop(Device):
    def __init__(self, robot_instance = True, print_dongle_status = True, lock=None, collision_mgmt=False):
        """
        Main controller for Stretch's gamepad that ships with the robot.

        Args:
            robot_instance (bool, optional): Set True if a robot instance is required within the class and not supplied externally.
            print_dongle_status (bool, optional): Print Dongle status when not plugged into.
            lock (_thread.lock, optional): Pass on lock object to be used while calling robot instance methods.
        """
        Device.__init__(self, 'stretch_gamepad')
        self.gamepad_controller = gc.GamePadController(print_dongle_status=print_dongle_status)
        self.precision_mode = False
        self.fast_base_mode = False
        self.robot = None
        self.collision_mgmt=collision_mgmt
        self._needs_robot_startup = robot_instance
        if robot_instance:
            self.robot = rb.Robot()
        self.controller_state = self.gamepad_controller.gamepad_state

        self.end_of_arm_tool =RobotParams().get_params()[1]['robot']['tool']

        self.sleep = 1/15
        self.print_mode = False
        self._i = 0
        
        self.fn_button_command = self.params['function_cmd'] # command to execute on pressing X(left button) for N seconds
        self.fn_button_detect_span = self.params['press_time_span'] #s
        
        self._last_fn_btn_press = None
        self._last_shutdwon_btn_press = None
            
        self.base_command = gamepad_joints.CommandBase()
        self.lift_command = gamepad_joints.CommandLift()
        self.arm_command = gamepad_joints.CommandArm()
        self.wirst_yaw_command = gamepad_joints.CommandWristYaw()
        self.head_pan_command = gamepad_joints.CommandHeadPan()
        self.head_tilt_command =  gamepad_joints.CommandHeadTilt()
        
        self.gripper = None
        self.wrist_pitch_command = None
        self.wrist_roll_command = None
        self.is_gamepad_dongle = False
        
        if self.using_stretch_gripper():
            self.gripper = gamepad_joints.CommandGripperPosition()
        if self.using_dexwrist():
            self.wrist_pitch_command = gamepad_joints.CommandWristPitch()
            self.wrist_roll_command = gamepad_joints.CommandWristRoll()
            
        print(f"Key mapped to End-Of-Arm Tool: {self.end_of_arm_tool}")
        self.lock = lock
        if not self.lock:
            self.lock = threading.Lock()
        
        self.dexwrist_ctrl_switch = True
        self.skip_x_button = False

        self.left_stick_button_fn = None
        self.right_stick_button_fn = None

        self.currently_stowing = False

    def using_stretch_gripper(self):
        return self.end_of_arm_tool == 'tool_stretch_dex_wrist' or self.end_of_arm_tool == 'eoa_wrist_dw3_tool_sg3' \
            or self.end_of_arm_tool == 'tool_stretch_gripper'
    def using_dexwrist(self):
        return self.end_of_arm_tool == 'tool_stretch_dex_wrist' or self.end_of_arm_tool == 'eoa_wrist_dw3_tool_sg3' \
            or self.end_of_arm_tool == 'eoa_wrist_dw3_tool_nil' or self.end_of_arm_tool == 'eoa_wrist_dw3_tool_tablet_12in'

    def command_robot_joints(self, robot):
        """
        Override this method create custom gamepad mappings.

        The gamepad stick/buttons states are mapped to robot motion in this method. 
        A user must modify this method to create cutom gamepad key mapping.

        The following buttons are pre-assigned to high priority robot functions and should not be used:
        START/start_button_pressed - Press to home robot if not calibrated
        BACK/select_button_pressed - Long press to trigger a shutdown
        MDDLE RING BUTTON/middle_led_ring_button_pressed - Controller hardware specific controls
        X/left_button_pressed - Function command exection button

        Parameters
        ----------
        robot : robot.Robot 
            Valid robot instance
        """

        # Set control modes flags
        self.precision_mode = self.controller_state['left_trigger_pulled'] > 0.9
        self.fast_base_mode = self.controller_state['right_trigger_pulled'] > 0.9 # specific to base motion 

        dxl_zero_vel_set_division_factor = 3 
        # Note: Coninuously commanding stop_motion()(set zero velocities) to chained Dxls above 15 Hz might cause thread blocking issues 
        # while used in multithreaded executors (E.g. ROS2). So using a division factor to downscale the stop_motion() call rate.

        # Wrist Yaw Control        
        if self.controller_state['right_shoulder_button_pressed']:
            self.wirst_yaw_command.command_button_to_motion(-1,robot)
            
        elif self.controller_state['left_shoulder_button_pressed']:
            self.wirst_yaw_command.command_button_to_motion(1,robot)
        else:
            if self._i % dxl_zero_vel_set_division_factor == 0:
                self.wirst_yaw_command.stop_motion(robot)
        
        if not self.using_dexwrist() or (self.using_dexwrist() and not self.dexwrist_ctrl_switch):
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

            if self.using_dexwrist():
                self.wrist_pitch_command.stop_motion(robot)
                self.wrist_roll_command.stop_motion(robot)

        elif self.using_dexwrist() and self.dexwrist_ctrl_switch:
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

            self.head_pan_command.stop_motion(robot)
            self.head_tilt_command.stop_motion(robot)

                
        self.arm_command.command_stick_to_motion(self.controller_state['right_stick_x'],robot)
        self.lift_command.command_stick_to_motion(self.controller_state['right_stick_y'],robot)
        self.base_command.command_stick_to_motion(self.controller_state['left_stick_x'],self.controller_state['left_stick_y'],robot)

        if self.gripper:
            if self.controller_state['right_button_pressed']:
                self.gripper.open_gripper(robot)
            elif self.controller_state['bottom_button_pressed']:
                self.gripper.close_gripper(robot)
            else:
                self.gripper.stop_gripper(robot)


        # Switches the D-Pad control to DexWrist or Head on X/left_button press
        # `dexwrist_ctrl_switch` and `skip_x_button`  are convinience booleans for holding the toggled D-Pad info
        if self.controller_state['left_button_pressed'] and not self.skip_x_button:
            if not self.dexwrist_ctrl_switch:
                self.dexwrist_ctrl_switch = True
                print("Switch D-Pad to DexWrist Control")
                self.do_single_beep(robot)
                self.skip_x_button = True
            else:
                self.dexwrist_ctrl_switch = False
                print("Switch D-Pad to Head Control")
                self.do_single_beep(robot)
                self.skip_x_button = True
            self.time_since_dexwrist_switch = time.perf_counter()
        # skip x button press for 1 second after a toggle 
        if self.skip_x_button:
            if (time.perf_counter() - self.time_since_dexwrist_switch) > 0.5:
                self.skip_x_button = False

            

        self.manage_robot_stow(robot,self.controller_state['top_button_pressed']) # Stow the robot on Y/top_button long 2s press
        self.manage_shutdown(robot) # Stows the robot and performs a PC shutdown when the Back/SELECT_BUTTON is long pressed for 2s. Comment to turn off

        # Optional custom function button feature / Recommended to use with a non-confliction button key
        # Executes the command assigned to the function_cmd param when the given button key is pressed for a defined duration.
        # self.manage_fn_button(robot,self.controller_state['left_button_pressed']) 
        self.manage_left_stick_fn_button(self.controller_state['left_stick_button_pressed'])
        self.manage_right_stick_fn_button(self.controller_state['right_stick_button_pressed'])

    def do_motion(self, state = None, robot = None):
        """
        This method should called in the control loop (mainloop())
    
        Parameters
        ----------
        state : Dict
            Override the gamepad controller state providing custom state, Checkout method GamePadController.get_state()
        robot : robot.Robot 
            Valid robot instance
        """
        if not robot:
            robot = self.robot
        self._i = self._i + 1 
        self._update_state(state)
        self._update_modes()
        with self.lock:
            if not robot.is_homed() and self.controller_state['start_button_pressed']:
                self.do_single_beep(robot)
                robot.home()
            if robot.is_homed() and not self.currently_stowing:
                if self.gamepad_controller.is_gamepad_dongle or state:
                    self.command_robot_joints(robot)
                else:
                    self._safety_stop(robot)
            else:   
                if self._i % 100 == 0 and not self.currently_stowing: 
                    print('press the start button to calibrate the robot')

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
        """Start the gamepad controller thread and robot thread if required.

        Args:
            robot (robot.Robot, optional): Valid robot instance if required.
        """
        if self.robot:
            robot = self.robot
        self.gamepad_controller.setDaemon(True)
        self.gamepad_controller.start()
        if self.robot:
            if self.robot.startup():
                self.do_double_beep()
        else:
            self.do_double_beep(robot)
        if  self.collision_mgmt:
            robot.enable_collision_mgmt()
    
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
                    
    def _update_modes(self):
        self.arm_command.precision_mode = self.precision_mode
        self.lift_command.precision_mode = self.precision_mode
        self.base_command.precision_mode = self.precision_mode
        self.base_command.fast_base_mode = self.fast_base_mode
        self.wirst_yaw_command.precision_mode = self.precision_mode
        if self.gripper:
            self.gripper.precision_mode = self.precision_mode
        if self.using_dexwrist():
            self.wrist_pitch_command.precision_mode = self.precision_mode
            self.wrist_roll_command.precision_mode = self.precision_mode
        self.head_pan_command.precision_mode = self.precision_mode
        self.head_tilt_command.precision_mode = self.precision_mode
            
    def manage_robot_stow(self, robot, button_state):
        """Switch the D-Pad between DexWrist and Head Control by a 2s button press
        """    
        if button_state:
            if not self._last_fn_btn_press:
                self._last_fn_btn_press = time.time()

            if time.time() - self._last_fn_btn_press >= 2 and not self.currently_stowing:
                self.do_single_beep(robot)
                self._last_fn_btn_press = None
                threading.Thread(target=self.stow_robot,args=(robot,),daemon=True).start()
                # self.stow_robot(robot)
        else:
            self._last_fn_btn_press = None

    def manage_left_stick_fn_button(self, button_state):
        """Trigger custom user function for left stick
        """
        if self.left_stick_button_fn == None:
            return

        if button_state:
            if not self._last_left_stick_fn_btn_press:
                self._last_left_stick_fn_btn_press = time.time()

            if time.time() - self._last_left_stick_fn_btn_press >= self.fn_button_detect_span:
                click.secho("Executing Left Stick Custom Function", fg="green", bold=True)
                self.left_stick_button_fn()
                self._last_left_stick_fn_btn_press = None
        else:
            self._last_left_stick_fn_btn_press = None

    def manage_right_stick_fn_button(self, button_state):
        """Trigger custom user function for right stick
        """
        if self.right_stick_button_fn == None:
            return

        if button_state:
            if not self._last_right_stick_fn_btn_press:
                self._last_right_stick_fn_btn_press = time.time()

            if time.time() - self._last_right_stick_fn_btn_press >= self.fn_button_detect_span:
                click.secho("Executing right Stick Custom Function", fg="green", bold=True)
                self.right_stick_button_fn()
                self._last_right_stick_fn_btn_press = None
        else:
            self._last_right_stick_fn_btn_press = None

    def manage_fn_button(self, robot, button_state):
        """Detect function button press
        """    
        if self.params['enable_fn_button']: 
            if button_state:
                if not self._last_fn_btn_press:
                    self._last_fn_btn_press = time.time()

                if time.time() - self._last_fn_btn_press >= self.fn_button_detect_span:
                    self._last_fn_btn_press = None
                    click.secho(f"Executing Function command: {self.fn_button_command}", fg="green", bold=True)
                    self.do_four_beep(robot)
                    self._execute_fn_cmd()
            else:
                self._last_fn_btn_press = None
    
    def _execute_fn_cmd(self):
        if self.fn_button_command:
            execute_command_non_blocking(self.fn_button_command)
    
    def _safety_stop(self, robot):
        self.wirst_yaw_command.command_stick_to_motion(0, robot)
        self.arm_command.command_stick_to_motion(0, robot)
        self.lift_command.command_stick_to_motion(0, robot)
        self.head_pan_command.command_stick_to_motion(0, robot)
        self.head_tilt_command.command_stick_to_motion(0, robot)
        self.base_command.command_stick_to_motion(0,0, robot)
        if self.using_dexwrist():
            self.wrist_pitch_command.command_stick_to_motion(0, robot)
            self.wrist_roll_command.command_stick_to_motion(0, robot)

    def stow_robot(self, robot):
        if robot.is_homed():
            # Reset motion params as fast for xbox
            self.currently_stowing = True
            v = robot.end_of_arm.motors['wrist_yaw'].params['motion']['default']['vel']
            a = robot.end_of_arm.motors['wrist_yaw'].params['motion']['default']['accel']
            robot.end_of_arm.motors['wrist_yaw'].set_motion_params(v, a)
            robot.stow()
            self.do_single_beep(robot)
            self.currently_stowing = False
    
    def stop(self):
        if self._needs_robot_startup:
            self.robot.stop()
        if not self.gamepad_controller.stop_thread:
            self.gamepad_controller.shutdown_flag.set()
            self.gamepad_controller.join(1)
    
    def manage_shutdown(self, robot):
        """Detect shutdown button press (Button select)

        Args:
            robot (robot.Robot): Valid robot instance.
        """
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

    def step_mainloop(self,robot=None):
        if not robot:
            robot = self.robot
        self.do_motion(robot=robot)
        robot.push_command()
        time.sleep(self.sleep)

    def mainloop(self):
        """Run the main control loop
        """
        try:
            while True:
                self.step_mainloop()
        except (ThreadServiceExit, KeyboardInterrupt, SystemExit, UnpluggedError, DynamixelCommError):
            self.gamepad_controller.stop()
            self.robot.stop()

def execute_command_non_blocking(command):
    try:
        # Use subprocess.Popen to start the command in a separate process that wont get killed
        # when the main self process is killed
        process = subprocess.Popen(
            command,
            stdin=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            preexec_fn=os.setpgrp  # Detach the child process from the parent
        )
        
        # Optionally, you can save the process ID (PID) for later management if needed
        with open("/tmp/gamepad_fn_command_process.pid", "w") as pid_file:
            print(f"Process PID ID saved to `/tmp/gamepad_fn_command_process.pid`")
            pid_file.write(str(process.pid))

    except Exception as e:
        print(f"An error occurred: {e}")
        
if __name__ == "__main__":
   gamepad_teleop = GamePadTeleop()
   gamepad_teleop.startup()
   gamepad_teleop.mainloop()
