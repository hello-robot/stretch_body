#!/usr/bin/env python3
from __future__ import print_function
import stretch_body.gamepad_controller as gc
import stretch_body.robot as rb
from stretch_body.hello_utils import *
import os
import time
import argparse
import random
import math


print_stretch_re_use()

parser = argparse.ArgumentParser(description=
                                 'Jog the robot from an XBox Controller  \n' +
                                 '-------------------------------------\n' +
                                 'Left Stick X:\t Rotate base \n' +
                                 'Left Stick Y:\t Translate base \n' +
                                 'Right Trigger:\t Fast base motion \n' +
                                 'Right Stick X:\t Translate arm \n' +
                                 'Right Stick Y:\t Translate lift \n' +
                                 'Left Button:\t Rotate wrist CCW \n' +
                                 'Right Button:\t Rotate wrist CW \n' +
                                 'A/B Buttons:\t Close/Open gripper \n' +
                                 'Left/Right Pad:\t Head Pan \n' +
                                 'Top/Bottom Pad:\t Head tilt \n' +
                                 'Y Button :\t Go to stow position \n ' +
                                 'Start Button:\t Home robot \n ' +
                                 'Back Button (2 sec):\t Shutdown computer \n ' +
                                 '-------------------------------------\n',
                                 formatter_class=argparse.RawTextHelpFormatter)

args = parser.parse_args()


class CommandToLinearMotion():
    def __init__(self, command_dead_zone_value, move_duration_s, max_distance_m, accel_m):
        # This expects
        # a command value with a magnitude between 0.0 and 1.0, inclusive
        # a dead_zone with a magnitude greater than or equal to 0.0 and less than 1.0

        self.dead_zone = abs(command_dead_zone_value)
        self.move_duration_s = abs(move_duration_s)
        self.max_distance_m = abs(max_distance_m)
        self.accel_m = abs(accel_m)

        # check that the values are reasonable
        assert self.dead_zone >= 0.0
        assert self.dead_zone <= 0.9, 'WARNING: CommandToLinearMotion.__init__ command_dead_zone_value is strangely large command_dead_zone_value = abs({0}) > 0.9.'.format(
            command_dead_zone_value)
        assert self.move_duration_s > 0.01, 'WARNING: CommandToLinearMotion.__init__ move_duration_s = abs({0}) <= 0.01 seconds, which is a short time for a single move.'.format(
            move_duration_s)
        assert self.move_duration_s <= 1.0, 'WARNING: CommandToLinearMotion.__init__ move_duration_s = abs({0}) > 1.0 seconds, which is a long time for a single move.'.format(
            move_duration_s)
        assert self.max_distance_m <= 0.3, 'WARNING: CommandToLinearMotion.__init__ max_distance_m = abs({0}) > 0.3 meters, which is a long distance for a single move.'.format(
            max_distance_m)
        assert self.accel_m <= 30.0, 'WARNING: CommandToLinearMotion.__init__ accel_m = abs({0}) > 30.0 m/s^2, which is very high (> 3 g).'.format(
            accel_m)

    def get_dist_vel_accel(self, output_sign, command_value):
        # Larger commands attempt to move over larger distances in the
        # same amount of time by moving at higher velocities.
        c_val = abs(command_value)

        assert c_val <= 1.0, 'ERROR: CommandToLinearMotion.get_dist_vel_accel given command value > 1.0, command_value = {0}'.format(
            command_value)
        assert c_val > self.dead_zone, 'ERROR: CommandToLinearMotion.get_dist_vel_accel the command should not be executed due to its value being within the dead zone: abs(command_value) = abs({0}) <= {1} = self.dead_zone'.format(
            command_value, self.dead_zone)
        if 1:
            scale = (c_val - self.dead_zone) / (1.0 - self.dead_zone) ** 2
        else:
            scale = c_val - self.dead_zone
        d_m = (scale * (self.max_distance_m / (1.0 - self.dead_zone)))
        d_m = math.copysign(d_m, output_sign)
        v_m = d_m / self.move_duration_s  # average m/s for a move of distance d_m to last for time move_s
        a_m = self.accel_m
        return d_m, v_m, a_m


class CommandToRotaryMotion():
    def __init__(self, command_dead_zone_value, move_duration_s, max_angle_rad, accel_rad):
        # This expects
        # a command value with a magnitude between 0.0 and 1.0, inclusive
        # a dead_zone with a magnitude greater than or equal to 0.0 and less than 1.0

        self.dead_zone = abs(command_dead_zone_value)
        self.move_duration_s = abs(move_duration_s)
        self.max_angle_rad = abs(max_angle_rad)
        self.accel_rad = abs(accel_rad)

        # check that the values are reasonable
        assert self.dead_zone >= 0.0
        assert self.dead_zone <= 0.9, 'WARNING: CommandToRotaryMotion.__init__ command_dead_zone_value is strangely large command_dead_zone_value = abs({0}) > 0.9.'.format(
            command_dead_zone_value)
        assert self.move_duration_s > 0.01, 'WARNING: CommandToRotaryMotion.__init__ move_duration_s = abs({0}) <= 0.01 second, which is a short time for a single move.'.format(
            move_duration_s)
        assert self.move_duration_s <= 1.0, 'WARNING: CommandToRotaryMotion.__init__ move_duration_s = abs({0}) > 1.0 second, which is a long time for a single move.'.format(
            move_duration_s)
        assert self.max_angle_rad <= 0.7, 'WARNING: CommandToRotaryMotion.__init__ max_angle_rad = abs({0}) > 0.7 , which is a large angle for a single move (~40.0 deg).'.format(
            max_angle_rad)
        assert self.accel_rad <= 4.0 * 10, 'WARNING: CommandToRotaryMotion.__init__ accel_rad = abs({0}) > 4.0 rad/s^2, which is high.'.format(
            accel_rad)

    def get_dist_vel_accel(self, output_sign, command_value):
        # Larger commands attempt to move over larger distances in the
        # same amount of time by moving at higher velocities.
        c_val = abs(command_value)
        assert c_val <= 1.0, 'ERROR: CommandToRotaryMotion.get_dist_vel_accel given command value > 1.0, command_value = {0}'.format(
            command_value)
        assert c_val > self.dead_zone, 'ERROR: CommandToRotaryMotion.get_dist_vel_accel the command should not be executed due to its value being within the dead zone: abs(command_value) = abs({0}) <= {1} = self.dead_zone'.format(
            command_value, self.dead_zone)

        scale = c_val - self.dead_zone
        d_r = (scale * (self.max_angle_rad / (1.0 - self.dead_zone)))
        d_r = math.copysign(d_r, output_sign)
        v_r = d_r / self.move_duration_s  # average m/s for a move of distance d_m to last for time move_s
        a_r = self.accel_rad
        return d_r, v_r, a_r

# ######################### BASE ########################################
# Regular Motion
dead_zone = 0.1  # 0.25 #0.1 #0.2 #0.3 #0.4
move_s = 0.6
max_dist_m = 0.06  # 0.04 #0.05
accel_m = 0.2  # 0.1
command_to_linear_motion = CommandToLinearMotion(dead_zone, move_s, max_dist_m, accel_m)

move_s = 0.05
max_dist_rad = 0.10  # 0.2 #0.25 #0.1 #0.09
accel_rad = 0.8  # 0.05
command_to_rotary_motion = CommandToRotaryMotion(dead_zone, move_s, max_dist_rad, accel_rad)
############################
# Fast Motion
fast_move_s = 0.6
fast_max_dist_m = 0.12
fast_accel_m = 0.8
# fast, but unstable on thresholds: 0.6 s, 0.15 m, 0.8 m/s^2

fast_command_to_linear_motion = CommandToLinearMotion(dead_zone, fast_move_s, fast_max_dist_m, fast_accel_m)
fast_move_s = 0.2
fast_max_dist_rad = 0.6
fast_accel_rad = 0.8
fast_command_to_rotary_motion = CommandToRotaryMotion(dead_zone, fast_move_s, fast_max_dist_rad, fast_accel_rad)


loop_itr=0
last_dir='x'
def manage_base(b,controller_state):
    global loop_itr, last_dir

    xy_scale = 0.5  # 0.5
    w_scale = 3.0
    a_xy_des = 0.75
    a_w_des = 6.0

    if controller_state['right_trigger_pulled']>0.5: #slow mode
        scale=0.25
        xy_scale=xy_scale*scale
        w_scale = w_scale * scale
        a_xy_des=a_xy_des*scale
        a_w_des=a_w_des*scale


    loop_itr=loop_itr+1
    #print('----------------- %d ----------------'%loop_itr)
    side_command = controller_state['left_stick_x']
    forward_command = controller_state['left_stick_y']
    rotate_command = controller_state['right_stick_x']
    if abs(side_command)<dead_zone:
        side_command=0
    if abs(forward_command)<dead_zone:
        forward_command=0
    if abs(rotate_command)<dead_zone:
        rotate_command=0

    go_forward= abs(forward_command)>abs(side_command) and abs(forward_command)>abs(rotate_command)
    go_side = abs(side_command) > abs(forward_command) and abs(side_command) > abs(rotate_command)
    go_rotate = abs(rotate_command) > abs(side_command) and abs(rotate_command) > abs(forward_command)

    #Note: Bug as we're hacking the handling of accel/decel
    #If the below is dir 'y' then one wheel will keep spinning as deccel u0=0
    #Keep as x until we handle motion planning correctly
    if not go_forward and not go_side and not go_rotate:
        if last_dir=='w':
            b.set_omni_velocity(dir=last_dir, v_des=0, a_des=a_w_des)
        else:
            b.set_omni_velocity(dir=last_dir, v_des=0, a_des=a_xy_des)


    if go_forward:# and abs(forward_command) > dead_zone:
        #print("Forward",forward_command)
        b.set_omni_velocity(dir='y', v_des=xy_scale*forward_command,a_des=a_xy_des)
        last_dir='y'

    if go_side:
            #print("Side",side_command)
            b.set_omni_velocity(dir='x', v_des=xy_scale*side_command,a_des=a_xy_des)
            last_dir='x'

    if go_rotate:# and abs(rotate_command) > dead_zone:
        #print('Rotate',rotate_command)
        b.set_omni_velocity(dir='w', v_des=w_scale*rotate_command,a_des=a_w_des)
        last_dir='w'

    #
    # if abs(side_command) > dead_zone:
    #     ay=a_scale*side_command
    # if abs(rotate_command) > dead_zone:
    #     w=w_scale*rotate_command
    # b.set_omni_velocity(ax,ay,w,a_des)


# ######################### SHUTDOWN  ########################################
shutdown_pc = False
ts_shutdown_start = 0


def manage_shutdown(robot, controller_state):
    global shutdown_pc, ts_shutdown_start
    if controller_state['select_button_pressed']:
        if not shutdown_pc:
            ts_shutdown_start = time.time()
            shutdown_pc = True
        if time.time() - ts_shutdown_start > 2.0:
            robot.pimu.trigger_beep()
            robot.stow()
            robot.stop()
            time.sleep(1.0)
            os.system(
                'paplay --device=alsa_output.pci-0000_00_1f.3.analog-stereo /usr/share/sounds/ubuntu/stereo/desktop-logout.ogg')
            os.system('sudo shutdown now')  # sudoers should be set up to not need a password
    else:
        shutdown_pc = False


########################### Check and wait for USB Devices ###########################

def check_usb_devices(wait_timeout=5):
    hello_devices = ['hello-omni-1',
                     'hello-omni-2',
                     'hello-omni-3']

    print('Waiting for all the hello* devices ...')
    all_found = True
    for dev in hello_devices:
        if not wait_till_usb(dev, wait_timeout):
            all_found = False
    if all_found:
        print('Found all hello* devices.')
    return all_found


def wait_till_usb(usb, wait_timeout):
    s_ts = time.time()
    while time.time() - s_ts <= wait_timeout:
        devices = os.listdir('/dev')
        hello_devs = [dev for dev in devices if 'hello' in dev]
        if usb in hello_devs:
            return True
    print('{} device not found.'.format(usb))
    return False


def do_double_beep(robot):
    robot.pimu.trigger_beep()
    robot.push_command()
    time.sleep(0.5)
    robot.pimu.trigger_beep()
    robot.push_command()
    time.sleep(0.5)

# ######################### MAIN ########################################
from stretch_body.omni_base import OmniBase
from stretch_body.pimu import Pimu
def main():
    xbox_controller = gc.GamePadController()
    xbox_controller.start()
    check_usb_devices(wait_timeout=5)
    b = OmniBase()
    b.startup()
    p=Pimu()
    p.startup()
    try:
        while True:
            controller_state = xbox_controller.get_state()
            manage_base(b,controller_state)
            b.push_command()
            p.trigger_motor_sync()
            p.push_command()
            time.sleep(0.05)
    except (ThreadServiceExit, KeyboardInterrupt, SystemExit):
        b.stop()
        p.stop()
        xbox_controller.stop()


if __name__ == "__main__":
    main()
