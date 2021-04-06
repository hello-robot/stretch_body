#!/usr/bin/env python
from __future__ import print_function
import stretch_body.xbox_controller as xc
import stretch_body.robot as rb
from stretch_body.hello_utils import *
import os
import time
import argparse
print_stretch_re_use()

parser=argparse.ArgumentParser(description=
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
    '-------------------------------------\n',formatter_class=argparse.RawTextHelpFormatter)
parser.add_argument("--enable_autostart", help="Enable running of controller on boot",action="store_true")
parser.add_argument("--disable_autostart", help="Disable running of controller on boot",action="store_true")
args=parser.parse_args()

if args.disable_autostart:
    if os.path.isfile(os.environ['HOME'] + '/.config/autostart/hello_robot_xbox_teleop.desktop'):
        os.system('rm '+ os.environ['HOME']+'/.config/autostart/hello_robot_xbox_teleop.desktop')
        print('Autostart disabled...')
    else:
        print('Autostart already disabled...')
    exit()

if args.enable_autostart:
    if os.path.isfile('/etc/hello-robot/hello_robot_xbox_teleop.desktop'):
        os.system('cp /etc/hello-robot/hello_robot_xbox_teleop.desktop '+ os.environ['HOME']+'/.config/autostart/')
        print('Autostart enabled...')
    else:
        print('Autostart file is missing. Unable to enable...')
    exit()


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
        assert self.accel_rad <= 4.0*10, 'WARNING: CommandToRotaryMotion.__init__ accel_rad = abs({0}) > 4.0 rad/s^2, which is high.'.format(
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



def main():
    ##################
    # Setup Xbox controller
    xbox_controller = xc.XboxController()
    xbox_controller.start()

    ############################
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

    ############################
    # Fast motion for navigation with the mobile base

    fast_move_s = 0.6
    fast_max_dist_m = 0.12
    fast_accel_m = 0.8

    # fast, but unstable on thresholds: 0.6 s, 0.15 m, 0.8 m/s^2

    fast_command_to_linear_motion = CommandToLinearMotion(dead_zone, fast_move_s, fast_max_dist_m, fast_accel_m)
    fast_move_s = 0.2
    fast_max_dist_rad = 0.6
    fast_accel_rad = 0.8

    fast_command_to_rotary_motion = CommandToRotaryMotion(dead_zone, fast_move_s, fast_max_dist_rad, fast_accel_rad)
    ############################

    head_stopped = False

    stop_loop = False
    
    robot = rb.Robot()

    robot.startup()
    robot_calibrated = robot.is_calibrated()

    robot.pimu.trigger_beep()
    robot.push_command()
    time.sleep(0.5)

    robot.pimu.trigger_beep()
    robot.push_command()
    time.sleep(0.5)

    last_time=time.time()
    t_last=time.time()
    itr=0
    shutdown_pc=False
    ts_shutdown_start=0
    first_home_warn=True
    wrist_yaw_target=0.0
    head_pan_target=0.0
    head_tilt_target=0.0

    while not stop_loop:
        try:
            ##################
            #xbox_controller.update()
            # get latest controller state
            controller_state = xbox_controller.get_state()

            ##################
            # convert controller state to new robot commands
            calibrate_the_robot = controller_state['start_button_pressed']

            if controller_state['select_button_pressed']:
                if not shutdown_pc:
                    ts_shutdown_start = time.time()
                    shutdown_pc=True
                if time.time()-ts_shutdown_start > 2.0:
                    robot.pimu.trigger_beep()
                    robot.stow()
                    robot.stop()
                    time.sleep(1.0)
                    os.system('paplay --device=alsa_output.pci-0000_00_1f.3.analog-stereo /usr/share/sounds/ubuntu/stereo/desktop-logout.ogg')
                    os.system('sudo shutdown now') #sudoers should be set up to not need a password
            else:
                shutdown_pc=False

            #run_stop = controller_state['middle_led_ring_button_pressed']
            #run_stop = controller_state['select_button_pressed']
            #if run_stop:
            #    print('runstop event trigger not yet implemented')
                # robot.pimu.runstop_event_trigger()
            forward_command = controller_state['left_stick_y']
            turn_command = controller_state['left_stick_x']
            lift_command = controller_state['right_stick_y']
            arm_command = controller_state['right_stick_x']

            wrist_yaw_left = controller_state['left_shoulder_button_pressed']
            wrist_yaw_right = controller_state['right_shoulder_button_pressed']

            # controller_state['right_trigger_pulled']
            close_gripper = controller_state['bottom_button_pressed']
            open_gripper = controller_state['right_button_pressed']
            stow_robot = controller_state['top_button_pressed']

            #turn_on_the_relay = controller_state['left_button_pressed']

            camera_pan_right = controller_state['right_pad_pressed']
            camera_pan_left = controller_state['left_pad_pressed']
            camera_tilt_up = controller_state['top_pad_pressed']
            camera_tilt_down = controller_state['bottom_pad_pressed']

            lift_height_m = robot.lift.status['pos']
            arm_extension_m = robot.arm.status['pos']
            if robot.end_of_arm is not None:
                wrist_yaw_rad = robot.end_of_arm.motors['wrist_yaw'].status['pos']
            else:
                wrist_yaw_rad=0
            print_joints = False
            if print_joints:
                print('lift_height_m =', lift_height_m)
                print('arm_extension_m =', arm_extension_m)
                print('wrist_yaw_rad =', wrist_yaw_rad)

            # FAST NAVIGATION MODE
            fast_navigation_mode = False
            navigation_mode_trigger = controller_state['right_trigger_pulled']
            if (navigation_mode_trigger > 0.5):
                fast_navigation_mode = True


            ##################
            # convert robot commands to robot movement
            # only allow a pure translation or a pure rotation command
            if abs(forward_command) > abs(turn_command):
                if abs(forward_command) > dead_zone:
                    output_sign = math.copysign(1, forward_command)
                    if not fast_navigation_mode:
                        d_m, v_m, a_m = command_to_linear_motion.get_dist_vel_accel(output_sign, forward_command)
                    else:
                        d_m, v_m, a_m = fast_command_to_linear_motion.get_dist_vel_accel(output_sign, forward_command)
                    #print('Translate %f %.2f %.2f %.2f'%(time.time(),d_m,v_m,a_m))
                    tt=time.time()
                    #print('DT %f',tt-last_time)
                    last_time=tt
                    robot.base.translate_by(d_m, v_m, a_m)
            else:
                if abs(turn_command) > dead_zone:
                    output_sign = -math.copysign(1, turn_command)
                    if not fast_navigation_mode:
                        d_rad, v_rad, a_rad = command_to_rotary_motion.get_dist_vel_accel(output_sign, turn_command)
                    else:
                        d_rad, v_rad, a_rad = fast_command_to_rotary_motion.get_dist_vel_accel(output_sign, turn_command)
                    #print('DR', d_rad,'J',turn_command)
                    robot.base.rotate_by(d_rad, v_rad, a_rad)
            # if (abs(forward_command) < dead_zone) and (abs(turn_command) < dead_zone):
            # robot.base.translate_by(0.0)
            # robot.base.rotate_by(0.0)


            if abs(lift_command) > dead_zone:
                output_sign = math.copysign(1, lift_command)
                d_m, v_m, a_m = command_to_linear_motion.get_dist_vel_accel(output_sign, lift_command)
                robot.lift.move_by(d_m, v_m, a_m)

            if abs(arm_command) > dead_zone:
                output_sign = math.copysign(1, arm_command)
                d_m, v_m, a_m = command_to_linear_motion.get_dist_vel_accel(output_sign, arm_command)
                robot.arm.move_by(d_m, v_m, a_m)


            if robot.end_of_arm is not None:
                wrist_rotate_rad = deg_to_rad(25.0)  # 60.0 #10.0 #5.0
                wrist_accel = 15.0  # 25.0 #30.0 #15.0 #8.0 #15.0
                wrist_vel = 25.0  # 10.0 #6.0 #3.0 #6.0
                wrist_slew_down=0.15
                wrist_slew_up=.07
                #Slew target to zero if no buttons pushed
                if not wrist_yaw_left and not wrist_yaw_right:
                    if wrist_yaw_target>=0:
                        wrist_yaw_target=max(0,wrist_yaw_target-wrist_slew_down)
                    else:
                        wrist_yaw_target=min(0,wrist_yaw_target+wrist_slew_down)
                #Or slew up to max
                if wrist_yaw_left:
                    wrist_yaw_target=min(wrist_yaw_target+wrist_slew_up,wrist_rotate_rad)
                if wrist_yaw_right:
                    wrist_yaw_target =max(wrist_yaw_target-wrist_slew_up,-wrist_rotate_rad)
                robot.end_of_arm.move_by('wrist_yaw',wrist_yaw_target, wrist_vel, wrist_accel)


            head_pan_rad = deg_to_rad(40.0)  # 25.0 #40.0
            head_pan_accel = 14.0  # 15.0
            head_pan_vel = 7.0  # 6.0
            head_pan_slew_down = 0.15
            head_pan_slew_up = .15
            # Slew target to zero if no buttons pushed
            if not camera_pan_left and not camera_pan_right:
                if head_pan_target >= 0:
                    head_pan_target = max(0, head_pan_target - head_pan_slew_down)
                else:
                    head_pan_target = min(0, head_pan_target + head_pan_slew_down)
            # Or slew up to max
            if camera_pan_right:
                head_pan_target = max(head_pan_target - head_pan_slew_up, -head_pan_rad)
            if camera_pan_left:
                head_pan_target = min(head_pan_target + head_pan_slew_up, head_pan_rad)
            robot.head.move_by('head_pan', head_pan_target, head_pan_vel, head_pan_accel)

            # Head tilt
            head_tilt_rad = deg_to_rad(40.0)  # 25.0 #40.0
            head_tilt_accel = 14.0  # 12.0  # 15.0
            head_tilt_vel = 7.0  # 10.0  # 6.0
            head_tilt_slew_down = 0.15
            head_tilt_slew_up = .15
            # Slew target to zero if no buttons pushed
            if not camera_tilt_up and not camera_tilt_down:
                if head_tilt_target >= 0:
                    head_tilt_target = max(0, head_tilt_target - head_tilt_slew_down)
                else:
                    head_tilt_target = min(0, head_tilt_target + head_tilt_slew_down)
            # Or slew up to max
            if camera_tilt_down:
                head_tilt_target = max(head_tilt_target - head_tilt_slew_up, -head_tilt_rad)
            if camera_tilt_up:
                head_tilt_target = min(head_tilt_target + head_tilt_slew_up, head_tilt_rad)
            robot.head.move_by('head_tilt', head_tilt_target, head_tilt_vel, head_tilt_accel)


            if robot.end_of_arm is not None and robot.end_of_arm.is_tool_present('StretchGripper'):
                gripper_rotate_pct = 10.0
                gripper_accel = robot.end_of_arm.motors['stretch_gripper'].params['motion']['max']['accel']
                gripper_vel = robot.end_of_arm.motors['stretch_gripper'].params['motion']['max']['vel']


                if open_gripper:
                    robot.end_of_arm.move_by('stretch_gripper', gripper_rotate_pct, gripper_vel, gripper_accel)
                elif close_gripper:
                    robot.end_of_arm.move_by('stretch_gripper', -gripper_rotate_pct, gripper_vel, gripper_accel)

            ##################
            # execute robot's motor movements
            if stow_robot and robot_calibrated:
                #Reset motion params as fast for xbox
                v=robot.end_of_arm.motors['wrist_yaw'].params['motion']['default']['vel']
                a=robot.end_of_arm.motors['wrist_yaw'].params['motion']['default']['accel']
                robot.end_of_arm.motors['wrist_yaw'].set_motion_params(v,a)
                robot.stow()
            else:
                if robot_calibrated:
                    robot.push_command()
                elif calibrate_the_robot:
                    print('begin calibrating the robot')
                    robot.home()
                    print('finished calibrating the robot')
                    robot_calibrated = True
                else:
                    if first_home_warn:
                        print('press the start button to calibrate the robot')
                    else:
                        first_home_warn=False

            time.sleep(0.05)

        except (ThreadServiceExit, KeyboardInterrupt, SystemExit):
            stop_loop = True
            xbox_controller.stop()
            robot.stop()



if __name__ == "__main__":
    main()
