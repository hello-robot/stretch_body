#!/usr/bin/env python3


from urdfpy import URDF
import argparse
import os
import math
import yaml

parser=argparse.ArgumentParser(description='Python based URDF visualization')
parser.add_argument("--motion", help="Turn motor on",action="store_true")
parser.add_argument("--fake", help="Show fake robot",action="store_true")
args=parser.parse_args()

urdf_file=os.environ['HELLO_FLEET_PATH']+'/'+os.environ['HELLO_FLEET_ID']+'/exported_urdf/stretch.urdf'
controller_parameters_filename =os.environ['HELLO_FLEET_PATH']+'/'+os.environ['HELLO_FLEET_ID']+'/exported_urdf/controller_calibration_head.yaml'

########################################################################################
### The classes below demonstrate how the robot mechanisms are modeled relative to the URDF
### Currently the stretch_body package relies on Python2.7 and the visualization tool Python3
### For now we use FakeRobot as a placeholder until stretch_body moves to Python3

#import stretch_body.robot as rb
class FakeRobot:
    def __init__(self):
        pass

    def startup(self):
        pass

    def is_calibrated(self):
        return True

    def stop(self):
        pass

    def get_status(self):
        return {'arm': {'pos': 0.4},
                'lift': {'pos': 0.4},
                'end_of_arm': {'wrist_yaw': {'pos': 0.0},
                               'stretch_gripper': {'pos': 0.0}},
                'head': {'head_pan': {'pos': 0.0},
                         'head_tilt': {'pos': 0.0}}
                }


class GripperConversion:
    def __init__(self):
        # robotis position values (gripper.py)
        #      0 is very close to closed (fingers almost or barely touching)
        #     50 is maximally open
        #   -100 is maximally closed (maximum force applied to the object - might be risky for a large object)
        #
        # aperture is ~12.5 cm wide when open (0.125 m, 125 mm)

        self.finger_length_m = 0.17

        self.open_aperture_m = 0.125
        self.closed_aperture_m = 0.0

        # 0.52 rad ~= 30 deg
        self.open_gripper_rad = 0.52
        self.closed_gripper_rad = 0.0

        self.open_robotis = 5.0
        self.closed_robotis = 0.0
        self.max_grip_force_robotis = -1.0

        self.robotis_to_aperture_slope = (
                    (self.open_aperture_m - self.closed_aperture_m) / (self.open_robotis - self.closed_robotis))

    def robotis_to_aperture(self, robotis_in):
        # linear model
        aperture_m = (self.robotis_to_aperture_slope * (robotis_in - self.closed_robotis)) + self.closed_aperture_m
        return aperture_m

    def aperture_to_robotis(self, aperture_m):
        # linear model
        robotis_out = ((aperture_m - self.closed_aperture_m) / self.robotis_to_aperture_slope) + self.closed_robotis
        return robotis_out

    def aperture_to_finger_rad(self, aperture_m):
        # arc length / radius = ang_rad
        finger_rad = (aperture_m / 2.0) / self.finger_length_m
        return finger_rad

    def finger_rad_to_aperture(self, finger_rad):
        aperture_m = 2.0 * (finger_rad * self.finger_length_m)
        return aperture_m

    def finger_to_robotis(self, finger_ang_rad):
        aperture_m = self.finger_rad_to_aperture(finger_ang_rad)
        robotis_out = self.aperture_to_robotis(aperture_m)
        return robotis_out

    def status_to_all(self, gripper_status):
        aperture_m = self.robotis_to_aperture(gripper_status['pos'])
        finger_rad = self.aperture_to_finger_rad(aperture_m)
        return aperture_m, finger_rad


class StretchState:
    def __init__(self, robot, controller_parameters):
        self.stretch = robot
        self.controller_parameters = controller_parameters.copy()
        self.gripper_conversion = GripperConversion()

    def get_urdf_configuration(self, backlash_state=None):
        if backlash_state is None:
            # use default backlash state
            backlash_state = {'wrist_extension_retracted': False,
                              'head_pan_looked_left': False}

        stretch_status = self.stretch.get_status()

        # set positions of the telescoping joints
        arm_status = stretch_status['arm']
        if backlash_state['wrist_extension_retracted']:
            arm_backlash_correction = self.controller_parameters['arm_retracted_offset']
        else:
            arm_backlash_correction = 0.0
        arm_m = arm_status['pos'] + arm_backlash_correction
        telescoping_link_m = arm_m / 4.0

        lift_status = stretch_status['lift']
        lift_m = lift_status['pos']

        wrist_yaw_status = stretch_status['end_of_arm']['wrist_yaw']
        wrist_yaw_rad = wrist_yaw_status['pos']

        gripper_status = stretch_status['end_of_arm']['stretch_gripper']
        gripper_aperture_m, gripper_finger_rad = self.gripper_conversion.status_to_all(gripper_status)

        head_pan_status = stretch_status['head']['head_pan']
        if backlash_state['head_pan_looked_left']:
            pan_backlash_correction = self.controller_parameters['pan_looked_left_offset']
        else:
            pan_backlash_correction = 0.0
        head_pan_rad = head_pan_status['pos'] + self.controller_parameters['pan_angle_offset'] + pan_backlash_correction

        head_tilt_status = stretch_status['head']['head_tilt']
        # ignores tilt backlash state
        raw_head_tilt_rad = head_tilt_status['pos']
        if raw_head_tilt_rad > self.controller_parameters['tilt_angle_backlash_transition']:
            tilt_backlash_correction = self.controller_parameters['tilt_looking_up_offset']
        else:
            tilt_backlash_correction = 0.0
        head_tilt_rad = head_tilt_status['pos'] + self.controller_parameters[
            'tilt_angle_offset'] + tilt_backlash_correction

        configuration = {
            'joint_left_wheel': 0.0,
            'joint_right_wheel': 0.0,
            'joint_lift': lift_m,
            'joint_arm_l0': telescoping_link_m,
            'joint_arm_l1': telescoping_link_m,
            'joint_arm_l2': telescoping_link_m,
            'joint_arm_l3': telescoping_link_m,
            'joint_wrist_yaw': wrist_yaw_rad,
            'joint_gripper_finger_left': gripper_finger_rad,
            'joint_gripper_finger_right': gripper_finger_rad,
            'joint_head_pan': head_pan_rad,
            'joint_head_tilt': head_tilt_rad
        }

        return configuration

# #######################################################################################


robot_model = URDF.load(urdf_file)

if args.motion:
    cfg_trajectory = {
        'joint_left_wheel': [0.0, math.pi],
        'joint_right_wheel': [0.0, math.pi],
        'joint_lift': [0.9, 0.7],
        'joint_arm_l0': [0.0, 0.1],
        'joint_arm_l1': [0.0, 0.1],
        'joint_arm_l2': [0.0, 0.1],
        'joint_arm_l3': [0.0, 0.1],
        'joint_wrist_yaw': [math.pi, 0.0],
        'joint_gripper_finger_left': [0.0, 0.25],
        'joint_gripper_finger_right': [0.0, 0.25],
        'joint_head_pan': [0.0, -(math.pi / 2.0)],
        'joint_head_tilt': [0.5, -0.5]
    }
    robot_model.animate(cfg_trajectory=cfg_trajectory)
elif args.fake:
    # stretch = rb.Robot()
    print('WARNING: Not actually visualizing the current state of the robot. Instead, using FakeRobot, since stretch_body.robot does not yet support Python 3.')
    stretch = FakeRobot()
    stretch.startup()
    stretch_calibrated = stretch.is_calibrated()
    if not stretch_calibrated:
        print('Exiting because the robot has not been calibrated')
        exit()

    fid = open(controller_parameters_filename, 'r')
    controller_parameters = yaml.load(fid)
    fid.close()
    stretch_state = StretchState(stretch, controller_parameters)
    cfg = stretch_state.get_urdf_configuration()
    robot_model.show(cfg=cfg)
    stretch.stop()
else:
    robot_model.show()


