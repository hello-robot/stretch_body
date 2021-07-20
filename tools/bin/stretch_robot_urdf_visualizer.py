#!/usr/bin/env python
import argparse
import math
import yaml
import pathlib
import urdfpy
import pyrender
import warnings

from hello_helpers.gripper_conversion import GripperConversion
import stretch_body.robot
import stretch_body.hello_utils as hu
hu.print_stretch_re_use()
warnings.filterwarnings("ignore")

class URDFVisualizer:
    """The `show` method in this class is modified from the
    original implementation of `urdfpy.URDF.show`. This class
    exists temporarily while the PR for this modification is
    in review.
    """

    def __init__(self, urdf):
        self.urdf = urdf
        self.nodes = None
        self.scene = None
        self.viewer = None

    def show(self, cfg=None, use_collision=False):
        """Visualize the URDF in a given configuration.
        Parameters
        ----------
        cfg : dict or (n), float
            A map from joints or joint names to configuration values for
            each joint, or a list containing a value for each actuated joint
            in sorted order from the base link.
            If not specified, all joints are assumed to be in their default
            configurations.
        use_collision : bool
            If True, the collision geometry is visualized instead of
            the visual geometry.
        """
        if use_collision:
            fk = self.urdf.collision_trimesh_fk(cfg=cfg)
        else:
            fk = self.urdf.visual_trimesh_fk(cfg=cfg)

        self.scene = pyrender.Scene()
        self.nodes = []
        for tm in fk:
            pose = fk[tm]
            mesh = pyrender.Mesh.from_trimesh(tm, smooth=False)
            mesh_node = self.scene.add(mesh, pose=pose)
            self.nodes.append(mesh_node)
        self.viewer = pyrender.Viewer(self.scene, run_in_thread=True, use_raymond_lighting=True)

    def update_pose(self, cfg=None, use_collision=False):
        if use_collision:
            fk = self.urdf.collision_trimesh_fk(cfg=cfg)
        else:
            fk = self.urdf.visual_trimesh_fk(cfg=cfg)

        self.viewer.render_lock.acquire()
        for i, tm in enumerate(fk):
            pose = fk[tm]
            self.scene.set_pose(self.nodes[i], pose=pose)
        self.viewer.render_lock.release()

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
        _, gripper_finger_rad, _, _ = self.gripper_conversion.status_to_all(gripper_status)

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


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Python based URDF visualization')
    parser.add_argument('-t', "--trajectory", help="Visualize predefined trajectory", action="store_true")
    parser.add_argument('-p', "--pose", help="Visualize predefined pose", action="store_true")
    parser.add_argument('-c', "--collision", help="Use collision meshes", action="store_true")
    args = parser.parse_args()

    calibration_dir = pathlib.Path(hu.get_fleet_directory()) / 'exported_urdf'
    urdf_path = calibration_dir / 'stretch.urdf'
    controller_params_path = calibration_dir / 'controller_calibration_head.yaml'
    urdf = urdfpy.URDF.load(str(urdf_path.absolute()))
    viz = URDFVisualizer(urdf)
    with open(str(controller_params_path.absolute()), 'r') as f:
        controller_params = yaml.load(f, Loader=yaml.FullLoader)

    if args.trajectory:
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
        urdf.animate(cfg_trajectory=cfg_trajectory, use_collision=args.collision)
    elif args.pose:
        cfg_pose = {
            'joint_left_wheel': 0.0,
            'joint_right_wheel': 0.0,
            'joint_lift': 0.6,
            'joint_arm_l0': 0.1,
            'joint_arm_l1': 0.1,
            'joint_arm_l2': 0.1,
            'joint_arm_l3': 0.1,
            'joint_wrist_yaw': math.pi,
            'joint_gripper_finger_left': 0.0,
            'joint_gripper_finger_right': 0.0,
            'joint_head_pan': -(math.pi / 2.0),
            'joint_head_tilt': -0.5
        }
        urdf.show(cfg=cfg_pose, use_collision=args.collision)
    else:
        r = stretch_body.robot.Robot()
        r.startup()
        if not r.is_calibrated():
            print('Exiting because the robot has not been calibrated')
            exit()

        stretch_state = StretchState(r, controller_params)
        viz.show(cfg=stretch_state.get_urdf_configuration(), use_collision=args.collision)
        while viz.viewer.is_active:
            viz.update_pose(cfg=stretch_state.get_urdf_configuration(), use_collision=args.collision)
        r.stop()
