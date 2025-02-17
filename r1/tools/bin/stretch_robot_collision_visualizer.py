#!/usr/bin/env python3
import argparse
import math
import yaml
import pathlib
import urchin as urdf_loader
import pyrender
import warnings
import numpy as np
import stretch_body.robot
import stretch_body.hello_utils as hu

import random

hu.print_stretch_re_use()
warnings.filterwarnings("ignore")
import argparse



class CollisionVisualizer:
    """The `show` method in this class is modified from the
    original implementation of `urdf_loader.URDF.show`. This class
    exists temporarily while the PR for this modification is
    in review.
    """

    def __init__(self, robot):
        self.robot = robot
        self.nodes = None
        self.scene = None
        self.viewer = None


    def show(self):
        """
        Create initial setup of PyRender scene

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
        cfg=self.robot.collision.get_joint_configuration()
        fk = self.robot.collision.urdf.collision_trimesh_fk(cfg=cfg)

        #Build map from collision mesh to link name
        lfk = self.robot.collision.urdf.link_fk(cfg=cfg, links=None)
        link_to_link_name={ell: ell.name for ell in lfk}
        self.collision_mesh_2_link_name={}
        for link in lfk:
            if link.collision_mesh is not None:
                self.collision_mesh_2_link_name[link.collision_mesh]=link_to_link_name[link]

        self.scene = pyrender.Scene(ambient_light=np.array([0.02, 0.02, 0.02, 1.0]))
        camera =pyrender.PerspectiveCamera(yfov=np.pi / 4.0, aspectRatio=1.0)

        self.nodes = []
        self.node_link_names=[]

        for tm in fk:
            pose = fk[tm]
            mesh = pyrender.Mesh.from_trimesh(tm, smooth=False)#,wireframe=True)#
            mesh_node = self.scene.add(mesh, pose=pose)
            self.nodes.append(mesh_node)

        self.viewer = pyrender.Viewer(self.scene, run_in_thread=True, shadows=False,use_raymond_lighting=True,viewport_size=(1200,1200))#,all_wireframe=True)

    def update_pose(self):

        fk = self.robot.collision.urdf.collision_trimesh_fk(cfg=self.robot.collision.get_joint_configuration())
        self.viewer.render_lock.acquire()

        for i, tm in enumerate(fk):
            pose = fk[tm]
            self.scene.set_pose(self.nodes[i], pose=pose)

            #if not self.robot.collision.was_link_in_collsion(self.collision_mesh_2_link_name[tm]) and self.robot.collision.is_link_in_collsion(self.collision_mesh_2_link_name[tm]):
            if self.robot.collision.is_link_in_collsion(self.collision_mesh_2_link_name[tm]):
                #New collision, update the color by deleting old node and making a new one
                self.scene.remove_node(self.nodes[i])
                mesh = pyrender.Mesh.from_trimesh(tm, smooth=False)#,wireframe=False)
                mesh_node = self.scene.add(mesh, pose=pose)
                self.nodes[i]=mesh_node
                color = np.array([1.0, 0.0, 0.0])
                vertex_colors = np.broadcast_to(color, mesh_node.mesh.primitives[0].positions.shape)
                mesh_node.mesh.primitives[0].color_0 = vertex_colors
            else: #lif not self.robot.collision.is_link_in_collsion(self.collision_mesh_2_link_name[tm]):
            #elif self.robot.collision.was_link_in_collsion(self.collision_mesh_2_link_name[tm]) and not self.robot.collision.is_link_in_collsion(self.collision_mesh_2_link_name[tm]):
                #Dropped a collision. update the color by deleting old node and making a new one
                self.scene.remove_node(self.nodes[i])
                mesh = pyrender.Mesh.from_trimesh(tm, smooth=False)#,wireframe=True)
                mesh_node = self.scene.add(mesh, pose=pose)
                self.nodes[i] = mesh_node
                color = np.array([0.5, 0.5, 0.5])
                vertex_colors = np.broadcast_to(color, mesh_node.mesh.primitives[0].positions.shape)
                mesh_node.mesh.primitives[0].color_0 = vertex_colors

        self.viewer.render_lock.release()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Visualize Stretch collision system ')
    #parser.add_argument("--mesh", help="View actual mesh models", action="store_true")
    parser.add_argument('-g', "--gamepad", help="Use gamepad to control pose", action="store_true")
    args = parser.parse_args()

    r = stretch_body.robot.Robot()

    # if not r.params['use_collision_manager']:
    #     print('Collision manager not enabled in robot.params')
    #     exit(0)

    r.startup()
    if not r.is_homed():
        print('Warning. Visualization may be inaccurate because the robot has not been calibrated')
        # exit()
    r.enable_collision_mgmt()
    gamepad=None
    if args.gamepad:
        import stretch_body.gamepad_teleop
        gamepad = stretch_body.gamepad_teleop.GamePadTeleop(robot_instance = False)
        gamepad.startup(robot=r)


    viz = CollisionVisualizer(robot=r)
    viz.show()
    while viz.viewer.is_active:
        viz.update_pose()
        if gamepad:
            gamepad.step_mainloop(r)
    r.stop()
    if gamepad:
        gamepad.gamepad_controller.stop()
