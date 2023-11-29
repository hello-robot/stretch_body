#! /usr/bin/env python

from stretch_body.device import Device
import urchin as urdf_loader
import meshio
import numpy as np
import importlib.resources as importlib_resources
import time
from audioplayer import AudioPlayer

# #######################################################################


class CollisionLink:
    def __init__(self,link_name,urdf,mesh_path,max_mesh_points):
        self.name=link_name
        self.link = urdf.link_map[link_name]
        stl_filename = str(mesh_path) + self.link.collisions[0].geometry.mesh.filename[1:]
        self.mesh = meshio.read(stl_filename)
        pts = self.mesh.points
        self.points = np.hstack((pts, np.ones([pts.shape[0], 1], dtype=np.float32)))  # One extend to Nx4 array
        self.in_collision= False
        self.was_in_collision = False
        self.is_xyz_cube=self.check_xyz_cube(self.points)
        self.is_valid=True
        if pts.shape[0] > max_mesh_points:
            print('Incorrect size of points for link:', link_name, pts.shape)
            print('Ignoring collision link %s' % link_name)
            self.is_valid=False
        self.pose=None

    def set_pose(self,p):
        self.pose=p

    def pretty_print(self):
        print('-- CollisionLink %s --'%self.name)
        print('XYZ Cube',self.is_xyz_cube)
        print('Is Valid', self.is_valid)
        print('In collision',self.in_collision)
        print('Was in collision',self.was_in_collision)
        print('Mesh size',self.points.shape)

    def check_xyz_cube(self,pts):
        """
        Check if points are aligned (roughly) to xyz and form a cube

        Parameters
        ----------
        pts of mesh (8x4)

        Returns
        -------
        True / False
        """
        # Check that each x,y,z of each point has two nearly (eps) idential values
        x = pts[:, 0]
        y = pts[:, 1]
        z = pts[:, 2]
        eps = .001  # meters
        for ridx in range(8):
            sx = sum(abs((x - x[ridx])) < eps)
            sy = sum(abs((y - y[ridx])) < eps)
            sz = sum(abs((y - y[ridx])) < eps)
            if sx + sy + sz != 12:
                return False
        return True

class CollisionPair:
    def __init__(self, link_pts,link_cube,motion_dir,joint_name):
        self.in_collision=False
        self.was_in_collision=False
        self.link_cube=link_cube
        self.link_pts=link_pts
        self.motion_dir=motion_dir
        self.joint_name=joint_name
        self.name_id='J_%s_LP_%s_LC_%s_DR_%s'%(joint_name,link_pts.name,link_cube.name,motion_dir)
        self.is_valid=self.link_cube.is_valid and self.link_pts.is_valid and self.link_cube.is_xyz_cube
        if not self.is_valid:
            print('Dropping monitor of collision pair %s'%self.name_id)

    def pretty_print(self):
        print('Collision pair: %s'%self.name_id)
        print('Is Valid',self.is_valid)
        print('--Link Cube--')
        self.link_cube.pretty_print()
        print('--Link Pts--')
        self.link_pts.pretty_print()
    def get_name_id(self):
        return self.name_id

class CollisionJoint:
    def __init__(self, joint_name,motor):
        self.name=joint_name
        self.motor=motor
        self.active_collisions=[]
        self.collision_pairs=[]
        self.in_collision={'pos':False,'neg':False}
        self.was_in_collision = {'pos': False, 'neg': False}
    def add_collision_pair(self,cp):
        self.collision_pairs.append(cp)
    def pretty_print(self):
        print('-------Collision Joint: %s-----------------'%self.name)
        for cp in self.collision_pairs:
            cp.pretty_print()
        print('--Active Collisions--')
        print(self.active_collisions)
        for ac in self.active_collisions:
            print('Active Collision: %s' % ac)


class RobotCollisionMgmt(Device):
    def __init__(self,robot):
        """
        RobotCollisionMgmt monitors for collisions between links.
        It utilizes the Collision mesh for collision estimation.
        Given the Cartesian structure of Stretch we simplify the collision detection in order to achieve real-time speeds.
        We simplify the problem by assuming:
        * One of the collision meshes ("cube") is a cube that is aligned with gravity (XYZ)
        * The other collision mesh ("pts") is simple shape of just a few points (eg, a cube, trapezoid, etc)<max_mesh_points

        The params define which links we want to monitor collisions between.
        Each link includes a parameter "scale_pct" which allows the mesh size to be expanded by a percentage around its centroid
        enabling the ability to increase the safety zone.
        """
        Device.__init__(self, name='robot_collision_mgmt')
        self.robot = robot
        self.collision_joints = {}
        self.collision_links = {}
        self.collision_pairs = {}
        self.beep=AudioPlayer('/usr/share/sounds/ubuntu/stereo/message.ogg')
        self.running=True

    def pretty_print(self):
        for j in self.collision_joints:
            self.collision_joints[j].pretty_print()

    def enable(self):
        self.running=True

    def disable(self):
        self.running=False
    def startup(self,threaded=False):
        Device.startup(self, threaded=False)
        pkg = str(importlib_resources.files("stretch_urdf"))  # .local/lib/python3.10/site-packages/stretch_urdf)
        model_name = self.robot.params['model_name']
        tool_name = self.robot.params['tool']
        urdf_name = pkg + '/%s/stretch_description_%s_%s.urdf' % (model_name, model_name, tool_name)
        mesh_path = pkg + '/%s/' % (model_name)

        try:
            self.urdf = urdf_loader.URDF.load(urdf_name)
        except ValueError:
            print('Unable to load URDF: %s' % urdf_name)
            self.urdf = None
            return

        #Build dict of potential collisions_pairs for each joint
        #Include those of standard robot body plus its defined tool
        # EG collision_joints={'lift':[{collision_1},{collision_2...}],'head_pan':[...]}
        cj=self.params[model_name]
        for tt in self.params[tool_name]:
            if tt in cj:
                cj[tt]+=self.params[tool_name][tt]
            else:
                cj[tt]=self.params[tool_name][tt]

        for joint_name in cj:
            self.collision_joints[joint_name]=CollisionJoint(joint_name,self.get_joint_motor(joint_name))
            for cp in cj[joint_name]: #eg cp={'motion_dir': 'neg', 'link_pts': 'link_lift', 'link_cube': 'base_link'}
                if cp['link_pts'] not in self.collision_links:
                    self.collision_links[cp['link_pts']]=CollisionLink(cp['link_pts'],self.urdf,mesh_path,self.params['max_mesh_points'])
                if cp['link_cube'] not in self.collision_links:
                    self.collision_links[cp['link_cube']]=CollisionLink(cp['link_cube'],self.urdf,mesh_path,self.params['max_mesh_points'])
                p=CollisionPair(self.collision_links[cp['link_pts']],self.collision_links[cp['link_cube']],cp['motion_dir'],joint_name)
                self.collision_pairs[p.get_name_id()]=p
                self.collision_joints[joint_name].add_collision_pair(p)

    def get_joint_motor(self,joint_name):
        if joint_name=='lift':
            return self.robot.lift
        if joint_name=='arm':
            return self.robot.arm
        if joint_name=='wrist_yaw':
            return self.robot.end_of_arm.get_joint('wrist_yaw')
        if joint_name=='head_pan':
            return self.robot.head.get_joint('head_pan')
        if joint_name=='head_tilt':
            return self.robot.head.get_joint('head_tilt')
        #Try the tool
        return self.robot.end_of_arm.get_joint(joint_name)


    def step(self,cfg=None):
        """
                Check for interference between cube pairs
        """
        if self.urdf is None or not self.running:
            return

        if cfg is None:
            cfg = self.get_joint_configuration(braked=True)#_braked()

        # Update forward kinematics of links
        lfk = self.urdf.link_fk(cfg=cfg, links=self.collision_links.keys(), use_names=True)

        # Update poses of links based on fk
        for link_name in lfk:
            self.collision_links[link_name].set_pose(lfk[link_name].dot(
                self.collision_links[link_name].points.transpose()).transpose())

        # Reset each link / joint status before updating
        for link_name in self.collision_links:
            self.collision_links[link_name].was_in_collision =self.collision_links[link_name].in_collision
            self.collision_links[link_name].in_collision=False

        for joint_name in self.collision_joints:
            self.collision_joints[joint_name].active_collisions=[]
            self.collision_joints[joint_name].was_in_collision = self.collision_joints[joint_name].in_collision.copy()
            self.collision_joints[joint_name].in_collision = {'pos': False, 'neg': False}

        # Test for collision between cube pairs
        for pair_name in self.collision_pairs:
            cp=self.collision_pairs[pair_name]
            if cp.is_valid:
                cp.was_in_collision=cp.in_collision
                cp.in_collision=self.check_pts_in_cube(cube=cp.link_cube.pose,pts=cp.link_pts.pose)
                if cp.in_collision:
                    cj = self.collision_joints[cp.joint_name]
                    cj.active_collisions.append(cp.get_name_id()) #Add collision to joint
                    cj.in_collision[cp.motion_dir] = True
                #Propogate to links
                self.collision_links[cp.link_cube.name].in_collision=self.collision_links[cp.link_cube.name].in_collision or cp.in_collision
                self.collision_links[cp.link_pts.name].in_collision =self.collision_links[cp.link_pts.name].in_collision or cp.in_collision

        #Update collision avoidance
        for joint_name in self.collision_joints:
            self.collision_joints[joint_name].motor.step_collision_avoidance(self.collision_joints[joint_name].in_collision)

        #Beep on new collision
        #Note: subtle issue. Playing the beep can take a variable amount of time depending on sys resources
        #Call it after we do the step_collision_avoidance so that the stop controller can be triggered as close to
        #Collision detection as possible (otherwise the longer time can allow a collision in worst case conditions)
        for pair_name in self.collision_pairs:
            if not self.collision_pairs[pair_name].was_in_collision and self.collision_pairs[pair_name].in_collision:
                self.beep.play(block=False)

    def is_link_in_collsion(self,link_name):
        if self.urdf is None:
            return False
        try:
            return self.collision_links[link_name].in_collision
        except KeyError: #Not all links will be monitored
            return False

    def was_link_in_collsion(self,link_name):
        if self.urdf is None:
            return False
        try:
            return self.collision_links[link_name].was_in_collision
        except KeyError: #Not all links will be monitored
            return False

    def check_pts_in_cube(self,cube, pts):
        """
        Check if any of the 'from' points lie inside the 'into' cube
        Parameters
        ----------
        cube_into: Array of points of cube (8x4)
        pts_from: Array of points to check

        Returns
        -------
        True/False
        """
        xmax = max(cube[:, 0])
        xmin = min(cube[:, 0])
        ymax = max(cube[:, 1])
        ymin = min(cube[:, 1])
        zmax = max(cube[:, 2])
        zmin = min(cube[:, 2])
        for p in pts:
            inside = p[0] <= xmax and p[0] >= xmin and p[1] <= ymax and p[1] >= ymin and p[2] <= zmax and p[2] >= zmin
            if inside:
                return True
        return False

    def get_joint_configuration(self,braked=False):
        """
        Construct a dictionary of robot's current pose
        """
        s = self.robot.get_status()

        if braked:
            da=self.params['k_brake_distance']['arm']*self.robot.arm.get_braking_distance()/4.0
            dl=self.params['k_brake_distance']['lift']*self.robot.lift.get_braking_distance()
            dwy=self.params['k_brake_distance']['wrist_yaw']*self.robot.end_of_arm.get_joint('wrist_yaw').get_braking_distance()
            dhp = self.params['k_brake_distance']['head_pan'] * self.robot.head.get_joint('head_pan').get_braking_distance()
            dht = self.params['k_brake_distance']['head_tilt'] * self.robot.head.get_joint('head_tilt').get_braking_distance()
        else:
            da=0.0
            dl=0.0
            dwy=0.0
            dhp=0.0
            dht=0.0

        configuration = {
            'joint_left_wheel': 0.0,
            'joint_right_wheel': 0.0,
            'joint_lift': dl+s['lift']['pos'],
            'joint_arm_l0': da+s['arm']['pos']/4.0,
            'joint_arm_l1': da+s['arm']['pos']/4.0,
            'joint_arm_l2': da+s['arm']['pos']/4.0,
            'joint_arm_l3': da+s['arm']['pos']/4.0,
            'joint_wrist_yaw': dwy+s['end_of_arm']['wrist_yaw']['pos'],
            'joint_head_pan': dhp+s['head']['head_pan']['pos'],
            'joint_head_tilt': dht+s['head']['head_tilt']['pos']
            }



        configuration.update(self.robot.end_of_arm.get_joint_configuration(braked))
        return configuration


class RobotCollision(Device):
    """
        Deprecated. Keep shell class (for now) to avoid breaking legacy code.
    """
    def __init__(self,robot):
        print('-----------')
        print('RobotCollision has been deprecated.')
        print('Use RobotCollisionMgmt instead')
        print('See KB post forum.hello-robot.com/xxx')
        print('-----------')

    def startup(self):
        pass

    def enable_model(self,name):
        pass

    def disable_model(self,name):
        pass


    def step(self):
        pass

