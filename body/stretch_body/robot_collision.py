#! /usr/bin/env python

from stretch_body.device import Device
import urchin as urdf_loader
import meshio
import numpy as np
import time
import threading
import chime

try:
    # works on ubunut 22.04
    import importlib.resources as importlib_resources
    str(importlib_resources.files("stretch_body"))
except AttributeError as e:
    # works on ubuntu 20.04
    import importlib_resources
    str(importlib_resources.files("stretch_body"))

# #######################################################################

def line_box_sat_intersection(line_point1, line_point2, aabb_min, aabb_max):
    """
    Checks if a line segment intersects an AABB using the Separating Axis Theorem (SAT).

    Args:
        line_point1: np.array of shape (3,). First point of the line segment.
        line_point2: np.array of shape (3,). Second point of the line segment.
        aabb_min: np.array of shape (3,). Minimum corner of the AABB.
        aabb_max: np.array of shape (3,). Maximum corner of the AABB.

    Returns:
        bool: True if the line intersects the AABB, False otherwise.
    """

    line_dir = line_point2 - line_point1

    # Check separating axes along AABB's edges
    for i in range(3):
        # Project line segment onto axis
        line_proj = np.dot(line_point1, np.eye(3)[i]) + np.dot(line_dir, np.eye(3)[i])
        aabb_proj = np.array([aabb_min[i], aabb_max[i]])

        # Check for overlap
        if not (np.min(line_proj) <= np.max(aabb_proj) and np.max(line_proj) >= np.min(aabb_proj)):
            return False  # No overlap on this axis

    # Check separating axes along line direction
    line_axis = line_dir / np.linalg.norm(line_dir)
    aabb_proj = np.dot(aabb_min, line_axis), np.dot(aabb_max, line_axis)
    line_proj = np.dot(line_point1, line_axis), np.dot(line_point1 + line_dir, line_axis)

    # Check for overlap
    if not (np.min(line_proj) <= np.max(aabb_proj) and np.max(line_proj) >= np.min(aabb_proj)):
        return False  # No overlap on the line axis

    return True  # No separating axis found, intersection exists


def check_pts_in_AABB_cube(cube, pts):
    """
    Check if any of the 'points lie inside the cube
    Parameters
    ----------
    cube: Array of points of cube (8x4)
    pts: Array of points to check

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

def check_ppd_edges_in_cube(cube,cube_edge,edge_indices):
    if len(edge_indices)!=12:
        print('Invalid PPD provided to check_ppd_edges_in_cube')
        return False
    aabb_min=np.array([min(cube[:, 0]),min(cube[:, 1]),min(cube[:, 2])],dtype=np.float32)
    aabb_max = np.array([max(cube[:, 0]),max(cube[:, 1]),max(cube[:, 2])],dtype=np.float32)
    print('EE',cube_edge)
    for ei in edge_indices:
        e0=cube_edge[ei][0][0:3]
        e1=cube_edge[ei][1][0:3]
        print('----',ei)
        print('e0',e0)
        print('e1', e1)
        print('aabb_min', aabb_min)
        print('aabb_max', aabb_max)
        if line_box_sat_intersection(e0,e1, aabb_min, aabb_max):
            return True
    return False

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
        self.is_aabb=self.check_AABB(self.points)
        self.is_valid=True
        if pts.shape[0] > max_mesh_points:
            print('Incorrect size of points for link:', link_name, pts.shape)
            print('Ignoring collision link %s' % link_name)
            self.is_valid=False
        self.pose=None
        #self.edge_indices_ppd=self.find_edge_indices_PPD()


    def is_ppd(self):
        return self.points.shape[0]==8

    def set_pose(self,p):
        self.pose=p

    def pretty_print(self):
        print('-- CollisionLink %s --'%self.name)
        print('AABB Cube',self.is_aabb)
        print('Is Valid', self.is_valid)
        print('In collision',self.in_collision)
        print('Was in collision',self.was_in_collision)
        print('Mesh size',self.points.shape)

    def find_edge_indices_PPD(self):
        """
        Return the indices for each edge assuming the link is a parallelpiped (PPD)
        """

        triangles=self.mesh.cells_dict['triangle']

        for t in triangles:
            idx_pairs=[[0,1],[0,2],[1,2]]
            edge_lens=[]
            for ii in idx_pairs:
                edge_lens.append(np.linalg.norm(self.points[ii[0]]-self.points[ii[2]]))
            exterior_edges = np.array(idx_pairs)[np.argsort(np.array(edge_lens))][0:2] #indices of two shortest legs of triangle




        px=self.points.shape[0]
        # if not self.is_ppd():
        #     return np.array([])
        idx=[]
        lens=[]
        #Toss out really short edge as is a mesh file artifact

        for i in range(px):
            for j in range(i+1,px):
                ll = np.linalg.norm(self.points[i] - self.points[j])
                if ll>eps:
                    idx.append([i,j])
                    lens.append(ll)
        #return 12 shortest lengths of all possible combinations
        q= np.array(idx)[np.argsort(np.array(lens))][0:12]
        lens.sort()
        print('LENS',lens)
        return q

    def check_AABB(self,pts):
        """
        Check if points are axis aligned (roughly) and form a rectangular parallelpiped (eg AABB)

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
    def __init__(self, name,link_pts,link_cube,detect_as):
        self.in_collision=False
        self.was_in_collision=False
        self.link_cube=link_cube
        self.link_pts=link_pts
        self.detect_as=detect_as
        self.name=name
        self.is_valid=self.link_cube.is_valid and self.link_pts.is_valid and self.link_cube.is_aabb
        if not self.is_valid:
            print('Dropping monitor of collision pair %s'%self.name_id)

    def pretty_print(self):
        print('-------- Collision Pair %s ----------------'%self.name_id.upper())
        print('In collision',self.in_collision)
        print('Was in collision',self.was_in_collision)
        print('Is Valid',self.is_valid)
        # print('--Link Cube--')
        # self.link_cube.pretty_print()
        # print('--Link Pts--')
        # self.link_pts.pretty_print()


class CollisionJoint:
    def __init__(self, joint_name,motor):
        self.name=joint_name
        self.motor=motor
        self.active_collisions=[]
        self.collision_pairs=[]
        self.collision_dirs={}
        self.in_collision={'pos':False,'neg':False}
        self.was_in_collision = {'pos': False, 'neg': False}
    def add_collision_pair(self,motion_dir, collision_pair):
        self.collision_pairs.append(collision_pair)
        self.collision_dirs[collision_pair.name]=motion_dir
    def pretty_print(self):
        print('-------Collision Joint: %s-----------------'%self.name)
        for cp in self.collision_pairs:
            cp.pretty_print()
            print('Motion dir: %s'%self.collision_dirs[cp.name])
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
        * One of the collision meshes ("cube") is a cube that is aligned with XYZ axis (eg AABB)
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
        #chime.theme('big-sur') #'material')#
        self.running=False
        self.urdf=None

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
        eoa_name= self.robot.params['tool']
        urdf_name = pkg + '/%s/stretch_description_%s_%s.urdf' % (model_name, model_name, eoa_name)
        mesh_path = pkg + '/%s/' % (model_name)

        if self.params[model_name]=={}:
            #self.logger.warning('Collision parameters not present. Disabling collision system.')
            self.running = False
            return

        try:
            self.urdf = urdf_loader.URDF.load(urdf_name)
        except ValueError:
            print('Unable to load URDF: %s. Disabling collision system.' % urdf_name)
            self.urdf = None
            self.running = False
            return

        #Construct collision pairs
        cp_dict = self.params[model_name]['collision_pairs']
        cp_dict.update(self.robot.end_of_arm.params['collision_mgmt']['collision_pairs'])

        for cp_name in cp_dict:
            cp=cp_dict[cp_name] #Eg {'link_pts': 'link_head_tilt', 'link_cube': 'link_arm_l4','detect_as':'pts'}
            if cp['link_pts'] not in self.collision_links:
                self.collision_links[cp['link_pts']] = CollisionLink(cp['link_pts'], self.urdf, mesh_path,
                                                                     self.params['max_mesh_points'])
            if cp['link_cube'] not in self.collision_links:
                self.collision_links[cp['link_cube']] = CollisionLink(cp['link_cube'], self.urdf, mesh_path,
                                                                      self.params['max_mesh_points'])
            self.collision_pairs[cp_name] = CollisionPair(name=cp_name,
                                                          link_pts=self.collision_links[cp['link_pts']],
                                                          link_cube=self.collision_links[cp['link_cube']],
                                                          detect_as=cp['detect_as'])

        #Assign collision pairs to each joint
        #Include those of standard robot body plus its defined tool
        # EG collision_joints={'lift':[{collision_1},{collision_2...}],'head_pan':[...]}
        cj_dict=self.params[model_name]['joints']
        eoa_cj_dict=self.robot.end_of_arm.params['collision_mgmt']['joints']

        for tt in eoa_cj_dict:
            if tt in cj_dict:
                cj_dict[tt]+=eoa_cj_dict[tt]
            else:
                cj_dict[tt]=eoa_cj_dict[tt]

        for joint_name in cj_dict:
            self.collision_joints[joint_name]=CollisionJoint(joint_name,self.get_joint_motor(joint_name))
            cp_list = cj_dict[joint_name]
            for cp in cp_list: #eg cp={'motion_dir': 'pos', 'collision_pair': 'link_head_tilt_TO_link_arm_l4'}
                self.collision_joints[joint_name].add_collision_pair(motion_dir=cp['motion_dir'],
                                                                     collision_pair=self.collision_pairs[cp['collision_pair']])

    def get_joint_motor(self,joint_name):
        if joint_name=='lift':
            return self.robot.lift
        if joint_name=='arm':
            return self.robot.arm
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

        # Test for collisions across all collision pairs
        for pair_name in self.collision_pairs:
            cp=self.collision_pairs[pair_name]
            if cp.is_valid:
                cp.was_in_collision=cp.in_collision
                if cp.detect_as=='pts':
                    cp.in_collision=check_pts_in_AABB_cube(cube=cp.link_cube.pose,pts=cp.link_pts.pose)
                # elif cp.detect_as=='edges':
                #     print('Checking', cp.name)
                #     cp.in_collision = check_ppd_edges_in_cube(cube=cp.link_cube.pose, cube_edge=cp.link_pts.pose,edge_indices=cp.link_pts.edge_indices_ppd)
                else:
                    cp.in_collision =False
                    #cp.pretty_print()

                #Propogate to links
                self.collision_links[cp.link_cube.name].in_collision=self.collision_links[cp.link_cube.name].in_collision or cp.in_collision
                self.collision_links[cp.link_pts.name].in_collision =self.collision_links[cp.link_pts.name].in_collision or cp.in_collision

                # Beep on new collision
                if not self.collision_pairs[pair_name].was_in_collision and self.collision_pairs[pair_name].in_collision:
                    print('New collision pair event: %s'%pair_name)
                    # print('\a')
                    self.alert()

        #Now update joint state
        for joint_name in self.collision_joints:
            cj = self.collision_joints[joint_name]
            for cp in cj.collision_pairs:
                if cp.in_collision:
                    cj.active_collisions.append(cp.name) #Add collision to joint
                    cj.in_collision[cj.collision_dirs[cp.name]] = True
            #Finally, update the collision state for each joint
            self.collision_joints[joint_name].motor.step_collision_avoidance(self.collision_joints[joint_name].in_collision)

    def alert(self):
        threading.Thread(target=chime.warning,daemon=True).start()

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


    def get_joint_configuration(self,braked=False):
        """
        Construct a dictionary of robot's current pose
        """
        s = self.robot.get_status()

        kbd = self.params[self.robot.params['model_name']]['k_brake_distance']
        if braked:
            da=kbd['arm']*self.robot.arm.get_braking_distance()/4.0
            dl=kbd['lift']*self.robot.lift.get_braking_distance()
            dhp = kbd['head_pan'] * self.robot.head.get_joint('head_pan').get_braking_distance()
            dht = kbd['head_tilt'] * self.robot.head.get_joint('head_tilt').get_braking_distance()
        else:
            da=0.0
            dl=0.0
            dhp=0.0
            dht=0.0

        configuration = {
            'joint_lift': dl+s['lift']['pos'],
            'joint_arm_l0': da+s['arm']['pos']/4.0,
            'joint_arm_l1': da+s['arm']['pos']/4.0,
            'joint_arm_l2': da+s['arm']['pos']/4.0,
            'joint_arm_l3': da+s['arm']['pos']/4.0,
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