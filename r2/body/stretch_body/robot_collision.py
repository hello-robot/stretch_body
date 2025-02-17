#! /usr/bin/env python

from stretch_body.device import Device
import urchin as urdf_loader
import meshio
import numpy as np
import time
import threading
import chime
import random
from stretch_body.robot_params import RobotParams
import multiprocessing
import signal
import ctypes
import sys

ENABLE_COLLISION_VISUALIZER = False


try:
    # works on ubunut 22.04
    import importlib.resources as importlib_resources
    str(importlib_resources.files("stretch_body"))
except Exception as e:
    # works on ubuntu 20.04
    import importlib_resources
    str(importlib_resources.files("stretch_body"))

if ENABLE_COLLISION_VISUALIZER:
    import pyrender
    import trimesh

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
            return True, p
    return False, None

def check_AABB_in_AABB_from_pts(pts1, pts2):
    """
    Check if an AABB intersects another AABB from the given two sets of points
    """
    print(pts1.shape,pts2.shape)
    xmax_1 = max(pts1[:, 0])
    xmin_1 = min(pts1[:, 0])
    ymax_1 = max(pts1[:, 1])
    ymin_1 = min(pts1[:, 1])
    zmax_1 = max(pts1[:, 2])
    zmin_1 = min(pts1[:, 2])

    xmax_2 = max(pts2[:, 0])
    xmin_2 = min(pts2[:, 0])
    ymax_2 = max(pts2[:, 1])
    ymin_2 = min(pts2[:, 1])
    zmax_2 = max(pts2[:, 2])
    zmin_2 = min(pts2[:, 2])

    cx = xmin_1<=xmax_2 and xmax_1>=xmin_2
    cy = ymin_1<=ymax_2 and ymax_1>=ymin_2
    cz = zmin_1<=zmax_2 and zmax_1>=zmin_2
    
    return cx and cy and cz


def check_mesh_triangle_edges_in_cube(mesh_triangles,cube):
    # Check a set of mesh's triangles intersect an AABB cube
    while len(mesh_triangles):
        # choose a random triangle indices
        random_index = random.randint(0, len(mesh_triangles) - 1)
        points = mesh_triangles[random_index]
        mesh_triangles.pop(random_index)
        # Barycentric Coordinates based points based edge points interpolation
        c,p = check_pts_in_AABB_cube(cube,sample_points_on_triangle_edges(np.array(points)))
        if c:
            return True, p
    return False, None

def get_triangle_edge_barycentric_coords(N):
    """
    Generate a Barycentric coordinate vectors of N points of a triangle edges
    This matrix is to be used as a constant.
    """
    barycentric_coords = []
    nums = np.linspace(0,1,num=N)
    for n1 in nums:
        for n2 in nums:
            for n3 in nums:
                c = False
                if abs(n3)<0.001:
                    c = True
                if abs(n2)<0.001:
                    c = True
                if abs(n1)<0.001:
                    c = True
                if c and abs((1-n2-n3)-n1)<0.001:
                    barycentric_coords.append([n1,n2,n3])

    return np.array(barycentric_coords)

BARYCENTRIC_COORDS = get_triangle_edge_barycentric_coords(25) # Sample a NX3 Barycentric Coord vector matrix

def sample_points_on_triangle_edges(points):
    # Convert barycentric coordinates to Cartesian coordinates
    points = BARYCENTRIC_COORDS[:, 0][:, np.newaxis] * points[0] \
           + BARYCENTRIC_COORDS[:, 1][:, np.newaxis] * points[1] \
           + BARYCENTRIC_COORDS[:, 2][:, np.newaxis] * points[2]
    return points

def scale_cuboid_points(vertices,scale_factor):
    # Calculate the centroid of the cuboid
    centroid = np.mean(vertices, axis=0)
    
    # Scale the vertices relative to the centroid
    scaled_vertices = centroid + scale_factor * (vertices - centroid)
    
    # Convert the scaled vertices back to a list of tuples and return
    return scaled_vertices

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

def closest_pair_3d(points1, points2):
    """
    Find the closest pair of 3D points from two lists of 3D points.
    """
    closest_distance = float('inf')
    closest_pair = None
    
    for p1 in points1:
        for p2 in points2:
            dist = np.linalg.norm(p1-p2)
            if dist < closest_distance:
                closest_distance = dist
                closest_pair = (p1, p2)
                
    return closest_pair, closest_distance
    
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
    
    def get_triangles(self):
        triangles_idx=self.mesh.cells_dict['triangle']
        triangles = []
        for t_set in triangles_idx:
            p1 = self.pose[t_set[0]]
            p2 = self.pose[t_set[1]]
            p3 = self.pose[t_set[2]]
            triangles.append([p1,p2,p3])
        return triangles

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
    def __init__(self, name,link_pts,link_cube,detect_as, cube_scale=1.2):
        self.in_collision=False
        self.was_in_collision=False
        self.link_cube=link_cube
        self.link_pts=link_pts
        self.detect_as=detect_as
        self.name=name
        self.cube_scale = cube_scale 
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
    def __init__(self, joint_name):
        self.name=joint_name
        self.active_collisions=[]
        self.collision_pairs=[]
        self.collision_dirs={}
        self.in_collision={'pos':False,'neg':False}
        self.was_in_collision = {'pos': False, 'neg': False}
        self.last_in_collision_cnt = 0

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

def _collision_compute_worker(name, shared_is_running, shared_joint_cfg, shared_collision_status, exit_event):
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    collision_compute = RobotCollisionCompute(name)
    if collision_compute.startup():
        collision_joints_status = {}
        time.sleep(0.5)
        while not exit_event.is_set():
            try:
                if shared_is_running.value:
                    collision_compute.step(shared_joint_cfg)
                    for joint_name in collision_compute.collision_joints:
                        collision_joints_status[joint_name] = collision_compute.collision_joints[joint_name].in_collision
                    shared_collision_status.put(collision_joints_status)
            except (BrokenPipeError,ConnectionResetError):
                pass
            time.sleep(collision_compute.sleep_time)

def signal_handler(signal_received, frame):
    sys.exit(0)

class RobotCollisionMgmt(Device):
    def __init__(self,robot,name='robot_collision_mgmt'):
        self.name = name
        self.robot = robot
        self.shared_joint_cfg = multiprocessing.Queue()
        self.shared_collision_status = multiprocessing.Queue()
        self.shared_is_running = multiprocessing.Value(ctypes.c_bool, False)
        self.exit_event = multiprocessing.Event()
        self.collision_compute_proccess = multiprocessing.Process(target=_collision_compute_worker,
                                                               args=(self.name,
                                                                     self.shared_is_running,
                                                                     self.shared_joint_cfg,
                                                                     self.shared_collision_status,
                                                                     self.exit_event,),daemon=True)
        self.running = False
        self.robot_params = RobotParams().get_params()[1]
        self.collision_status = {}
        self.brake_joints = {}

    def startup(self):
        self.collision_compute_proccess.start()
    
    def stop(self):
        try:
            self.exit_event.set()
            self.shared_is_running.value = False
            self.collision_compute_proccess.terminate()
            self.collision_compute_proccess.join()
        except Exception:
            pass
    
    def step(self):
        try:
            self.shared_is_running.value = self.running
            if self.running:
                config = self.get_joint_configuration(self.brake_joints)
                self.shared_joint_cfg.put(config)
                self.collision_status.update(self.shared_collision_status.get())
                for j in self.collision_status.keys():
                    jm = self.get_joint_motor(j)
                    jm.step_collision_avoidance(self.collision_status[j])
                    # if True in self.collision_status[j].values():
                    #     self.brake_joints[j] = True
                    # else:
                    #     self.brake_joints[j] = False

        except (BrokenPipeError,ConnectionResetError):
            pass
    
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

    def get_joint_configuration(self,brake_joints={}):
        """
        Construct a dictionary of robot's current pose
        """
        s = self.robot.get_status()
        kbd = self.robot_params['robot_collision_mgmt'][self.robot.params['model_name']]['k_brake_distance']
        da=0.0
        dl=0.0
        dhp=0.0
        dht=0.0
        try:
            if brake_joints['arm']:
                da=kbd['arm']*self.robot.arm.get_braking_distance()/4.0
        except KeyError:
            da=0.0
        try:
            if brake_joints['lift']:
                dl=kbd['lift']*self.robot.lift.get_braking_distance()
        except KeyError:
            dl=0.0
        try:
            if brake_joints['head_pan']:
                dhp = kbd['head_pan'] * self.robot.head.get_joint('head_pan').get_braking_distance()
        except KeyError:
            dhp=0.0
        try:
            if brake_joints['head_tilt']:
                dht = kbd['head_tilt'] * self.robot.head.get_joint('head_tilt').get_braking_distance()
        except KeyError:
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

        configuration.update(self.robot.end_of_arm.get_joint_configuration(brake_joints))
        return configuration
    
    def enable(self):
        self.running=True

    def disable(self):
        self.running=False
        for j in self.collision_status.keys():
            jm = self.get_joint_motor(j)
            jm.step_collision_avoidance({'pos':False,'neg':False})
            jm.forced_collision_stop_override = {'pos':False,'neg':False}

class RobotCollisionCompute(Device):
    def __init__(self,name='robot_collision_mgmt'):
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
        Device.__init__(self, name)
        self.collision_joints = {}
        self.collision_links = {}
        self.collision_pairs = {}
        chime.theme('big-sur') #'material')
        self.urdf=None
        self.prev_loop_start_ts = None
        self.robot_params = RobotParams().get_params()[1]
        self.viz = ENABLE_COLLISION_VISUALIZER
        self.sleep_time = 0.01
        if self.viz:
            self.first_frame = False

    def pretty_print(self):
        for j in self.collision_joints:
            self.collision_joints[j].pretty_print()


        
    def startup(self,threaded=False):
        Device.startup(self, threaded)
        pkg = str(importlib_resources.files("stretch_urdf"))  # .local/lib/python3.10/site-packages/stretch_urdf)
        model_name = self.robot_params['robot']['model_name']
        eoa_name= self.robot_params['robot']['tool']
        urdf_name = pkg + '/%s/stretch_description_%s_%s.urdf' % (model_name, model_name, eoa_name)
        mesh_path = pkg + '/%s/' % (model_name)

        if self.params[model_name]=={}:
            #self.logger.warning('Collision parameters not present. Disabling collision system.')
            self.running = False
            return False

        try:
            self.urdf = urdf_loader.URDF.load(urdf_name)
            if self.viz:
                print("Starting CollisionMgmt Debug Visualizer...")
                self.urf_viz = URDFVisualizer(self.urdf)
        except ValueError:
            print('Unable to load URDF: %s. Disabling collision system.' % urdf_name)
            self.urdf = None
            self.running = False
            return False

        #Construct collision pairs
        cp_dict = self.params[model_name]['collision_pairs']
        cp_dict.update(self.robot_params[eoa_name]['collision_mgmt']['collision_pairs'])

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
            if 'cube_scale' in list(cp.keys()):
                self.collision_pairs[cp_name].cube_scale = cp['cube_scale']

        #Assign collision pairs to each joint
        #Include those of standard robot body plus its defined tool
        # EG collision_joints={'lift':[{collision_1},{collision_2...}],'head_pan':[...]}
        cj_dict=self.params[model_name]['joints']
        eoa_cj_dict=self.robot_params[eoa_name]['collision_mgmt']['joints']

        for tt in eoa_cj_dict:
            if tt in cj_dict:
                cj_dict[tt]+=eoa_cj_dict[tt]
            else:
                cj_dict[tt]=eoa_cj_dict[tt]

        for joint_name in cj_dict:
            self.collision_joints[joint_name]=CollisionJoint(joint_name)
            cp_list = cj_dict[joint_name]
            for cp in cp_list: #eg cp={'motion_dir': 'pos', 'collision_pair': 'link_head_tilt_TO_link_arm_l4'}
                self.collision_joints[joint_name].add_collision_pair(motion_dir=cp['motion_dir'],
                                                                     collision_pair=self.collision_pairs[cp['collision_pair']])
        return True
    

    def step(self,cfg=None):
        """
                Check for interference between cube pairs
        """
        # if self.prev_loop_start_ts:
        #     print(f"[{self.name}] Step exec time: {(time.perf_counter()-self.prev_loop_start_ts)*1000}ms")
        
        if self.urdf is None:
            return

        # Update forward kinematics of links
        _cfg = cfg.get()
        if self.viz:
            if not self.first_frame:
                self.urf_viz.show(cfg=_cfg, use_collision=True)
                self.first_frame = True
            if self.urf_viz.viewer.is_active:
                self.urf_viz.update_pose(cfg=_cfg, use_collision=True)
        lfk = self.urdf.link_fk(cfg=_cfg, links=self.collision_links.keys(), use_names=True)

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

            # Release Collision Joints in_collision mode onlt after 100 cycles
            if self.collision_joints[joint_name].in_collision['pos'] or self.collision_joints[joint_name].in_collision['neg']:
                self.collision_joints[joint_name].last_in_collision_cnt = self.collision_joints[joint_name].last_in_collision_cnt + 1
            if self.collision_joints[joint_name].last_in_collision_cnt > 20:
                self.collision_joints[joint_name].in_collision['pos'] = False
                self.collision_joints[joint_name].in_collision['neg'] = False
                self.collision_joints[joint_name].last_in_collision_cnt = 0
        # Test for collisions across all collision pairs
        for pair_name in self.collision_pairs:
            cp=self.collision_pairs[pair_name]
            if cp.is_valid:
                cp.was_in_collision=cp.in_collision
                if cp.detect_as=='pts':
                    cp.in_collision, collision_point =check_pts_in_AABB_cube(cube = scale_cuboid_points(cp.link_cube.pose,cp.cube_scale),
                                                           pts = cp.link_pts.pose)
                    if cp.in_collision and self.viz:
                        self.urf_viz.collision_sphere(collision_point)
                    # cp.in_collision=check_AABB_in_AABB_from_pts(pts1=cp.link_cube.pose,pts2=cp.link_pts.pose)
                elif cp.detect_as=='edges':
                    cp.in_collision, collision_point = check_mesh_triangle_edges_in_cube(mesh_triangles = cp.link_pts.get_triangles(), 
                                                                        cube = scale_cuboid_points(cp.link_cube.pose,cp.cube_scale))
                    if cp.in_collision and self.viz:
                        self.urf_viz.collision_sphere(collision_point)
                else:
                    cp.in_collision =False
                    #cp.pretty_print()

                #Propogate to links
                self.collision_links[cp.link_cube.name].in_collision=self.collision_links[cp.link_cube.name].in_collision or cp.in_collision
                self.collision_links[cp.link_pts.name].in_collision =self.collision_links[cp.link_pts.name].in_collision or cp.in_collision

                # Beep on new collision
                if not self.collision_pairs[pair_name].was_in_collision and self.collision_pairs[pair_name].in_collision:
                    print(f'New collision pair event: {pair_name} [{time.time()}]' )
                    self.alert()

        #Now update joint state
        for joint_name in self.collision_joints:
            cj = self.collision_joints[joint_name]
            for cp in cj.collision_pairs:
                if cp.in_collision:
                    cj.active_collisions.append(cp.name) #Add collision to joint
                    cj.in_collision[cj.collision_dirs[cp.name]] = True
        self.prev_loop_start_ts = time.perf_counter()
        
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

class URDFVisualizer:
    """The `show` method in this class is modified from the
    original implementation of `urdf_loader.URDF.show`.
    """

    def __init__(self, urdf):
        self.urdf = urdf
        self.nodes = None
        self.scene = None
        self.viewer = None

        self.collision_points = []

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

        self.scene = pyrender.Scene(ambient_light = [0,0,0, 0.5])
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
        
        self.clean_collision_points()
        self.viewer.render_lock.release()
    
    def clean_collision_points(self):
        if len(self.collision_points):
            for c,t in self.collision_points:
                if time.perf_counter() - t > 1:
                    if self.scene.has_node(c):
                        self.scene.remove_node(c)
                        self.collision_points.remove((c,t))
                    else:
                        print(f"Unable to find {c}")
                        

    def collision_sphere(self, point):
        p = np.identity(4)
        p[:,3] = point
        mesh = trimesh.creation.icosphere(radius=0.015,subdivisions=1)
        mesh.visual.face_colors = np.random.randint(low=0,high=255,size=4)
        pmesh = pyrender.Mesh.from_trimesh(mesh,smooth=False)
        self.viewer.render_lock.acquire()
        mesh_node = self.scene.add(pmesh,pose=p)
        self.collision_points.append((mesh_node,time.perf_counter()))
        self.viewer.render_lock.release()
        

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
