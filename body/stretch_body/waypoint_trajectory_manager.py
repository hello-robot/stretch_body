#! /usr/bin/env python


from stretch_body.hello_utils import *
import time
import copy

TRAJECTORY_TYPE_INVALID = 0
TRAJECTORY_TYPE_CUBIC_SPLINE = 1
TRAJECTORY_TYPE_QUINTIC_SPLINE = 2

class WaypointTrajectoryManager:
    """
    Manage joint trajectories
    Trajectories are specified as a list of waypoints
    TRAJECTORY_TYPE_CUBIC_SPLINE:   [[time, pos,vel],...]
    TRAJECTORY_TYPE_QUINTIC_SPLINE: [[time, pos,vel,accel],...]

    These are then represented as a list of continuous trajectory segments (splines or lines)
    TRAJECTORY_TYPE_CUBIC_SPLINE:   [[duration, a0,a1,a2,a3],...]
    TRAJECTORY_TYPE_QUINTIC_SPLINE: [[duration, a0,a1,a2,a3,a4,a5],...]

    New waypoints can be added dynamically (so long as it is in the future),
    and the list of splines will be recomputed.

    The spline trajectory for each pair of waypoints is periodically pushed down to the joint controller
    for execution.
    """
    def __init__(self):
        self.__setup_new_trajectory()
        self.waypoints_last_trajectory = None

    def add_waypoints_to_trajectory(self,v):
        #V is a list of waypoints
        if self.num_segment_left()>0: #Append or insert into an existing trajectory
            if self.get_trajectory_type(v[0])!=self.traj_type:
                print('Invalid waypoint type')
                return
            tstart=self.waypoints[self.idx_waypoint][0]
            tend=self.waypoints[-1][0]
            tnew=v[0][0]
            if tnew<tstart:
                print('Unable to add waypoints to trajectory: tstart %f tnew %f'%(tstart,tnew))
                return
            idx_insert=len(self.waypoints)
            if tnew<tend: #Replace existing waypoints with new ones starting at tnew
                idx_insert=0
                for i in range(len(self.waypoints)):
                    if self.waypoints[i][0]>=tnew:
                        idx_insert=i
                        break
            self.waypoints =self.waypoints[0:idx_insert]+copy.deepcopy(v)
        else: #Start of a new trajectory
            self.__setup_new_trajectory()
            self.waypoints=copy.deepcopy(v)
            self.traj_type=self.get_trajectory_type(v[0])

        self.ts_duration=self.waypoints[-1][0] - self.waypoints[0][0]


    def get_trajectory_type(self,v):
        if len(v) == 4:
            return TRAJECTORY_TYPE_QUINTIC_SPLINE
        if len(v) == 3:
            return TRAJECTORY_TYPE_CUBIC_SPLINE
        return TRAJECTORY_TYPE_INVALID

    def __setup_new_trajectory(self):
        self.waypoints = []
        self.idx_waypoint = 0
        self.ts_start = None
        self.ts_duration = 0
        self.new_trajectory = True
        self.id_seg=0
        self.trajectory_loaded=False
        self.seg=None
        self.traj_type =TRAJECTORY_TYPE_INVALID
        self.last_setpoint_time = 0

    def num_segment_left(self):
        return max(0,len(self.waypoints)-self.idx_waypoint-1)


    def find_waypoint_before_time(self,t):
        #Return idx of waypoint just before/at the time
        for i in range(len(self.waypoints)-1):
            if t>=self.waypoints[i][0] and t<=self.waypoints[i+1][0] :
                return i
        return None

    def get_setpoint_at_time(self,t):
        #Get the target pos, vel, accel at time t
        if t>self.waypoints[-1][0]:
            return self.waypoints[-1][1:]
        idx_0=self.find_waypoint_before_time(t)
        if idx_0 is None:
            return None
        idx_1=idx_0+1
        if(idx_0>0):
            t_prior=self.waypoints[idx_0][0]
        else:
            t_prior=0
        self.last_setpoint_time = t
        if self.traj_type == TRAJECTORY_TYPE_CUBIC_SPLINE:
            #print 'Waypoint0',self.waypoints[idx_0],t-t_prior
            seg = self.generate_cubic_spline_segment(self.waypoints[idx_0],self.waypoints[idx_1])
            e=self.evaluate_cubic_spline(seg,t-t_prior)
            #print 'Vtarget',e[1]
            return e

        if self.traj_type == TRAJECTORY_TYPE_QUINTIC_SPLINE:
            seg = self.generate_quintic_spline_segment(self.waypoints[idx_0],self.waypoints[idx_1])
            return self.evaluate_quintic_spline(seg,t-t_prior)


    def get_first_segment(self):
        #Get first segment to send down, start loading
        #For non dynamixel joints
        self.idx_waypoint=0
        if (self.num_segment_left()<1):
            return None
        if self.traj_type == TRAJECTORY_TYPE_CUBIC_SPLINE:  # Recompute spline in case has changed
            self.seg = self.generate_cubic_spline_segment(self.waypoints[self.idx_waypoint], self.waypoints[self.idx_waypoint + 1])
            self.id_seg = 2
            self.seg.append(self.id_seg) #Id of first segment
        if self.traj_type == TRAJECTORY_TYPE_QUINTIC_SPLINE:  # Recompute spline in case has changed
            self.seg = self.generate_quintic_spline_segment(self.waypoints[self.idx_waypoint], self.waypoints[self.idx_waypoint + 1])
            self.id_seg = 2
            self.seg.append(self.id_seg) #Id of first segment
        return self.seg

    def get_next_segment(self,active_id):
        #The active id is the ID of the currently active trajectory segment as reported by the uC
        #Ids go from 2 to 255 and loop around
        #Id of 0 marks that the uC is in Idle state
        #Id of 1 marks that a trajectory is loaded but not executing
        #Id of 2 marks that the first segment has begun executing
        #This returns the trajectory segment subsequent to the active_id (seg_next)
        #The seg_next can be modified up until active_id==seg_next.id, allow for last minute revision
        #of the next segment to be pushed to the uC
        #An arbitrary number of segments can be sent do the uC sequentially
        #The end of a sequence is marked by sending a [0]*8 segment
        # For non dynamixel joints

        if active_id ==0 and self.trajectory_loaded: #Marks end of last segment of trajectory
            self.waypoints_last_trajectory=copy.deepcopy(self.waypoints) #Store this if want a trajectory history
            self.__setup_new_trajectory()
            return None

        if active_id ==1:   #Marks first segment of trajectory is loaded but not yet executing
            self.trajectory_loaded = True
            return self.seg

        if active_id == 2:
            if self.new_trajectory:
                self.new_trajectory = False
                self.ts_start = time.time()

        if active_id == self.id_seg:  #Marks the last pushed segment is executing, advance counters to push next one
            self.idx_waypoint=self.idx_waypoint+1
            self.id_seg = max(2,(self.id_seg+1)%255)

        if self.num_segment_left()>0 :
            if self.traj_type == TRAJECTORY_TYPE_CUBIC_SPLINE: #Recompute spline in case has changed
                self.seg= self.generate_cubic_spline_segment(self.waypoints[self.idx_waypoint],self.waypoints[self.idx_waypoint+1])
                self.seg.append(self.id_seg)
            if self.traj_type == TRAJECTORY_TYPE_QUINTIC_SPLINE: #Recompute spline in case has changed
                self.seg= self.generate_quintic_spline_segment(self.waypoints[self.idx_waypoint],self.waypoints[self.idx_waypoint+1])
                self.seg.append(self.id_seg)
        else:
            self.seg=[0]*8 #Marks no more segments to the uC
        return self.seg

    def duration_remaining(self):
        if self.ts_start is None:
            return self.ts_duration
        dt_elapased = time.time() - self.ts_start
        return max(0,self.ts_duration - dt_elapased)

    def generate_quintic_spline_segment(self, i,f):
        # waypoints are [[time, pos,vel,accel],...]
        duration = f[0] - i[0]
        a0 = i[1]
        a1 = i[2]
        a2 = i[3] / 2
        a3 = (20 * f[1] - 20 * i[1] - (8 * f[2] + 12 * i[2]) * (f[0] - i[0]) - (3 * i[3] - f[3]) * ((f[0] - i[0]) ** 2)) / (2 * ((f[0] - i[0]) ** 3))
        a4 = (30 * i[1] - 30 * f[1] + (14 * f[2] + 16 * i[2]) * (f[0] - i[0]) + (3 * i[3] - 2 * f[3]) * ((f[0] - i[0]) ** 2)) / (2 * ((f[0] - i[0]) ** 4))
        a5 = (12 * f[1] - 12 * i[1] - (6 * f[2] + 6 * i[2]) * (f[0] - i[0]) - (i[3] - f[3]) * ((f[0] - i[0]) ** 2)) / (2 * ((f[0] - i[0]) ** 5))
        return [duration, a0, a1, a2, a3, a4,a5]

    #See Craig Intro Robotics Ch7
    def generate_cubic_spline_segment(self, i,f):
        # waypoints are [[time, pos,vel],...]
        duration =f[0] - i[0]
        a0 = i[1]
        a1 = i[2]
        a2 = (3 / duration ** 2) * (f[1] - i[1]) - (2 / duration) * i[2] - (1 / duration) * f[2]
        a3 = (-2 / duration ** 3) * (f[1] - i[1]) + (1 / duration ** 2) * (f[2] + i[2])
        return [duration, a0, a1, a2, a3, 0, 0]

    def generate_linear_segment(self,i,f):
        # waypoints are [[time, pos],...]
        duration = f[0] - i[0]
        return [duration, i[1],f[1]]

    def evaluate_cubic_spline(self,s,t):
        #TRAJECTORY_TYPE_CUBIC_SPLINE:   [[duration, a0,a1,a2,a3],...]
        a=s[1:]
        pos= a[0]+(a[1]*t)+(a[2]*t**2)+(a[3]*t**3)
        vel= a[1] +(2*a[2]*t)+(3*a[3]*t**2)
        acc= 2*a[2] + 6*a[3]*t
        return [pos,vel,acc]

    def evaluate_quintic_spline(self,s,t):
        #TRAJECTORY_TYPE_QUINTIC_SPLINE: [[duration, a0,a1,a2,a3,a4,a5],...]
        a=s[1:]
        pos= a[0]+(a[1]*t)+(a[2]*t**2)+(a[3]*t**3)+(a[4]*t**4)+(a[5]*t**5)
        vel = a[1] +(2*a[2]*t)+(3*a[3]*t**2) + (4*a[4]*t**3) + (5*a[5]*t*4)
        accel = 2*a[2] + 6*a[3]*t + 12*a[4]*t**2 + 20*a[5]*t**3
        return [pos,vel,accel]

    def evaluate_linear_interpolate(self,s,t):
        #TRAJECTORY_TYPE_LINEAR: [[duration, pos0, pos1], ...]
        duration = s[0]
        pos0=s[1]
        pos1=s[2]
        pos = pos0 + t*(pos1-pos0)/duration
        vel = (pos1-pos0)/duration
        accel = None
        return [pos, vel, accel]

