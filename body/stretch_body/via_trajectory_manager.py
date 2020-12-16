#! /usr/bin/env python


from stretch_body.hello_utils import *
import time
import copy

TRAJECTORY_TYPE_LINEAR = 1
TRAJECTORY_TYPE_CUBIC_SPLINE = 2
TRAJECTORY_TYPE_QUINTIC_SPLINE = 3
SEG_IDX_ID = 0
SEG_IDX_TIME = 1

class ViaTrajectoryManager:
    """
    Manage joint trajectories
    Trajectories are specified as a list of via points
    TRAJECTORY_TYPE_LINEAR:         [[time, pos,],...]
    TRAJECTORY_TYPE_CUBIC_SPLINE:   [[time, pos,vel],...]
    TRAJECTORY_TYPE_QUINTIC_SPLINE: [[time, pos,vel,accel],...]

    These are then represented as a list of continuous trajectory segments (splines or lines)
    TRAJECTORY_TYPE_LINEAR:         [[duration, a0,a1],...]
    TRAJECTORY_TYPE_CUBIC_SPLINE:   [[duration, a0,a1,a2,a3],...]
    TRAJECTORY_TYPE_QUINTIC_SPLINE: [[duration, a0,a1,a2,a3,a4,a5],...]

    New via points can be added dynamically (so long as it is in the future),
    and the list of splines will be recomputed.

    The spline trajectory for each pair of via points is periodically pushed down to the joint controller
    for execution.
    """
    def __init__(self,traj_type=TRAJECTORY_TYPE_CUBIC_SPLINE):
        self.__traj_type=traj_type
        self.__setup_new_trajectory()
        self.trajectory_active=False
        self.vias_last_trajectory = None

    def add_vias_to_trajectory(self,v):
        #V is a list of vias
        if self.num_segment_left()>0: #Append or insert into an existing trajectory
            tstart=self.vias[self.idx_next_via][0]
            tend=self.vias[-1][0]
            tnew=v[0][0]
            if tnew<tstart:
                print('Unable to add vias to trajectory: tstart %f tnew %f'%(tstart,tnew))
                return
            idx_insert=len(self.vias)
            if tnew<tend: #Replace existing vias with new ones starting at tnew
                idx_insert=0
                for i in range(len(self.vias)):
                    if self.vias[i][0]>=tnew:
                        idx_insert=i
                        break
            self.vias =self.vias[0:idx_insert]+copy.deepcopy(v)
        else: #Start of a new trajectory
            self.__setup_new_trajectory()
            self.vias=copy.deepcopy(v)
        self.ts_duration=self.vias[-1][0] - self.vias[0][0]


    def __setup_new_trajectory(self):
        self.vias = []
        self.idx_next_via = 0
        self.ts_start = None
        self.ts_duration = 0
        self.new_trajectory = True
        self.id_seg_next=0
        self.seg_next=None
        self.trajectory_active=False

    def num_segment_left(self):
        return max(0,len(self.vias)-self.idx_next_via-1)
    

    def get_next_segment(self,active_id):
        #The active id is the ID of the currently executing trajectory segment on the uC
        #Ids go from 1 to 255 and loop around
        #Id of 0 marks uCno active segement running
        #This returns the trajectory segment subsequent to the active_id (seg_next)
        #The seg_next can be modified up until active_id==seg_next.id, allow for last minute revision
        #of the next segment to be pushed to the uC
        #An arbitrary number of segments can be sent do the uC sequentially
        #The end of a sequence is marked by sending a [0]*8 spline

        if active_id ==255 and self.trajectory_active: #Marks end of last segment of trajectory
            self.vias_last_trajectory=copy.deepcopy(self.vias) #Store this if want a trajectory history
            self.__setup_new_trajectory()
            return None

        if active_id ==0 and self.id_seg_next==0:   #Marks first segment of trajectory
            self.trajectory_active = True
            self.idx_next_via=0
            self.id_seg_next=1
        elif active_id == self.id_seg_next:         #Marks the seg_next has begun executing, advance counters
                self.idx_next_via=self.idx_next_via+1
                self.id_seg_next = max(1,(self.id_seg_next+1)%255)

        if self.num_segment_left()>0 :
            if self.__traj_type == TRAJECTORY_TYPE_CUBIC_SPLINE: #Recompute spline in case has changed
                self.seg_next= self.generate_cubic_spline_segment(self.vias[self.idx_next_via],self.vias[self.idx_next_via+1])
                self.seg_next.append(self.id_seg_next)
        else:
            self.seg_next=[0]*8 #Marks no more segments to the uC

        return self.seg_next

    def mark_start_of_trajectory(self):
        #Mark the time that the trajectory started
        if self.new_trajectory:
            self.new_trajectory=False
            self.ts_start = time.time()
            print 'Starting new trajectory at:',self.ts_start

    def duration_remaining(self):
        if self.ts_start is None:
            return self.ts_duration
        dt_elapased = time.time() - self.ts_start
        return max(0,self.ts_duration - dt_elapased)

    def generate_cubic_spline_segment(self, v0,v1):
        # Vias is [[pos,vel,time],...]
        #Pad out to 7 floats
        theta0 = v0[1]
        thetadot0 = v0[2]
        thetaf = v1[1]
        thetadotf = v1[2]
        duration =v1[0] - v0[0]
        a0 = theta0
        a1 = thetadot0
        a2 = (3 / duration ** 2) * (thetaf - theta0) - (2 / duration) * thetadot0 - (1 / duration) * thetadotf
        a3 = (-2 / duration ** 3) * (thetaf - theta0) + (1 / duration ** 2) * (thetadotf + thetadot0)
        return [duration, a0, a1, a2, a3, 0, 0]



