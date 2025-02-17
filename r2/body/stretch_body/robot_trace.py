from __future__ import print_function
import os
from stretch_body.device import Device
from stretch_body import hello_utils as hu
from os import makedirs
import yaml
from yaml import CDumper as Dumper
import glob
import time


class RobotTrace(Device):
    """
    The RobotTrace class logs a trace of key robot values to disk periodically
    """
    def __init__(self, robot):
        Device.__init__(self, 'robot_trace')
        self.robot=robot
        self.write_time=[]
        self.path=hu.get_stretch_directory()+'log/trace'
        try:
            makedirs(self.path)
        except OSError:
            pass #Exists
        self.fh=None
        self.time_string=hu.create_time_string()
        self.file_cnt=0
        self.n_samples=0
        samples_per_minute=60*self.robot.params['rates']['SystemMonitorThread_Hz']/self.robot.params['rates']['SystemMonitorThread_trace_downrate_int']
        files_per_minute= samples_per_minute/self.params['n_samples_per_file']
        self.trace_files_max = self.params['duration_limit_minutes']*files_per_minute
        self.trace_cbs=[]
        self.add_trace_callback(self.default_trace)


    def add_trace_callback(self,cb):
        """
        Allow user to add additional sensor fields to the trace logger
        Callback should take the form:

        def my_trace_cb(robot,data):
            data['wacc.ax']=robot.wacc.status['ax']
            ...
            return data
        """
        self.trace_cbs.append(cb)

    def default_trace(self,robot,data):
        data['timestamp']=time.time()
        data['pimu.voltage']=robot.pimu.status['voltage']
        data['pimu.current'] = robot.pimu.status['current']
        data['pimu.cpu_temp'] = robot.pimu.status['cpu_temp']
        data['pimu.charger_connected'] = robot.pimu.status['charger_connected']
        data['pimu.runstop_event'] = robot.pimu.status['runstop_event']
        data['pimu.low_voltage_alert'] = robot.pimu.status['low_voltage_alert']

        data['base.right_wheel.effort_pct']=robot.base.right_wheel.status['effort_pct']
        data['base.right_wheel.pos'] = robot.base.right_wheel.status['pos']
        data['base.right_wheel.vel'] = robot.base.right_wheel.status['vel']

        data['base.left_wheel.effort_pct'] = robot.base.left_wheel.status['effort_pct']
        data['base.left_wheel.pos'] = robot.base.left_wheel.status['pos']
        data['base.left_wheel.vel'] = robot.base.left_wheel.status['vel']

        data['arm.motor.effort_pct'] = robot.arm.motor.status['effort_pct']
        data['arm.pos'] = robot.arm.status['pos']
        data['arm.vel'] = robot.arm.status['vel']

        data['lift.motor.effort_pct'] = robot.lift.motor.status['effort_pct']
        data['lift.pos'] = robot.lift.status['pos']
        data['lift.vel'] = robot.lift.status['vel']
        return data

    def get_trace(self):
        data={}
        for c in self.trace_cbs:
            data=c(self.robot,data)
        return data

    def step(self):
        """Write trace of sensor values to YAML file"""
        ts = time.time()
        if self.fh is None:
            fn = self.path + '/trace_' + hu.get_fleet_id() + '_' +self.time_string+'_'+'%05d'%self.file_cnt+'.yaml'
            self.logger.debug('Creating trace: %s'%fn)
            self.fh = open(fn, 'w+')

        self.fh.write('###\n')
        data={'trace_%04d'%self.n_samples:self.get_trace()}
        yaml.dump(data, self.fh, encoding='utf-8', default_flow_style=False, Dumper=Dumper) #Use C YAML dumper for 5x speed increase over Python
        self.n_samples=self.n_samples+1


        if self.n_samples==self.params['n_samples_per_file']:
            #avg=sum(self.write_time)/len(self.write_time)
            #print('Average write time (ms): %f'%(avg*1000))
            self.fh.close()
            self.file_cnt = self.file_cnt+1
            self.fh=None
            self.n_samples=0
            self.cleanup()

        self.write_time.append(time.time() - ts)

    def cleanup(self):
        """
        Limit the total number of trace files to approximately self.trace_files_max
        To avoid filling up disk
        """
        all_files = glob.glob(hu.get_stretch_directory() + 'log/trace/*.yaml')
        if len(all_files)<=self.trace_files_max:
            return
        all_files.sort()  # Sorted chronological (id 0 is oldest)
        n_del = int(len(all_files)-self.trace_files_max)
        cmd='rm '
        for i in range(n_del):
            cmd=cmd+all_files[i]+' '
        os.system(cmd)
