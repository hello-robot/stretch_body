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
        makedirs(self.path, exist_ok=True)
        self.fh=None
        self.time_string=hu.create_time_string()
        self.file_cnt=0
        self.n_samples=0
        self.ts_last=time.time()

    def get_trace(self):
        d={}
        d['timestamp']=time.time()
        d['pimu.voltage']=self.robot.pimu.status['voltage']
        d['pimu.current'] = self.robot.pimu.status['current']
        d['pimu.temp'] = self.robot.pimu.status['temp']
        d['pimu.cpu_temp'] = self.robot.pimu.status['cpu_temp']
        d['pimu.charger_connected'] = self.robot.pimu.status['charger_connected']
        d['pimu.runstop_event'] = self.robot.pimu.status['runstop_event']
        return d

    def step(self):
        """Write trace of sensor values to YAML file"""
        steps_per_minute = self.robot.params['rates']['NonDXLStatusThread_Hz'] / self.robot.params['rates']['NonDXLStatusThread_trace_downrate_int']
        secs_per_file = self.params['n_samples_per_file'] / steps_per_minute
        print('STEP',time.time()-self.ts_last,steps_per_minute,secs_per_file)
        self.ts_last=time.time()
        if self.fh is None:
            fn = self.path + '/trace_' + hu.get_fleet_id() + '_' +self.time_string+'_'+'%05d'%self.file_cnt+'.yaml'
            self.logger.debug('Creating trace: %s'%fn)
            self.fh = open(fn, 'w+')
        ts=time.time()
        self.fh.write('###\n')
        data={'trace_%04d'%self.n_samples:self.get_trace()}
        yaml.dump(data, self.fh, encoding='utf-8', default_flow_style=False, Dumper=Dumper) #Use C YAML dumper for 5x speed increase over Python
        self.n_samples=self.n_samples+1
        self.write_time.append(time.time()-ts)
        #avg_ms=1000*sum(self.write_time)/len(self.write_time)
        #print("Trace dump avg time (ms): %f" % avg_ms)
        if self.n_samples==self.params['n_samples_per_file']:
            self.fh.close()
            self.file_cnt = self.file_cnt+1
            self.fh=None
            self.n_samples=0

    # def cleanup_trace_data(self,n_max=None):
    #     """
    #     Limit the total number of trace files to approximately 1 hours worth
    #     Remove all files that are older than one day
    #     """
    #     if n_max is None:
    #         steps_per_minute=self.params['rates']['NonDXLStatusThread_Hz']/self.robot.trace_downrate_int
    #         secs_per_file = self.params['n_samples_per_file']/steps_per_minute
    #         n_max = 60*files_per_minute
    #
    #     all_files = glob.glob(hu.get_stretch_directory() + 'log/trace/*.yaml')
    #     all_files.sort()  # Sorted chronological (id 0 is oldest)
    #     if len(all_files)<=n_max:
    #         return
    #     n_del = len(all_files)-n_max
    #     for i in range(n_del):
    #         os.system('rm %s' % all_files[i])
