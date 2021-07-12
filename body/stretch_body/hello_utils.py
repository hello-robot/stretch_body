from __future__ import print_function
import yaml
import math
import os
import time
import logging
import numpy

def print_stretch_re_use():
    print("For use with S T R E T C H (TM) RESEARCH EDITION from Hello Robot Inc.\n")

def create_time_string(time_format='%Y%m%d%H%M%S'):
    """Returns current time formatted as `time_format`

    Parameters
    ----------
    time_format : str
        Refer https://docs.python.org/3/library/time.html#time.strftime for options

    Returns
    -------
    str
        time as string in requested format
    """
    return time.strftime(time_format)

def deg_to_rad(x):
    return math.pi*x/180.0

def rad_to_deg(x):
    return 180.0*x/math.pi

def get_display():
    return os.environ.get('DISPLAY', None)

def get_fleet_id():
    return os.environ['HELLO_FLEET_ID']

def set_fleet_id(id):
    os.environ['HELLO_FLEET_ID']=id

def get_fleet_directory():
    return os.environ['HELLO_FLEET_PATH']+'/'+get_fleet_id()+'/'

def get_stretch_directory(sub_directory=''):
    """Returns path to stretch_user dir if HELLO_FLEET_PATH env var exists

    Parameters
    ----------
    sub_directory : str
        valid sub_directory within stretch_user/

    Returns
    -------
    str
        dirpath to stretch_user/ or dir within it if stretch_user/ exists, else /tmp
    """
    base_path = os.environ.get('HELLO_FLEET_PATH', None)
    full_path = base_path + '/' + sub_directory if base_path is not None else '/tmp/'
    return full_path

def read_fleet_yaml(f):
    """Reads yaml by filename from fleet directory

    Parameters
    ----------
    f : str
        filename of the yaml

    Returns
    -------
    dict
        yaml as dictionary if valid file, else empty dict
    """
    try:
        with open(get_fleet_directory()+f, 'r') as s:
            p = yaml.load(s,Loader=yaml.FullLoader)
            return {} if p is None else p
    except IOError:
        return {}

def write_fleet_yaml(fn,rp):
    with open(get_fleet_directory()+fn, 'w') as yaml_file:
        yaml.dump(rp, yaml_file, default_flow_style=False)

def overwrite_dict(overwritee_dict, overwriter_dict):
    for k in overwriter_dict.keys():
        if k in overwritee_dict:
            if type(overwritee_dict[k])==type(overwriter_dict[k]):
                if type(overwritee_dict[k])==dict:
                    overwrite_dict(overwritee_dict[k],overwriter_dict[k])
                else:
                    overwritee_dict[k]=overwriter_dict[k]
            else:
                print('Overwritting Factory Params with User Params. Type mismatch for key:',k)
        else: #If key not present, add anyhow (useful for adding new end_of_arm)
            overwritee_dict[k] = overwriter_dict[k]

def pretty_print_dict(title, d):
    """Print human readable representation of dictionary to terminal

    Parameters
    ----------
    title : str
        header title under which the dictionary is printed
    d : dict
        the dictionary to pretty print
    """
    print('-------- {0} --------'.format(title))
    for k in d.keys():
        if type(d[k]) != dict:
            print(k, ' : ', d[k])
    for k in d.keys():
        if type(d[k]) == dict:
            pretty_print_dict(k, d[k])



class LoopStats():
    """
    Track timing statistics for control loops
    """
    def __init__(self,loop_name,target_loop_rate):
        self.loop_name=loop_name
        self.target_loop_rate=target_loop_rate
        self.ts_loop_start=None
        self.ts_loop_end=None
        self.status={'loop_rate_hz':0, 'loop_rate_avg_hz':0, 'loop_rate_min_hz':10000000, 'loop_rate_max_hz':0,'loop_rate_std':0, 'execution_time_ms':0, 'loop_warns':0}
        self.logger = logging.getLogger()
        self.n_log=100
        self.log_idx=0
        self.rate_log=None
        self.log_rate_hz=1.0
        self.warned_yet=False
        self.sleep_time_s =0.001
        self.loop_cycles=0

    def pretty_print(self):
        print('--------- TimingStats %s -----------'%self.loop_name)
        print('Target rate: %f'%self.target_loop_rate)
        print('Current rate (Hz): %f' % self.status['loop_rate_hz'])
        print('Average rate (Hz): %f' % self.status['loop_rate_avg_hz'])
        print('Min rate (Hz): %f' % self.status['loop_rate_min_hz'])
        print('Max rate (Hz): %f' % self.status['loop_rate_max_hz'])
        print('Current execution time (ms): %f' % self.status['execution_time_ms'])
        print('Execution time supports rate of (Hz) %f'%(1000.0/self.status['execution_time_ms']))
        print('Warnings: %d out of %d'%(self.status['loop_warns'],self.loop_cycles))

    def mark_loop_start(self):
        self.ts_loop_start=time.time()

    def mark_loop_end(self):
        self.loop_cycles+=1
        #First two cycles initialize vars / log
        if not self.ts_loop_start:
            return
        if self.ts_loop_end is None:
            self.ts_loop_end=time.time()
            return
        end_last=self.ts_loop_end
        self.ts_loop_end=time.time()
        self.status['execution_time_ms']=(self.ts_loop_end-self.ts_loop_start)*1000
        self.status['loop_rate_hz']=1.0/(self.ts_loop_end-end_last)
        self.status['loop_rate_min_hz']=min(self.status['loop_rate_hz'], self.status['loop_rate_min_hz'])
        self.status['loop_rate_max_hz'] = max(self.status['loop_rate_hz'], self.status['loop_rate_max_hz'])
        if type(self.rate_log)==type(None):
            self.rate_log=numpy.array([self.status['loop_rate_hz']] * self.n_log)
        self.rate_log[self.log_idx]=self.status['loop_rate_hz']
        self.log_idx=(self.log_idx+1)%self.n_log
        self.status['loop_rate_avg_hz']=numpy.average(self.rate_log)

        self.sleep_time_s = (1 / self.target_loop_rate) - (self.status['execution_time_ms'] / 1000)
        if self.sleep_time_s < .001:
            self.status['loop_warns'] += 1
            if not self.warned_yet:
                self.warned_yet=True
                self.logger.debug('Target loop rate of %f Hz for %s not possible. Capable of %.2f Hz' % (self.target_loop_rate,self.loop_name,(1000.0/self.status['execution_time_ms'])))


    def display_rate_histogram(self):
        import matplotlib.pyplot as plt
        fig, axs = plt.subplots(1, 1, sharey=True, tight_layout=True)
        fig.suptitle('Distribution of loop rate (Hz). Target of %f '%self.target_loop_rate)
        axs.hist(x=self.rate_log, bins='auto', color='#0504aa', alpha=0.7, rwidth=0.85)
        plt.show()

    def get_loop_sleep_time(self):
        """
        :return: Time to sleep for to hit target loop rate
        """
        return max(.001,self.sleep_time_s)


    
class ThreadServiceExit(Exception):
    """
    Custom exception which is used to trigger the clean exit
    of all running threads and the main program.
    """
    pass


#Signal handler, must be set from main thread
def thread_service_shutdown(signum, frame):
    print('Caught signal %d' % signum)
    raise ThreadServiceExit

