from __future__ import print_function
import yaml
import math
import os
import time


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

class TimerStats():
    def __init__(self):
        self.av = None
        self.mx = None
        self.count = 0


    def update(self, duration):
        if self.av is None:
            self.av = duration
        else:
            self.av = ((self.count * self.av) + duration) / (self.count + 1)
            
        if self.mx is None:
            self.mx = duration
        elif self.mx < duration:
            self.mx = duration

        self.count = self.count + 1

    def output_string(self):
        out = 'timer: av = ' + str(self.av) + ' , max = ' + str(self.mx)
        return out

    def pretty_print(self):
        print('Timer Stat -- Avg: ', str(self.av), 'Max: ', str(self.mx))

    
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

