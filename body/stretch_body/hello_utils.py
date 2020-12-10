import yaml
import math
import os
import time


def print_stretch_re_use():
    print("For use with S T R E T C H (TM) RESEARCH EDITION from Hello Robot Inc.\n")

def create_time_string():
    t = time.localtime()
    time_string = str(t.tm_year) + str(t.tm_mon).zfill(2) + str(t.tm_mday).zfill(2) + str(t.tm_hour).zfill(2) + str(t.tm_min).zfill(2) + str(t.tm_sec).zfill(2)
    return time_string


def deg_to_rad(x):
    return math.pi*x/180.0

def rad_to_deg(x):
    return 180.0*x/math.pi


def get_fleet_id():
    return os.environ['HELLO_FLEET_ID']

def set_fleet_id(id):
    os.environ['HELLO_FLEET_ID']=id

def get_fleet_directory():
    return os.environ['HELLO_FLEET_PATH']+'/'+get_fleet_id()+'/'

def read_fleet_yaml(fn):
    s = file(get_fleet_directory()+fn, 'r')
    p = yaml.load(s,Loader=yaml.FullLoader)
    if p is None:
        return {}
    else:
        return p

def write_fleet_yaml(fn,rp):
    with open(get_fleet_directory()+fn, 'w') as yaml_file:
        yaml.dump(rp, yaml_file, default_flow_style=False)

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
        print 'Timer Stat -- Avg: ', str(self.av), 'Max: ', str(self.mx)

    
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


class SystemTimestamp:
    def __init__(self, secs=0, nsecs=0):
        """
        secs: seconds since epoch
        nsecs: nanoseconds since seconds (since epoch)
        """
        self.secs = int(secs)
        self.nsecs = int(nsecs)


    def from_wall_time(self):
        return self.from_secs(time.time())

    def from_secs(self,secs):
        s = int(secs)
        n = int((secs - s) * 1000000000)
        return SystemTimestamp(s,n)

    def from_nsecs(self,nsecs):
        s=int(nsecs)/1000000000
        n=int(nsecs-s*1000000000)
        return SystemTimestamp(s, n)

    def from_usecs(self,usecs):
        s=int(usecs)/1000000
        n=int(usecs-s*1000000)*1000
        return SystemTimestamp(s, n)

    def to_secs(self):
        return self.secs+self.nsecs/1000000000.0

    def to_msecs(self):
        return self.to_secs()*1000

    def to_nsecs(self):
        return self.secs*1000000000 + self.nsecs

    def to_usecs(self):
        return self.secs * 1000000 + self.nsecs/1000

    def __add__(self,ts ):
        return SystemTimestamp(self.secs + ts.secs, self.nsecs+ts.nsecs)
    def __sub__(self,ts ):
        return SystemTimestamp(self.secs - ts.secs, self.nsecs-ts.nsecs)
    def __repr__(self):
        return '%.6f' % self.to_secs()
