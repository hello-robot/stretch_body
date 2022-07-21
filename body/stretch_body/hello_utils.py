from __future__ import print_function
import yaml
import math
import os
import time
import logging
import numpy as np


def print_stretch_re_use():
    print("For use with S T R E T C H (R) RESEARCH EDITION from Hello Robot Inc.")
    print("---------------------------------------------------------------------\n")

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

def confirm(question):
    reply = None
    while reply not in ("y", "n"):
        reply = input(question + " (y/n)").lower()
    return (reply == "y")


def get_display():
    return os.environ.get('DISPLAY', None)

def get_fleet_id():
    return os.environ['HELLO_FLEET_ID']

def set_fleet_id(id):
    os.environ['HELLO_FLEET_ID']=id

def get_fleet_directory():
    return os.environ['HELLO_FLEET_PATH']+'/'+get_fleet_id()+'/'

def set_fleet_directory(fleet_path,fleet_id):
    os.environ['HELLO_FLEET_ID'] = fleet_id
    os.environ['HELLO_FLEET_PATH'] = fleet_path

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

def read_fleet_yaml(f,fleet_dir=None):
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
        if fleet_dir is None:
            fleet_dir=get_fleet_directory()
        with open(fleet_dir+f, 'r') as s:
            p = yaml.load(s,Loader=yaml.FullLoader)
            return {} if p is None else p
    except IOError:
        return {}

def write_fleet_yaml(fn,rp,fleet_dir=None,header=None):
    if fleet_dir is None:
        fleet_dir = get_fleet_directory()
    if fleet_dir[-1]!='/':
        fleet_dir+='/'
    with open(fleet_dir+fn, 'w') as yaml_file:
        if header is not None:
            yaml_file.write(header)
        yaml.dump(rp, yaml_file, default_flow_style=False)


def overwrite_dict(overwritee_dict, overwriter_dict):
    for k in overwriter_dict.keys():
        if k in overwritee_dict:
            if (type(overwritee_dict[k])==type(overwriter_dict[k])) or (type(overwritee_dict[k])==int and type(overwriter_dict[k])==float) or (type(overwritee_dict[k])==float and type(overwriter_dict[k])==int):
                if type(overwritee_dict[k])==dict:
                    overwrite_dict(overwritee_dict[k],overwriter_dict[k])
                else:
                    overwritee_dict[k]=overwriter_dict[k]
            else:
                print('Overwritting Robot Params. Type mismatch for key:',k)
                print('Overwriter',overwriter_dict[k])
                print('Overwritee', overwritee_dict[k])
        else: #If key not present, add anyhow (useful for adding new params)
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
    """Track timing statistics for control loops
    """

    def __init__(self, loop_name, target_loop_rate):
        self.loop_name = loop_name
        self.target_loop_rate = target_loop_rate
        self.ts_loop_start = None
        self.ts_loop_end = None
        self.last_ts_loop_end = None
        self.status = {'execution_time_s': 0,
                       'curr_rate_hz': 0,
                       'avg_rate_hz': 0,
                       'supportable_rate_hz': 0,
                       'min_rate_hz': float('inf'),
                       'max_rate_hz': 0,
                       'std_rate_hz': 0,
                       'missed_loops': 0,
                       'num_loops': 0}
        self.logger = logging.getLogger(self.loop_name)
        self.curr_rate_history = []
        self.supportable_rate_history = []
        self.n_history = 100
        self.debug_freq = 50
        self.sleep_time_s = 0.0

    def pretty_print(self):
        print('--------- TimingStats %s -----------' % self.loop_name)
        print('Target rate (Hz): %.2f' % self.target_loop_rate)
        print('Current rate (Hz): %.2f' % self.status['curr_rate_hz'])
        print('Average rate (Hz): %.2f' % self.status['avg_rate_hz'])
        print('Standard deviation of rate history (Hz): %.2f' % self.status['std_rate_hz'])
        print('Min rate (Hz): %.2f' % self.status['min_rate_hz'])
        print('Max rate (Hz): %.2f' % self.status['max_rate_hz'])
        print('Supportable rate (Hz): %.2f' % self.status['supportable_rate_hz'])
        print('Warnings: %d out of %d' % (self.status['missed_loops'], self.status['num_loops']))

    def mark_loop_start(self):
        self.ts_loop_start=time.time()

    def mark_loop_end(self):
        self.status['num_loops'] += 1

        # First two cycles initialize vars / log
        if not self.ts_loop_start:
            return
        if self.ts_loop_end is None:
            self.ts_loop_end = time.time()
            return
        if self.last_ts_loop_end is None:
            self.last_ts_loop_end = self.ts_loop_end
            self.ts_loop_end = time.time()
            self.status['execution_time_s'] = self.ts_loop_end - self.ts_loop_start
            self.status['curr_rate_hz'] = 1.0 / (self.ts_loop_end - self.last_ts_loop_end)
            return

        # Calculate average and supportable loop rate **must be done before marking loop end**
        if len(self.curr_rate_history) >= self.n_history:
            self.curr_rate_history.pop(0)
        self.curr_rate_history.append(self.status['curr_rate_hz'])
        self.status['avg_rate_hz'] = np.mean(self.curr_rate_history)
        self.status['std_rate_hz'] = np.std(self.curr_rate_history)
        if len(self.supportable_rate_history) >= self.n_history:
            self.supportable_rate_history.pop(0)
        self.supportable_rate_history.append(1.0 / self.status['execution_time_s'])
        self.status['supportable_rate_hz'] = np.mean(self.supportable_rate_history)

        # Log timing stats **must be done before marking loop end**
        if self.status['num_loops'] % self.debug_freq == 0:
            self.logger.debug('--------- TimingStats %s %d -----------' % (self.loop_name, self.status['num_loops']))
            self.logger.debug('Target rate: %f' % self.target_loop_rate)
            self.logger.debug('Current rate (Hz): %f' % self.status['curr_rate_hz'])
            self.logger.debug('Average rate (Hz): %f' % self.status['avg_rate_hz'])
            self.logger.debug('Standard deviation of rate history (Hz): %f' % self.status['std_rate_hz'])
            self.logger.debug('Min rate (Hz): %f' % self.status['min_rate_hz'])
            self.logger.debug('Max rate (Hz): %f' % self.status['max_rate_hz'])
            self.logger.debug('Supportable rate (Hz): %f' % self.status['supportable_rate_hz'])
            self.logger.debug('Standard deviation of supportable rate history (Hz): %f' % np.std(self.supportable_rate_history))
            self.logger.debug('Warnings: %d out of %d' % (self.status['missed_loops'], self.status['num_loops']))
            self.logger.debug('Sleep time (s): %f' % self.sleep_time_s)

        # Calculate current loop rate & execution time
        self.last_ts_loop_end = self.ts_loop_end
        self.ts_loop_end = time.time()
        self.status['execution_time_s'] = self.ts_loop_end - self.ts_loop_start
        self.status['curr_rate_hz'] = 1.0 / (self.ts_loop_end - self.last_ts_loop_end)
        self.status['min_rate_hz'] = min(self.status['curr_rate_hz'], self.status['min_rate_hz'])
        self.status['max_rate_hz'] = max(self.status['curr_rate_hz'], self.status['max_rate_hz'])

        # Calculate sleep time to achieve desired loop rate
        self.sleep_time_s = (1 / self.target_loop_rate) - self.status['execution_time_s']
        if self.sleep_time_s < 0.0:
            self.status['missed_loops'] += 1
            if self.status['missed_loops'] == 1:
                self.logger.debug('Missed target loop rate of %.2f Hz for %s. Currently %.2f Hz' % (self.target_loop_rate, self.loop_name, self.status['curr_rate_hz']))

    def generate_rate_histogram(self, save=None):
        import matplotlib.pyplot as plt
        fig, axs = plt.subplots(1, 1, sharey=True, tight_layout=True)
        fig.suptitle('Distribution of loop rate (Hz). Target of %.2f ' % self.target_loop_rate)
        axs.hist(x=self.curr_rate_history, bins='auto', color='#0504aa', alpha=0.7, rwidth=0.85)
        plt.show() if save is None else plt.savefig(save)

    def get_loop_sleep_time(self):
        """
        Returns
        -------
        float : Time to sleep for to hit target loop rate
        """
        return max(0.0, self.sleep_time_s)

    
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

def evaluate_polynomial_at(poly, t):
    """Evaluate a quintic polynomial at a given time.

    Parameters
    ----------
    poly : List(float)
        Represents a quintic polynomial as a coefficients array [a0, a1, a2, a3, a4, a5].
        The polynomial is f(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
    t : float
        the time in seconds at which to evaluate the polynomial

    Returns
    -------
    Tuple(float)
        array with three elements: evaluated position, velocity, and acceleration.
    """
    a = [float(elem) for elem in poly]
    t = float(t)
    pos = a[0] + (a[1]*t) + (a[2]*t**2) + (a[3]*t**3) + (a[4]*t**4) + (a[5]*t**5)
    vel = a[1] + (2*a[2]*t) + (3*a[3]*t**2) + (4*a[4]*t**3) + (5*a[5]*t**4)
    accel = (2*a[2]) + (6*a[3]*t) + (12*a[4]*t**2) + (20*a[5]*t**3)
    return (pos, vel, accel)

def is_segment_feasible(segment, v_des, a_des, t=0.0, inc=0.1):
    """Determine whether a segment adheres to dynamic limits.

    Parameters
    ----------
    segment : List
        Represents a segment of a waypoint trajectory as a list of length eight,
        structured like [duration_s, a0, a1, a2, a3, a4, a5, segment_id].
    v_des : float
        Velocity limit that the segment shouldn't exceed
    a_des : float
        Acceleration limit that the segment shouldn't exceed
    t : float
        optional, time in seconds at which to begin checking segment
    inc : float
        optional, increment in seconds at which the polynomial is evaluated along the segment

    Returns
    -------
    bool
        whether the segment is feasible
    """
    v_des = float(v_des)
    a_des = float(a_des)
    while t < segment[0]:
        _, vel_t, acc_t = evaluate_polynomial_at(segment[1:-1], t)
        if abs(vel_t) > v_des or abs(acc_t) > a_des:
            return False

        t = min(segment[0], t + inc)

    return True

def generate_quintic_polynomial(i, f):
    """Generate quintic polynomial from two points

    Parameters
    ----------
    i : List(float)
        Represents the first waypoint as a list, [time, pos, vel, accel]
    f : List(float)
        Represents the second waypoint as a list, [time, pos, vel, accel]

    Returns
    -------
    List(float)
        Represents a quintic polynomial as a coefficients + duration array [duration, a0, a1, a2, a3, a4, a5].
        The polynomial is f(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
    """
    i = [float(elem) for elem in i]
    f = [float(elem) for elem in f]
    duration = f[0] - i[0]
    a0 = i[1]
    a1 = i[2]
    a2 = i[3] / 2
    a3 = (20 * f[1] - 20 * i[1] - (8 * f[2] + 12 * i[2]) * duration - (3 * i[3] - f[3]) * (duration ** 2)) / (2 * (duration ** 3))
    a4 = (30 * i[1] - 30 * f[1] + (14 * f[2] + 16 * i[2]) * duration + (3 * i[3] - 2 * f[3]) * (duration ** 2)) / (2 * (duration ** 4))
    a5 = (12 * f[1] - 12 * i[1] - (6 * f[2] + 6 * i[2]) * duration - (i[3] - f[3]) * (duration ** 2)) / (2 * (duration ** 5))
    return [duration, a0, a1, a2, a3, a4, a5]

def generate_cubic_polynomial(i, f):
    """Generate cubic polynomial from two points

    Parameters
    ----------
    i : List(float)
        Represents the first waypoint as a list, [time, pos, vel]
    f : List(float)
        Represents the second waypoint as a list, [time, pos, vel]

    Returns
    -------
    List(float)
        Represents a cubic polynomial as a coefficients + duration array [duration, a0, a1, a2, a3, 0, 0].
        The polynomial is f(t) = a0 + a1*t + a2*t^2 + a3*t^3
    """
    i = [float(elem) for elem in i]
    f = [float(elem) for elem in f]
    duration = f[0] - i[0]
    a0 = i[1]
    a1 = i[2]
    a2 = (3 / duration ** 2) * (f[1] - i[1]) - (2 / duration) * i[2] - (1 / duration) * f[2]
    a3 = (-2 / duration ** 3) * (f[1] - i[1]) + (1 / duration ** 2) * (f[2] + i[2])
    return [duration, a0, a1, a2, a3, 0, 0]

def generate_linear_polynomial(i, f):
    """Generate linear polynomial from two points

    Parameters
    ----------
    i : List(float)
        Represents the first waypoint as a list, [time, pos]
    f : List(float)
        Represents the second waypoint as a list, [time, pos]

    Returns
    -------
    List(float)
        Represents a linear polynomial as a coefficients + duration array [duration, a0, a1, 0, 0, 0, 0].
        The polynomial is f(t) = a0 + a1*t
    """
    i = [float(elem) for elem in i]
    f = [float(elem) for elem in f]
    duration = f[0] - i[0]
    a0 = i[1]
    a1 = (f[1] - i[1]) / duration
    return [duration, a0, a1, 0, 0, 0, 0]

def get_pose_diff(pose0, pose1, translation_atol=2e-3, rotation_atol=2e-2):
    """Return the motion required to get from pose 0 to pose 1.

    Assumed that between pose 0 and pose 1, there has only been
    either a translation or rotation motion.

    Parameters
    ----------
    pose0 : Tuple(float, float, float)
        x, y, theta in meters and radians
    pose1 : Tuple(float, float, float)
        x, y, theta in meters and radians

    Returns
    -------
    float, float
        Tuple (dx, dtheta) of translation and rotation required to
        move from pose0 to pose1
    """
    x0, y0, theta0 = pose0
    x1, y1, theta1 = pose1
    theta0 = np.arctan2(np.sin(theta0), np.cos(theta0)) # constrains to [-pi, pi]
    theta1 = np.arctan2(np.sin(theta1), np.cos(theta1)) # constrains to [-pi, pi]

    # TODO: For now, we use a simplified motion model where we assume
    # that every motion is either a translation OR a rotation,
    # and the translation is either straight forward or straight back
    if np.isclose(x0, x1, atol=translation_atol) and np.isclose(y0, y1, atol=translation_atol):
        return 0.0, theta1 - theta0

    if np.isclose(theta0, theta1, atol=rotation_atol):
        dx = x1 - x0
        dy = y1 - y0
        drive_angle = math.atan2(dy, dx)
        distance = math.hypot(dy, dx)
        if np.isclose(drive_angle, theta0, atol=rotation_atol):
            return distance, 0.0
        opposite_theta0 = np.arctan2(np.sin(theta0 + np.pi), np.cos(theta0 + np.pi)) # constrains to [-pi, pi]
        if np.isclose(drive_angle, opposite_theta0, atol=rotation_atol):
            return -distance, 0.0

    return 0.0, 0.0
