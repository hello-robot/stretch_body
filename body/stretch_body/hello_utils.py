from __future__ import print_function
import yaml
import math
import os
import pwd
import time
import logging
import numpy as np
import sys
import signal
import pathlib
import numbers
import subprocess
import pyrealsense2 as rs
import cv2
from filelock import FileLock, Timeout


def print_stretch_re_use():
    print("For use with S T R E T C H (R) from Hello Robot Inc.")
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
        else:
            if fleet_dir[-1] != '/':
                fleet_dir = fleet_dir + '/'
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
    """Merge two dictionaries while overwriting common keys and
    report errors when values of the same key differ in Python
    type. The result gets stored in `overwritee_dict`.

    Parameters
    ----------
    overwritee_dict : dict
        The dictionary which will be overwritten. Use this as the merged result.
    overwriter_dict : dict
        The dictionary which will overwrite.

    Returns
    -------
    bool
        True if no mismatches were found during the overwrite, False otherwise.
    """
    no_mismatches = True
    for k in overwriter_dict.keys():
        if k in overwritee_dict:
            if (isinstance(overwritee_dict[k], dict) and isinstance(overwriter_dict[k], dict)):
                sub_no_mismatches = overwrite_dict(overwritee_dict[k], overwriter_dict[k])
                no_mismatches = no_mismatches and sub_no_mismatches
            else:
                if (type(overwritee_dict[k]) == type(overwriter_dict[k])) or (isinstance(overwritee_dict[k], numbers.Real) and isinstance(overwriter_dict[k], numbers.Real)):
                    overwritee_dict[k] = overwriter_dict[k]
                else:
                    no_mismatches = False
                    print('stretch_body.hello_utils.overwrite_dict ERROR: Type mismatch for key={0}, between overwritee={1} and overwriter={2}'.format(k, overwritee_dict[k], overwriter_dict[k]), file=sys.stderr)
        else: #If key not present, add anyhow (useful for overlaying params)
            overwritee_dict[k] = overwriter_dict[k]
    return no_mismatches

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
        self.last_ts_loop_start = None
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
        self.ts_0=time.time()

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
        self.status['num_loops'] += 1
        self.ts_loop_start=time.time()

        if self.last_ts_loop_start is None: #Wait until have sufficient data
            self.last_ts_loop_start=self.ts_loop_start
            return

        self.status['curr_rate_hz'] = 1.0 / (self.ts_loop_start - self.last_ts_loop_start)
        self.status['min_rate_hz'] = min(self.status['curr_rate_hz'], self.status['min_rate_hz'])
        self.status['max_rate_hz'] = max(self.status['curr_rate_hz'], self.status['max_rate_hz'])


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

        self.last_ts_loop_start = self.ts_loop_start

        # Calculate sleep time to achieve desired loop rate
        self.sleep_time_s = (1 / self.target_loop_rate) - self.status['execution_time_s']
        if self.sleep_time_s < 0.0 and time.time()-self.ts_0>5.0: #Allow 5s for timing to stabilize on startup
            self.status['missed_loops'] += 1
            if self.status['missed_loops'] == 1:
                self.logger.debug('Missed target loop rate of %.2f Hz for %s. Currently %.2f Hz' % (self.target_loop_rate, self.loop_name, self.status['curr_rate_hz']))

    def mark_loop_end(self):
        # First two cycles initialize vars / log
        if self.ts_loop_start is None:
            return
        self.ts_loop_end = time.time()
        self.status['execution_time_s'] = self.ts_loop_end - self.ts_loop_start


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

    def wait_until_ready_to_run(self,sleep=.0005):
        if self.ts_loop_start is None:
            time.sleep(.01)
            return True
        while time.time()-self.ts_loop_start<(1/self.target_loop_rate):
            time.sleep(sleep)


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
    success: bool
        whether the segment is feasible
    max_v: float
        Maximum velocity of spline
    max_a: float
        Maximum acceleration of spline
    """
    v_des = float(v_des)
    a_des = float(a_des)
    max_v = 0
    max_a =0
    success=True
    while t < segment[0]:
        x_t, vel_t, acc_t = evaluate_polynomial_at(segment[1:-1], t)
        max_v=max(max_v,abs(vel_t))
        max_a = max(max_a, abs(acc_t))
        if abs(vel_t) > v_des or abs(acc_t) > a_des:
            success=False
        t = min(segment[0], t + inc)
    return success, max_v, max_a

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

def pseudo_N_to_effort_pct(joint_name,contact_thresh_N):
    import stretch_body.robot_params
    d = stretch_body.robot_params.RobotParams.get_params()[1] #Get complete param dict
    motor_name = {'arm':'hello-motor-arm', 'lift': 'hello-motor-lift', 'base':'hello-motor-left-wheel'}[joint_name]
    i_feedforward = 0 if joint_name =='base' else d[joint_name]['i_feedforward']
    iMax_name = 'iMax_neg' if contact_thresh_N<0 else 'iMax_pos'
    contact_A = (contact_thresh_N / d[joint_name]['force_N_per_A'])+i_feedforward
    return 100*contact_A / abs(d[motor_name]['gains'][iMax_name])


def check_deprecated_contact_model_base(joint,method_name, contact_thresh_N,contact_thresh ):
    """
    With RE2 we are transitioning entire stretch fleet to use new API (and effort_pct for the contact model)
    Catch older code that is using the older API and require updating of code
    """

    #Check if old parameters still found in YAML
    if ('contact_thresh_max_N' in joint.params) or ('contact_thresh_N' in joint.params):
        msg="Robot is using out-of-date contact parameters"
        msg=msg+'Please run tool RE1_migrate_contacts.py before continuing.\n'
        msg=msg+'For more details, see https://forum.hello-robot.com/t/476 \n'
        msg = msg + 'In method %s.%s' % (joint.name, method_name)
        print(msg)
        joint.logger.error(msg)
        sys.exit(1)

    #Check if code is passing in old values
    if contact_thresh_N is not None:
        msg='Use of parameter contact_thresh_N is no longer supported\n'
        msg= msg + 'Update your code to use (contact_thresh)\n'
        msg = msg +  'For more details, see https://forum.hello-robot.com/t/476\n'
        msg=msg+'In method %s.%s'%(joint.name,method_name)
        print(msg)
        joint.logger.error(msg)
        sys.exit(1)

def check_deprecated_contact_model_prismatic_joint(joint,method_name, contact_thresh_pos_N,contact_thresh_neg_N,contact_thresh_pos,contact_thresh_neg ):
    """
    With RE2 we are transitioning entire stretch fleet to use new API (and effort_pct for the contact model)
    Catch older code that is using the older API and require updating of code
    For code that was, for example:
        arm.move_to(x_m=0.1, contact_thresh_pos_N=30.0, contact_thresh_neg_N=-30.0)
    Should now be:
        arm.move_to(x_m=0.1, contact_thresh_pos=pseudo_N_to_effort_pct(30.0),
            contact_thresh_neg=pseudo_N_to_effort_pct(-30.0))
    """

    #Check if old parameters still found in YAML
    if ('contact_thresh_max_N' in joint.params) or ('contact_thresh_N' in joint.params) or ('homing_force_N' in joint.params):
        msg="Robot is using out-of-date contact parameters\n"
        msg=msg+'Please run tool RE1_migrate_contacts.py before continuing.\n'
        msg=msg+'For more details, see https://forum.hello-robot.com/t/476 \n'
        msg = msg + 'In method %s.%s' % (joint.name, method_name)
        print(msg)
        joint.logger.error(msg)
        sys.exit(1)

    #Check if code is passing in old values
    if contact_thresh_pos_N is not None or contact_thresh_neg_N is not None:
        msg='Use of parameters contact_thresh_pos_N and contact_thresh_neg_N is no longer supported\n'
        msg= msg + 'Update your code to use (contact_thresh_pos, contact_thresh_neg)\n'
        msg = msg +  'For more details, see https://forum.hello-robot.com/t/476\n'
        msg=msg+'In method %s.%s'%(joint.name,method_name)
        print(msg)
        joint.logger.error(msg)
        sys.exit(1)

    #Check if code is passing in new values but not yet migrated
    if contact_thresh_pos is not None or contact_thresh_neg is not None \
            or (contact_thresh_pos is None and contact_thresh_neg is None):
        if ('contact_models' not in joint.params) or ('effort_pct' not in joint.params['contact_models']) or\
                ('contact_thresh_default' not in joint.params['contact_models']['effort_pct']) or\
                ('contact_thresh_homing' not in joint.params['contact_models']['effort_pct']) :
            msg='Effort_Pct contact parameters not available\n'
            msg = msg + 'Please run tool RE1_migrate_contacts.py before continuing.\n'
            msg = msg + 'For more details, see https://forum.hello-robot.com/t/476 \n'
            msg=msg+'In method %s.%s'%(joint.name,method_name)
            print(msg)
            joint.logger.error(msg)
            sys.exit(1)

def check_file_exists(fn):
    if os.path.exists(fn):
        return True
    else:
        print(f"Unable to find file: {fn}")
        return False

def to_parabola_transform(x):
    if x<0:
        return  -1*(abs(x)**2)
    else:
        return x**2

def map_to_range(value, new_min, new_max):
    # Ensure value is between 0 and 1
    value = max(0, min(1, value))
    mapped_value = (value - 0) * (new_max - new_min) / (1 - 0) + new_min
    return mapped_value

def get_video_devices():
    """
    Returns dictionary of all the enumerated video devices found in the robot 
    """
    command = 'v4l2-ctl --list-devices'
    lines = subprocess.getoutput(command).split('\n')
    lines = [l.strip() for l in lines if l != '']
    cameras = [l for l in lines if not ('/dev/' in l)]
    devices = [l for l in lines if '/dev/' in l]

    all_camera_devices = {}
    camera_devices = []
    current_camera = None
    for line in lines:
        if line in cameras:
            if (current_camera is not None) and camera_devices:
                all_camera_devices[current_camera] = camera_devices
                camera_devices = []
            current_camera = line
        elif line in devices:
            camera_devices.append(line)
    if (current_camera is not None) and camera_devices:
        all_camera_devices[current_camera] = camera_devices

    return all_camera_devices

def setup_realsense_camera(serial_number, color_size, depth_size, fps):
    """
    Returns a Realsense camera pipeline used for accessing D435i & D405's video streams
    """
    pipeline = rs.pipeline()
    config = rs.config()

    if serial_number:
        config.enable_device(serial_number)

    config.enable_stream(rs.stream.color, color_size[0], color_size[1], rs.format.bgr8, fps)
    config.enable_stream(rs.stream.depth, depth_size[0], depth_size[1], rs.format.z16, fps)

    profile = pipeline.start(config)
    return pipeline

def setup_uvc_camera(device_index, size=None, fps=None, format = None):
    """
    Returns Opencv capture object of the UVC video divice
    """
    cap = cv2.VideoCapture(device_index)
    if format:
        fourcc_value = cv2.VideoWriter_fourcc(*f'{format}')
        cap.set(cv2.CAP_PROP_FOURCC, fourcc_value)
    if size:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, size[0])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, size[1])
    if fps:
        cap.set(cv2.CAP_PROP_FPS, fps)
    return cap

def get_video_device_port(camera_name):
    """
    Returns the video device port based on the given camera name match
    """
    camera_devices = get_video_devices()
    camera_device = None
    for k,v in camera_devices.items():
        if camera_name in k:
            camera_device = v[0]
            print(f"Found Camera={k} at port={camera_device} ")
            return camera_device
    print('ERROR: Did not find the specified camera_name = ' + str(camera_name))
    return  camera_device

BODY_FILE = '/tmp/stretch_pid_dir/stretch_body_robot_pid.txt'
BODY_FILELOCK = f'{BODY_FILE}.lock'

def acquire_body_filelock():
    whoami = pwd.getpwuid(os.getuid()).pw_name
    pid_file = pathlib.Path(BODY_FILE)
    filelock_path = pathlib.Path(BODY_FILELOCK)
    # 1. If the '/tmp/stretch_pid_dir' does not exist, make it. Note, it's important we create
    #    these files within a subdirectory instead of /tmp directly because /tmp is a "sticky"
    #    directory (i.e. only the user that created the file can edit it).
    if not pid_file.parent.is_dir():
        pid_file.parent.mkdir(parents=True, exist_ok=True)
    file_lock = FileLock(BODY_FILELOCK)
    try:
        file_lock.acquire(timeout=1)
        # 2. If we acquire the lock as the file's owner, the lock's permissions will have changed
        #    to limit write privileges. We use chmod to enable all users to write to the file.
        if filelock_path.owner() == whoami or whoami == "root":
            filelock_path.chmod(0o777)
        # 3. Write this process's PID to a file so this process can be freed by others.
        with open(str(pid_file), 'w') as f:
            f.write(str(os.getpid()))
        if pid_file.owner() == whoami or whoami == "root":
            pid_file.chmod(0o777)
    except Timeout:
        # 4. If we failed to acquire the lock as the file's owner, the lock's permissions will have
        #    changed to limit write privileges. We use chmod to enable all users to write to the file.
        if filelock_path.owner() == whoami or whoami == "root":
            filelock_path.chmod(0o777)
        return False, file_lock
    return True, file_lock

def free_body_filelock():
    whoami = pwd.getpwuid(os.getuid()).pw_name
    pid_file = pathlib.Path(BODY_FILE)
    filelock_path = pathlib.Path(BODY_FILELOCK)
    # 1. If the '/tmp/stretch_pid_dir' does not exist, no robot process has created it.
    if not pid_file.parent.is_dir():
        return True
    file_lock = FileLock(BODY_FILELOCK)
    try:
        file_lock.acquire(timeout=1)
        file_lock.release()
        # 2. If we acquire the lock as the file's owner, the lock's permissions will have changed
        #    to limit write privileges. We use chmod to enable all users to write to the file.
        if filelock_path.owner() == whoami or whoami == "root":
            filelock_path.chmod(0o777)
    except Timeout:
        # 3. If we failed to acquire the lock as the file's owner, the lock's permissions will have
        #    changed to limit write privileges. We use chmod to enable all users to write to the file.
        if filelock_path.owner() == whoami or whoami == "root":
            filelock_path.chmod(0o777)
        with open(pid_file, 'r') as f:
            tokill_pid = int(f.read())
            # 4. Send SIGTERM a few times because some processes (e.g. ipython) try to stall on exit.
            try:
                os.kill(tokill_pid, signal.SIGTERM)
                time.sleep(0.2)
                os.kill(tokill_pid, signal.SIGTERM)
                time.sleep(0.2)
                os.kill(tokill_pid, signal.SIGTERM)
            except PermissionError:
                # 5. os.kill will fail to kill PIDs not owned by this user. Root user can kill anything.
                return False
            except ProcessLookupError:
                pass
    return True
