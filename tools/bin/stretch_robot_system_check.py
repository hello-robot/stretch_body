#!/usr/bin/env python3
from __future__ import print_function
import sh
import re
import apt
import git
import distro
import pathlib
from packaging import version
import yaml
import time
import datetime
import stretch_body.robot as robot
import os, fnmatch
import subprocess
from colorama import Fore, Back, Style
import argparse
import stretch_body.hello_utils as hu
from stretch_body.dynamixel_XL430 import *
import stretch_body.device
hu.print_stretch_re_use()


parser=argparse.ArgumentParser(description='Check that all robot hardware is present and reporting sane values')
args=parser.parse_args()

# #####################################################
def val_in_range(val_name, val,vmin, vmax):
    p=val <=vmax and val>=vmin
    if p:
        print(Fore.GREEN +'[Pass] ' + val_name + ' = ' + str(val))
    else:
        print(Fore.RED +'[Fail] ' + val_name + ' = ' +str(val)+ ' out of range ' +str(vmin) + ' to ' + str(vmax))

def val_in_range_warn(val_name, val,vmin, vmax, hint=""):
    p=val <=vmax and val>=vmin
    if p:
        print(Fore.GREEN +'[Pass] ' + val_name + ' = ' + str(val))
    else:
        print(Fore.YELLOW +'[Warn] ' + val_name + ' = ' +str(val)+ ' out of range ' +str(vmin) + ' to ' + str(vmax) + hint)

def val_is_not(val_name, val,vnot):
    if val is not vnot:
        print(Fore.GREEN +'[Pass] ' + val_name + ' = ' + str(val))
    else:
        print(Fore.RED +'[Fail] ' + val_name + ' = ' +str(val))

#Turn off logging so get a clean output
import logging
logging.getLogger('sh').setLevel(logging.CRITICAL)
#logging.disable(logging.CRITICAL)
r=robot.Robot()
r.startup()

# #####################################################
print(Style.RESET_ALL)
print('---- Checking Devices ----')
robot_devices={'hello-wacc':0, 'hello-motor-left-wheel':0,'hello-pimu':0, 'hello-lrf':0,'hello-dynamixel-head':0,'hello-dynamixel-wrist':0,'hello-motor-arm':0,'hello-motor-right-wheel':0,
               'hello-motor-lift':0,'hello-respeaker':0}

listOfFiles = os.listdir('/dev')
pattern = "hello*"
for entry in listOfFiles:
    if fnmatch.fnmatch(entry, pattern):
            robot_devices[entry]=1
for k in robot_devices.keys():
    if robot_devices[k]:
        print(Fore.GREEN +'[Pass] : '+k)
    else:
        print(Fore.RED +'[Fail] : '+ k)
# #####################################################

print(Style.RESET_ALL)
if robot_devices['hello-pimu']:
    print('---- Checking Pimu ----')
    p=r.pimu
    val_in_range('Voltage',p.status['voltage'], vmin=p.config['low_voltage_alert'], vmax=14.5)
    val_in_range('Current',p.status['current'], vmin=0.5, vmax=p.config['high_current_alert'])
    val_in_range('Temperature',p.status['temp'], vmin=10, vmax=40)
    val_in_range_warn('Cliff-0',p.status['cliff_range'][0], vmin=p.config['cliff_thresh'], vmax=60, hint=' - calibrate using REx_cliff_sensor_calibrate.py')
    val_in_range_warn('Cliff-1',p.status['cliff_range'][1], vmin=p.config['cliff_thresh'], vmax=60, hint=' - calibrate using REx_cliff_sensor_calibrate.py')
    val_in_range_warn('Cliff-2',p.status['cliff_range'][2], vmin=p.config['cliff_thresh'], vmax=60, hint=' - calibrate using REx_cliff_sensor_calibrate.py')
    val_in_range_warn('Cliff-3',p.status['cliff_range'][3], vmin=p.config['cliff_thresh'], vmax=60, hint=' - calibrate using REx_cliff_sensor_calibrate.py')
    val_in_range('IMU AZ',p.status['imu']['az'], vmin=-10.1, vmax=-9.5)
    val_in_range('IMU Pitch', hu.rad_to_deg(p.status['imu']['pitch']), vmin=-12, vmax=12)
    val_in_range('IMU Roll', hu.rad_to_deg(p.status['imu']['roll']), vmin=-12, vmax=12)
    print(Style.RESET_ALL)

# #####################################################
print(Style.RESET_ALL)

if robot_devices['hello-dynamixel-wrist']:
    print('---- Checking EndOfArm ----')
    w = r.end_of_arm
    try:
        for mk in w.motors.keys():
            if w.motors[mk].do_ping():
                print(Fore.GREEN +'[Pass] Ping of: '+mk)
                if w.motors[mk].params['req_calibration']:
                    if w.motors[mk].motor.is_calibrated():
                        print(Fore.GREEN + '[Pass] Homed: ' + mk)
                    else:
                        print(Fore.RED + '[Fail] Not Homed: ' + mk)
            else:
                print(Fore.RED + '[Fail] Ping of: ' + mk)
            print(Style.RESET_ALL)
    except(IOError, DynamixelCommError):
        print(Fore.RED + '[Fail] Startup of EndOfArm')
# #####################################################
print(Style.RESET_ALL)
if robot_devices['hello-dynamixel-head']:
    print('---- Checking Head ----')
    h = r.head
    try:
        for mk in h.motors.keys():
            if h.motors[mk].do_ping():
                print(Fore.GREEN +'[Pass] Ping of: '+mk)
            else:
                print(Fore.RED + '[Fail] Ping of: ' + mk)
            print(Style.RESET_ALL)
    except(IOError, DynamixelCommError):
        print(Fore.RED + '[Fail] Startup of EndOfArm')
# #####################################################
print(Style.RESET_ALL)
if robot_devices['hello-wacc']:
    print('---- Checking Wacc ----')
    w=r.wacc
    val_in_range('AX',w.status['ax'], vmin=8.0, vmax=11.0)
    print(Style.RESET_ALL)

# #####################################################
print(Style.RESET_ALL)
if robot_devices['hello-motor-left-wheel']:
    print('---- Checking hello-motor-left-wheel ----')
    m = r.base.left_wheel
    val_is_not('Position',m.status['pos'], vnot=0)
    print(Style.RESET_ALL)

# #####################################################
print(Style.RESET_ALL)
if robot_devices['hello-motor-right-wheel']:
    print('---- Checking hello-motor-right-wheel ----')
    m = r.base.right_wheel
    val_is_not('Position',m.status['pos'], vnot=0)
    print(Style.RESET_ALL)

# #####################################################
print(Style.RESET_ALL)
if robot_devices['hello-motor-arm']:
    print('---- Checking hello-motor-arm ----')
    m = r.arm.motor
    val_is_not('Position',m.status['pos'], vnot=0)
    val_is_not('Position Homed', m.status['pos_calibrated'], vnot=False)
    print(Style.RESET_ALL)

# #####################################################
print(Style.RESET_ALL)
if robot_devices['hello-motor-lift']:
    print('---- Checking hello-motor-lift ----')
    m = r.lift.motor
    val_is_not('Position',m.status['pos'], vnot=0)
    val_is_not('Position Homed', m.status['pos_calibrated'], vnot=False)
    print(Style.RESET_ALL)

# #####################################################
print(Style.RESET_ALL)
print ('---- Checking for Intel D435i ----')
cmd = "lsusb -d 8086:0b3a"
returned_value = subprocess.call(cmd,shell=True)  # returns the exit code in unix
if returned_value==0:
    print(Fore.GREEN + '[Pass] : Device found ')
else:
    print(Fore.RED + '[Fail] : No device found')
# #####################################################
try: # TODO: remove try/catch after sw check verified to work reliably
    def get_ip():
        # https://stackoverflow.com/a/28950776
        import socket
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.settimeout(0)
        try:
            # doesn't even have to be reachable
            s.connect(('10.254.254.254', 1))
            IP = s.getsockname()[0]
        except Exception:
            IP = '127.0.0.1'
        finally:
            s.close()
        return IP

    scan_dict = None
    def find_latest_updates_scan():
        global scan_dict
        logpath = pathlib.Path('~/stretch_user/log/updates_logger/').expanduser()
        if logpath.is_dir():
            for scan in sorted(logpath.glob('updates_scan.*.yaml'), reverse=True):
                # filters out scans that are older than 30 days (stale) or newer than 15 days old (gives a buffer for releases to stablize)
                scan_datetime_str = str(scan).split('.')[1]
                scan_datetime = datetime.datetime.strptime(scan_datetime_str, '%Y%m%d%H%M%S') # raises ValueError if parsing fails
                now_datetime = datetime.datetime.now()
                days15ago_datetime = now_datetime + datetime.timedelta(days=-15)
                days30ago_datetime = now_datetime + datetime.timedelta(days=-30)
                if scan_datetime > days30ago_datetime and scan_datetime < days15ago_datetime:
                    with open(str(scan), 'r') as s:
                        scan_dict = yaml.load(s, Loader=yaml.FullLoader)
                        break
    find_latest_updates_scan()

    def is_distribution_okay():
        ubuntu_str = f'{distro.name()} {distro.version()}'
        ubuntu_version = distro.version()
        if ubuntu_version == '18.04':
            return Fore.RED + f'[Fail] {ubuntu_str} is deprecated'
        elif ubuntu_version == '20.04':
            return Fore.GREEN + f'[Pass] {ubuntu_str} is ready'
        elif ubuntu_version == '22.04':
            return Fore.GREEN + f'[Pass] {ubuntu_str} is ready'
        else:
            return Fore.RED + f'[Fail] {ubuntu_str} is unknown'
    def all_apt_correct():
        apt_expectations = {
            '18.04': {
                'ros-melodic-librealsense2': False,
                'librealsense2': True,
            },
            '20.04': {
                'ros-noetic-librealsense2': False,
                'ros-galactic-librealsense2': False,
                'librealsense2': True,
            },
            '22.04': {
                'ros-iron-librealsense2': False,
                'librealsense2': True,
            }
        }
        apt_list = apt.Cache()
        for pkg, is_install_expected in apt_expectations[distro.version()].items():
            if pkg not in apt_list and is_install_expected:
                return False, f"{pkg} should be installed"
            if pkg in apt_list and is_install_expected != apt_list[pkg].is_installed:
                return False, f"{pkg} should {'not' if not is_install_expected else ''} be installed"
        return True, ''
    def all_firmware_uptodate():
        # get current fw versions
        def get_fw_version(hello_device):
            try:
                return f"v{hello_device.board_info['firmware_version'].split('.v', 1)[1]}"
            except:
                return 'v0.0.0p-1'
        current_fw_versions = {
            'pimu': get_fw_version(r.pimu),
            'wacc': get_fw_version(r.wacc),
            'arm': get_fw_version(r.arm.motor),
            'lift': get_fw_version(r.lift.motor),
            'left-wheel': get_fw_version(r.base.left_wheel),
            'right-wheel': get_fw_version(r.base.right_wheel),
        }

        # get latest fw versions
        latest_fw_versions = {
            'pimu': 'Pimu.v0.0.1p0',
            'wacc': 'Wacc.v0.0.1p0',
            'arm': 'Stepper.v0.0.1p0',
            'lift': 'Stepper.v0.0.1p0',
            'left-wheel': 'Stepper.v0.0.1p0',
            'right-wheel': 'Stepper.v0.0.1p0',
        }
        if scan_dict:
            latest_fw_versions['pimu'] = scan_dict['firmware']['hello-pimu']
            latest_fw_versions['wacc'] = scan_dict['firmware']['hello-wacc']
            latest_fw_versions['arm'] = scan_dict['firmware']['hello-motor-arm']
            latest_fw_versions['lift'] = scan_dict['firmware']['hello-motor-lift']
            latest_fw_versions['right-wheel'] = scan_dict['firmware']['hello-motor-right-wheel']
            latest_fw_versions['left-wheel'] = scan_dict['firmware']['hello-motor-left-wheel']
        latest_fw_versions = {hello_device: f"v{latest_fw_versions[hello_device].split('.v', 1)[1]}" for hello_device in latest_fw_versions}

        # check current against latest
        for hello_device in current_fw_versions:
            currentf = version.parse(current_fw_versions[hello_device].split('p', 1)[0])
            latestf = version.parse(latest_fw_versions[hello_device].split('p', 1)[0])
            if currentf < latestf:
                return False, current_fw_versions
        return True, current_fw_versions
    def all_pip_uptodate():
        # get current pip versions and their optional editable locations
        pip_versions = {
            'hello-robot-stretch-body': None,
            'hello-robot-stretch-body-tools': None,
            'hello-robot-stretch-tool-share': None,
            'hello-robot-stretch-factory': None,
            'hello-robot-stretch-diagnostics': None,
            'hello-robot-stretch-urdf': None,
        }
        pip_editable_locations = {
            'hello-robot-stretch-body': None,
            'hello-robot-stretch-body-tools': None,
            'hello-robot-stretch-tool-share': None,
            'hello-robot-stretch-factory': None,
            'hello-robot-stretch-diagnostics': None,
            'hello-robot-stretch-urdf': None,
        }
        for line in sh.pip.list(_iter=True):
            pip_pkg_data = re.split(r'\s+(?=[\d/])', line.strip())
            if len(pip_pkg_data) >= 2 and pip_pkg_data[0] in pip_versions:
                pip_versions[pip_pkg_data[0]] = pip_pkg_data[1]
                if len(pip_pkg_data) >= 3:
                    p = pip_pkg_data[2]
                    pip_editable_locations[pip_pkg_data[0]] = p.replace(str(pathlib.Path(p).home()), '~')

        # get latest pip versions
        latest_pip_version = {
            'hello-robot-stretch-body': version.parse('0.4.32'),
            'hello-robot-stretch-body-tools': version.parse('0.4.16'),
            'hello-robot-stretch-tool-share': version.parse('0.2.7'),
            'hello-robot-stretch-factory': version.parse('0.4.6'),
            'hello-robot-stretch-diagnostics': version.parse('0.0.13'),
            'hello-robot-stretch-urdf': version.parse('0.0.11'),
        }
        if scan_dict:
            latest_pip_version = {p: version.parse(scan_dict['pip'].get(p, '0.0.0')) for p in latest_pip_version}

        # check current against latest
        try: # The try/except catches pip pkgs that aren't installed
            for pip_pkg in pip_versions:
                p = version.parse(pip_versions[pip_pkg])
                if p < latest_pip_version[pip_pkg]:
                    return False, pip_versions, pip_editable_locations
        except:
            return False, pip_versions, pip_editable_locations
        return True, pip_versions, pip_editable_locations
    def all_ros_correct():
        ros1_distros = ['noetic', 'melodic', 'lunar', 'kinetic', 'jade', 'indigo']
        ros2_distros = ['rolling', 'jazzy', 'iron', 'humble', 'galactic', 'foxy', 'eloquent', 'dashing']
        ros_expectations = { # the actual set of supported ROS distros
            'melodic': {
                'stretch_core': True,
                'realsense2_camera': True,
                'respeaker_ros': False,
                'realsense_gazebo_plugin': True,
            },
            'noetic': {
                'stretch_core': True,
                'realsense2_camera': True,
                'respeaker_ros': True,
                'realsense_gazebo_plugin': True,
            },
            # 'galactic': { # Deprecated August 2023
            #     'stretch_core': True,
            #     'realsense2_camera': True,
            #     'sllidar_ros2': True,
            #     'ros2_numpy': True,
            #     'stretch_moveit_plugins': True,
            # },
            'humble': {
                'stretch_core': True,
                'realsense2_camera': True,
                'sllidar_ros2': True,
                'ros2_numpy': True,
            }
        }
        # create ros_distro
        ros_distro = os.getenv('ROS_DISTRO')
        if not ros_distro:
            return False, '', False, '', ''

        # create ros_name
        if ros_distro in ros1_distros:
            ros_name = f'ROS {ros_distro.capitalize()}'
        elif ros_distro in ros2_distros:
            ros_name = f'ROS2 {ros_distro.capitalize()}'
        else:
            ros_name = f'Unknown ROS {ros_distro.capitalize()}'

        # check ros expectations
        if ros_distro not in ros_expectations:
            return False, ros_name, False, '', ''
        if ros_distro in ros1_distros:
            import rospkg
            rospack = rospkg.RosPack()
            rospack_list = rospack.list()
            for pkg, is_install_expected in ros_expectations[ros_distro].items():
                if is_install_expected != (pkg in rospack_list):
                    return True, ros_name, False, f"{pkg} should {'not' if not is_install_expected else ''} be installed", ''
            p = rospack.get_path('stretch_core')
            ws_paths = [str(par.parent) for par in pathlib.Path(p).parents if str(par).endswith('src')]
            ws_path = ''
            if len(ws_paths) > 0:
                ws_path = ws_paths[0].replace(str(pathlib.Path(p).home()), '~')
                ws_path = str(pathlib.Path(ws_path) / 'src' / 'stretch_ros')
            if pathlib.Path(ws_path).expanduser().is_dir():
                if scan_dict:
                    latest_stretchros_git_commit = scan_dict['ros']['stretch_ros']
                    if latest_stretchros_git_commit is not None:
                        repo = git.Repo(str(pathlib.Path(ws_path).expanduser()))
                        last200_localstretchros_git_commits = [str(repo.commit(f'HEAD~{i}')) for i in range(100)]
                        if latest_stretchros_git_commit not in last200_localstretchros_git_commits:
                            return True, ros_name, False, 'Stretch ROS not up-to-date', ws_path
            else:
                return True, ros_name, False, 'Unable to find workspace', ''
            ros_ip = os.getenv('ROS_IP')
            if ros_ip and ros_ip != get_ip():
                return True, ros_name, False, f'Remote master ROS_IP should be {get_ip()}', ws_path
            return True, ros_name, True, '', ws_path
        elif ros_distro in ros2_distros:
            from ament_index_python.packages import get_package_share_directory, get_package_prefix
            for pkg, is_install_expected in ros_expectations[ros_distro].items():
                try: # use the try/except to catch whether the pkg is installed
                    get_package_share_directory(pkg)
                    if not is_install_expected:
                        return True, ros_name, False, f"{pkg} should not be installed", ''
                except:
                    if is_install_expected:
                        return True, ros_name, False, f"{pkg} should be installed", ''
            p = get_package_prefix('stretch_core')
            ws_path = str(pathlib.Path(p).parent.parent / 'src' / 'stretch_ros2').replace(str(pathlib.Path(p).home()), '~')
            if pathlib.Path(ws_path).expanduser().is_dir():
                if scan_dict:
                    latest_stretchros_git_commit = scan_dict['ros']['stretch_ros2']
                    if latest_stretchros_git_commit is not None:
                        repo = git.Repo(str(pathlib.Path(ws_path).expanduser()))
                        last200_localstretchros_git_commits = [str(repo.commit(f'HEAD~{i}')) for i in range(100)]
                        if latest_stretchros_git_commit not in last200_localstretchros_git_commits:
                            return True, ros_name, False, 'Stretch ROS2 not up-to-date', ws_path
            else:
                return True, ros_name, False, 'Unable to find workspace', ''
            return True, ros_name, True, '', ws_path
        return True, ros_name, False, 'Unable to list pkgs for this distribution', ''
    print(Style.RESET_ALL)
    print ('---- Checking Software ----')
    # Ubuntu
    print(is_distribution_okay())
    # APT
    apt_correct, apt_err_msg = all_apt_correct()
    if apt_correct:
        print(Fore.GREEN + f'[Pass] All APT pkgs are setup correctly')
    else:
        print(Fore.YELLOW + f'[Warn] Not all APT pkgs are setup correctly ({apt_err_msg})')
    # Firmware
    fw_uptodate, fw_versions = all_firmware_uptodate()
    if fw_uptodate:
        print(Fore.GREEN + '[Pass] Firmware is up-to-date')
    else:
        print(Fore.YELLOW + '[Warn] Firmware not up-to-date (try REx_firmware_updater.py --recommended)')
    print(Fore.LIGHTBLUE_EX + '         hello-pimu = ' + Fore.CYAN + fw_versions['pimu'])
    print(Fore.LIGHTBLUE_EX + '         hello-wacc = ' + Fore.CYAN + fw_versions['wacc'])
    print(Fore.LIGHTBLUE_EX + '         hello-motor-arm = ' + Fore.CYAN + fw_versions['arm'])
    print(Fore.LIGHTBLUE_EX + '         hello-motor-lift = ' + Fore.CYAN + fw_versions['lift'])
    print(Fore.LIGHTBLUE_EX + '         hello-motor-left-wheel = ' + Fore.CYAN + fw_versions['left-wheel'])
    print(Fore.LIGHTBLUE_EX + '         hello-motor-right-wheel = ' + Fore.CYAN + fw_versions['right-wheel'])
    # Python
    pip_uptodate, pip_versions, pip_editable_locations = all_pip_uptodate()
    if pip_uptodate:
        print(Fore.GREEN + '[Pass] Python pkgs are up-to-date')
    else:
        print(Fore.YELLOW + '[Warn] Python pkgs not up-to-date')
    for bname in ['hello-robot-stretch-body', 'hello-robot-stretch-body-tools', 'hello-robot-stretch-tool-share', 'hello-robot-stretch-factory', 'hello-robot-stretch-diagnostics', 'hello-robot-stretch-urdf']:
        print(Fore.LIGHTBLUE_EX + f'         {bname} = ' + Fore.CYAN + f"{pip_versions[bname] if pip_versions[bname] else 'Not Installed'}" + Fore.LIGHTBLUE_EX + f"{f' (installed locally at {pip_editable_locations[bname]})' if pip_editable_locations[bname] else ''}")
    # ROS
    ros_enabled, ros_name, ros_ready, ros_err_msg, ros_ws_path = all_ros_correct()
    if ros_enabled:
        if ros_ready:
            print(Fore.GREEN + f'[Pass] {ros_name} is ready')
            if ros_ws_path:
                print(Fore.LIGHTBLUE_EX + f'         Workspace at {ros_ws_path}')
        else:
            print(Fore.YELLOW + f'[Warn] {ros_name} not ready ({ros_err_msg})')
            if ros_ws_path:
                print(Fore.LIGHTBLUE_EX + f'         Workspace at {ros_ws_path}')
    else:
        if ros_name:
            print(Fore.YELLOW + f'[Warn] {ros_name} not supported')
        else:
            print(Fore.YELLOW + '[Warn] No version of ROS enabled')
except:
    show_sw_exc = stretch_body.device.Device(name='system_check', req_params=False).params.get('show_sw_exc', False)
    if show_sw_exc:
        raise

r.stop()
