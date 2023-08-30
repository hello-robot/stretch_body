#!/usr/bin/env python3
from __future__ import print_function
import sh
import re
import apt
import distro
import pathlib
import rospkg
from packaging import version
import time
import stretch_body.robot as robot
import os, fnmatch
import subprocess
from colorama import Fore, Back, Style
import argparse
import stretch_body.hello_utils as hu
from stretch_body.dynamixel_XL430 import *
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
                        print(Fore.GREEN + '[Pass] Calibrated: ' + mk)
                    else:
                        print(Fore.RED + '[Fail] Not Calibrated: ' + mk)
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
    val_is_not('Position Calibrated', m.status['pos_calibrated'], vnot=False)
    print(Style.RESET_ALL)

# #####################################################
print(Style.RESET_ALL)
if robot_devices['hello-motor-lift']:
    print('---- Checking hello-motor-lift ----')
    m = r.lift.motor
    val_is_not('Position',m.status['pos'], vnot=0)
    val_is_not('Position Calibrated', m.status['pos_calibrated'], vnot=False)
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
            if pkg not in apt_list or is_install_expected != apt_list[pkg].is_installed:
                return False, f'{pkg} not set-up correctly'
        return True, ''
    def all_firmware_uptodate():
        def get_fw_version(hello_device):
            try:
                return f"v{hello_device.board_info['firmware_version'].split('.v', 1)[1]}"
            except:
                return 'v0.0.0p-1'
        fw_versions = {
            'pimu': get_fw_version(r.pimu),
            'wacc': get_fw_version(r.wacc),
            'arm': get_fw_version(r.arm.motor),
            'lift': get_fw_version(r.lift.motor),
            'left-wheel': get_fw_version(r.base.left_wheel),
            'right-wheel': get_fw_version(r.base.right_wheel),
        }
        # as of Jun 25, 2023
        latest_fw_versions = {
            'pimu': version.parse('v0.1.0'),
            'wacc': version.parse('v0.1.0'),
            'arm': version.parse('v0.1.0'),
            'lift': version.parse('v0.1.0'),
            'left-wheel': version.parse('v0.1.0'),
            'right-wheel': version.parse('v0.1.0'),
        }
        for hello_device in fw_versions:
            f = version.parse(fw_versions[hello_device].split('p', 1)[0])
            if f < latest_fw_versions[hello_device]:
                return False, fw_versions
        return True, fw_versions
    def all_pip_uptodate():
        pip_versions = {
            'hello-robot-stretch-body': None,
            'hello-robot-stretch-body-tools': None,
            'hello-robot-stretch-tool-share': None,
            'hello-robot-stretch-factory': None,
            'hello-robot-stretch-diagnostics': None,
        }
        pip_editable_locations = {
            'hello-robot-stretch-body': None,
            'hello-robot-stretch-body-tools': None,
            'hello-robot-stretch-tool-share': None,
            'hello-robot-stretch-factory': None,
            'hello-robot-stretch-diagnostics': None,
        }
        latest_pip_version = {
            'hello-robot-stretch-body': version.parse('0.4.32'),
            'hello-robot-stretch-body-tools': version.parse('0.4.16'),
            'hello-robot-stretch-tool-share': version.parse('0.2.7'),
            'hello-robot-stretch-factory': version.parse('0.4.6'),
            'hello-robot-stretch-diagnostics': version.parse('0.0.13'),
        }
        for line in sh.pip.list(_iter=True):
            pip_pkg_data = re.split(r'\s+(?=[\d/])', line.strip())
            if len(pip_pkg_data) >= 2 and pip_pkg_data[0] in pip_versions:
                pip_versions[pip_pkg_data[0]] = pip_pkg_data[1]
                if len(pip_pkg_data) >= 3:
                    p = pip_pkg_data[2]
                    pip_editable_locations[pip_pkg_data[0]] = p.replace(str(pathlib.Path(p).home()), '~')
        try:
            for pip_pkg in pip_versions:
                p = version.parse(pip_versions[pip_pkg])
                if p < latest_pip_version[pip_pkg]:
                    return False, pip_versions, pip_editable_locations
        except:
            return False, pip_versions, pip_editable_locations
        return True, pip_versions, pip_editable_locations
    def all_ros_correct():
        ros_expectations = {
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
            'galactic': {
                'stretch_core': True,
                'realsense2_camera': True,
                'sllidar_ros2': True,
                'ros2_numpy': True,
                'stretch_moveit_plugins': True,
            },
            'iron': {
                'stretch_core': True,
                'realsense2_camera': True,
                'sllidar_ros2': True,
                'ros2_numpy': True,
                'stretch_moveit_plugins': True,
            }
        }
        if not os.getenv('ROS_DISTRO'):
            return False, '', False, '', ''
        ros_name = f'ROS {os.getenv("ROS_DISTRO").capitalize()}'
        if os.getenv('ROS_DISTRO') not in ros_expectations:
            return False, ros_name, False, '', ''
        rospack = rospkg.RosPack()
        rospack_list = rospack.list()
        for pkg, is_install_expected in ros_expectations[os.getenv('ROS_DISTRO')].items():
            if is_install_expected != (pkg in rospack_list):
                return True, ros_name, False, f'{pkg} missing', ''
        p = rospack.get_path('stretch_core')
        ws_paths = [str(par.parent) for par in pathlib.Path(p).parents if str(par).endswith('src')]
        ws_path = ''
        if len(ws_paths) > 0:
            ws_path = ws_paths[0].replace(str(pathlib.Path(p).home()), '~')
        return True, ros_name, True, '', ws_path
    print(Style.RESET_ALL)
    print ('---- Checking Software ----')
    # Ubuntu APT
    apt_ready, apt_err_msg = all_apt_correct()
    ubuntu_str = f'{distro.name()} {distro.version()}'
    if apt_ready:
        print(Fore.GREEN + f'[Pass] {ubuntu_str} is ready')
    else:
        print(Fore.YELLOW + f'[Warn] {ubuntu_str} not ready ({apt_err_msg})')
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
    bname = 'hello-robot-stretch-body'
    print(Fore.LIGHTBLUE_EX + '         Stretch Body = ' + Fore.CYAN + f"{pip_versions[bname] if pip_versions[bname] else 'Not Installed'}" + Fore.LIGHTBLUE_EX + f"{f' (installed locally at {pip_editable_locations[bname]})' if pip_editable_locations[bname] else ''}")
    bname = 'hello-robot-stretch-body-tools'
    print(Fore.LIGHTBLUE_EX + '         Stretch Body Tools = ' + Fore.CYAN + f"{pip_versions[bname] if pip_versions[bname] else 'Not Installed'}" + Fore.LIGHTBLUE_EX + f"{f' (installed locally at {pip_editable_locations[bname]})' if pip_editable_locations[bname] else ''}")
    bname = 'hello-robot-stretch-tool-share'
    print(Fore.LIGHTBLUE_EX + '         Stretch Tool Share = ' + Fore.CYAN + f"{pip_versions[bname] if pip_versions[bname] else 'Not Installed'}" + Fore.LIGHTBLUE_EX + f"{f' (installed locally at {pip_editable_locations[bname]})' if pip_editable_locations[bname] else ''}")
    bname = 'hello-robot-stretch-factory'
    print(Fore.LIGHTBLUE_EX + '         Stretch Factory = ' + Fore.CYAN + f"{pip_versions[bname] if pip_versions[bname] else 'Not Installed'}" + Fore.LIGHTBLUE_EX + f"{f' (installed locally at {pip_editable_locations[bname]})' if pip_editable_locations[bname] else ''}")
    bname = 'hello-robot-stretch-diagnostics'
    print(Fore.LIGHTBLUE_EX + '         Stretch Diagnostics = ' + Fore.CYAN + f"{pip_versions[bname] if pip_versions[bname] else 'Not Installed'}" + Fore.LIGHTBLUE_EX + f"{f' (installed locally at {pip_editable_locations[bname]})' if pip_editable_locations[bname] else ''}")
    # ROS
    ros_enabled, ros_name, ros_ready, ros_err_msg, ros_ws_path = all_ros_correct()
    if ros_enabled:
        if ros_ready:
            print(Fore.GREEN + f'[Pass] {ros_name} is ready')
            print(Fore.LIGHTBLUE_EX + f'         Workspace at {ros_ws_path}')
        else:
            print(Fore.YELLOW + f'[Warn] {ros_name} not ready ({ros_err_msg})')
    else:
        if ros_name:
            print(Fore.YELLOW + f'[Warn] {ros_name} not supported')
        else:
            print(Fore.YELLOW + '[Warn] No version of ROS enabled')
except:
    pass

r.stop()
