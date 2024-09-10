#!/usr/bin/env python3

import stretch_body.hello_utils as hu
hu.print_stretch_re_use()

import os
import sh
import re
import sys
import apt
import git
import yaml
import distro
import fnmatch
import pathlib
import datetime
import argparse
from packaging import version
from colorama import Fore, Back, Style

import rplidar
import pyrealsense2 as rs
import stretch_body.device
import stretch_body.robot as robot
import stretch_factory.hello_device_utils as hdu

parser=argparse.ArgumentParser(description='Check that all robot hardware is present and reporting sane values')
parser.add_argument('-v', "--verbose", help="Prints more information", action="store_true")
args=parser.parse_args()

# ###################   SETUP    ######################
def val_in_range(val_name, val,vmin, vmax):
    p=val <=vmax and val>=vmin
    if p:
        return True, f"{val_name} = {val:.2f}"
    else:
        return False, f"{val_name} = {val:.2f} out of range {vmin} to {vmax}"

# Turn off logging so get a clean output
import logging
logging.getLogger('sh').setLevel(logging.CRITICAL)
logging.getLogger('rplidar').setLevel(logging.CRITICAL)

# print robot info
device = stretch_body.device.Device(name='robot', req_params=False)
stretch_serial_no = device.params.get('serial_no', '')
stretch_model = device.params.get('model_name', '')
stretch_batch = device.params.get('batch_name', '')
stretch_tool = device.params.get('tool', '')
if stretch_model == "SE3":
    print(Fore.LIGHTBLUE_EX + 'Model = ' + Fore.CYAN + 'Stretch 3')
elif stretch_model == "RE2V0":
    print(Fore.LIGHTBLUE_EX + 'Model = ' + Fore.CYAN + 'Stretch 2')
elif stretch_model == "RE1V0":
    print(Fore.LIGHTBLUE_EX + 'Model = ' + Fore.CYAN + 'Stretch RE1')
if stretch_tool == "tool_none":
    print(Fore.LIGHTBLUE_EX + 'Tool = ' + Fore.CYAN + 'None')
elif stretch_tool == "tool_stretch_gripper":
    print(Fore.LIGHTBLUE_EX + 'Tool = ' + Fore.CYAN + 'Standard Gripper')
elif stretch_tool == "tool_stretch_dex_wrist":
    print(Fore.LIGHTBLUE_EX + 'Tool = ' + Fore.CYAN + 'DexWrist 2 w/ Gripper')
elif stretch_tool == "eoa_wrist_dw3_tool_nil":
    print(Fore.LIGHTBLUE_EX + 'Tool = ' + Fore.CYAN + 'DexWrist 3 w/ no attached tool')
elif stretch_tool == "eoa_wrist_dw3_tool_sg3":
    print(Fore.LIGHTBLUE_EX + 'Tool = ' + Fore.CYAN + 'DexWrist 3 w/ Gripper')
elif stretch_tool == "eoa_wrist_dw3_tool_tablet_12in":
    print(Fore.LIGHTBLUE_EX + 'Tool = ' + Fore.CYAN + 'DexWrist 3 w/ 12" Tablet')
else:
    print(Fore.LIGHTBLUE_EX + 'Tool = ' + Fore.CYAN + stretch_tool)
if args.verbose:
    print(Fore.LIGHTBLUE_EX + 'Batch = ' + Fore.CYAN + stretch_batch)
print(Fore.LIGHTBLUE_EX + 'Serial Number = ' + Fore.CYAN + stretch_serial_no)
# create robot instance
print(Style.RESET_ALL)
r=robot.Robot()
if not r.startup():
    sys.exit(1)

r.monitor.logger.setLevel('WARN')

# ###################  HARDWARE  ######################
def is_comms_ready():
    # Establish what USB devices we expect to see
    usb_device_seen = {
        'hello-wacc': False,
        'hello-motor-left-wheel': False,
        'hello-pimu': False,
        'hello-lrf': False,
        'hello-dynamixel-head': False,
        'hello-dynamixel-wrist': False,
        'hello-motor-arm': False,
        'hello-motor-right-wheel': False,
        'hello-motor-lift': False,
        'hello-respeaker': False,
    }
    if stretch_model == "SE3":
        usb_device_seen['hello-nav-head-camera'] = False

    # Mark which USB devices we actually see
    listOfFiles = os.listdir('/dev')
    pattern = "hello*"
    for entry in listOfFiles:
        if fnmatch.fnmatch(entry, pattern):
                usb_device_seen[entry] = True

    # Return error if not all USB devices seen
    for d in usb_device_seen:
        if not usb_device_seen[d]:
            return False, f"missing /dev/{d}", usb_device_seen, []

    # Ping all dxl motors
    ping_list = []
    for chain in [r.end_of_arm, r.head]:
        for mk in chain.motors.keys():
            if not chain.motors[mk].do_ping():
                return False, f"failed to ping {mk}", usb_device_seen, ping_list
            ping_list.append(mk)

    # Check hello steppers getting real data
    for stepper in [r.lift.motor, r.arm.motor, r.base.left_wheel, r.base.right_wheel]:
        if stepper.status['pos'] == 0:
            return False, f"failed to pull data for {stepper.name}", usb_device_seen, ping_list
        ping_list.append(stepper.name)

    return True, "", usb_device_seen, ping_list
def are_actuators_ready():
    # Check hello steppers' self recognized type matches SDK's expectation
    if 'stepper_type' in r.lift.motor.board_info and r.lift.motor.board_info['stepper_type'] != 0 and r.lift.motor.board_info['stepper_type'] != None:
        if r.lift.motor.board_info['stepper_type'] != 'hello-motor-lift':
            return False, "stepper type mismatch on lift motor"
    if 'stepper_type' in r.arm.motor.board_info and r.arm.motor.board_info['stepper_type'] != 0 and r.arm.motor.board_info['stepper_type'] != None:
        if r.arm.motor.board_info['stepper_type'] != 'hello-motor-arm':
            return False, "stepper type mismatch on arm motor"
    if 'stepper_type' in r.base.left_wheel.board_info and r.base.left_wheel.board_info['stepper_type'] != 0 and r.base.left_wheel.board_info['stepper_type'] != None:
        if r.base.left_wheel.board_info['stepper_type'] != 'hello-motor-left-wheel':
            return False, "stepper type mismatch on left wheel motor"
    if 'stepper_type' in r.base.right_wheel.board_info and r.base.right_wheel.board_info['stepper_type'] != 0 and r.base.right_wheel.board_info['stepper_type'] != None:
        if r.base.right_wheel.board_info['stepper_type'] != 'hello-motor-right-wheel':
            return False, "stepper type mismatch on right wheel motor"

    # Check dxl motors homed
    for chain in [r.end_of_arm, r.head]:
        for mk in chain.motors.keys():
            if chain.motors[mk].params['req_calibration'] and not chain.motors[mk].motor.is_calibrated():
                return False, "robot not homed, run stretch_robot_home.py"

    # Check hello steppers homed
    for stepper in [r.lift.motor, r.arm.motor]:
        if not stepper.status['pos_calibrated']:
            return False, "robot not homed, run stretch_robot_home.py"

    # Check robot agrees everything homed
    if not r.is_homed():
        return False, "robot not homed, run stretch_robot_home.py"

    return True, ""
def are_sensors_ready():
    # Establish which cameras we expect to see
    cameras_seen = {
        'D435': False,
    }
    if stretch_model == "SE3":
        cameras_seen['D405'] = False
        cameras_seen['OV9782'] = False

    # Mark which Realsense cameras we actually see
    rs_cameras = [{'name': device.get_info(rs.camera_info.name), 'serial_number': device.get_info(rs.camera_info.serial_number)}
        for device in rs.context().devices]
    for rs_cam in rs_cameras:
        for expected_cam in cameras_seen:
            if expected_cam in rs_cam['name']:
                cameras_seen[expected_cam] = True

    # Mark which UVC cameras we actually see
    try:
        nhc_model = hdu.extract_udevadm_info('/dev/hello-nav-head-camera', 'ID_MODEL')
        # nav head cam (nhc) model should be 'Arducam_OV9782_USB_Camera'
        if 'OV9782' in nhc_model:
            cameras_seen['OV9782'] = True
    except:
        pass

    # Return error if not all cameras seen
    for s in cameras_seen:
        if not cameras_seen[s]:
            return False, False, f"missing {s} camera"

    # Check for lidar
    try:
        lidar_dev = stretch_body.device.Device('lidar')
        lidar_usb = lidar_dev.params['usb_name']
        lidar = rplidar.RPLidar(lidar_usb, baudrate=lidar_dev.params['baud'])
        lidar.stop_motor()
    except rplidar.RPLidarException:
        return False, False, "missing lidar"

    # TODO: Check for microphone array

    # Check pimu and wacc
    p=r.pimu
    w=r.wacc
    checks = [
        # val_in_range('Voltage',p.status['voltage'], vmin=p.config['low_voltage_alert'], vmax=14.5),
        val_in_range('Current',p.status['current'], vmin=0.5, vmax=p.config['high_current_alert']),
        val_in_range('Temperature',p.status['temp'], vmin=10, vmax=45),
        val_in_range('IMU AZ',p.status['imu']['az'], vmin=-10.1, vmax=-9.5),
        # val_in_range('IMU Pitch', hu.rad_to_deg(p.status['imu']['pitch']), vmin=-12, vmax=12), # TODO
        # val_in_range('IMU Roll', hu.rad_to_deg(p.status['imu']['roll']), vmin=-12, vmax=12), # TODO
        val_in_range('AX',w.status['ax'], vmin=8.0, vmax=11.0),
    ]
    for c in checks:
        check_succeeded, check_msg = c
        if not check_succeeded:
            return True, False, check_msg

    return True, True, ""
def is_battery_ready():
    # TODO: Check charged connected but not charging

    # Check battery voltage
    p=r.pimu
    voltage_check_succeeded, _ = val_in_range('Voltage',p.status['voltage'], vmin=p.config['low_voltage_alert'], vmax=14.5)
    return voltage_check_succeeded, p.status['voltage']
print(Style.RESET_ALL)
print ('---- Checking Hardware ----')
comms_ready, comms_err_msg, comms_usb_device_seen, comms_ping_list = is_comms_ready()
if comms_ready:
    print(Fore.GREEN + '[Pass] Comms are ready')
else:
    print(Fore.RED + f'[Fail] Comms not ready ({comms_err_msg})')
if args.verbose:
    for d in comms_usb_device_seen:
        if comms_usb_device_seen[d]:
            print(Fore.LIGHTBLUE_EX + f'         {d} present')
        else:
            print(Fore.RED + f'         {d} missing')
    for p in comms_ping_list:
        print(Fore.LIGHTBLUE_EX + f'         {p} pinged')
actuators_ready, actuators_err_msg = are_actuators_ready()
if actuators_ready:
    print(Fore.GREEN + '[Pass] Actuators are ready')
else:
    print(Fore.RED + f'[Fail] Actuators not ready ({actuators_err_msg})')
sensors_ready, sensors_total, sensors_err_msg = are_sensors_ready()
if sensors_ready:
    if sensors_total:
        print(Fore.GREEN + '[Pass] Sensors are ready')
    else:
        print(Fore.YELLOW + f'[Warn] Sensors not ready ({sensors_err_msg})')
else:
    print(Fore.RED + f'[Fail] Sensors not ready ({sensors_err_msg})')
battery_ready, battery_voltage = is_battery_ready()
if battery_ready:
    print(Fore.GREEN + f'[Pass] Battery voltage is {battery_voltage:.1f} V')
else:
    print(Fore.RED + f'[Fail] Battery voltage is {battery_voltage:.1f} V')

# ###################  SOFTWARE  ######################
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
                'ros-humble-librealsense2': False,
                'librealsense2': True,
            }
        }
        apt_list = apt.Cache()
        for pkg, is_install_expected in apt_expectations[distro.version()].items():
            if pkg not in apt_list and is_install_expected:
                return False, f"{pkg} should be installed"
            if pkg in apt_list and is_install_expected != apt_list[pkg].is_installed:
                return False, f"{pkg} should {'not' if not is_install_expected else ''}be installed"
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
            pip_pkg_data[0] = pip_pkg_data[0].replace('_', '-')
            if len(pip_pkg_data) >= 2 and pip_pkg_data[0] in pip_versions:
                pip_versions[pip_pkg_data[0]] = pip_pkg_data[1]
                if len(pip_pkg_data) >= 3:
                    p = pip_pkg_data[2]
                    pip_editable_locations[pip_pkg_data[0]] = p.replace(str(pathlib.Path(p).home()), '~')

        # get latest pip versions
        latest_pip_version = {
            'hello-robot-stretch-body': version.parse('0.7.0'),
            'hello-robot-stretch-body-tools': version.parse('0.7.0'),
            'hello-robot-stretch-tool-share': version.parse('0.2.8'),
            'hello-robot-stretch-factory': version.parse('0.5.0'),
            'hello-robot-stretch-diagnostics': version.parse('0.0.14'),
            'hello-robot-stretch-urdf': version.parse('0.0.18'),
        }
        if scan_dict:
            latest_pip_version = {p: version.parse(scan_dict['pip'].get(p, '0.0.0')) for p in latest_pip_version}

        # check current against latest
        for pip_pkg in pip_versions:
            if pip_versions[pip_pkg] is None:
                return False, f"run pip3 install -U {pip_pkg}", pip_versions, pip_editable_locations
            p = version.parse(pip_versions[pip_pkg])
            if p < latest_pip_version[pip_pkg]:
                return False, f"run pip3 install -U {pip_pkg}", pip_versions, pip_editable_locations
        return True, "", pip_versions, pip_editable_locations
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
                    return True, ros_name, False, f"{pkg} should {'not' if not is_install_expected else ''}be installed", ''
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
        print(Fore.YELLOW + '[Warn] Firmware not up-to-date (run REx_firmware_updater.py --install)')
    if args.verbose:
        print(Fore.LIGHTBLUE_EX + '         hello-pimu = ' + Fore.CYAN + fw_versions['pimu'])
        print(Fore.LIGHTBLUE_EX + '         hello-wacc = ' + Fore.CYAN + fw_versions['wacc'])
        print(Fore.LIGHTBLUE_EX + '         hello-motor-arm = ' + Fore.CYAN + fw_versions['arm'])
        print(Fore.LIGHTBLUE_EX + '         hello-motor-lift = ' + Fore.CYAN + fw_versions['lift'])
        print(Fore.LIGHTBLUE_EX + '         hello-motor-left-wheel = ' + Fore.CYAN + fw_versions['left-wheel'])
        print(Fore.LIGHTBLUE_EX + '         hello-motor-right-wheel = ' + Fore.CYAN + fw_versions['right-wheel'])
    # Python
    pip_uptodate, pip_err_msg, pip_versions, pip_editable_locations = all_pip_uptodate()
    if pip_uptodate:
        print(Fore.GREEN + '[Pass] Python pkgs are up-to-date')
    else:
        print(Fore.YELLOW + f'[Warn] Python pkgs not up-to-date ({pip_err_msg})')
    if args.verbose:
        for bname in ['hello-robot-stretch-body', 'hello-robot-stretch-body-tools', 'hello-robot-stretch-tool-share', 'hello-robot-stretch-factory', 'hello-robot-stretch-diagnostics', 'hello-robot-stretch-urdf']:
            print(Fore.LIGHTBLUE_EX + f'         {bname} = ' + Fore.CYAN + f"{pip_versions[bname] if pip_versions[bname] else 'Not Installed'}" + Fore.LIGHTBLUE_EX + f"{f' (installed locally at {pip_editable_locations[bname]})' if pip_editable_locations[bname] else ''}")
    # ROS
    ros_enabled, ros_name, ros_ready, ros_err_msg, ros_ws_path = all_ros_correct()
    if ros_enabled:
        if ros_ready:
            print(Fore.GREEN + f'[Pass] {ros_name} is ready')
            if ros_ws_path and args.verbose:
                print(Fore.LIGHTBLUE_EX + f'         Workspace at {ros_ws_path}')
        else:
            print(Fore.YELLOW + f'[Warn] {ros_name} not ready ({ros_err_msg})')
            if ros_ws_path and args.verbose:
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
