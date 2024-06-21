#!/usr/bin/env python3
import stretch_body.robot_params
stretch_body.robot_params.RobotParams.set_logging_level("DEBUG")
stretch_body.robot_params.RobotParams.set_logging_formatter("brief_console_formatter")

import stretch_body.hello_utils as hu
hu.print_stretch_re_use()

import re
import os
import sys
import shlex
import distro
import logging
import argparse
import subprocess
from os.path import exists
import stretch_body.device
import stretch_body.hello_utils
from colorama import Fore, Back, Style

parser=argparse.ArgumentParser(description="Configure Stretch's software for the attached tool")
parser.add_argument('-v', "--verbose", help="Prints more information", action="store_true")
args=parser.parse_args()

cli_device = stretch_body.device.Device(name='stretch_configure_tool.py', req_params=False)
cli_device.logger.setLevel('DEBUG' if args.verbose else 'INFO')
logging.getLogger().setLevel(logging.CRITICAL)

def is_wrist_present():
    return exists('/dev/hello-dynamixel-wrist')

def how_many_dxl_on_wrist_chain():
    from stretch_body.dynamixel_XL430 import DynamixelXL430, DynamixelCommError
    cli_device.logger.debug("Scanning wrist dxl chain for motors...")
    nfound = 0
    for dxl_id in range(25):
        try:
            baud = DynamixelXL430.identify_baud_rate(dxl_id, '/dev/hello-dynamixel-wrist')
            if baud != -1:
                m = DynamixelXL430(dxl_id, '/dev/hello-dynamixel-wrist', baud=baud)
                m.startup()
                if not m.hw_valid:
                    continue
                if not m.do_ping(verbose=False):
                    continue
                m.stop()
                nfound += 1
        except DynamixelCommError:
            continue
    return nfound

def is_d405_present():
    import pyrealsense2 as rs
    rs_cameras = [{'name': device.get_info(rs.camera_info.name), 'serial_number': device.get_info(rs.camera_info.serial_number)}
        for device in rs.context().devices]
    for rs_cam in rs_cameras:
        if 'D405' in rs_cam['name']:
            return True
    return False

def run_cmd(cmdstr):
    cli_device.logger.debug(f'Executing command: {cmdstr}')
    returncode = os.system(cmdstr + ' > /dev/null 2>&1')
    if (returncode != 0):
        cli_device.logger.info(f"ERROR executing {cmdstr}")
        sys.exit(1)

cli_device.logger.info('Loading...')
robot_device = stretch_body.device.Device(name='robot')
stretch_model = robot_device.params.get('model_name', '')
stretch_tool = robot_device.params.get('tool', '')
supported_tools = robot_device.robot_params.get('supported_eoa', [])
if not is_wrist_present():
    cli_device.logger.info('ERROR: Unable to find wrist hardware. Contact Hello Robot Support.')
    sys.exit(1)
num_wrist_dxls = how_many_dxl_on_wrist_chain()
d405_present = is_d405_present()

def does_tool_need_to_change():
    cli_device.logger.info(f'Your software is currently configured for the {Fore.CYAN + stretch_tool + Style.RESET_ALL} tool')

    # check if current tool is supported
    if stretch_tool not in supported_tools:
        cli_device.logger.info(f"But {Fore.CYAN + stretch_tool + Style.RESET_ALL} isn't a supported tool for your robot")
        cli_device.logger.debug(f"The supported tools for your robot are: {Fore.YELLOW + str(supported_tools) + Style.RESET_ALL}")
        return True
    # check if the existing hardware, num dxls, matches the current tool
    tool_numdxls_d405_map = {
        'tool_none': (1, False),
        'tool_stretch_gripper': (2, False),
        'tool_stretch_dex_wrist': (4, False),
        'eoa_wrist_dw3_tool_nil': (3, True),
        'eoa_wrist_dw3_tool_sg3': (4, True),
        'eoa_wrist_dw3_tool_tablet_12in': (3, True),
    }
    expected_num_wrist_dxls, expected_d405_present = tool_numdxls_d405_map.get(stretch_tool, (-1, False))
    if num_wrist_dxls != expected_num_wrist_dxls:
        cli_device.logger.info(f"But the wrist chain has the wrong number of motors")
        cli_device.logger.debug(f"Num Dxls: {num_wrist_dxls}, Expected Num Dxls: {expected_num_wrist_dxls}")
        return True

    # check if the existing hardware, d405 present, matches the current tool
    if d405_present != expected_d405_present:
        cli_device.logger.info(f"""But the gripper camera {"should" if expected_d405_present else "shouldn't"} be present""")
        return True

    cli_device.logger.info("Which seems correct based on the hardware connected to your robot")
    cli_device.logger.info("Done!")
    return False

def determine_what_tool_is_correct():
    cli_device.logger.info('Determining what tool is correct for your robot...')

    # start with the supported list
    matches = supported_tools
    cli_device.logger.debug(f"Starting with the supported tools for your model: {Fore.YELLOW + str(matches) + Style.RESET_ALL}")

    # filter by num dxls
    numdxls_tool_map = {
        1: ['tool_none'],
        2: ['tool_stretch_gripper'],
        3: ['eoa_wrist_dw3_tool_nil', 'eoa_wrist_dw3_tool_tablet_12in'],
        4: ['tool_stretch_dex_wrist', 'eoa_wrist_dw3_tool_sg3']
    }
    numdxls_match = numdxls_tool_map.get(num_wrist_dxls, [])
    cli_device.logger.debug(f"These tools match based on {num_wrist_dxls} number of wrist dxls: {Fore.YELLOW + str(numdxls_match) + Style.RESET_ALL}")
    matches = list(set(matches) & set(numdxls_match))
    cli_device.logger.debug(f"Filtering based on this brings the matches to: {Fore.YELLOW + str(matches) + Style.RESET_ALL}")

    # filter by d405 present
    d405_tool_map = {
        False: ['tool_none', 'tool_stretch_gripper', 'tool_stretch_dex_wrist', 'eoa_wrist_dw3_tool_nil'],
        True: ['eoa_wrist_dw3_tool_sg3','eoa_wrist_dw3_tool_tablet_12in'],
    }
    d405_match = d405_tool_map.get(d405_present, [])
    cli_device.logger.debug(f"These tools match based on present={d405_present} gripper camera: {Fore.YELLOW + str(d405_match) + Style.RESET_ALL}")
    matches = list(set(matches) & set(d405_match))
    cli_device.logger.debug(f"Filtering based on this brings the matches to: {Fore.YELLOW + str(matches) + Style.RESET_ALL}")

    if len(matches) == 0:
        cli_device.logger.info('Unable to find any tool that matches the hardware connected to your robot. Contact Hello Robot support for help.')
        cli_device.logger.info("Not changing anything. Exiting.")
        sys.exit(1)
    elif len(matches) > 1:
        cli_device.logger.info('Found multiple possible tools that could work:')
        for i, match in enumerate(matches):
            cli_device.logger.info(f'    {i}) {match}')
        user_select = int(input('Select one. If unsure, Ctrl C to exit & contact Hello Robot support. Input #: '))
        if user_select >= len(matches):
            cli_device.logger.info('Invalid selection. Exiting.')
            sys.exit(1)
        return matches[user_select]
    else:
        return matches[0]

def configure_tool(target_tool_name):
    cli_device.logger.info(f'Configuring your software to use the {Fore.GREEN + target_tool_name + Style.RESET_ALL} tool')
    user_input = input('Proceed? (y/n): ')
    if user_input not in ['y', 'Y', 'yes', 'YES']:
        cli_device.logger.info('Not changing anything. Exiting.')
        sys.exit(1)

    # check config params exist
    if not exists(stretch_body.hello_utils.get_fleet_directory()+'stretch_configuration_params.yaml'):
        cli_device.logger.info('Please run RE1_migrate_params.py before continuing. For more details, see https://forum.hello-robot.com/t/425')
        sys.exit(1)

    # check stretch_urdf exists
    ubuntu_version = distro.version()
    if ubuntu_version == '18.04':
        cli_device.logger.info('This CLI doesnt support Ubuntu 18.04. Consider upgrading your robots operating system.')
        sys.exit(1)
    elif ubuntu_version == '20.04':
        import importlib_resources
        root_dir = str(importlib_resources.files("stretch_urdf"))
        data_dir = f"{root_dir}/{stretch_model}"
        cli_device.logger.debug(f'data_dir={data_dir}')
    elif ubuntu_version == '22.04':
        import importlib.resources as importlib_resources
        root_dir = importlib_resources.files("stretch_urdf")
        data_dir = f"{root_dir}/{stretch_model}"
        cli_device.logger.debug(f'data_dir={data_dir}')
    else:
        cli_device.logger.info(f'This CLI doesnt support {ubuntu_version}. Consider upgrading your robots operating system.')
        sys.exit(1)

    # check stretch_urdf has target_tool_name
    target_tool_urdf = f"stretch_description_{stretch_model}_{target_tool_name}.urdf"
    target_tool_xacro = f"stretch_description_{stretch_model}_{target_tool_name}.xacro"
    if target_tool_urdf not in os.listdir(data_dir) or target_tool_xacro not in os.listdir(data_dir + '/xacro'):
        cli_device.logger.info(f'Cannot find URDF for this tool. Contact Hello Robot support.')
        cli_device.logger.debug(f"Target URDF={target_tool_urdf}. Target XACRO={target_tool_xacro}. Stretch URDF has these: {os.listdir(data_dir)}")
        cli_device.logger.info("Not changing anything. Exiting.")
        sys.exit(1)

    # check ROS workspace exists
    ros_distro = os.getenv('ROS_DISTRO')
    if ros_distro == 'noetic':
        ros_repo_path = f'/home/{os.getenv("USER")}/catkin_ws/src/stretch_ros'
        if not exists(ros_repo_path):
            cli_device.logger.info(f'Cannot find ROS workspace. Consider creating a new ROS workspace.')
            cli_device.logger.debug(f"Checked for ROS workspace here: {ros_repo_path}")
            cli_device.logger.info("Not changing anything. Exiting.")
            sys.exit(1)
    elif ros_distro == 'humble':
        ros_repo_path = f'/home/{os.getenv("USER")}/ament_ws/src/stretch_ros2'
        if not exists(ros_repo_path):
            cli_device.logger.info(f'Cannot find ROS workspace. Consider creating a new ROS workspace.')
            cli_device.logger.debug(f"Check for ROS workspace here: {ros_repo_path}")
            cli_device.logger.info("Not changing anything. Exiting.")
            sys.exit(1)

    # update the robot parameters for the next tool
    if stretch_model == 'RE1V0': # Stretch RE1
        tool_feedforward_map = {
            'tool_none': 0.4,
            'tool_stretch_gripper': 0.4,
            'tool_stretch_dex_wrist': 0.75,
            'eoa_wrist_dw3_tool_nil': 0.75,
            'eoa_wrist_dw3_tool_tablet_12in': 0.75,
            'eoa_wrist_dw3_tool_sg3': 0.75,
        }
        feedforward_value = tool_feedforward_map.get(target_tool_name, 0.8)
        cli_device.logger.debug(f'For model={stretch_model} and tool={target_tool_name}, choosing i_feedforward={feedforward_value}')

        tool_yaml = {
            'robot': {'tool': target_tool_name},
            'lift': {'i_feedforward': feedforward_value},
            'hello-motor-lift': {'gains': {'i_safety_feedforward': feedforward_value}}
        }
    else: # Stretch 2 and Stretch 3
        tool_feedforward_map = {
            'tool_none': 1.2,
            'tool_stretch_gripper': 1.2,
            'tool_stretch_dex_wrist': 1.8,
            'eoa_wrist_dw3_tool_nil': 1.8,
            'eoa_wrist_dw3_tool_tablet_12in': 1.8,
            'eoa_wrist_dw3_tool_sg3': 1.8,
        }
        feedforward_value = tool_feedforward_map.get(target_tool_name, 1.9)
        cli_device.logger.debug(f'For model={stretch_model} and tool={target_tool_name}, choosing i_feedforward={feedforward_value}')

        tool_yaml = {
            'robot': {'tool': target_tool_name},
            'lift': {'i_feedforward': feedforward_value},
            'hello-motor-lift': {'gains': {'i_safety_feedforward': feedforward_value}}
        }

    # push new parameters to yaml
    configuration_yaml = stretch_body.hello_utils.read_fleet_yaml('stretch_configuration_params.yaml')
    stretch_body.hello_utils.overwrite_dict(overwritee_dict=configuration_yaml, overwriter_dict=tool_yaml)
    stretch_body.hello_utils.write_fleet_yaml('stretch_configuration_params.yaml', configuration_yaml,
                                              header=stretch_body.robot_params.RobotParams().get_configuration_params_header())

    # get rid of export_urdf and exported_urdf_previous directories
    run_cmd(f"rm -rf {ros_repo_path}/stretch_description/urdf/exported_urdf")
    run_cmd(f"rm -rf {ros_repo_path}/stretch_description/urdf/exported_urdf_previous")

    # copy URDF mesh files to ROS workspace
    src = f"{data_dir}/meshes/*"
    dst = f"{ros_repo_path}/stretch_description/meshes/"
    cmd = f"cp -r {src} {dst}"
    run_cmd(cmd)

    # copy URDF xacro files to ROS workspace
    src = f"{data_dir}/xacro/*"
    dst = f"{ros_repo_path}/stretch_description/urdf/"
    cmd = f"cp -r {src} {dst}"
    run_cmd(cmd)

    # replace mesh paths to package:// format
    def search_and_replace(file_path, search_word, replace_word):
        if file_path.endswith('.sh') or file_path.endswith('.md'):
            return
        with open(file_path, 'r') as file:
            file_contents = file.read()
            updated_contents = file_contents.replace(search_word, replace_word)
        with open(file_path, 'w') as file:
            file.write(updated_contents)
    def remove_lines_between_patterns(text, start_pattern, end_pattern):
        start_regex = re.compile(re.escape(start_pattern) + r'.*?' + re.escape(end_pattern), re.DOTALL)
        end_regex = re.compile(re.escape(end_pattern) + r'.*?' + re.escape(start_pattern), re.DOTALL)
        cleaned_text = start_regex.sub(start_pattern + end_pattern, text)
        cleaned_text = end_regex.sub(start_pattern + end_pattern, cleaned_text)
        cleaned_text = cleaned_text.replace(start_pattern + end_pattern, "")
        return cleaned_text
    for f in os.listdir(dst):
        try:
            search_and_replace(f"{dst}/{f}",'./meshes','package://stretch_description/meshes')
        except IsADirectoryError:
            for f2 in os.listdir(f"{dst}/{f}"):
                search_and_replace(f"{dst}/{f}/{f2}",'./meshes','package://stretch_description/meshes')

        # Manually remove the link_ground and joint_ground
        if f == 'stretch_main.xacro':
            main = f"{dst}/{f}"
            file = open(main)
            r = remove_lines_between_patterns(file.read(),'<link\n    name="link_ground">','</link>')
            r = remove_lines_between_patterns(r,'<joint\n    name="joint_ground"','</joint>')
            file.close()
            run_cmd(f"rm {main}")
            f = open(main, "a")
            f.write(r)
            f.close()

    # set stretch_description.xacro
    cli_device.logger.debug(f"Updating stretch_description.xacro from {target_tool_xacro}")
    cmd = f"cp {ros_repo_path}/stretch_description/urdf/{target_tool_xacro} {ros_repo_path}/stretch_description/urdf/stretch_description.xacro"
    run_cmd(cmd)

    # generate the calibrated URDF
    if ros_distro == 'noetic':
        cli_device.logger.debug("Starting roscore in background...")
        run_cmd('roscore &')
        cli_device.logger.debug("Updating URDF after xacro change...")
        run_cmd('rosrun stretch_calibration update_urdf_after_xacro_change.sh')
    elif ros_distro == 'humble':
        cli_device.logger.debug("Updating URDF after xacro change...")
        run_cmd('ros2 run stretch_calibration update_urdf_after_xacro_change')
        cli_device.logger.debug("Rebuild stretch_description package...")
        run_cmd('cd ~/ament_ws;colcon build --packages-select stretch_description')

    # export the calibrated URDF
    if ros_distro == 'noetic':
        cli_device.logger.debug("Export URDF...")
        run_cmd('cd ~/catkin_ws/src/stretch_ros/stretch_description/urdf; ./export_urdf.sh')
    elif ros_distro == 'humble':
        cli_device.logger.debug("Export URDF...")
        run_cmd('cd ~/ament_ws/src/stretch_ros2/stretch_description/urdf; ./export_urdf.sh')

    cli_device.logger.info('Done!')

if does_tool_need_to_change():
    target_tool_name = determine_what_tool_is_correct()
    configure_tool(target_tool_name)
