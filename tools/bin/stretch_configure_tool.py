#!/usr/bin/env python3
import stretch_body.robot_params
stretch_body.robot_params.RobotParams.set_logging_level("DEBUG")
stretch_body.robot_params.RobotParams.set_logging_formatter("brief_console_formatter")

import stretch_body.hello_utils as hu
hu.print_stretch_re_use()

import sys
import logging
import argparse
import stretch_body.device
from colorama import Fore, Back, Style

parser=argparse.ArgumentParser(description="Configure Stretch's software for the attached tool")
parser.add_argument('-v', "--verbose", help="Prints more information", action="store_true")
args=parser.parse_args()

cli_device = stretch_body.device.Device(name='stretch_configure_tool.py', req_params=False)
cli_device.logger.setLevel('DEBUG' if args.verbose else 'INFO')
logging.getLogger().setLevel(logging.CRITICAL)

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

cli_device.logger.info('Loading...')
robot_device = stretch_body.device.Device(name='robot')
stretch_tool = robot_device.params.get('tool', '')
# supported_tools = robot_device.robot_params.get('supported_eoa', []) # TODO
supported_tools = ['tool_none', 'tool_stretch_gripper', 'tool_stretch_dex_wrist', 'eoa_wrist_dw3_tool_nil', 'eoa_wrist_dw3_tool_sg3']
# num_wrist_dxls = how_many_dxl_on_wrist_chain() # TODO
num_wrist_dxls = 4
# d405_present = is_d405_present() # TODO
d405_present = True

def does_tool_need_to_change():
    cli_device.logger.info(f'Your software is currently configured for the {Fore.CYAN + stretch_tool + Style.RESET_ALL} tool')

    # check if current tool is supported
    if stretch_tool not in supported_tools:
        cli_device.logger.info(f"But {Fore.CYAN + stretch_tool + Style.RESET_ALL} isn't a supported tool for your robot")
        cli_device.logger.debug(f"The supported tools for your robot are: {Fore.CYAN + str(supported_tools) + Style.RESET_ALL}")
        return True

    # check if the existing hardware, num dxls, matches the current tool
    tool_numdxls_d405_map = {
        'tool_none': (1, False),
        'tool_stretch_gripper': (2, False),
        'tool_stretch_dex_wrist': (4, False),
        'eoa_wrist_dw3_tool_nil': (3, True),
        'eoa_wrist_dw3_tool_sg3': (4, True),
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
    cli_device.logger.info("Not changing anything. Exiting.")
    return False

def determine_what_tool_is_correct():
    cli_device.logger.info('Determining what tool is correct for your robot...')

    # start with the supported list
    matches = supported_tools
    cli_device.logger.debug(f"Starting with the supported tools for your model: {Fore.CYAN + str(matches) + Style.RESET_ALL}")

    # filter by num dxls
    numdxls_tool_map = {
        1: ['tool_none'],
        2: ['tool_stretch_gripper'],
        3: ['eoa_wrist_dw3_tool_nil'],
        4: ['tool_stretch_dex_wrist', 'eoa_wrist_dw3_tool_sg3']
    }
    numdxls_match = numdxls_tool_map.get(num_wrist_dxls, [])
    cli_device.logger.debug(f"These tools match based on {num_wrist_dxls} number of wrist dxls: {Fore.CYAN + str(numdxls_match) + Style.RESET_ALL}")
    matches = list(set(matches) & set(numdxls_match))
    cli_device.logger.debug(f"Filtering based on this brings the matches to: {Fore.CYAN + str(matches) + Style.RESET_ALL}")

    # filter by d405 present
    d405_tool_map = {
        False: ['tool_none', 'tool_stretch_gripper', 'tool_stretch_dex_wrist'],
        True: ['eoa_wrist_dw3_tool_nil', 'eoa_wrist_dw3_tool_sg3'],
    }
    d405_match = d405_tool_map.get(d405_present, [])
    cli_device.logger.debug(f"These tools match based on present={d405_present} gripper camera: {Fore.CYAN + str(d405_match) + Style.RESET_ALL}")
    matches = list(set(matches) & set(d405_match))
    cli_device.logger.debug(f"Filtering based on this brings the matches to: {Fore.CYAN + str(matches) + Style.RESET_ALL}")

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
            cli_device.logger.info('Invalid selection. Exiting...')
            sys.exit(1)
        return matches[user_select]
    else:
        return matches[0]

if does_tool_need_to_change():
    target_tool_name = determine_what_tool_is_correct()
    print(target_tool_name)
