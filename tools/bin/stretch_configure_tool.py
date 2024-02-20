#!/usr/bin/env python3
import stretch_body.robot_params
stretch_body.robot_params.RobotParams.set_logging_level("DEBUG")
stretch_body.robot_params.RobotParams.set_logging_formatter("brief_console_formatter")

import stretch_body.hello_utils as hu
hu.print_stretch_re_use()

import logging
import argparse
import stretch_body.device

parser=argparse.ArgumentParser(description="Configure Stretch's software for the attached tool")
parser.add_argument('-v', "--verbose", help="Prints more information", action="store_true")
args=parser.parse_args()

cli_device = stretch_body.device.Device(name='stretch_configure_tool.py', req_params=False)
cli_device.logger.setLevel('DEBUG' if args.verbose else 'INFO')

from colorama import Fore, Back, Style
cli_device.logger.debug('hey debug')
cli_device.logger.info(Fore.RED + 'hey info')
cli_device.logger.warning(Fore.CYAN + 'hey warning')
cli_device.logger.error('hey error')
cli_device.logger.critical('hey critical')

