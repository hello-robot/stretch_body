#!/usr/bin/env python
from __future__ import print_function
import argparse
parser = argparse.ArgumentParser(description='Check that all robot hardware is present and reporting sane values')
parser.add_argument('-v', "--verbose", help="verbose logging", action="store_true")
parser.add_argument('-n', "--nocolor", help="no color", action="store_true")
args = parser.parse_args()

import stretch_body.robot_params
stretch_body.robot_params.RobotParams.set_logging_level("DEBUG" if args.verbose else "CRITICAL")
import stretch_body.device # must be imported directly after using RobotParams.set_logging_level

import unittest
import sys
sys.path.insert(0, "/home/hello-robot/repos/stretch_body/tools")
import test.test_system
from colorama import Fore, Back, Style
import stretch_body.hello_utils as hu
hu.print_stretch_re_use()


if args.nocolor:
    Fore.GREEN = ''
    Fore.RED = ''
    Fore.YELLOW = ''
    Fore.CYAN = ''
    Style.RESET_ALL = ''


class SystemCheckTestResult(unittest.result.TestResult):

    def __init__(self, stream):
        super(SystemCheckTestResult, self).__init__(stream, None, None)
        self.stream = stream

    def startTest(self, test):
        super(SystemCheckTestResult, self).startTest(test)
        test_description = test.shortDescription()
        if test_description is None:
            print('SYSTEM CHECK ERROR: Each method in the test case must have method-level docstrings\n')

        self.stream.write(test_description)
        self.stream.write("... ")
        self.stream.flush()

    def addSuccess(self, test):
        super(SystemCheckTestResult, self).addSuccess(test)
        self.stream.writeln(Fore.GREEN + "PASS" + Style.RESET_ALL)
        self.stream.flush()

    def addError(self, test, err):
        super(SystemCheckTestResult, self).addError(test, err)
        self.stream.writeln(Fore.RED + "FAIL ({0})".format(err[1]) + Style.RESET_ALL)
        self.stream.flush()

    def addFailure(self, test, err):
        super(SystemCheckTestResult, self).addFailure(test, err)
        print_as_warn = False
        what_to_warn = None
        testMethod = getattr(test, test._testMethodName, None)
        if testMethod is not None:
            print_as_warn = getattr(testMethod, "__system_check_warn__", False)
            what_to_warn = getattr(testMethod, "__system_check_warning__", None)
        if print_as_warn:
            msg = err[1] if what_to_warn is None else what_to_warn
            self.stream.writeln(Fore.YELLOW + "WARN ({0})".format(msg) + Style.RESET_ALL)
        else:
            self.stream.writeln(Fore.RED + "FAIL ({0})".format(err[1]) + Style.RESET_ALL)
        self.stream.flush()

    def addSkip(self, test, reason):
        super(SystemCheckTestResult, self).addSkip(test, reason)
        self.stream.writeln(Fore.CYAN + "SKIP" + Style.RESET_ALL)
        self.stream.flush()

    def addExpectedFailure(self, test, err):
        super(SystemCheckTestResult, self).addExpectedFailure(test, err)
        self.stream.writeln(Fore.GREEN + "PASS" + Style.RESET_ALL)
        self.stream.flush()

    def addUnexpectedSuccess(self, test):
        super(SystemCheckTestResult, self).addUnexpectedSuccess(test)
        self.stream.writeln(Fore.RED + "FAIL" + Style.RESET_ALL)
        self.stream.flush()


class SystemCheckTestRunner(unittest.TextTestRunner):

    def run(self, suite):
        if len(suite._tests) == 0:
            print('SYSTEM CHECK ERROR: A test suite must have at least one test\n')
            return
        class_doc = suite._tests[0].__doc__
        if class_doc is None:
            print('SYSTEM CHECK ERROR: A test case must have a class-level docstring\n')
            return

        self.stream.write("---- ")
        self.stream.write(class_doc.split("\n")[0].strip())
        self.stream.writeln(" ----")
        self.stream.flush()
        result = SystemCheckTestResult(self.stream)
        suite(result)
        self.stream.writeln()
        self.stream.flush()
        return result

usbdevices_suite = unittest.TestSuite()
usbdevices_suite.addTests([
    test.test_system.TestUSBDevices('test_usb_aliases_present'),
    test.test_system.TestUSBDevices('test_num_acm_devices_present'),
])

head_suite = unittest.TestSuite()
head_suite.addTests([
    test.test_system.TestHead('test_joints_pingable'),
    test.test_system.TestHead('test_joints_homed'),
    test.test_system.TestHead('test_head_camera_present'),
])

endofarm_suite = unittest.TestSuite()
endofarm_suite.addTests([
    test.test_system.TestEndOfArm('test_joints_pingable'),
    test.test_system.TestEndOfArm('test_joints_homed'),
])

arm_suite = unittest.TestSuite()
arm_suite.addTests([
    test.test_system.TestArm('test_valid_status'),
    test.test_system.TestArm('test_arm_homed'),
])

lift_suite = unittest.TestSuite()
lift_suite.addTests([
    test.test_system.TestLift('test_valid_status'),
    test.test_system.TestLift('test_lift_homed'),
])

base_suite = unittest.TestSuite()
base_suite.addTests([
    test.test_system.TestBase('test_valid_lwheel_status'),
    test.test_system.TestBase('test_valid_rwheel_status'),
])

wacc_suite = unittest.TestSuite()
wacc_suite.addTests([
    test.test_system.TestWACC('test_valid_accelx'),
    test.test_system.TestWACC('test_valid_digital_inputs'),
])

pimu_suite = unittest.TestSuite()
pimu_suite.addTests([
    test.test_system.TestPIMU('test_valid_voltage'),
    test.test_system.TestPIMU('test_valid_current'),
    test.test_system.TestPIMU('test_valid_temperature'),
    test.test_system.TestPIMU('test_valid_cliff0'),
    test.test_system.TestPIMU('test_valid_cliff1'),
    test.test_system.TestPIMU('test_valid_cliff2'),
    test.test_system.TestPIMU('test_valid_cliff3'),
    test.test_system.TestPIMU('test_valid_imu_az'),
    test.test_system.TestPIMU('test_valid_imu_pitch'),
    test.test_system.TestPIMU('test_valid_imu_roll'),
])

software_suite = unittest.TestSuite()
software_suite.addTests([
    test.test_system.TestSoftware('test_latest_hello_pip_packages'),
    test.test_system.TestSoftware('test_latest_firmware'),
])

runner = SystemCheckTestRunner(stream=sys.stdout)
runner.run(usbdevices_suite)
runner.run(head_suite)
runner.run(endofarm_suite)
runner.run(arm_suite)
runner.run(lift_suite)
runner.run(base_suite)
runner.run(wacc_suite)
runner.run(pimu_suite)
runner.run(software_suite)
