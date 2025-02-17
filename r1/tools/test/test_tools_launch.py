import unittest
import stretch_body.robot_params
stretch_body.robot_params.RobotParams.set_logging_level("DEBUG")

# import subprocess
# import sys

import subprocess, threading

import shlex
from subprocess import Popen, PIPE
from threading import Timer

def run(cmd, timeout_sec):
    proc = Popen(shlex.split(cmd), stdout=PIPE, stderr=PIPE)
    timer = Timer(timeout_sec, proc.kill)
    try:
        timer.start()
        stdout, stderr = proc.communicate()
        print(stderr)
    finally:
        print('\r\n')
        timer.cancel()
        return proc.returncode

tools=['stretch_about.py','stretch_arm_home.py -h','stretch_arm_jog.py','stretch_audio_test.py',
        'stretch_base_jog.py','stretch_gripper_home.py -h', 'stretch_gripper_jog.py','stretch_hardware_echo.py',
        'stretch_head_jog.py','stretch_lift_home.py -h','stretch_lift_jog.py', 'stretch_params.py','stretch_pimu_jog.py',
        'stretch_pimu_scope.py --ax','stretch_respeaker_test.py', 'stretch_robot_battery_check.py','stretch_robot_dynamixel_reboot.py',
        'stretch_robot_home.py -h','stretch_robot_jog.py','stretch_robot_keyboard_teleop.py','stretch_robot_monitor.py',
        'stretch_robot_system_check.py','stretch_rp_lidar_jog.py --range',
        'stretch_wacc_jog.py','stretch_wacc_scope.py','stretch_wrist_yaw_jog.py','stretch_xbox_controller_teleop.py']

tool_py3_only=['stretch_robot_urdf_visualizer.py']

#This has an issue where stdout / console gets in a state
#Where print statement dont' new line. To fix...
class TestToolsLaunch(unittest.TestCase):

    def test_tools_launch_py2(self):
        path='../bin/'
        for t in tools:
            print('Py2: Launching %s ...'%t)
            r=run('python2 '+path+t,timeout_sec=2.0)
            self.assertNotEqual(r,1)

    def test_tools_launch_py3(self):
        path='../bin/'
        for t in tools:
            print('Py3: Launching %s ...'%t)
            r=run('python3 '+path+t,timeout_sec=2.0)
            self.assertNotEqual(r,1)
        for t in tool_py3_only:
            print('Py3: Launching %s ...' % t)
            r = run('python3 ' + path + t, timeout_sec=2.0)
            self.assertNotEqual(r, 1)