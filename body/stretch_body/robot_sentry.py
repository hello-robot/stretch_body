#! /usr/bin/env python

from stretch_body.device import Device

class RobotSentry(Device):
    """
    The RobotSentry class allows Devices to keep hardware
    within safe operating conditions.
    It is managed by the Robot class, allowing for system level Status data
    to inform the local Device controller context
    Devices that implement Sentry functions may override user control targets to ensure safe operation
    Sentry functions may be turn on/off/configured via YAML
    It runs at 12.5Hz
    """
    def __init__(self,robot):
        Device.__init__(self)
        self.robot=robot
        self.param=self.robot_params['robot_sentry']

    def startup(self):
        pass

    def step(self):
        if self.param['wrist_yaw_overload'] and self.robot.end_of_arm is not None:
            self.robot.end_of_arm.motors['wrist_yaw'].step_sentry()

        if self.param['stretch_gripper_overload'] and self.robot.end_of_arm is not None:
            if self.robot.end_of_arm.is_tool_present('StretchGripper'):
                self.robot.end_of_arm.motors['stretch_gripper'].step_sentry()

        if self.param['base_max_velocity'] and self.robot.base is not None:
            self.robot.base.step_sentry(self.robot.lift.status['pos'], self.robot.arm.status['pos'],self.robot.end_of_arm.motors['wrist_yaw'].status['pos'])


        if self.param['base_fan_control'] and self.robot.pimu is not None:
            self.robot.pimu.step_sentry('base_fan_control')

        if self.param.has_key('dynamixel_stop_on_runstop') and self.param['dynamixel_stop_on_runstop'] and self.robot.pimu is not None:
            if self.robot.head is not None:
                self.robot.head.step_sentry(runstop=self.robot.pimu.status['runstop_event'])
            if self.robot.end_of_arm is not None:
                self.robot.end_of_arm.step_sentry(runstop=self.robot.pimu.status['runstop_event'])
