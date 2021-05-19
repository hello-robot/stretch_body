from __future__ import print_function
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
        Device.__init__(self,'robot_sentry')
        self.robot=robot

    def startup(self):
        return True

    def step(self):
        if self.params['wrist_yaw_overload'] and self.robot.end_of_arm is not None:
            self.robot.end_of_arm.motors['wrist_yaw'].step_sentry()

        if self.params['stretch_gripper_overload'] and self.robot.end_of_arm is not None:
            if self.robot.end_of_arm.is_tool_present('StretchGripper'):
                self.robot.end_of_arm.motors['stretch_gripper'].step_sentry()

        if self.params['base_max_velocity'] and self.robot.base is not None:
            self.robot.base.step_sentry(self.robot.lift.status['pos'], self.robot.arm.status['pos'],self.robot.end_of_arm.motors['wrist_yaw'].status['pos'])


        if self.params['base_fan_control'] and self.robot.pimu is not None:
            self.robot.pimu.step_sentry('base_fan_control')

        if 'dynamixel_stop_on_runstop' in self.params and self.params['dynamixel_stop_on_runstop']:
            if self.robot.head is not None:
                self.robot.head.step_sentry(runstop=self.robot.pimu.status['runstop_event'])
            if self.robot.end_of_arm is not None:
                self.robot.end_of_arm.step_sentry(runstop=self.robot.pimu.status['runstop_event'])
