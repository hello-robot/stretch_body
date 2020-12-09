#! /usr/bin/env python
import time
from stretch_body.device import Device
from stretch_body.hello_utils import *

class RobotTimestampManager(Device):
    """
      This is updated from Robot._pull_status_non_dynamixel()

      It computes the PC timestamp for each non-dynamixel subsystems sensor data
      The Pimu hardware clock is the reference for the Pimu and steppers
      The Wacc hardware clock is the reference for the Wacc

      All timestamps are relative to the Linux 'wall time'
      The Pimu and Wacc ClockManagers attempt to keep the skew between the
      respective hardware clocks and wall time minimal (<~2ms) when use_skew_compensation is enabled

      sync_mode_enabled: 0/1 : required to be enabled to use synchronous data capture and motor commands
      use_skew_compensation: 0/1: compensate the hardware clock timebase so stays aligned with PC (time-constant 10s of seconds)
      time_align_status: 0/1: interpolate between status frames
    """
    def __init__(self,robot):
        Device.__init__(self)
        self.robot=robot
        self.param=self.robot_params['robot_timestamp_manager']
        self.status_history=[None]*25 #Store last 25 status (approx 1 second of data)
        self.status_time_aligned=None
        self.status_id=0

    def startup(self): #This should be called prior to starting of Robot threads

        if self.param['sync_mode_enabled']:
            self.__enable_sync_mode()
        else:
            self.__disable_sync_mode()

    def __enable_sync_mode(self):
        self.robot.pimu.enable_sync_mode()
        self.robot.pimu.push_command()
        self.robot.arm.motor.enable_sync_mode()
        self.robot.arm.push_command()
        self.robot.lift.motor.enable_sync_mode()
        self.robot.lift.push_command()
        self.robot.base.left_wheel.enable_sync_mode()
        self.robot.base.right_wheel.enable_sync_mode()
        self.robot.base.push_command()
        self.robot.wacc.enable_sync_mode()
        self.robot.wacc.push_command()

    def __disable_sync_mode(self):
        self.robot.pimu.disable_sync_mode()
        self.robot.pimu.push_command()
        self.robot.arm.motor.disable_sync_mode()
        self.robot.arm.push_command()
        self.robot.lift.motor.disable_sync_mode()
        self.robot.lift.push_command()
        self.robot.base.left_wheel.disable_sync_mode()
        self.robot.base.right_wheel.disable_sync_mode()
        self.robot.base.push_command()
        self.robot.wacc.disable_sync_mode()
        self.robot.wacc.push_command()

    def step(self):
        self.robot.status['timestamps']['non_dynamixel_wall_time']=SystemTimestamp().from_wall_time()
        if not self.param['sync_mode_enabled']:
            self.robot.status['timestamps']['pimu_imu'] = SystemTimestamp().from_secs(self.robot.pimu.status['timestamp_pc'])
            self.robot.status['timestamps']['lift_enc'] =  SystemTimestamp().from_secs(self.robot.lift.status['timestamp_pc'])
            self.robot.status['timestamps']['arm_enc'] =  SystemTimestamp().from_secs(self.robot.arm.status['timestamp_pc'])
            self.robot.status['timestamps']['left_wheel_enc'] =  SystemTimestamp().from_secs(self.base.status['timestamp_pc'])
            self.robot.status['timestamps']['right_wheel_enc'] =  SystemTimestamp().from_secs(self.base.status['timestamp_pc'])
            self.robot.status['timestamps']['wacc_acc'] = SystemTimestamp().from_secs(self.robot.wacc.status['timestamp_pc'])
        else:
            ts_pimu_base =  self.robot.pimu.status['timestamp_line_sync'] #The time that the sync line was toggled
            dt_pimu_imu = self.robot.pimu.status['timestamp']-ts_pimu_base

            ts_wacc_base = self.robot.wacc.status['timestamp_status_sync']  # The time that the status sync RPC was handled
            dt_wacc_acc = self.robot.wacc.status['timestamp'] - ts_wacc_base

            dt_lift_enc = self.robot.lift.motor.status['timestamp']-self.robot.lift.motor.status['timestamp_line_sync']
            dt_arm_enc = self.robot.arm.motor.status['timestamp'] - self.robot.arm.motor.status['timestamp_line_sync']
            dt_left_wheel_enc = self.robot.base.left_wheel.status['timestamp'] - self.robot.base.left_wheel.status['timestamp_line_sync']
            dt_right_wheel_enc = self.robot.base.right_wheel.status['timestamp'] - self.robot.base.right_wheel.status['timestamp_line_sync']

            self.robot.status['timestamps']['hw_sync'] = self.robot.pimu.clock_manager.HW_to_PC_timestamp(ts_pimu_base)
            self.robot.status['timestamps']['pimu_imu'] = self.robot.pimu.clock_manager.HW_to_PC_timestamp(dt_pimu_imu+ts_pimu_base)
            self.robot.status['timestamps']['lift_enc'] = self.robot.pimu.clock_manager.HW_to_PC_timestamp(dt_lift_enc + ts_pimu_base)
            self.robot.status['timestamps']['arm_enc'] = self.robot.pimu.clock_manager.HW_to_PC_timestamp(dt_arm_enc + ts_pimu_base)
            self.robot.status['timestamps']['left_wheel_enc'] = self.robot.pimu.clock_manager.HW_to_PC_timestamp(dt_left_wheel_enc + ts_pimu_base)
            self.robot.status['timestamps']['right_wheel_enc'] = self.robot.pimu.clock_manager.HW_to_PC_timestamp(dt_right_wheel_enc + ts_pimu_base)
            self.robot.status['timestamps']['wacc_acc'] = self.robot.wacc.clock_manager.HW_to_PC_timestamp(dt_wacc_acc + ts_wacc_base)

            if self.param['time_align_status']:
                pass #Not yet implemented

        # Record the current status
        self.status_history.append(self.robot.status.copy())
        self.status_history = self.status_history[1:]
