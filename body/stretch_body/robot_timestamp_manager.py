from __future__ import print_function
from stretch_body.device import Device
from pyquaternion import Quaternion

class RobotTimestampManager(Device):
    """
      This is updated from Robot._pull_status_non_dynamixel()

      It computes the PC timestamp for each non-dynamixel subsystems sensor data
      The Pimu hardware clock is the reference for the Pimu and steppers
      The Wacc hardware clock is the reference for the Wacc

      All timestamps are relative to the Linux 'wall time'
      The Pimu and Wacc ClockManagers attempt to keep the skew between the
      respective hardware clocks and wall time minimal (<~2ms) when use_skew_compensation is enabled

      robot.params.sync_mode_enabled: 0/1 : required to be enabled to use synchronous data capture and motor commands
      use_skew_compensation: 0/1: compensate the hardware clock timebase so stays aligned with PC (time-constant 10s of seconds)
      time_align_status: 0/1: interpolate between status frames
    """
    def __init__(self,robot):
        Device.__init__(self)
        self.robot=robot
        self.param=self.robot_params['robot_timestamp_manager']
        self.status_time_aligned=None
        self.status_id=0

    def startup(self): #This should be called prior to starting of Robot threads
        pass

    def pretty_print(self):
        print('------ Timestamp Manager -----')
        s=self.robot.status_history[-1][1]
        print('Sync mode enabled    : '+str(self.robot.params['sync_mode_enabled']))
        print('Status ID            : '+str(self.robot.status_history[-1][0]))
        print('Wall time            : '+str(s['timestamps']['non_dynamixel_wall_time']))
        print('Hardware sync        : ' + str(s['timestamps']['hw_sync']))
        print('Pimu IMU             : '+str(s['timestamps']['pimu_imu']))
        print('Lift Encdoer         : ' + str(s['timestamps']['lift_enc']))
        print('Arm Encoder          : ' + str(s['timestamps']['arm_enc']))
        print('Right Wheel Encoder  : ' + str(s['timestamps']['right_wheel_enc']))
        print('Left Wheel Encoder   : ' + str(s['timestamps']['left_wheel_enc']))
        print('Wacc Accel           : ' + str(s['timestamps']['wacc_acc']))

    def step(self): #Called at approx 25Hz
        self.robot.status['timestamps']['non_dynamixel_wall_time']=SystemTimestamp().from_wall_time()
        if not self.robot.params['sync_mode_enabled']:
            self.robot.status['timestamps']['pimu_imu'] = SystemTimestamp().from_secs(self.robot.pimu.status['timestamp_pc'])
            self.robot.status['timestamps']['lift_enc'] =  SystemTimestamp().from_secs(self.robot.lift.status['timestamp_pc'])
            self.robot.status['timestamps']['arm_enc'] =  SystemTimestamp().from_secs(self.robot.arm.status['timestamp_pc'])
            self.robot.status['timestamps']['left_wheel_enc'] =  SystemTimestamp().from_secs(self.robot.base.status['timestamp_pc'])
            self.robot.status['timestamps']['right_wheel_enc'] =  SystemTimestamp().from_secs(self.robot.base.status['timestamp_pc'])
            self.robot.status['timestamps']['wacc_acc'] = SystemTimestamp().from_secs(self.robot.wacc.status['timestamp_pc'])
            self.robot.status['timestamps']['hw_sync'] =SystemTimestamp().from_wall_time() #Not meaningful but provide a nominal value
        else:
            ts_pimu_base =  self.robot.pimu.status['timestamp_line_sync'] #The time that the sync line was toggled (In HW usecs)
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



        #if self.param['time_align_status']:
        #    self.time_align_status()


    def time_align_status(self):
        if len(self.robot.status_history)>1:
            #Align sensor data to the time of the most recent Pimu line sync
            s2 = self.robot.status_history[-1][1] #Most recent
            s1 = self.robot.status_history[-2][1] #Prior
            t = s2['timestamps']['hw_sync']
            self.__align_sensor(s2,s1,'arm','pos','arm_enc',t)
            self.__align_sensor(s2, s1, 'arm', 'vel', 'arm_enc', t)
            self.__align_sensor(s2, s1, 'arm', 'force', 'arm_enc', t)

            self.__align_sensor(s2, s1, 'lift', 'pos', 'lift_enc', t)
            self.__align_sensor(s2, s1, 'lift', 'vel', 'lift_enc', t)
            self.__align_sensor(s2, s1, 'lift', 'force', 'lift_enc', t)

            self.__align_sensor(s2, s1, 'base', 'pos', 'left_wheel_enc', t,device_sub='left_wheel')
            self.__align_sensor(s2, s1, 'base', 'vel', 'left_wheel_enc', t, device_sub='left_wheel')
            self.__align_sensor(s2, s1, 'base', 'effort', 'left_wheel_enc', t, device_sub='left_wheel')

            self.__align_sensor(s2, s1, 'base', 'pos', 'right_wheel_enc', t, device_sub='right_wheel')
            self.__align_sensor(s2, s1, 'base', 'vel', 'right_wheel_enc', t, device_sub='right_wheel')
            self.__align_sensor(s2, s1, 'base', 'effort', 'right_wheel_enc', t, device_sub='right_wheel')

            self.__align_sensor(s2, s1, 'wacc', 'ax', 'wacc_acc', t)
            self.__align_sensor(s2, s1, 'wacc', 'ay', 'wacc_acc', t)
            self.__align_sensor(s2, s1, 'wacc', 'az', 'wacc_acc', t)

            self.__align_sensor(s2, s1, 'pimu', 'ax', 'pimu_imu', t,device_sub='imu')
            self.__align_sensor(s2, s1, 'pimu', 'ay', 'pimu_imu', t, device_sub='imu')
            self.__align_sensor(s2, s1, 'pimu', 'az', 'pimu_imu', t, device_sub='imu')

            self.__align_sensor(s2, s1, 'pimu', 'gx', 'pimu_imu', t, device_sub='imu')
            self.__align_sensor(s2, s1, 'pimu', 'gy', 'pimu_imu', t, device_sub='imu')
            self.__align_sensor(s2, s1, 'pimu', 'gz', 'pimu_imu', t, device_sub='imu')

            self.__align_sensor(s2, s1, 'pimu', 'mx', 'pimu_imu', t, device_sub='imu')
            self.__align_sensor(s2, s1, 'pimu', 'my', 'pimu_imu', t, device_sub='imu')
            self.__align_sensor(s2, s1, 'pimu', 'mz', 'pimu_imu', t, device_sub='imu')


            q2=Quaternion(w=s2['pimu']['imu']['qw'],x=s2['pimu']['imu']['qx'],y=s2['pimu']['imu']['qy'],z=s2['pimu']['imu']['qz'])
            q1 = Quaternion(w=s1['pimu']['imu']['qw'], x=s1['pimu']['imu']['qx'], y=s1['pimu']['imu']['qy'],z=s1['pimu']['imu']['qz'])
            t2 = s2['timestamps']['pimu_imu'].to_secs()
            t1 = s1['timestamps']['pimu_imu'].to_secs()
            if (t2-t1)>0:
                dt = (t.to_secs()-t1)/(t2-t1) #Amount to interpolate from t1 (prior)
                if dt>1.0: #extrapolate instead of interpolate (https://answers.unity.com/questions/168779/extrapolating-quaternion-rotation.html)
                    rot = q2 * q1.inverse
                    axis = rot.axis
                    ang = rot.degrees
                    if (ang > 180):
                        ang -= 360
                    ang = ang * dt % 360
                    qi = Quaternion(axis=axis, degrees=ang) * q1
                else: #interpolate
                    qi=Quaternion.slerp(q1,q2,dt)
                self.robot.status['pimu']['imu']['qw'] = qi.w
                self.robot.status['pimu']['imu']['qx'] = qi.x
                self.robot.status['pimu']['imu']['qy'] = qi.y
                self.robot.status['pimu']['imu']['qz'] = qi.z


    def __align_sensor(self,status,prior,device,sensor,ts_sensor,to_time,device_sub=None,verbose=False):
        if device_sub is None:
            y2 = status[device][sensor]
            y1 = prior[device][sensor]
        else:
            y2 = status[device][device_sub][sensor]
            y1 = prior[device][device_sub][sensor]
        t2 = status['timestamps'][ts_sensor].to_secs()
        t1 = prior['timestamps'][ts_sensor].to_secs()
        dt=t2-t1
        dy=y2-y1

        if dt>0:
            self.robot.status['timestamps'][ts_sensor] = to_time
            y = self.__interpolate(t1, t2, y1, y2, to_time.to_secs())
            if device_sub is None:
                self.robot.status[device][sensor] = y
            else:
                self.robot.status[device][device_sub][sensor]=y

            if verbose:
                print('----------------')
                print(device + ':' + str(device_sub) + ':' + sensor)
                print('DT: ' + str(dt))
                print('DY: ' + str(dy))
                print('T2: ' + str(t2) + ' T1: ' + str(t1))
                print('Y2: ' + str(y2) + ' Y1: ' + str(y1))
                print('To time: '+str(to_time))
                print('Inteporlated Y: '+str(y))



    def __interpolate(self,t1,t2,y1,y2,t):
        m=(y2-y1)/(t2-t1)
        y = m*(t - t1)+y1
        return y

