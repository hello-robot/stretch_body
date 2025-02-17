from __future__ import print_function
from stretch_body.transport import *
from stretch_body.device import Device
from stretch_body.hello_utils import *
import textwrap
import threading
import psutil
import time
import array as arr

# ######################## PIMU #################################

"""
The PIMU is the power and IMU Arduino board in the base
"""


class IMUBase(Device):
    """
    API to the Stretch IMU found in the base
    """
    def __init__(self):
        Device.__init__(self, 'imu',req_params=False)
        #pitch; //-180 to 180, rolls over
        #roll; //-90 to  90, rolls over at 180
        #heading; //0-360.0, rolls over
        self.status={'ax':0,'ay':0,'az':0,'gx':0,'gy':0,'gz':0,'mx':0,'my':0,'mz':0,'roll':0,'pitch':0,'heading':0,'timestamp':0,'qw':0,'qx':0,'qy':0,'qz':0,'bump':0}

    def get_status(self):
        s=self.status.copy()
        return s

    def get_quaternion(self):
        return [self.status['qw'],self.status['qx'],self.status['qy'],self.status['qz']]
    # ####################################################

    def pretty_print(self):
        print('----------IMU -------------')
        print('AX (m/s^2)', self.status['ax'])
        print('AY (m/s^2)', self.status['ay'])
        print('AZ (m/s^2)', self.status['az'])
        print('GX (rad/s)', self.status['gx'])
        print('GY (rad/s)', self.status['gy'])
        print('GZ (rad/s)', self.status['gz'])
        print('MX (uTesla)', self.status['mx'])
        print('MY (uTesla)', self.status['my'])
        print('MZ (uTesla)', self.status['mz'])
        print('QW', self.status['qw'])
        print('QX', self.status['qx'])
        print('QY', self.status['qy'])
        print('QZ', self.status['qz'])
        print('Roll (deg)', rad_to_deg(self.status['roll']))
        print('Pitch (deg)', rad_to_deg(self.status['pitch']))
        print('Heading (deg)', rad_to_deg(self.status['heading']))
        print('Bump', self.status['bump'])
        print('Timestamp (s)', self.status['timestamp'])
        print('-----------------------')

    def unpack_status(self, s, unpack_to=None):
        if unpack_to is None:
            unpack_to = self.status
        sidx = 0
        unpack_to['ax'] = unpack_float_t(s[sidx:]);sidx += 4
        unpack_to['ay'] = unpack_float_t(s[sidx:]);sidx += 4
        unpack_to['az'] = unpack_float_t(s[sidx:]);sidx += 4
        unpack_to['gx'] = unpack_float_t(s[sidx:]);sidx += 4
        unpack_to['gy'] = unpack_float_t(s[sidx:]);sidx += 4
        unpack_to['gz'] = unpack_float_t(s[sidx:]);sidx += 4
        unpack_to['mx'] = unpack_float_t(s[sidx:]);sidx += 4
        unpack_to['my'] = unpack_float_t(s[sidx:]);sidx += 4
        unpack_to['mz'] = unpack_float_t(s[sidx:]);sidx += 4
        unpack_to['roll'] = deg_to_rad(unpack_float_t(s[sidx:]));sidx += 4
        unpack_to['pitch'] = deg_to_rad(unpack_float_t(s[sidx:]));sidx += 4
        unpack_to['heading'] = deg_to_rad(unpack_float_t(s[sidx:]));sidx += 4
        unpack_to['qw'] = unpack_float_t(s[sidx:]);sidx += 4
        unpack_to['qx'] = unpack_float_t(s[sidx:]);sidx += 4
        unpack_to['qy'] = unpack_float_t(s[sidx:]);sidx += 4
        unpack_to['qz'] = unpack_float_t(s[sidx:]);sidx += 4
        unpack_to['bump'] = unpack_float_t(s[sidx:]);sidx += 4
        return sidx


# ######################## IMU PROTOCOL P0 #################################
class IMU_Protocol_P0(IMUBase):
    def unpack_status(self, s, unpack_to=None):
        if unpack_to is None:
            unpack_to = self.status
        sidx =  IMUBase.unpack_status(self, s, unpack_to)
        self.status['timestamp'] = self.timestamp.set(unpack_uint32_t(s[sidx:]));sidx += 4
        return sidx

# ######################## IMU PROTOCOL P1 #################################
class IMU_Protocol_P1(IMUBase):
    def unpack_status(self, s, unpack_to=None):
        if unpack_to is None:
            unpack_to = self.status
        sidx = IMUBase.unpack_status(self, s, unpack_to)
        return sidx

# ######################## IMU #################################
class IMU(IMUBase):
    def __init__(self):
        IMUBase.__init__(self)
        # Order in descending order so more recent protocols/methods override less recent
        self.supported_protocols = {'p0': (IMU_Protocol_P0,), 
                                    'p1': (IMU_Protocol_P1,IMU_Protocol_P0,),
                                    'p2': (IMU_Protocol_P1,IMU_Protocol_P0,),
                                    'p3': (IMU_Protocol_P1,IMU_Protocol_P0,),
                                    'p4': (IMU_Protocol_P1,IMU_Protocol_P0,),
                                    'p5': (IMU_Protocol_P1,IMU_Protocol_P0,),
                                    'p6': (IMU_Protocol_P1,IMU_Protocol_P0,)}

# ##################################################################################
class PimuBase(Device):
    """
    API to the Stretch Power and IMU board (Pimu)
    """
    RPC_SET_PIMU_CONFIG = 1
    RPC_REPLY_PIMU_CONFIG = 2
    RPC_GET_PIMU_STATUS = 3
    RPC_REPLY_PIMU_STATUS = 4
    RPC_SET_PIMU_TRIGGER = 5
    RPC_REPLY_PIMU_TRIGGER = 6
    RPC_GET_PIMU_BOARD_INFO = 7
    RPC_REPLY_PIMU_BOARD_INFO = 8
    RPC_SET_MOTOR_SYNC = 9
    RPC_REPLY_MOTOR_SYNC = 10
    RPC_READ_TRACE =11
    RPC_REPLY_READ_TRACE =12
    RPC_GET_PIMU_STATUS_AUX = 13
    RPC_REPLY_PIMU_STATUS_AUX = 14
    RPC_LOAD_TEST_PULL = 15
    RPC_REPLY_LOAD_TEST_PULL = 16
    RPC_LOAD_TEST_PUSH = 17
    RPC_REPLY_LOAD_TEST_PUSH = 18

    STATE_AT_CLIFF_0 = 1
    STATE_AT_CLIFF_1 = 2
    STATE_AT_CLIFF_2 = 4
    STATE_AT_CLIFF_3 = 8
    STATE_RUNSTOP_EVENT = 16
    STATE_CLIFF_EVENT = 32
    STATE_FAN_ON = 64
    STATE_BUZZER_ON = 128
    STATE_LOW_VOLTAGE_ALERT = 256
    STATE_OVER_TILT_ALERT = 512
    STATE_HIGH_CURRENT_ALERT = 1024
    STATE_CHARGER_CONNECTED = 2048
    STATE_BOOT_DETECTED = 4096
    STATE_IS_TRACE_ON = 8192
    STATE_IS_CHARGER_CHARGING = 16384

    TRIGGER_BOARD_RESET = 1
    TRIGGER_RUNSTOP_RESET = 2
    TRIGGER_CLIFF_EVENT_RESET = 4
    TRIGGER_BUZZER_ON = 8
    TRIGGER_BUZZER_OFF = 16
    TRIGGER_FAN_ON = 32
    TRIGGER_FAN_OFF = 64
    TRIGGER_IMU_RESET = 128
    TRIGGER_RUNSTOP_ON = 256
    TRIGGER_BEEP = 512
    TRIGGER_LIGHTBAR_TEST = 1024
    TRIGGER_ENABLE_TRACE= 2048
    TRIGGER_DISABLE_TRACE=4096


    TRACE_TYPE_STATUS = 0
    TRACE_TYPE_DEBUG = 1
    TRACE_TYPE_PRINT = 2


    def __init__(self, event_reset=False, usb=None):
        Device.__init__(self, 'pimu')
        self.config = self.params['config']
        self.imu = IMU()
        self._dirty_config = True
        self._dirty_trigger = False
        self.frame_id_last = None
        self.frame_id_base = 0
        if usb is None:
            usb = self.params['usb_name']
        self.transport = Transport(usb=usb, logger=self.logger)
        self.status = {'voltage': 0, 'current': 0, 'temp': 0,'cpu_temp': 0, 'cliff_range':[0,0,0,0], 'frame_id': 0,
                       'timestamp': 0,'at_cliff':[False,False,False,False], 'runstop_event': False, 'bump_event_cnt': 0,
                       'cliff_event': False, 'fan_on': False, 'buzzer_on': False, 'low_voltage_alert':False,'high_current_alert':False,'over_tilt_alert':False,
                       'charger_connected':False, 'boot_detected':False,'imu': self.imu.status,'debug':0,'state':0,'trace_on':0,
                       'motor_sync_rate': 0, 'motor_sync_cnt': 0, 'motor_sync_queues': 0, 'motor_sync_drop': 0,
                       'transport': self.transport.status, 'current_charge':0, 'charger_is_charging':False, 'over_tilt_type':0}

        self.status_zero=self.status.copy()
        self.status_aux = {'foo': 0}

        self._trigger=0
        self.ts_last_fan_on=None
        self.fan_on_last=False
        #Reset PIMU state so that Ctrl-C and re-instantiate Pimu class is efficient way to get out of an event
        if event_reset:
            self.runstop_event_reset()
            self.cliff_event_reset()

        self.board_info = {'board_variant': None, 'firmware_version': None, 'protocol_version': None,'hardware_id':0}
        self.hw_valid = False
        self.ts_last_motor_sync=None
        self.ts_last_motor_sync_warn=None
        self.load_test_payload = arr.array('B', range(256)) * 4
        self.ts_get_cpu_temp=0
        self.get_cpu_temp()

    # ###########  Device Methods #############

    def startup(self, threaded=False):
        try:
            Device.startup(self, threaded=threaded)
            self.hw_valid = self.transport.startup()
            if self.hw_valid:
                payload = arr.array('B', [self.RPC_GET_PIMU_BOARD_INFO])
                self.transport.do_pull_rpc_sync(payload, self.rpc_board_info_reply)
                self.transport.configure_version(self.board_info['firmware_version'])
                return True
            return False
        except KeyError:
            self.hw_valid =False
            return False


    def stop(self):
        Device.stop(self)
        if not self.hw_valid:
            return
        self.set_fan_off()
        self.push_command(exiting=True)
        self.transport.stop()
        self.hw_valid = False

    def set_config(self,c):
        self.config=c.copy()
        self._dirty_config = True

    def pull_status(self, exiting=False):
        if not self.hw_valid:
            return
        payload = arr.array('B', [self.RPC_GET_PIMU_STATUS])
        self.transport.do_pull_rpc_sync(payload, self.rpc_status_reply)

    async def pull_status_async(self, exiting=False):
        if not self.hw_valid:
            return
        payload = arr.array('B', [self.RPC_GET_PIMU_STATUS])
        await self.transport.do_pull_rpc_async(payload, self.rpc_status_reply, exiting=exiting)

    async def push_command_async(self, exiting=False):
        if not self.hw_valid:
            return
        payload = self.transport.get_empty_payload()
        if self._dirty_config:
            payload[0] = self.RPC_SET_PIMU_CONFIG
            sidx = self.pack_config(payload, 1)
            await self.transport.do_push_rpc_async(payload[:sidx], self.rpc_config_reply)
            self._dirty_config = False

        if self._dirty_trigger:
            payload[0] = self.RPC_SET_PIMU_TRIGGER
            sidx = self.pack_trigger(payload, 1)
            await self.transport.do_push_rpc_async(payload[:sidx], self.rpc_trigger_reply)
            self._trigger = 0
            self._dirty_trigger = False

    def push_command(self, exiting=False):
        if not self.hw_valid:
            return
        payload = self.transport.get_empty_payload()
        if self._dirty_config:
            payload[0] = self.RPC_SET_PIMU_CONFIG
            sidx = self.pack_config(payload, 1)
            self.transport.do_push_rpc_sync(payload[:sidx], self.rpc_config_reply)
            self._dirty_config = False

        if self._dirty_trigger:
            payload[0] = self.RPC_SET_PIMU_TRIGGER
            sidx = self.pack_trigger(payload, 1)
            self.transport.do_push_rpc_sync(payload[:sidx], self.rpc_trigger_reply)
            self._trigger = 0
            self._dirty_trigger = False

    def pretty_print(self):
        print('------ Pimu -----')
        print('Voltage',self.status['voltage'])
        print('Current', self.status['current'])
        if self.board_info['hardware_id']>=3:
            print('Current Charge',self.status['current_charge'])
        print('CPU Temp',self.status['cpu_temp'])
        print('Board Temp', self.status['temp'])
        print('State', self.status['state'])
        print('At Cliff', self.status['at_cliff'])
        print('Cliff Range', self.status['cliff_range'])
        print('Cliff Event', self.status['cliff_event'])
        print('Runstop Event', self.status['runstop_event'])
        print('Bump Event Cnt', self.status['bump_event_cnt'])
        print('Fan On', self.status['fan_on'])
        print('Buzzer On', self.status['buzzer_on'])
        print('Low Voltage Alert', self.status['low_voltage_alert'])
        print('High Current Alert', self.status['high_current_alert'])
        print('Over Tilt Alert',self.status['over_tilt_alert'])
        print('Trace on:', self.status['trace_on'])
        if self.board_info['hardware_id']>0:
            print('Charger Connected', self.status['charger_connected'])
            print('Boot Detected', self.status['boot_detected'])
        print('Debug', self.status['debug'])
        print('Timestamp (s)', self.status['timestamp'])
        #print('Read error', self.transport.status['read_error'])
        print('Queued motor sync', self.status['motor_sync_queues'])
        print('Motor sync rate', self.status['motor_sync_rate'])
        print('Motor sync cnt', self.status['motor_sync_cnt'])
        print('Board variant:',self.board_info['board_variant'])
        print('Firmware version:', self.board_info['firmware_version'])
        print('Transport version:', self.transport.version)
        self.imu.pretty_print()

    # ####################### User Functions #######################################################


    def runstop_event_reset(self):
        """
        Reset the robot runstop, allowing motion to continue
        """
        self._trigger=self._trigger | self.TRIGGER_RUNSTOP_RESET
        self._dirty_trigger=True

    def runstop_event_trigger(self):
        """
        Trigger the robot runstop, stopping motion
        """
        self._trigger=self._trigger | self.TRIGGER_RUNSTOP_ON
        self._dirty_trigger=True

    def trigger_beep(self):
        """
        Generate a single short beep
        """
        self._trigger=self._trigger | self.TRIGGER_BEEP
        self._dirty_trigger=True

    def trigger_lightbar_test(self):
        self._trigger = self._trigger | self.TRIGGER_LIGHTBAR_TEST
        self._dirty_trigger = True

    # ####################### Utility functions ####################################################
    def imu_reset(self):
        self._trigger=self._trigger | self.TRIGGER_IMU_RESET
        self._dirty_trigger=True


    def is_ready_for_sync(self):
        # For RE1.0 robots (hardware_id==0) the runstop and sync line are shared
        # This limits the maximum rate that the motor sync can be triggered
        # For RE2.0 robots the sync rate is limited by the min pulse width (~10ms)
        #Track the rate that the sync is triggered
        # This is called once prior to issuing stepper push_commands
        # If it is True then trigger_motor_sync is safe to call after the push_commands
        # By calling it first, it ensures that the sync rate is accurate (eg, not measuring timing of RPC, etc)
        t = time.time()
        if self.ts_last_motor_sync is not None:
            rate=1 / (t - self.ts_last_motor_sync)
            if rate>self.params['max_sync_rate_hz']:
                self.status['motor_sync_drop'] += 1
                if self.ts_last_motor_sync_warn is None or t-self.ts_last_motor_sync_warn>5.0:
                    print('Warning: Rate of calls to Pimu:trigger_motor_sync rate of %f above maximum frequency of %.2f Hz. Motor commands dropped: %d'%(rate,self.params['max_sync_rate_hz'],self.status['motor_sync_drop']))
                    self.ts_last_motor_sync_warn=t
                return False
            self.status['motor_sync_rate'] = rate #Only update rate if it is ready
        return True

    def trigger_motor_sync(self):
        #Push out immediately
        if not self.hw_valid:
            return
        payload = arr.array('B', [self.RPC_SET_MOTOR_SYNC])
        self.transport.do_push_rpc_sync(payload, self.rpc_motor_sync_reply)
        self.ts_last_motor_sync = time.time()


    def set_fan_on(self):
        self._trigger=self._trigger | self.TRIGGER_FAN_ON
        self._dirty_trigger=True

    def set_fan_off(self):
        self._trigger=self._trigger | self.TRIGGER_FAN_OFF
        self._dirty_trigger=True

    def set_buzzer_on(self):
        self._trigger=self._trigger | self.TRIGGER_BUZZER_ON
        self._dirty_trigger=True

    def set_buzzer_off(self):
        self._trigger=self._trigger | self.TRIGGER_BUZZER_OFF
        self._dirty_trigger=True

    def board_reset(self):
        self._trigger=self._trigger | self.TRIGGER_BOARD_RESET
        self._dirty_trigger=True

    def cliff_event_reset(self):
        self._trigger=self._trigger | self.TRIGGER_CLIFF_EVENT_RESET
        self._dirty_trigger=True

    # ########### Sensor Calibration #################
    def get_voltage(self,raw):
        raw_to_V = 20.0/1024 #10bit adc, 0-20V per 0-3.3V reading
        return raw*raw_to_V


    def get_temp(self,raw):
        raw_to_mV = 3300/1024.0
        mV = raw*raw_to_mV - 400 #400mV at 0C per spec
        C = mV/19.5 #19.5mV per C per spec
        return C

    def get_current(self,raw):
        if self.board_info['hardware_id']>=3:
            return self.get_current_efuse(raw)
        else:
            return self.get_current_shunt(raw)
    def get_current_shunt(self,raw):
        """
        RE1 / RE2 Pimus using shunt resistor for current measurement
        """
        raw_to_mV = 3300 / 1024.0
        mV = raw * raw_to_mV
        mA = mV/.408 # conversion per circuit
        return mA/1000.0

    def get_current_efuse(self,raw):
        """
        S3 Pimu's using Efuse current measurement
        """
        raw_to_mV = 3300 / 1024.0
        mV = raw * raw_to_mV
        mA = mV/.224 # conversion per circuit
        return mA/1000.0

    def get_current_charge(self,raw):
        """
        S3 Pimu's shunt measurement of charger current
        """
        raw_to_mV = 3300 / 1024.0
        mV = raw * raw_to_mV
        mA = mV/.215 # conversion per circuit
        return mA/1000.0
    
    def get_tilt_type(self, raw):
        tilt_type = {
            1: 'Left Tilt',
            2: 'Right Tilt',
            3: 'Front Tilt'
        }
        return tilt_type.get(raw, None)
    # ################Data Packing #####################

    def unpack_board_info(self,s):
        sidx=0
        self.board_info['board_variant'] = unpack_string_t(s[sidx:], 20)
        self.board_info['hardware_id'] = 0
        if len(self.board_info['board_variant'])==6: #New format of Pimu.x Older format of Pimu.BoardName.Vx' If older format,default to 0
            self.board_info['hardware_id']=int(self.board_info['board_variant'][-1])
        sidx += 20
        self.board_info['firmware_version'] = unpack_string_t(s[sidx:], 20)
        self.board_info['protocol_version'] = self.board_info['firmware_version'][self.board_info['firmware_version'].rfind('p'):]
        sidx += 20
        return sidx

    def pack_config(self,s,sidx):
        for i in range(4):
            pack_float_t(s, sidx, self.config['cliff_zero'][i])
            sidx += 4
        pack_float_t(s, sidx, self.config['cliff_thresh']);sidx += 4
        pack_float_t(s, sidx, self.config['cliff_LPF']);sidx += 4
        pack_float_t(s, sidx, self.config['voltage_LPF']);sidx += 4
        pack_float_t(s, sidx, self.config['current_LPF']);sidx += 4
        pack_float_t(s, sidx, self.config['temp_LPF']);sidx += 4
        pack_uint8_t(s, sidx, self.config['stop_at_cliff']); sidx += 1
        pack_uint8_t(s, sidx, self.config['stop_at_runstop']);sidx += 1
        pack_uint8_t(s, sidx, self.config['stop_at_tilt']);sidx += 1
        pack_uint8_t(s, sidx, self.config['stop_at_low_voltage']);sidx += 1
        pack_uint8_t(s, sidx, self.config['stop_at_high_current']); sidx += 1
        for i in range(3):
            pack_float_t(s, sidx, self.config['mag_offsets'][i])
            sidx += 4
        for i in range(9):
            pack_float_t(s, sidx, self.config['mag_softiron_matrix'][i])
            sidx += 4
        for i in range(3):
            pack_float_t(s, sidx, self.config['gyro_zero_offsets'][i])
            sidx += 4
        pack_float_t(s, sidx, self.config['rate_gyro_vector_scale']); sidx += 4
        pack_float_t(s, sidx, self.config['gravity_vector_scale']); sidx += 4
        pack_float_t(s, sidx, self.config['accel_LPF']); sidx += 4
        pack_float_t(s, sidx, self.config['bump_thresh']); sidx += 4
        pack_float_t(s, sidx, self.config['low_voltage_alert']);sidx += 4
        pack_float_t(s, sidx, self.config['high_current_alert']);sidx += 4
        pack_float_t(s, sidx, self.config['over_tilt_alert']); sidx += 4
        return sidx

    def pack_trigger(self,s,sidx):
        pack_uint32_t(s,sidx,self._trigger); sidx+=4
        return sidx

    def unpack_status(self,s, unpack_to=None):
        raise NotImplementedError('This method not supported for firmware on protocol {0}.'
            .format(self.board_info['protocol_version']))

    def read_firmware_trace(self):
        raise NotImplementedError('This method not supported for firmware on protocol {0}.'
                                  .format(self.board_info['protocol_version']))

    def rpc_read_firmware_trace_reply(self, reply):
        raise NotImplementedError('This method not supported for firmware on protocol {0}.'
                                  .format(self.board_info['protocol_version']))

    def enable_firmware_trace(self):
        raise NotImplementedError('This method not supported for firmware on protocol {0}.'
                                  .format(self.board_info['protocol_version']))

    def disable_firmware_trace(self):
        raise NotImplementedError('This method not supported for firmware on protocol {0}.'
                                  .format(self.board_info['protocol_version']))


    def pull_status_aux(self):
        raise NotImplementedError('This method not supported for firmware on protocol {0}.'
            .format(self.board_info['protocol_version']))


    def push_load_test(self):
        raise NotImplementedError('This method not supported for firmware on protocol {0}.'
                                  .format(self.board_info['protocol_version']))

    def pull_load_test(self):
        raise NotImplementedError('This method not supported for firmware on protocol {0}.'
                                  .format(self.board_info['protocol_version']))

        # ################Transport Callbacks #####################

    def rpc_motor_sync_reply(self,reply):
        if reply[0] != self.RPC_REPLY_MOTOR_SYNC:
            self.logger.warning('Error RPC_REPLY_MOTOR_SYNC', reply[0])

    def rpc_config_reply(self,reply):
        if reply[0] != self.RPC_REPLY_PIMU_CONFIG:
            self.logger.warning('Error RPC_REPLY_PIMU_CONFIG', reply[0])

    def rpc_board_info_reply(self,reply):
        if reply[0] == self.RPC_REPLY_PIMU_BOARD_INFO:
            self.unpack_board_info(reply[1:])
        else:
            self.logger.warning('Error RPC_REPLY_PIMU_BOARD_INFO', reply[0])

    def rpc_trigger_reply(self,reply):
        if reply[0] != self.RPC_REPLY_PIMU_TRIGGER:
            self.logger.warning('Error RPC_REPLY_PIMU_TRIGGER', reply[0])
        else:
            tt=unpack_uint32_t(reply[1:])

    def rpc_status_reply(self,reply):
        if reply[0] == self.RPC_REPLY_PIMU_STATUS:
            self.unpack_status(reply[1:])
        else:
            self.logger.warning('Error RPC_REPLY_PIMU_STATUS', reply[0])


    # ################ Sentry #####################
    def get_cpu_temp(self):
        #Note, this call can be slow
        cpu_temp = 0
        try:
            t = psutil.sensors_temperatures()['coretemp']
            for c in t:
                cpu_temp = max(cpu_temp, c.current)
        except(KeyError, IOError): #May not be available on virtual machines
            cpu_temp=25.0
        return cpu_temp

    def step_sentry(self,robot=None):
        if self.hw_valid and self.robot_params['robot_sentry']['base_fan_control']:
            #Manage CPU temp using the mobile base fan
            #See https://www.intel.com/content/www/us/en/support/articles/000005946/intel-nuc.html

            if time.time()-self.ts_get_cpu_temp>10.0: #Once every 10s as is slow changing and is a slow sys call
                self.status['cpu_temp']=self.get_cpu_temp()
                self.ts_get_cpu_temp=time.time()
            if self.status['cpu_temp']>self.params['base_fan_on']:
                if self.ts_last_fan_on is None or time.time()-self.ts_last_fan_on>3.0: #Will turn itself off if don't refresh command
                    self.set_fan_on()
                    self.push_command()
                    self.ts_last_fan_on = time.time()
                if  not self.status['fan_on']:
                    self.logger.debug('Base fan turned on')

            if self.fan_on_last and not self.status['fan_on']:
                self.logger.debug('Base fan turned off')

            if self.status['cpu_temp']<self.params['base_fan_off']and self.status['fan_on']:
                self.set_fan_off()
                self.push_command()
            self.fan_on_last = self.status['fan_on']



# ######################## PIMU PROTOCOL PO #################################


class Pimu_Protocol_P0(PimuBase):
    def unpack_status(self,s, unpack_to=None):
        if unpack_to is None:
            unpack_to = self.status
        sidx=0
        sidx +=self.imu.unpack_status((s[sidx:]))
        unpack_to['voltage']=self.get_voltage(unpack_float_t(s[sidx:]));sidx+=4
        unpack_to['current'] = self.get_current(unpack_float_t(s[sidx:]));sidx+=4
        unpack_to['temp'] = self.get_temp(unpack_float_t(s[sidx:]));sidx+=4

        for i in range(4):
            unpack_to['cliff_range'][i]=unpack_float_t(s[sidx:])
            sidx+=4

        unpack_to['state'] = unpack_uint32_t(s[sidx:])
        sidx += 4

        unpack_to['at_cliff']=[]
        unpack_to['at_cliff'].append((unpack_to['state'] & self.STATE_AT_CLIFF_0) != 0)
        unpack_to['at_cliff'].append((unpack_to['state'] & self.STATE_AT_CLIFF_1) != 0)
        unpack_to['at_cliff'].append((unpack_to['state'] & self.STATE_AT_CLIFF_2) != 0)
        unpack_to['at_cliff'].append((unpack_to['state'] & self.STATE_AT_CLIFF_3) != 0)
        unpack_to['runstop_event'] = (unpack_to['state'] & self.STATE_RUNSTOP_EVENT) != 0
        unpack_to['cliff_event'] = (unpack_to['state'] & self.STATE_CLIFF_EVENT) != 0
        unpack_to['fan_on'] = (unpack_to['state'] & self.STATE_FAN_ON) != 0
        unpack_to['buzzer_on'] = (unpack_to['state'] & self.STATE_BUZZER_ON) != 0
        unpack_to['low_voltage_alert'] = (unpack_to['state'] & self.STATE_LOW_VOLTAGE_ALERT) != 0
        unpack_to['high_current_alert'] = (unpack_to['state'] & self.STATE_HIGH_CURRENT_ALERT) != 0
        unpack_to['over_tilt_alert'] = (unpack_to['state'] & self.STATE_OVER_TILT_ALERT) != 0
        unpack_to['charger_connected'] = (unpack_to['state'] & self.STATE_CHARGER_CONNECTED) != 0
        unpack_to['boot_detected'] = (unpack_to['state'] & self.STATE_BOOT_DETECTED) != 0
        unpack_to['timestamp'] = self.timestamp.set(unpack_uint32_t(s[sidx:])); sidx += 4
        unpack_to['bump_event_cnt'] = unpack_uint16_t(s[sidx:]);sidx += 2
        unpack_to['debug'] = unpack_float_t(s[sidx:]); sidx += 4
        return sidx

# ######################## PIMU PROTOCOL P1 #################################
class Pimu_Protocol_P1(PimuBase):
    def unpack_status(self, s, unpack_to=None):
        if unpack_to is None:
            unpack_to = self.status

        sidx=0
        sidx +=self.imu.unpack_status((s[sidx:]))
        unpack_to['voltage']=self.get_voltage(unpack_float_t(s[sidx:]));sidx+=4
        unpack_to['current'] = self.get_current(unpack_float_t(s[sidx:]));sidx+=4
        unpack_to['temp'] = self.get_temp(unpack_float_t(s[sidx:]));sidx+=4

        for i in range(4):
            unpack_to['cliff_range'][i]=unpack_float_t(s[sidx:])
            sidx+=4

        unpack_to['state'] = unpack_uint32_t(s[sidx:])
        sidx += 4
        unpack_to['at_cliff']=[]
        unpack_to['at_cliff'].append((unpack_to['state'] & PimuBase.STATE_AT_CLIFF_0) != 0)
        unpack_to['at_cliff'].append((unpack_to['state'] & self.STATE_AT_CLIFF_1) != 0)
        unpack_to['at_cliff'].append((unpack_to['state'] & self.STATE_AT_CLIFF_2) != 0)
        unpack_to['at_cliff'].append((unpack_to['state'] & self.STATE_AT_CLIFF_3) != 0)
        unpack_to['runstop_event'] = (unpack_to['state'] & self.STATE_RUNSTOP_EVENT) != 0
        unpack_to['cliff_event'] = (unpack_to['state'] & self.STATE_CLIFF_EVENT) != 0
        unpack_to['fan_on'] = (unpack_to['state'] & self.STATE_FAN_ON) != 0
        unpack_to['buzzer_on'] = (unpack_to['state'] & self.STATE_BUZZER_ON) != 0
        unpack_to['low_voltage_alert'] = (unpack_to['state'] & self.STATE_LOW_VOLTAGE_ALERT) != 0
        unpack_to['high_current_alert'] = (unpack_to['state'] & self.STATE_HIGH_CURRENT_ALERT) != 0
        unpack_to['over_tilt_alert'] = (unpack_to['state'] & self.STATE_OVER_TILT_ALERT) != 0
        if self.board_info['hardware_id']>0:
            unpack_to['charger_connected'] = (unpack_to['state'] & self.STATE_CHARGER_CONNECTED) != 0
            unpack_to['boot_detected'] = (unpack_to['state'] & self.STATE_BOOT_DETECTED) != 0
        unpack_to['timestamp'] = self.timestamp.set(unpack_uint64_t(s[sidx:])); sidx += 8
        self.imu.status['timestamp'] = unpack_to['timestamp']
        unpack_to['bump_event_cnt'] = unpack_uint16_t(s[sidx:]);sidx += 2
        unpack_to['debug'] = unpack_float_t(s[sidx:]); sidx += 4
        return sidx




# ######################## PIMU PROTOCOL P2 #################################
class Pimu_Protocol_P2(PimuBase):

    def read_firmware_trace(self):
        self.trace_buf = []
        self.timestamp.reset() #Timestamp holds state, reset within lock to avoid threading issues
        self.n_trace_read=1
        ts=time.time()
        while ( self.n_trace_read) and time.time()-ts<60.0:
            payload = arr.array('B', [self.RPC_READ_TRACE])
            self.transport.do_pull_rpc_sync(payload, self.rpc_read_firmware_trace_reply)
            time.sleep(.001)
        return self.trace_buf

    def unpack_debug_trace(self,s,unpack_to):
        sidx=0
        unpack_to['u8_1']=unpack_uint8_t(s[sidx:]);sidx+=1
        unpack_to['u8_2'] = unpack_uint8_t(s[sidx:]);sidx += 1
        unpack_to['f_1'] = unpack_float_t(s[sidx:]);sidx += 4
        unpack_to['f_2'] = unpack_float_t(s[sidx:]);sidx += 4
        unpack_to['f_3'] = unpack_float_t(s[sidx:]);sidx += 4
        return sidx

    def unpack_print_trace(self,s,unpack_to):
        sidx=0
        line_len=32
        unpack_to['timestamp']=self.timestamp.set(unpack_uint64_t(s[sidx:]));sidx += 8
        unpack_to['line'] = unpack_string_t(s[sidx:], line_len); sidx += line_len
        unpack_to['x'] = unpack_float_t(s[sidx:]);sidx += 4
        return sidx

    def rpc_read_firmware_trace_reply(self, reply):
        if len(reply)>0 and reply[0] == self.RPC_REPLY_READ_TRACE:
            self.n_trace_read=reply[1]
            self.trace_buf.append({'id': len(self.trace_buf), 'status': {},'debug':{},'print':{}})
            if reply[2]==self.TRACE_TYPE_STATUS:
                self.trace_buf[-1]['status']= self.status_zero.copy()
                self.unpack_status(reply[3:],unpack_to=self.trace_buf[-1]['status'])
            elif reply[2]==self.TRACE_TYPE_DEBUG:
                self.unpack_debug_trace(reply[3:],unpack_to=self.trace_buf[-1]['debug'])
            elif reply[2]==self.TRACE_TYPE_PRINT:
                self.unpack_print_trace(reply[3:],unpack_to=self.trace_buf[-1]['print'])
            else:
                print('Unrecognized trace type %d'%reply[2])
        else:
            print('Error RPC_REPLY_READ_TRACE')
            self.n_trace_read=0
            self.trace_buf = []
    def enable_firmware_trace(self):
        self._trigger = self._trigger | self.TRIGGER_ENABLE_TRACE
        self._dirty_trigger = True

    def disable_firmware_trace(self):
        self._trigger = self._trigger | self.TRIGGER_DISABLE_TRACE
        self._dirty_trigger = True

    def unpack_status(self, s, unpack_to=None): #P2
        if unpack_to is None:
            unpack_to = self.status

        sidx=0
        sidx +=self.imu.unpack_status((s[sidx:]))
        unpack_to['voltage']=self.get_voltage(unpack_float_t(s[sidx:]));sidx+=4
        unpack_to['current'] = self.get_current(unpack_float_t(s[sidx:]));sidx+=4
        unpack_to['temp'] = self.get_temp(unpack_float_t(s[sidx:]));sidx+=4

        for i in range(4):
            unpack_to['cliff_range'][i]=unpack_float_t(s[sidx:])
            sidx+=4

        unpack_to['state'] = unpack_uint32_t(s[sidx:])
        sidx += 4

        unpack_to['at_cliff']=[]
        unpack_to['at_cliff'].append((unpack_to['state'] & PimuBase.STATE_AT_CLIFF_0) != 0)
        unpack_to['at_cliff'].append((unpack_to['state'] & self.STATE_AT_CLIFF_1) != 0)
        unpack_to['at_cliff'].append((unpack_to['state'] & self.STATE_AT_CLIFF_2) != 0)
        unpack_to['at_cliff'].append((unpack_to['state'] & self.STATE_AT_CLIFF_3) != 0)
        unpack_to['runstop_event'] = (unpack_to['state'] & self.STATE_RUNSTOP_EVENT) != 0
        unpack_to['cliff_event'] = (unpack_to['state'] & self.STATE_CLIFF_EVENT) != 0
        unpack_to['fan_on'] = (unpack_to['state'] & self.STATE_FAN_ON) != 0
        unpack_to['buzzer_on'] = (unpack_to['state'] & self.STATE_BUZZER_ON) != 0
        unpack_to['low_voltage_alert'] = (unpack_to['state'] & self.STATE_LOW_VOLTAGE_ALERT) != 0
        unpack_to['high_current_alert'] = (unpack_to['state'] & self.STATE_HIGH_CURRENT_ALERT) != 0
        unpack_to['over_tilt_alert'] = (unpack_to['state'] & self.STATE_OVER_TILT_ALERT) != 0
        if self.board_info['hardware_id']>0:
            unpack_to['charger_connected'] = (unpack_to['state'] & self.STATE_CHARGER_CONNECTED) != 0
            unpack_to['boot_detected'] = (unpack_to['state'] & self.STATE_BOOT_DETECTED) != 0
        unpack_to['trace_on'] = (unpack_to['state'] & self.STATE_IS_TRACE_ON) != 0
        unpack_to['timestamp'] = self.timestamp.set(unpack_uint64_t(s[sidx:])); sidx += 8
        self.imu.status['timestamp'] = unpack_to['timestamp']
        unpack_to['bump_event_cnt'] = unpack_uint16_t(s[sidx:]);sidx += 2
        unpack_to['debug'] = unpack_float_t(s[sidx:]); sidx += 4
        return sidx

# ######################## PIMU PROTOCOL P3 #################################
class Pimu_Protocol_P3(PimuBase):


    def is_ready_for_sync(self):
        """
        With P3 this function is no longer meaningful as sync overruns are handled on the fw level
        """
        return True
    def trigger_motor_sync(self):
        # Push out immediately
        if not self.hw_valid:
            return
        payload = arr.array('B', [self.RPC_SET_MOTOR_SYNC])
        old_sync_cnt = self.status['motor_sync_cnt']
        self.transport.do_push_rpc_sync(payload, self.rpc_motor_sync_reply)

        t=time.time()
        # Should motor_sync_cnt should increment with each call to trigger_motor_sync, if not it is an overrun
        if self.status['motor_sync_cnt'] == old_sync_cnt:
            self.status['motor_sync_queues']=self.status['motor_sync_queues']+1
            print('Warning: Queued motor_sync as trigger_motor_sync calls above maximum rate. Overruns: %d' % (self.status['motor_sync_queues']))
            self.ts_last_motor_sync_warn = t

        if self.ts_last_motor_sync is not None:
            self.status['motor_sync_rate']=1 / (t - self.ts_last_motor_sync)
        self.ts_last_motor_sync = t

    def rpc_motor_sync_reply(self,reply):
        if reply[0] != self.RPC_REPLY_MOTOR_SYNC:
            self.logger.warning('Error RPC_REPLY_MOTOR_SYNC', reply[0])
        else:
            self.unpack_motor_sync_reply(reply[1:])


    def unpack_motor_sync_reply(self, s):
        # take in an array of bytes
        # this needs to exactly match the C struct format
        sidx=0
        self.status['motor_sync_cnt']=  unpack_int16_t(s[sidx:])
        sidx += 2
        return sidx

    def pull_status_aux(self):
        if not self.hw_valid:
            return
        payload = arr.array('B', [self.RPC_GET_PIMU_STATUS_AUX])
        self.transport.do_pull_rpc_sync(payload, self.rpc_status_aux_reply)


    def rpc_status_aux_reply(self,reply):
        if reply[0] == self.RPC_REPLY_PIMU_STATUS_AUX:
            self.unpack_status_aux(reply[1:])
        else:
            self.logger.warning('Error RPC_REPLY_PIMU_AUX_STATUS', reply[0])

    def unpack_status_aux(self, s):
        # take in an array of bytes
        # this needs to exactly match the C struct format
        sidx=0
        self.status_aux['foo']=  unpack_int16_t(s[sidx:]);sidx += 2
        return sidx

    def push_load_test(self):
        if not self.hw_valid:
            return
        payload=self.transport.get_empty_payload()
        payload[0] = self.RPC_LOAD_TEST_PUSH
        payload[1:] = self.load_test_payload
        self.transport.do_push_rpc_sync(payload, self.rpc_load_test_push_reply)

    def pull_load_test(self):
        if not self.hw_valid:
            return
        payload = arr.array('B',[self.RPC_LOAD_TEST_PULL])
        self.transport.do_pull_rpc_sync(payload, self.rpc_load_test_pull_reply)


    def rpc_load_test_push_reply(self, reply):
        if reply[0] != self.RPC_REPLY_LOAD_TEST_PUSH:
            print('Error RPC_REPLY_LOAD_TEST_PUSH', reply[0])

    def rpc_load_test_pull_reply(self, reply):
        if reply[0] == self.RPC_REPLY_LOAD_TEST_PULL:
            d = reply[1:]
            for i in range(1024):
                if d[i] != self.load_test_payload[(i + 1) % 1024]:
                    print('Load test pull bad data', d[i], self.load_test_payload[(i + 1) % 1024])
            self.load_test_payload = d
            print('Successful load test pull')
        else:
            print('Error RPC_REPLY_LOAD_TEST_PULL', reply[0])

# ######################## PIMU PROTOCOL P4 #################################
class Pimu_Protocol_P4(PimuBase):
    def unpack_status(self, s, unpack_to=None):  # P4
        if unpack_to is None:
            unpack_to = self.status
        sidx = 0
        sidx = sidx + Pimu_Protocol_P2.unpack_status(self, s, unpack_to)
        self.status['current_charge'] = self.get_current_charge(unpack_float_t(s[sidx:]));sidx += 4
        return sidx

# ######################## PIMU PROTOCOL P5 #################################
class Pimu_Protocol_P5(PimuBase):
    def unpack_status(self, s, unpack_to=None):  # P5
        if unpack_to is None:
            unpack_to = self.status
        sidx = 0
        sidx = sidx + Pimu_Protocol_P4.unpack_status(self, s, unpack_to)
        unpack_to['charger_is_charging'] = (unpack_to['state'] & self.STATE_IS_CHARGER_CHARGING) != 0
        return sidx

# ######################## PIMU PROTOCOL P6 #################################
class Pimu_Protocol_P6(PimuBase):
    def unpack_status(self, s, unpack_to=None):  # P6
        if unpack_to is None:
            unpack_to = self.status
        sidx = 0
        sidx = sidx + Pimu_Protocol_P5.unpack_status(self, s, unpack_to)
        unpack_to['over_tilt_type'] = self.get_tilt_type(unpack_uint8_t(s[sidx:]));sidx += 1
        return sidx
# ######################## PIMU #################################
class Pimu(PimuBase):
    """
    API to the Stretch Power and IMU board (Pimu)
    """
    def __init__(self, event_reset=False, usb=None):
        PimuBase.__init__(self, event_reset,usb)
        # Order in descending order so more recent protocols/methods override less recent
        self.supported_protocols = {'p0': (Pimu_Protocol_P0,),
                                    'p1': (Pimu_Protocol_P1, Pimu_Protocol_P0,),
                                    'p2': (Pimu_Protocol_P2, Pimu_Protocol_P1, Pimu_Protocol_P0,),
                                    'p3': (Pimu_Protocol_P3, Pimu_Protocol_P2, Pimu_Protocol_P1, Pimu_Protocol_P0,),
                                    'p4': (Pimu_Protocol_P4, Pimu_Protocol_P3, Pimu_Protocol_P2, Pimu_Protocol_P1, Pimu_Protocol_P0,),
                                    'p5': (Pimu_Protocol_P5, Pimu_Protocol_P4, Pimu_Protocol_P3, Pimu_Protocol_P2, Pimu_Protocol_P1, Pimu_Protocol_P0,),
                                    'p6': (Pimu_Protocol_P6, Pimu_Protocol_P5, Pimu_Protocol_P4, Pimu_Protocol_P3, Pimu_Protocol_P2, Pimu_Protocol_P1, Pimu_Protocol_P0,)
                                    }

    def startup(self, threaded=False):
        """
        First determine which protocol version the uC firmware is running.
        Based on that version, replaces PimuBase class inheritance with a inheritance to a child class of PimuBase that supports that protocol
        """
        PimuBase.startup(self, threaded=threaded)
        if self.hw_valid:
            if self.board_info['protocol_version'] in self.supported_protocols:
                Pimu.__bases__ = self.supported_protocols[self.board_info['protocol_version']]
                IMU.__bases__= self.imu.supported_protocols[self.board_info['protocol_version']]
            else:
                if self.board_info['protocol_version'] is None:
                    protocol_msg = """
                                    ----------------
                                    Failure in communications for {0} on startup.
                                    Please power cycle the robot and try again.
                                    ----------------
                                    """.format(self.name)
                else:
                    protocol_msg = """
                    ----------------
                    Firmware protocol mismatch on {0}.
                    Protocol on board is {1}.
                    Valid protocols are: {2}.
                    Disabling device.
                    Please upgrade the firmware and/or version of Stretch Body.
                    ----------------
                    """.format(self.name, self.board_info['protocol_version'], self.supported_protocols.keys())
                self.logger.warning(textwrap.dedent(protocol_msg))
                self.hw_valid = False
                self.transport.stop()
        if self.hw_valid:
            self.push_command()
            self.pull_status()
        return self.hw_valid