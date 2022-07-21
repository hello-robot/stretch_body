from __future__ import print_function
from stretch_body.transport import *
from stretch_body.device import Device
from stretch_body.hello_utils import *
import textwrap
import threading
import psutil
import time


# ######################## PIMU #################################

"""
The PIMU is the power and IMU Arduino board in the base
"""


class IMUBase(Device):
    """
    API to the Stretch RE1 IMU found in the base
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

    def unpack_status(self, s):
        raise NotImplementedError('This method not supported for firmware on protocol {0}.'
            .format(self.board_info['protocol_version']))

# ######################## IMU PROTOCOL P0 #################################
class IMU_Protocol_P0(IMUBase):
    def unpack_status(self, s):
        # take in an array of bytes
        # this needs to exactly match the C struct format
        sidx=0
        self.status['ax']=  unpack_float_t(s[sidx:]);sidx += 4
        self.status['ay'] = unpack_float_t(s[sidx:]);sidx += 4
        self.status['az'] = unpack_float_t(s[sidx:]);sidx += 4
        self.status['gx'] = unpack_float_t(s[sidx:]);sidx += 4
        self.status['gy'] = unpack_float_t(s[sidx:]);sidx += 4
        self.status['gz'] = unpack_float_t(s[sidx:]);sidx += 4
        self.status['mx'] = unpack_float_t(s[sidx:]);sidx += 4
        self.status['my'] = unpack_float_t(s[sidx:]);sidx += 4
        self.status['mz'] = unpack_float_t(s[sidx:]);sidx += 4
        self.status['roll'] = deg_to_rad(unpack_float_t(s[sidx:]));sidx += 4
        self.status['pitch'] = deg_to_rad(unpack_float_t(s[sidx:]));sidx += 4
        self.status['heading'] = deg_to_rad(unpack_float_t(s[sidx:]));sidx += 4
        self.status['qw'] = unpack_float_t(s[sidx:]);sidx += 4
        self.status['qx'] = unpack_float_t(s[sidx:]);sidx += 4
        self.status['qy'] = unpack_float_t(s[sidx:]);sidx += 4
        self.status['qz'] = unpack_float_t(s[sidx:]);sidx += 4
        self.status['bump'] = unpack_float_t(s[sidx:]);sidx += 4
        self.status['timestamp'] = self.timestamp.set(unpack_uint32_t(s[sidx:]));sidx += 4
        return sidx

# ######################## IMU PROTOCOL P1 #################################
class IMU_Protocol_P1(IMUBase):
    def unpack_status(self, s):
        # take in an array of bytes
        # this needs to exactly match the C struct format
        sidx=0
        self.status['ax']=  unpack_float_t(s[sidx:]);sidx += 4
        self.status['ay'] = unpack_float_t(s[sidx:]);sidx += 4
        self.status['az'] = unpack_float_t(s[sidx:]);sidx += 4
        self.status['gx'] = unpack_float_t(s[sidx:]);sidx += 4
        self.status['gy'] = unpack_float_t(s[sidx:]);sidx += 4
        self.status['gz'] = unpack_float_t(s[sidx:]);sidx += 4
        self.status['mx'] = unpack_float_t(s[sidx:]);sidx += 4
        self.status['my'] = unpack_float_t(s[sidx:]);sidx += 4
        self.status['mz'] = unpack_float_t(s[sidx:]);sidx += 4
        self.status['roll'] = deg_to_rad(unpack_float_t(s[sidx:]));sidx += 4
        self.status['pitch'] = deg_to_rad(unpack_float_t(s[sidx:]));sidx += 4
        self.status['heading'] = deg_to_rad(unpack_float_t(s[sidx:]));sidx += 4
        self.status['qw'] = unpack_float_t(s[sidx:]);sidx += 4
        self.status['qx'] = unpack_float_t(s[sidx:]);sidx += 4
        self.status['qy'] = unpack_float_t(s[sidx:]);sidx += 4
        self.status['qz'] = unpack_float_t(s[sidx:]);sidx += 4
        self.status['bump'] = unpack_float_t(s[sidx:]);sidx += 4
        return sidx

# ######################## IMU #################################
class IMU(IMUBase):
    def __init__(self):
        IMUBase.__init__(self)
        # Order in descending order so more recent protocols/methods override less recent
        self.supported_protocols = {'p0': (IMU_Protocol_P0,), 'p1': (IMU_Protocol_P1,IMU_Protocol_P0,)}

# ##################################################################################
class PimuBase(Device):
    """
    API to the Stretch RE1 Power and IMU board (Pimu)
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


    def __init__(self, event_reset=False):
        Device.__init__(self, 'pimu')
        self.config = self.params['config']
        self.lock = threading.RLock()
        self.imu = IMU()
        self._dirty_config = True
        self._dirty_trigger = False
        self.frame_id_last = None
        self.frame_id_base = 0
        self.transport = Transport(usb='/dev/hello-pimu', logger=self.logger)
        self.status = {'voltage': 0, 'current': 0, 'temp': 0,'cpu_temp': 0, 'cliff_range':[0,0,0,0], 'frame_id': 0,
                       'timestamp': 0,'at_cliff':[False,False,False,False], 'runstop_event': False, 'bump_event_cnt': 0,
                       'cliff_event': False, 'fan_on': False, 'buzzer_on': False, 'low_voltage_alert':False,'high_current_alert':False,'over_tilt_alert':False,
                       'charger_connected':False, 'boot_detected':False,'imu': self.imu.status,'debug':0,'state':0,'motor_sync_drop':0,
                       'transport': self.transport.status}
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


    # ###########  Device Methods #############

    def startup(self, threaded=False):
        try:
            Device.startup(self, threaded=threaded)
            with self.lock:
                self.hw_valid = self.transport.startup()
                if self.hw_valid:
                    # Pull board info
                    self.transport.payload_out[0] = self.RPC_GET_PIMU_BOARD_INFO
                    self.transport.queue_rpc(1, self.rpc_board_info_reply)
                    self.transport.step(exiting=False)
                    return True
                return False
        except KeyError:
            self.hw_valid =False
            return False



    def stop(self):
        Device.stop(self)
        if not self.hw_valid:
            return
        with self.lock:
            self.set_fan_off()
            self.push_command(exiting=True)
            self.transport.stop()
            self.hw_valid = False

    def pull_status(self,exiting=False):
        if not self.hw_valid:
            return
        with self.lock:
            # Queue Body Status RPC
            self.transport.payload_out[0] = self.RPC_GET_PIMU_STATUS
            self.transport.queue_rpc(1, self.rpc_status_reply)
            self.transport.step(exiting=exiting)

    def push_command(self,exiting=False):
        if not self.hw_valid:
            return
        with self.lock:
            if self._dirty_config:
                self.transport.payload_out[0] = self.RPC_SET_PIMU_CONFIG
                sidx = self.pack_config(self.transport.payload_out, 1)
                self.transport.queue_rpc2(sidx, self.rpc_config_reply)
                self._dirty_config=False

            if self._dirty_trigger:
                self.transport.payload_out[0] = self.RPC_SET_PIMU_TRIGGER
                sidx = self.pack_trigger(self.transport.payload_out, 1)
                self.transport.queue_rpc2(sidx, self.rpc_trigger_reply)
                self._trigger=0
                self._dirty_trigger=False
            self.transport.step2(exiting=exiting)

    def pretty_print(self):
        print('------ Pimu -----')
        print('Voltage',self.status['voltage'])
        print('Current', self.status['current'])
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
        if self.board_info['hardware_id']>0:
            print('Charger Connected', self.status['charger_connected'])
            print('Boot Detected', self.status['boot_detected'])
        print('Debug', self.status['debug'])
        print('Timestamp (s)', self.status['timestamp'])
        print('Read error', self.transport.status['read_error'])
        print('Dropped motor sync',self.status['motor_sync_drop'])
        print('Board variant:',self.board_info['board_variant'])
        print('Firmware version:', self.board_info['firmware_version'])
        self.imu.pretty_print()

    # ####################### User Functions #######################################################

    def runstop_event_reset(self):
        """
        Reset the robot runstop, allowing motion to continue
        """
        with self.lock:
            self._trigger=self._trigger | self.TRIGGER_RUNSTOP_RESET
            self._dirty_trigger=True

    def runstop_event_trigger(self):
        """
        Trigger the robot runstop, stopping motion
        """
        with self.lock:
            self._trigger=self._trigger | self.TRIGGER_RUNSTOP_ON
            self._dirty_trigger=True

    def trigger_beep(self):
        """
        Generate a single short beep
        """
        with self.lock:
            self._trigger=self._trigger | self.TRIGGER_BEEP
            self._dirty_trigger=True

    # ####################### Utility functions ####################################################
    def imu_reset(self):
        with self.lock:
            self._trigger=self._trigger | self.TRIGGER_IMU_RESET
            self._dirty_trigger=True

    def trigger_motor_sync(self):
        #Push out immediately
        if not self.hw_valid:
            return

        t = time.time()
        if self.ts_last_motor_sync is not None and t-self.ts_last_motor_sync<1.0/self.params['max_sync_rate_hz']:
            self.status['motor_sync_drop'] += 1
            if self.ts_last_motor_sync_warn is None or t-self.ts_last_motor_sync_warn>5.0:
                print('Warning: Rate of calls to Pimu:trigger_motor_sync above maximum frequency of %.2f Hz. Motor commands dropped: %d'%(self.params['max_sync_rate_hz'],self.status['motor_sync_drop']))
                self.ts_last_motor_sync_warn=t
            return

        with self.lock:
            self.transport.payload_out[0] = self.RPC_SET_MOTOR_SYNC
            self.transport.queue_rpc(1, self.rpc_motor_sync_reply)
            self.transport.step()
            self.ts_last_motor_sync = t

    def set_fan_on(self):
        with self.lock:
            self._trigger=self._trigger | self.TRIGGER_FAN_ON
            self._dirty_trigger=True

    def set_fan_off(self):
        with self.lock:
            self._trigger=self._trigger | self.TRIGGER_FAN_OFF
            self._dirty_trigger=True

    def set_buzzer_on(self):
        with self.lock:
            self._trigger=self._trigger | self.TRIGGER_BUZZER_ON
            self._dirty_trigger=True

    def set_buzzer_off(self):
        with self.lock:
            self._trigger=self._trigger | self.TRIGGER_BUZZER_OFF
            self._dirty_trigger=True

    def board_reset(self):
        with self.lock:
            self._trigger=self._trigger | self.TRIGGER_BOARD_RESET
            self._dirty_trigger=True

    def cliff_event_reset(self):
        with self.lock:
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
        raw_to_mV = 3300 / 1024.0
        mV = raw * raw_to_mV
        mA = mV/.408 # conversion per circuit
        return mA/1000.0
    # ################Data Packing #####################

    def unpack_board_info(self,s):
        with self.lock:
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
        with self.lock:
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
        with self.lock:
            pack_uint32_t(s,sidx,self._trigger); sidx+=4
            return sidx

    def unpack_status(self,s):
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
            cpu_temp=self.get_cpu_temp()
            if cpu_temp>self.params['base_fan_on']:
                if self.ts_last_fan_on is None or time.time()-self.ts_last_fan_on>3.0: #Will turn itself off if don't refresh command
                    self.set_fan_on()
                    self.push_command()
                    self.ts_last_fan_on = time.time()
                if  not self.status['fan_on']:
                    self.logger.debug('Base fan turned on')

            if self.fan_on_last and not self.status['fan_on']:
                self.logger.debug('Base fan turned off')

            if cpu_temp<self.params['base_fan_off']and self.status['fan_on']:
                self.set_fan_off()
                self.push_command()
            self.fan_on_last = self.status['fan_on']


# ######################## PIMU PROTOCOL PO #################################

class Pimu_Protocol_P0(PimuBase):
    def unpack_status(self,s):
        with self.lock:
            sidx=0
            sidx +=self.imu.unpack_status((s[sidx:]))
            self.status['voltage']=self.get_voltage(unpack_float_t(s[sidx:]));sidx+=4
            self.status['current'] = self.get_current(unpack_float_t(s[sidx:]));sidx+=4
            self.status['temp'] = self.get_temp(unpack_float_t(s[sidx:]));sidx+=4

            for i in range(4):
                self.status['cliff_range'][i]=unpack_float_t(s[sidx:])
                sidx+=4

            self.status['state'] = unpack_uint32_t(s[sidx:])
            sidx += 4

            self.status['at_cliff']=[]
            self.status['at_cliff'].append((self.status['state'] & self.STATE_AT_CLIFF_0) != 0)
            self.status['at_cliff'].append((self.status['state'] & self.STATE_AT_CLIFF_1) != 0)
            self.status['at_cliff'].append((self.status['state'] & self.STATE_AT_CLIFF_2) != 0)
            self.status['at_cliff'].append((self.status['state'] & self.STATE_AT_CLIFF_3) != 0)
            self.status['runstop_event'] = (self.status['state'] & self.STATE_RUNSTOP_EVENT) != 0
            self.status['cliff_event'] = (self.status['state'] & self.STATE_CLIFF_EVENT) != 0
            self.status['fan_on'] = (self.status['state'] & self.STATE_FAN_ON) != 0
            self.status['buzzer_on'] = (self.status['state'] & self.STATE_BUZZER_ON) != 0
            self.status['low_voltage_alert'] = (self.status['state'] & self.STATE_LOW_VOLTAGE_ALERT) != 0
            self.status['high_current_alert'] = (self.status['state'] & self.STATE_HIGH_CURRENT_ALERT) != 0
            self.status['over_tilt_alert'] = (self.status['state'] & self.STATE_OVER_TILT_ALERT) != 0
            self.status['charger_connected'] = (self.status['state'] & self.STATE_CHARGER_CONNECTED) != 0
            self.status['boot_detected'] = (self.status['state'] & self.STATE_BOOT_DETECTED) != 0
            self.status['timestamp'] = self.timestamp.set(unpack_uint32_t(s[sidx:])); sidx += 4
            self.status['bump_event_cnt'] = unpack_uint16_t(s[sidx:]);sidx += 2
            self.status['debug'] = unpack_float_t(s[sidx:]); sidx += 4
            self.status['cpu_temp']=self.get_cpu_temp()
            return sidx

# ######################## PIMU PROTOCOL P1 #################################
class Pimu_Protocol_P1(PimuBase):
    def unpack_status(self,s):
        with self.lock:
            sidx=0
            sidx +=self.imu.unpack_status((s[sidx:]))
            self.status['voltage']=self.get_voltage(unpack_float_t(s[sidx:]));sidx+=4
            self.status['current'] = self.get_current(unpack_float_t(s[sidx:]));sidx+=4
            self.status['temp'] = self.get_temp(unpack_float_t(s[sidx:]));sidx+=4

            for i in range(4):
                self.status['cliff_range'][i]=unpack_float_t(s[sidx:])
                sidx+=4

            self.status['state'] = unpack_uint32_t(s[sidx:])
            sidx += 4

            self.status['at_cliff']=[]
            self.status['at_cliff'].append((self.status['state'] & PimuBase.STATE_AT_CLIFF_0) != 0)
            self.status['at_cliff'].append((self.status['state'] & self.STATE_AT_CLIFF_1) != 0)
            self.status['at_cliff'].append((self.status['state'] & self.STATE_AT_CLIFF_2) != 0)
            self.status['at_cliff'].append((self.status['state'] & self.STATE_AT_CLIFF_3) != 0)
            self.status['runstop_event'] = (self.status['state'] & self.STATE_RUNSTOP_EVENT) != 0
            self.status['cliff_event'] = (self.status['state'] & self.STATE_CLIFF_EVENT) != 0
            self.status['fan_on'] = (self.status['state'] & self.STATE_FAN_ON) != 0
            self.status['buzzer_on'] = (self.status['state'] & self.STATE_BUZZER_ON) != 0
            self.status['low_voltage_alert'] = (self.status['state'] & self.STATE_LOW_VOLTAGE_ALERT) != 0
            self.status['high_current_alert'] = (self.status['state'] & self.STATE_HIGH_CURRENT_ALERT) != 0
            self.status['over_tilt_alert'] = (self.status['state'] & self.STATE_OVER_TILT_ALERT) != 0
            if self.board_info['hardware_id']>0:
                self.status['charger_connected'] = (self.status['state'] & self.STATE_CHARGER_CONNECTED) != 0
                self.status['boot_detected'] = (self.status['state'] & self.STATE_BOOT_DETECTED) != 0
            self.status['timestamp'] = self.timestamp.set(unpack_uint64_t(s[sidx:])); sidx += 8
            self.imu.status['timestamp'] = self.status['timestamp']
            self.status['bump_event_cnt'] = unpack_uint16_t(s[sidx:]);sidx += 2
            self.status['debug'] = unpack_float_t(s[sidx:]); sidx += 4
            self.status['cpu_temp']=self.get_cpu_temp()
            return sidx

# ######################## PIMU #################################
class Pimu(PimuBase):
    """
    API to the Stretch RE1 Power and IMU board (Pimu)
    """
    def __init__(self, event_reset=False):
        PimuBase.__init__(self, event_reset)
        # Order in descending order so more recent protocols/methods override less recent
        self.supported_protocols = {'p0': (Pimu_Protocol_P0,), 'p1': (Pimu_Protocol_P1,Pimu_Protocol_P0,)}

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