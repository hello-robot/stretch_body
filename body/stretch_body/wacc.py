from __future__ import print_function
from stretch_body.transport import *
from stretch_body.device import Device, DeviceTimestamp
import threading
import textwrap
import array as arr


# ######################## WACC #################################

class WaccBase(Device):
    """
    API to the Stretch wrist+accelerometer (Wacc) board
    The Wacc has:
    -- 3-axis accelerometer reported as Ax,Ay,and Az
    -- Two digital inputs D0, D1
    -- Two digital outputs D2, D3
    -- One analog input: A0
    -- A single tap count based on the accelerometer

    ext_status_cb: Callback to handle custom status data
    ext_command_cb: Callback to handle custom command data
    """
    RPC_SET_WACC_CONFIG = 1
    RPC_REPLY_WACC_CONFIG = 2
    RPC_GET_WACC_STATUS = 3
    RPC_REPLY_WACC_STATUS = 4
    RPC_SET_WACC_COMMAND = 5
    RPC_REPLY_WACC_COMMAND = 6
    RPC_GET_WACC_BOARD_INFO = 7
    RPC_REPLY_WACC_BOARD_INFO = 8
    RPC_READ_TRACE =9
    RPC_REPLY_READ_TRACE =10
    RPC_LOAD_TEST_PUSH = 11
    RPC_REPLY_LOAD_TEST_PUSH = 12
    RPC_LOAD_TEST_PULL = 13
    RPC_REPLY_LOAD_TEST_PULL = 14

    TRIGGER_BOARD_RESET = 1
    TRIGGER_ENABLE_TRACE = 2
    TRIGGER_DISABLE_TRACE = 4

    STATE_IS_TRACE_ON = 1
    TRACE_TYPE_STATUS = 0
    TRACE_TYPE_DEBUG = 1
    TRACE_TYPE_PRINT = 2


    def __init__(self, ext_status_cb=None, ext_command_cb=None, usb=None):
        Device.__init__(self, 'wacc')
        self.ext_status_cb=ext_status_cb
        self.ext_command_cb=ext_command_cb
        self._dirty_config = True #Force push down
        self._dirty_command = False
        self._command = {'d2':0,'d3':0, 'trigger':0}
        if usb is None:
            usb = self.params['usb_name']
        self.transport = Transport(usb=usb, logger=self.logger)
        self.status = { 'ax':0,'ay':0,'az':0,'a0':0,'d0':0,'d1':0, 'd2':0,'d3':0,'single_tap_count': 0, 'state':0, 'debug':0,
                       'timestamp': 0,'trace_on':0,
                       'transport': self.transport.status}
        self.status_zero = self.status.copy()
        self.board_info = {'board_variant': None, 'firmware_version': None, 'protocol_version': None,'hardware_id':0}
        self.hw_valid = False
        self.load_test_payload = arr.array('B', range(256)) * 4
    # ###########  Device Methods #############

    def startup(self, threaded=False):
        try:
            self.config = self.params['config']
            Device.startup(self, threaded=threaded)
            self.hw_valid = self.transport.startup()
            if self.hw_valid:
                # Pull board info
                payload=arr.array('B',[self.RPC_GET_WACC_BOARD_INFO])
                self.transport.do_pull_rpc_sync(payload,self.rpc_board_info_reply)
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

        self.push_command(exiting=True)
        self.transport.stop()
        self.hw_valid = False



    def set_D2(self,on):#0 or 1
        """
        Set the Digital Out 2 on the Wacc expansion header
        """
        self._command['d2']=bool(on)
        self._dirty_command = True

    def set_D3(self,on): #0 or 1
        """
        Set the Digital Out 3 on the Wacc expansion header
        """
        self._command['d3']=bool(on)
        self._dirty_command = True

    def pull_status(self,exiting=False):
        if not self.hw_valid:
            return
        # Queue Status RPC
        payload = arr.array('B',[self.RPC_GET_WACC_STATUS])
        self.transport.do_pull_rpc_sync(payload, self.rpc_status_reply,exiting=exiting )

    async def pull_status_async(self,exiting=False):
        if not self.hw_valid:
            return
        payload = arr.array('B', [self.RPC_GET_WACC_STATUS])
        await self.transport.do_pull_rpc_async(payload, self.rpc_status_reply, exiting=exiting)

    async def push_command_async(self,exiting=False):
        if not self.hw_valid:
            return
        payload = self.transport.get_empty_payload()
        if self._dirty_config:
            payload[0] = self.RPC_SET_WACC_CONFIG
            sidx = self.pack_config(payload, 1)
            await self.transport.do_push_rpc_async(payload[:sidx], self.rpc_config_reply,exiting=exiting)
            self._dirty_config=False

        if self._dirty_command:
            payload[0] = self.RPC_SET_WACC_COMMAND
            sidx = self.pack_command( payload, 1)
            await self.transport.do_push_rpc_async(payload[:sidx], self.rpc_command_reply,exiting=exiting)
            self._command['trigger'] =0
            self._dirty_command=False

    def push_command(self,exiting=False):
        if not self.hw_valid:
            return
        payload = self.transport.get_empty_payload()
        if self._dirty_config:
            payload[0] = self.RPC_SET_WACC_CONFIG
            sidx = self.pack_config(payload, 1)
            self.transport.do_push_rpc_sync(payload[:sidx], self.rpc_config_reply,exiting=exiting)
            self._dirty_config=False

        if self._dirty_command:
            payload[0] = self.RPC_SET_WACC_COMMAND
            sidx = self.pack_command( payload, 1)
            self.transport.do_push_rpc_sync(payload[:sidx], self.rpc_command_reply,exiting=exiting)
            self._command['trigger'] =0
            self._dirty_command=False

    def pretty_print(self):
        print('------------------------------')
        print('Ax (m/s^2)',self.status['ax'])
        print('Ay (m/s^2)', self.status['ay'])
        print('Az (m/s^2)', self.status['az'])
        print('A0', self.status['a0'])
        print('D0 (In)', self.status['d0'])
        print('D1 (In)', self.status['d1'])
        print('D2 (Out)', self.status['d2'])
        print('D3 (Out)', self.status['d3'])
        print('Single Tap Count', self.status['single_tap_count'])
        print('State ', self.status['state'])
        print('Trace on',self.status['trace_on'])
        print('Debug',self.status['debug'])
        print('Timestamp (s)', self.status['timestamp'])
        print('Board variant:', self.board_info['board_variant'])
        print('Firmware version:', self.board_info['firmware_version'])
        print('Transport version:', self.transport.version)
    # ####################### Utility functions ####################################################
    def board_reset(self):
        self._command['trigger']=self._command['trigger']| self.TRIGGER_BOARD_RESET
        self._dirty_command=True

    # ################Data Packing #####################

    def unpack_board_info(self,s):
        sidx=0
        self.board_info['board_variant'] = unpack_string_t(s[sidx:], 20)
        self.board_info['hardware_id'] = 0
        if len(self.board_info['board_variant'])==6: #New format of Wacc.x  Older format of Wacc.BoardName.Vx' If older format,default to 0
            self.board_info['hardware_id']=int(self.board_info['board_variant'][-1])
        sidx += 20
        self.board_info['firmware_version'] = unpack_string_t(s[sidx:], 20)
        self.board_info['protocol_version'] = self.board_info['firmware_version'][self.board_info['firmware_version'].rfind('p'):]
        sidx += 20
        return sidx


    def pack_command(self,s,sidx):
        if self.ext_command_cb is not None:  # Pack custom data first
            sidx += self.ext_command_cb(s, sidx)
        pack_uint8_t(s, sidx, self._command['d2'])
        sidx += 1
        pack_uint8_t(s, sidx, self._command['d3'])
        sidx += 1
        pack_uint32_t(s, sidx, self._command['trigger'])
        sidx += 4
        return sidx

    def pack_config(self,s,sidx):
        pack_uint8_t(s, sidx, self.config['accel_range_g'])
        sidx += 1
        pack_float_t(s, sidx, self.config['accel_LPF'])
        sidx += 4
        pack_float_t(s, sidx, self.config['ana_LPF'])
        sidx += 4
        pack_uint8_t(s, sidx, self.config['accel_single_tap_dur'])
        sidx += 1
        pack_uint8_t(s, sidx, self.config['accel_single_tap_thresh'])
        sidx += 1
        pack_float_t(s, sidx, self.config['accel_gravity_scale'])
        sidx += 4
        return sidx

    def unpack_status(self,s,unpack_to=None):
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


    def push_load_test(self):
        raise NotImplementedError('This method not supported for firmware on protocol {0}.'
                                  .format(self.board_info['protocol_version']))

    def pull_load_test(self):
        raise NotImplementedError('This method not supported for firmware on protocol {0}.'
                                  .format(self.board_info['protocol_version']))

    # ################Transport Callbacks #####################


    def rpc_enable_transport_v1_reply(self, reply):
        raise NotImplementedError('This method not supported for firmware on protocol {0}.'
                                  .format(self.board_info['protocol_version']))

    def rpc_board_info_reply(self,reply):
        if reply[0] == self.RPC_REPLY_WACC_BOARD_INFO:
            self.unpack_board_info(reply[1:])
        else:
            self.logger.warning('Error RPC_REPLY_WACC_BOARD_INFO', reply[0])

    def rpc_command_reply(self,reply):
        if reply[0] != self.RPC_REPLY_WACC_COMMAND:
            self.logger.warning('Error RPC_REPLY_WACC_COMMAND', reply[0])

    def rpc_config_reply(self,reply):
        if reply[0] != self.RPC_REPLY_WACC_CONFIG:
            self.logger.warning('Error RPC_REPLY_WACC_CONFIG', reply[0])

    def rpc_status_reply(self,reply):
        if reply[0] == self.RPC_REPLY_WACC_STATUS:
            self.unpack_status(reply[1:])
        else:
            self.logger.warning('Error RPC_REPLY_WACC_STATUS', reply[0])


# ######################## Wacc PROTOCOL PO #################################

class Wacc_Protocol_P0(WaccBase):
    def unpack_status(self,s,unpack_to=None):
        if unpack_to is None:
            unpack_to=self.status
        sidx=0
        if self.ext_status_cb is not None:
            sidx+=self.ext_status_cb(s[sidx:])
        unpack_to['ax'] = unpack_float_t(s[sidx:]);sidx+=4
        unpack_to['ay'] = unpack_float_t(s[sidx:]);sidx+=4
        unpack_to['az'] = unpack_float_t(s[sidx:]);sidx+=4

        unpack_to['a0'] = unpack_int16_t(s[sidx:]);sidx+=2
        unpack_to['d0'] = unpack_uint8_t(s[sidx:]); sidx += 1
        unpack_to['d1'] = unpack_uint8_t(s[sidx:]); sidx += 1
        unpack_to['d2'] = unpack_uint8_t(s[sidx:]); sidx += 1
        unpack_to['d3'] = unpack_uint8_t(s[sidx:]); sidx += 1
        unpack_to['single_tap_count'] = unpack_uint32_t(s[sidx:]);sidx += 4
        unpack_to['state'] = unpack_uint32_t(s[sidx:]); sidx += 4
        unpack_to['timestamp'] = self.timestamp.set(unpack_uint32_t(s[sidx:]));sidx += 4
        unpack_to['debug'] = unpack_uint32_t(s[sidx:]);sidx += 4
        return sidx

# ######################## Wacc PROTOCOL P1 #################################
class Wacc_Protocol_P1(WaccBase):
    def unpack_status(self,s,unpack_to=None):
        if unpack_to is None:
            unpack_to=self.status
        sidx=0
        if self.ext_status_cb is not None:
            sidx+=self.ext_status_cb(s[sidx:])
        unpack_to['ax'] = unpack_float_t(s[sidx:]);sidx+=4
        unpack_to['ay'] = unpack_float_t(s[sidx:]);sidx+=4
        unpack_to['az'] = unpack_float_t(s[sidx:]);sidx+=4
        unpack_to['a0'] = unpack_int16_t(s[sidx:]);sidx+=2
        unpack_to['d0'] = unpack_uint8_t(s[sidx:]); sidx += 1
        unpack_to['d1'] = unpack_uint8_t(s[sidx:]); sidx += 1
        unpack_to['d2'] = unpack_uint8_t(s[sidx:]); sidx += 1
        unpack_to['d3'] = unpack_uint8_t(s[sidx:]); sidx += 1
        unpack_to['single_tap_count'] = unpack_uint32_t(s[sidx:]);sidx += 4
        unpack_to['state'] = unpack_uint32_t(s[sidx:]); sidx += 4
        unpack_to['timestamp'] = self.timestamp.set(unpack_uint64_t(s[sidx:]));sidx += 8
        unpack_to['debug'] = unpack_uint32_t(s[sidx:]);sidx += 4
        return sidx

# ######################## Wacc PROTOCOL P1 #################################
class Wacc_Protocol_P2(WaccBase):
    def unpack_status(self,s,unpack_to=None):
        if unpack_to is None:
            unpack_to=self.status
        sidx=0
        if self.ext_status_cb is not None:
            sidx+=self.ext_status_cb(s[sidx:])
        unpack_to['ax'] = unpack_float_t(s[sidx:]);sidx+=4
        unpack_to['ay'] = unpack_float_t(s[sidx:]);sidx+=4
        unpack_to['az'] = unpack_float_t(s[sidx:]);sidx+=4
        unpack_to['a0'] = unpack_int16_t(s[sidx:]);sidx+=2
        unpack_to['d0'] = unpack_uint8_t(s[sidx:]); sidx += 1
        unpack_to['d1'] = unpack_uint8_t(s[sidx:]); sidx += 1
        unpack_to['d2'] = unpack_uint8_t(s[sidx:]); sidx += 1
        unpack_to['d3'] = unpack_uint8_t(s[sidx:]); sidx += 1
        unpack_to['single_tap_count'] = unpack_uint32_t(s[sidx:]);sidx += 4
        unpack_to['state'] = unpack_uint32_t(s[sidx:]); sidx += 4
        unpack_to['trace_on'] = unpack_to['state'] & self.STATE_IS_TRACE_ON > 0
        unpack_to['timestamp'] = self.timestamp.set(unpack_uint64_t(s[sidx:]));sidx += 8
        unpack_to['debug'] = unpack_uint32_t(s[sidx:]);sidx += 4
        return sidx

    def read_firmware_trace(self):
        self.trace_buf = []
        self.timestamp.reset() #Timestamp holds state, reset within lock to avoid threading issues
        self.n_trace_read=1
        ts=time.time()
        payload = arr.array('B',[self.RPC_READ_TRACE])
        while ( self.n_trace_read) and time.time()-ts<60.0:
            self.transport.do_pull_rpc(payload, self.rpc_read_firmware_trace_reply)
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
        self._command['trigger']=self._command['trigger']| self.TRIGGER_ENABLE_TRACE
        self._dirty_command = True

    def disable_firmware_trace(self):
        self._command['trigger']=self._command['trigger']| self.TRIGGER_DISABLE_TRACE
        self._dirty_command = True

class Wacc_Protocol_P3(WaccBase):
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
# ######################## Wacc #################################
class Wacc(WaccBase):
    """
    API to the Stretch Wrist Accelerometer (Wacc) Board
    """
    def __init__(self, usb=None):
        WaccBase.__init__(self, usb=usb)
        #Order in descending order so more recent protocols/methods override less recent
        self.supported_protocols = {'p0': (Wacc_Protocol_P0,), 'p1': (Wacc_Protocol_P1, Wacc_Protocol_P0),'p2': (Wacc_Protocol_P2, Wacc_Protocol_P1, Wacc_Protocol_P0,),
                                    'p3': (Wacc_Protocol_P3, Wacc_Protocol_P2, Wacc_Protocol_P1, Wacc_Protocol_P0,),}

    def startup(self, threaded=False):
        """
        First determine which protocol version the uC firmware is running.
        Based on that version, replaces PimuBase class inheritance with a inheritance to a child class of PimuBase that supports that protocol
        """
        WaccBase.startup(self, threaded=threaded)
        if self.hw_valid:
            if self.board_info['protocol_version'] in self.supported_protocols:
                Wacc.__bases__ = self.supported_protocols[self.board_info['protocol_version']]
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
