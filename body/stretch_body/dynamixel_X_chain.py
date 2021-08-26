from __future__ import print_function
import stretch_body.hello_utils as hello_utils
import time

# The code can be found in the following directory:
# /opt/ros/melodic/lib/python2.7/dist-packages/dynamixel_sdk/
from dynamixel_sdk.robotis_def import *
from stretch_body.dynamixel_XL430 import *
from stretch_body.dynamixel_hello_XL430 import DynamixelCommErrorStats
import dynamixel_sdk.port_handler as prh
import dynamixel_sdk.packet_handler as pch
import dynamixel_sdk.group_sync_read as gsr


class DynamixelXChain(Device):
    """
    This class manages a daisy chain of Dynamixel X-Series servos
    It allows adding more than one servo at run time
    It allos manage group reading of status data from servos so as to not overload the control bus
    """
    def __init__(self, usb, name):
        Device.__init__(self, name)
        self.usb = usb
        self.pt_lock = threading.RLock()

        try:
            prh.LATENCY_TIMER = self.params['dxl_latency_timer']
            self.port_handler = prh.PortHandler(usb)
            self.port_handler.openPort()
            self.port_handler.setBaudRate(int(self.params['baud']))
            self.packet_handler = pch.PacketHandler(2.0)
            self.hw_valid = True
        except serial.SerialException as e:
            self.packet_handler = None
            self.port_handler = None
            self.hw_valid =False
            self.logger.error("SerialException({0}): {1}".format(e.errno, e.strerror))

        self.status={}
        self.motors = {}
        self.readers={}
        self.comm_errors = DynamixelCommErrorStats(name, logger=self.logger)
        self.status_mux_id = 0

    def add_motor(self,m):
        self.motors[m.name]=m

    def get_motor(self,motor_name):
        try:
            return self.motors[motor_name]
        except (AttributeError, KeyError):
            return None

    def startup(self):
        for mk in self.motors.keys():  # Provide nop data in case comm failures
            self.status[mk] = self.motors[mk].status
        if not self.hw_valid:
            return False
        if len(self.motors.keys()):
            try:
                if self.params['use_group_sync_read']:
                    self.readers['pos'] = gsr.GroupSyncRead(self.port_handler, self.packet_handler,
                                                            XL430_ADDR_PRESENT_POSITION, 4)
                    self.readers['effort'] = gsr.GroupSyncRead(self.port_handler, self.packet_handler,
                                                               XL430_ADDR_PRESENT_LOAD, 2)
                    self.readers['vel'] = gsr.GroupSyncRead(self.port_handler, self.packet_handler,
                                                            XL430_ADDR_PRESENT_VELOCITY, 4)
                    self.readers['temp'] = gsr.GroupSyncRead(self.port_handler, self.packet_handler,
                                                             XL430_ADDR_PRESENT_TEMPERATURE, 1)
                    self.readers['hardware_error'] = gsr.GroupSyncRead(self.port_handler, self.packet_handler,
                                                                       XL430_ADDR_HARDWARE_ERROR_STATUS, 1)
                    for mk in self.motors.keys():
                        for k in self.readers.keys():
                            if not self.readers[k].addParam(self.motors[mk].motor.dxl_id):
                                self.logger.error('Dynamixel X sync read initialization failed.')
                                raise DynamixelCommError
                for mk in self.motors.keys():
                    self.motors[mk].startup()
                    self.status[mk] = self.motors[mk].status
                self.pull_status()
            except DynamixelCommError:
                self.comm_errors.add_error(rx=True,gsr=True)
                self.hw_valid = False
                return False
        return True

    def stop(self):
        if not self.hw_valid:
            return
        for mk in self.motors.keys():
            self.motors[mk].stop()
        self.hw_valid = False


    def pull_status(self):
        if not self.hw_valid:
            return
        try:
            ts = time.time()
            if self.params['use_group_sync_read']:
                pos = self.sync_read(self.readers['pos'])
                if pos==None and self.params['retry_on_comm_failure']:
                    pos = self.sync_read(self.readers['pos'])

                vel = self.sync_read(self.readers['vel'])
                if vel == None and self.params['retry_on_comm_failure']:
                    vel = self.sync_read(self.readers['vel'])

                if self.status_mux_id == 0:
                    effort = self.sync_read(self.readers['effort'])
                    if effort == None and self.params['retry_on_comm_failure']:
                        effort = self.sync_read(self.readers['effort'])
                else:
                    effort = None

                if self.status_mux_id == 1:
                    temp = self.sync_read(self.readers['temp'])
                    if temp == None and self.params['retry_on_comm_failure']:
                        temp = self.sync_read(self.readers['temp'])
                else:
                    temp = None

                if self.status_mux_id == 2:
                    hardware_error = self.sync_read(self.readers['hardware_error'])
                    if hardware_error == None and self.params['retry_on_comm_failure']:
                        hardware_error = self.sync_read(self.readers['hardware_error'])
                else:
                    hardware_error = None

                self.status_mux_id = (self.status_mux_id + 1) % 3

                idx = 0
                # Build dictionary of status data and push to each motor status
                # None may indicate comm error or the field wasn't read on this mux cycle
                for mk in self.motors.keys():
                    data = {'ts': time.time()}
                    if pos is not None:
                        data['x'] = pos[idx]
                    else:
                        data['x'] = self.motors[mk].status['pos_ticks']
                    if vel is not None:
                        data['v'] = vel[idx]
                    else:
                        data['v'] = self.motors[mk].status['vel_ticks']
                    if effort is not None:
                        data['eff'] = effort[idx]
                    else:
                        data['eff'] = self.motors[mk].status['effort_ticks']
                    if temp is not None:
                        data['temp'] = temp[idx]
                    else:
                        data['temp'] = self.motors[mk].status['temp']
                    if hardware_error is not None:
                        data['err'] = hardware_error[idx]
                    else:
                        data['err'] = self.motors[mk].status['hardware_error']
                    self.motors[mk].pull_status(data)
                    idx = idx + 1
            else:
                for m in self.motors:
                    with self.pt_lock:
                        self.motors[m].pull_status()
        except(DynamixelCommError, IOError):
            self.comm_errors.add_error(rx=True, gsr=True)
            self.port_handler.ser.reset_output_buffer()
            self.port_handler.ser.reset_input_buffer()

    def pretty_print(self):
        print('--- Dynamixel X Chain ---')
        print('USB', self.usb)
        for mk in self.motors.keys():
            self.motors[mk].pretty_print()

    def sync_read(self, reader):
        if not self.hw_valid:
            return None
        with self.pt_lock:
            result = reader.txRxPacket()
        if result != COMM_SUCCESS:
            self.logger.debug('Dynamixel X sync read txRxPacket failed with error code = ' + str(result))
            raise DynamixelCommError

        def get_val(id_num):
            b = reader.getData(id_num, reader.start_address, reader.data_length)
            # The error code seems to be 0, yet there are also
            # circumstances where b will be 0 without an error, such
            # as being at position = 0. For now, I am commenting out
            # this error reporting code. One possibility is to edit
            # the SDK module. Another possibility is to use the
            # Available command separately.
            #
            # if (b == 0):
            #     print('Dynamixel X sync read getData failed.')
            #     return None

            if reader.data_length == 4:
                val = struct.unpack('i', arr.array('B', [DXL_LOBYTE(DXL_LOWORD(b)), DXL_HIBYTE(DXL_LOWORD(b)),
                                                         DXL_LOBYTE(DXL_HIWORD(b)), DXL_HIBYTE(DXL_HIWORD(b))]))[0]
            if reader.data_length == 2:
                val = struct.unpack('h', arr.array('B', [DXL_LOBYTE(b), DXL_HIBYTE(b)]))[0]
            if reader.data_length == 1:
                val = struct.unpack('b', arr.array('B', [b]))[0]
            return val
        values = [get_val(self.motors[mk].motor.dxl_id) for mk in self.motors.keys()]
        return values


    def step_sentry(self,robot):
        """This sentry places the Dynamixel servos in torque_disabled
        mode when the runstop is enabled.
        """
        for k in self.motors.keys():
            self.motors[k].step_sentry(robot)
