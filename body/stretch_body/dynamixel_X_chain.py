from __future__ import print_function
import stretch_body.hello_utils as hello_utils
import time

# The code can be found in the following directory:
# /opt/ros/melodic/lib/python2.7/dist-packages/dynamixel_sdk/
from dynamixel_sdk.robotis_def import *
from stretch_body.dynamixel_XL430 import *
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
        self.runstop_last=None


    def add_motor(self,m):
        self.motors[m.name]=m

    def startup(self):
        if not self.hw_valid:
            return False
        if len(self.motors.keys()):
            if self.params['use_group_sync_read']:
                self.readers['pos']=gsr.GroupSyncRead(self.port_handler, self.packet_handler, XL430_ADDR_PRESENT_POSITION,4)
                self.readers['effort']=gsr.GroupSyncRead(self.port_handler, self.packet_handler, XL430_ADDR_PRESENT_LOAD,2)
                self.readers['vel'] = gsr.GroupSyncRead(self.port_handler, self.packet_handler, XL430_ADDR_PRESENT_VELOCITY,4)
                self.readers['temp'] = gsr.GroupSyncRead(self.port_handler, self.packet_handler, XL430_ADDR_PRESENT_TEMPERATURE,1)
                self.readers['hardware_error'] = gsr.GroupSyncRead(self.port_handler, self.packet_handler, XL430_ADDR_HARDWARE_ERROR_STATUS, 1)
                for mk in self.motors.keys():
                    for k in self.readers.keys():
                        if not self.readers[k].addParam(self.motors[mk].motor.dxl_id):
                            raise IOError('Dynamixel X sync read initialization failed.')
            for mk in self.motors.keys():
                self.motors[mk].startup()
                self.status[mk] = self.motors[mk].status
            self.pull_status()
        return True

    def stop(self):
        if not self.hw_valid:
            return
        self.hw_valid = False
        for mk in self.motors.keys():
            self.motors[mk].stop()


    def pull_status(self):
        if not self.hw_valid:
            return
        try:
            ts = time.time()
            if self.params['use_group_sync_read']:

                pos = self.sync_read(self.readers['pos'])
                if pos==None and self.params['retry_on_comm_failure']:
                    pos = self.sync_read(self.readers['pos'])

                effort = self.sync_read(self.readers['effort'])
                if effort == None and self.params['retry_on_comm_failure']:
                    effort = self.sync_read(self.readers['effort'])

                vel = self.sync_read(self.readers['vel'])
                if vel == None and self.params['retry_on_comm_failure']:
                    vel = self.sync_read(self.readers['vel'])

                temp = self.sync_read(self.readers['temp'])
                if temp == None and self.params['retry_on_comm_failure']:
                    temp = self.sync_read(self.readers['temp'])

                hardware_error = self.sync_read(self.readers['hardware_error'])
                if hardware_error == None and self.params['retry_on_comm_failure']:
                    hardware_error = self.sync_read(self.readers['hardware_error'])

                if pos is not None and effort is not None and vel is not None and temp is not None and hardware_error is not None:
                    idx=0
                    for mk in self.motors.keys():
                        data = {'x':pos[idx], 'v': vel[idx],'eff': effort[idx], 'ts': ts, 'temp': temp[idx], 'err':hardware_error[idx]}
                        self.motors[mk].pull_status(data)
                        idx=idx+1
                #else:
                #    print("Communication error. Failed to pull status on %s"%self.usb)
            else:
                for m in self.motors:
                    with self.pt_lock:
                        self.motors[m].pull_status()
        except IOError:
            self.logger.error('Pull Status IOError on: %s'%self.usb)
        except IndexError:
            self.logger.error('Pull Status IndexError on: %s'%self.usb)

    def pretty_print(self):
        print('--- Dynamixel X Chain ---')
        print('USB', self.usb)
        for mk in self.motors.keys():
            self.motors[mk].pretty_print()

    def sync_read(self, reader):
        if not self.hw_valid:
            return
        with self.pt_lock:
            result = reader.txRxPacket()
        if result != COMM_SUCCESS:
            raise IOError('Dynamixel X sync read txRxPacket failed with error code = ' + str(result))


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

        if self.hw_valid and self.robot_params['robot_sentry']['dynamixel_stop_on_runstop']:
            runstop=robot.pimu.status['runstop_event']
            if runstop is not self.runstop_last:
                if runstop:
                    for mk in self.motors.keys():
                        self.motors[mk].disable_torque()
                else:
                    for mk in self.motors.keys():
                        self.motors[mk].enable_torque()
            self.runstop_last=runstop


