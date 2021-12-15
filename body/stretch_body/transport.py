from __future__ import print_function
import serial
import time
import struct
import array as arr
import stretch_body.cobbs_framing as cobbs_framing
import copy
import fcntl
import logging

"""

Loop protocol is:



RPC Data is sent over a COBBS encoding with CRC error detection.

The  packet is:

Data can be up to X bytes.

Data is manually packed / unpacked into dictionaries (Python) and C-structs (Arduino).
Care should be taken that the pack/unpack size and types are consistent between the two.
This is not automated.
"""

# ##################### TRANSPORT ####################################



class TransportError(Exception):
    """Base class for exceptions in this module."""
    pass



RPC_START_NEW_RPC =100
RPC_ACK_NEW_RPC= 101

RPC_SEND_BLOCK_MORE = 102
RPC_ACK_SEND_BLOCK_MORE = 103
RPC_SEND_BLOCK_LAST = 104
RPC_ACK_SEND_BLOCK_LAST = 105

RPC_GET_BLOCK = 106
RPC_ACK_GET_BLOCK_MORE = 107
RPC_ACK_GET_BLOCK_LAST = 108

RPC_BLOCK_SIZE = 32
RPC_DATA_SIZE = 1024

dbg_on = 0


class Transport():
    """
    Handle serial communication with Devices
    """
    def __init__(self, usb, logger=logging.getLogger()):
        self.usb = usb
        self.logger = logger
        self.payload_out = arr.array('B', [0] * (RPC_DATA_SIZE+1))
        self.payload_in = arr.array('B', [0] * (RPC_DATA_SIZE+1))
        self.buf = arr.array('B', [0] *(RPC_BLOCK_SIZE*2))

        self.write_error=0
        self.read_error = 0
        self.rpc_queue=[]
        self.rpc_queue2 = []
        self.itr = 0
        self.itr_time = 0
        self.tlast = 0
        self.logger.debug('Starting TransportConnection on: ' + self.usb)
        try:
            self.ser = serial.Serial(self.usb, write_timeout=1.0)#PosixPollSerial(self.usb)#Serial(self.usb)# 115200)  # , write_timeout=1.0)  # Baud not important since USB comms
            if self.ser.isOpen():
                try:
                    fcntl.flock(self.ser.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
                except IOError:
                    self.logger.error('Port %s is busy. Check if another Stretch Body process is already running'%self.usb)
                    self.ser.close()
                    self.ser=None
        except serial.SerialException as e:
            self.logger.error("SerialException({0}): {1}".format(e.errno, e.strerror))
            self.ser = None
        if self.ser==None:
            self.logger.warning('Unable to open serial port for device %s'%self.usb)
        self.framer=cobbs_framing.CobbsFraming( )
        self.status={'rate':0,'read_error':0,'write_error':0,'itr':0,'transaction_time_avg':0,'transaction_time_max':0,'timestamp_pc':0}


    def startup(self):
        try:
            self.ser = serial.Serial(self.usb, write_timeout=1.0)#PosixPollSerial(self.usb)#Serial(self.usb)# 115200)  # , write_timeout=1.0)  # Baud not important since USB comms
            if self.ser.isOpen():
                try:
                    fcntl.flock(self.ser.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
                except IOError:
                    self.logger.error('Port %s is busy. Check if another Stretch Body process is already running'%self.usb)
                    self.ser.close()
                    self.ser=None
        except serial.SerialException as e:
            self.logger.error("SerialException({0}): {1}".format(e.errno, e.strerror))
            self.ser = None
        if self.ser==None:
            self.logger.warning('Unable to open serial port for device %s'%self.usb)
        return self.ser is not None #return if hardware connection valid

    def stop(self):
        if self.ser:
            self.logger.debug('Shutting down TransportConnection on: ' + self.usb)
            self.ser.close()
            self.ser = None

    def queue_rpc(self,n,reply_callback):
        if self.ser:
            self.rpc_queue.append((copy.copy(self.payload_out[:n]),reply_callback))

    def queue_rpc2(self,n,reply_callback):
        if self.ser:
            self.rpc_queue2.append((copy.copy(self.payload_out[:n]),reply_callback))

    def step_rpc(self,rpc,rpc_callback): #Handle a single RPC transaction
        if not self.ser:
            self.logger.debug('Transport Serial not present for: %s' % self.usb)
            return

        dbg_buf = ''
        try:
            ts = time.time()
            if dbg_on:
                dbg_buf=dbg_buf+'--------------- New RPC -------------------------\n'
            ########## Initiate new RPC
            self.buf[0]=RPC_START_NEW_RPC
            self.framer.sendFramedData(self.buf, 1, self.ser)
            if dbg_on:
                dbg_buf=dbg_buf+'Framer sent RPC_START_NEW_RPC\n'
            crc, nr = self.framer.receiveFramedData(self.buf, self.ser)
            if dbg_on:
                if nr:
                    dbg_buf=dbg_buf+'Framer rcvd on RPC_ACK_NEW_RPC CRC: '+str(crc)+' NR: '+str(nr)+' B0: '+str(self.buf[0])+' Expected B0: '+str(RPC_ACK_NEW_RPC) +':'+self.usb
                else:
                    dbg_buf = dbg_buf +'Framer rcvd 0 bytes on RPC_ACK_NEW_RPC'

            if crc!=1 or self.buf[0] != RPC_ACK_NEW_RPC:
                self.logger.error('Transport RX Error on RPC_ACK_NEW_RPC {0} {1} {2}'.format(crc, nr, self.buf[0]))
                raise TransportError
            #if dbg_on:
            #    print('New RPC initiated, len',len(rpc))
            ########### Send all blocks
            ntx=0
            while ntx<len(rpc):
                nb=min(len(rpc)-ntx,RPC_BLOCK_SIZE) #Num bytes to send
                b = rpc[ntx:ntx + nb]
                ntx=ntx+nb
                if ntx==len(rpc):#Last block
                    self.buf[0] = RPC_SEND_BLOCK_LAST
                    self.buf[1:len(b) + 1] = b
                    #if dbg_on:
                    #    print('Sending last block',ntx)
                    self.framer.sendFramedData(self.buf, nb+1, self.ser)
                    if dbg_on:
                        dbg_buf = dbg_buf + 'Framer sent RPC_SEND_BLOCK_LAST\n'
                    #if dbg_on:
                    #    print('Getting last block ack',ntx)
                    crc, nr = self.framer.receiveFramedData(self.buf, self.ser)
                    if dbg_on:
                        if nr:
                            dbg_buf = dbg_buf + 'Framer rcvd on RPC_SEND_BLOCK_LAST CRC: ' + str(crc) + ' NR: ' + str(nr) + ' B0: ' + str(self.buf[0]) + ' Expected B0: ' + str(RPC_ACK_SEND_BLOCK_LAST)+':'+self.usb+'\n'
                        else:
                            dbg_buf = dbg_buf + 'Framer rcvd 0 bytes on RPC_SEND_BLOCK_LAST'
                    #if dbg_on:
                    #    print('Last block ack rcvd',ntx)
                    if crc!=1 or self.buf[0]!=RPC_ACK_SEND_BLOCK_LAST:
                        self.logger.error('Transport RX Error on RPC_ACK_SEND_BLOCK_LAST {0} {1} {2}'.format(crc, nr, self.buf[0]))
                        raise TransportError
                else:
                    self.buf[0] = RPC_SEND_BLOCK_MORE
                    self.buf[1:len(b) + 1] = b
                    #if dbg_on:
                    #    print('Sending next block',ntx)
                    self.framer.sendFramedData(self.buf, nb + 1, self.ser)
                    if dbg_on:
                        dbg_buf = dbg_buf + 'Framer sent RPC_SEND_BLOCK_MORE\n'
                    #if dbg_on:
                    #    print('Sent next block',ntx)
                    crc, nr = self.framer.receiveFramedData(self.buf, self.ser)
                    if dbg_on:
                        if nr:
                            dbg_buf = dbg_buf + 'Framer rcvd on RPC_SEND_BLOCK_MORE CRC: ' + str(crc) + ' NR: ' + str(nr) + ' B0: ' + str(self.buf[0]) + ' Expected B0: ' + str(RPC_ACK_SEND_BLOCK_MORE)+':'+self.usb+'\n'
                        else:
                            dbg_buf = dbg_buf + 'Framer rcvd 0 bytes on RPC_SEND_BLOCK_MORE'
                    if crc!=1 or self.buf[0]!=RPC_ACK_SEND_BLOCK_MORE:
                        self.logger.error('Transport RX Error on RPC_ACK_SEND_BLOCK_MORE {0} {1} {2}'.format(crc, nr, self.buf[0]))
                        raise TransportError
            ########### Receive all blocks
            reply = arr.array('B')
            #if dbg_on:
            #    print('Receiving RPC reply')
            while True:
                self.buf[0] = RPC_GET_BLOCK
                #if dbg_on:
                #    print('Block requested')
                self.framer.sendFramedData(self.buf,1, self.ser)
                crc, nr = self.framer.receiveFramedData(self.buf, self.ser)
                #if dbg_on:
                #    print('Block request success')
                if crc != 1 or not (self.buf[0] == RPC_ACK_GET_BLOCK_MORE or self.buf[0] == RPC_ACK_GET_BLOCK_LAST):
                    self.logger.error('Transport RX Error on RPC_GET_BLOCK {0} {1} {2}'.format(crc, nr, self.buf[0]))
                    raise TransportError
                reply=reply+self.buf[1:nr]

                if self.buf[0] == RPC_ACK_GET_BLOCK_LAST:
                    break
            # Now process the reply
            #if dbg_on:
            #    print('Got reply',len(reply))
            #print('---------------------- RPC complete, elapsed time------------------:',time.time()-ts)
            rpc_callback(reply)
        except TransportError as e:
            if dbg_on:
                print('---- Debug Exception')
                print(dbg_buf)
            self.read_error = self.read_error + 1
            self.ser.reset_output_buffer()
            self.ser.reset_input_buffer()
            self.logger.error("TransportError: %s : %s" % (self.usb, str(e)))
        except serial.SerialTimeoutException as e:
            self.write_error += 1
            self.ser=None
            self.logger.error("SerialTimeoutException: %s : %s"%(self.usb, str(e)))
        except serial.SerialException as e:
            self.logger.error("SerialException: %s : %s"%(self.usb, str(e)))
            self.ser=None
        except TypeError as e:
            self.logger.error("TypeError: %s : %s" % (self.usb, str(e)))
            self.ser=None

    def is_step_complete(self):
        return self.rt.dirty_step==False
    def is_step2_complete(self):
        return self.rt.dirty_step2==False

    def step(self,exiting=False):
        if not self.ser:
            return
        if exiting:
            time.sleep(0.1) #May have been a hard exit, give time for bad data to land, remove, do final RPC
            self.ser.reset_output_buffer()
            self.ser.reset_input_buffer()

        #This will block until all RPCs have been completed
        try:
            #called by body thread at cyclic rate
            self.itr += 1
            self.itr_time = time.time() - self.tlast
            self.tlast = time.time()
            #Now run RPC calls
            while len(self.rpc_queue):
                rpc,reply_callback=self.rpc_queue[0]
                #if dbg_on:
                #    print('RPC of',reply_callback)
                self.step_rpc(rpc,reply_callback)
                self.rpc_queue = self.rpc_queue[1:]
        except IOError as e:
            print("IOError({0}): {1}".format(e.errno, e.strerror))
            self.read_error=self.read_error+1
        except serial.SerialTimeoutException as e:
            self.write_error += 1
            print("SerialException({0}): {1}".format(e.errno, e.strerror))

        # Update status
        if self.itr_time != 0:
            self.status['rate'] = 1 / self.itr_time
        else:
            self.status['rate'] = 0
        self.status['read_error'] = self.read_error
        self.status['write_error'] = self.write_error
        self.status['itr'] = self.itr

    def step2(self,exiting=False):
        if not self.ser:
            return
        if exiting:
            time.sleep(0.1)  # May have been a hard exit, give time for bad data to land, remove, do final RPC
            self.ser.reset_output_buffer()
            self.ser.reset_input_buffer()

        #This will block until all RPCs have been completed
        try:
            while len(self.rpc_queue2):
                rpc,reply_callback=self.rpc_queue2[0]
                self.step_rpc(rpc,reply_callback)
                self.rpc_queue2 = self.rpc_queue2[1:]
        except IOError as e:
            print("IOError({0}): {1}".format(e.errno, e.strerror))
            self.read_error=self.read_error+1
        except serial.SerialTimeoutException as e:
            self.write_error += 1
            print("SerialException({0}): {1}".format(e.errno, e.strerror))


# #####################################
def pack_string_t(s,sidx,x):
    n=len(x)
    return struct.pack_into(str(n)+'s',s,sidx,x)

def unpack_string_t(s,n):
    return (struct.unpack(str(n)+'s', s[:n])[0].strip(b'\x00')).decode('utf-8')

def unpack_int32_t(s):
    return struct.unpack('i',s[:4])[0]

def unpack_uint32_t(s):
    return struct.unpack('I',s[:4])[0]

def unpack_int64_t(s):
    return struct.unpack('q',s[:8])[0]

def unpack_uint64_t(s):
    return struct.unpack('Q',s[:8])[0]

def unpack_int16_t(s):
    return struct.unpack('h',s[:2])[0]

def unpack_uint16_t(s):
    return struct.unpack('H',s[:2])[0]

def unpack_uint8_t(s):
    return struct.unpack('B',s[:1])[0]

def unpack_float_t(s):
    return struct.unpack('f', s[:4])[0]

def unpack_double_t(s):
    return struct.unpack('d', s[:8])[0]

def pack_float_t(s,sidx,x):
    return struct.pack_into('f',s,sidx,x)

def pack_double_t(s,sidx,x):
    return struct.pack_into('d',s,sidx,x)

def pack_int32_t(s,sidx,x):
    return struct.pack_into('i',s,sidx,x)

def pack_uint32_t(s,sidx,x):
    return struct.pack_into('I',s,sidx,x)

def pack_int16_t(s,sidx,x):
    return struct.pack_into('h',s,sidx,x)

def pack_uint16_t(s,sidx,x):
    return struct.pack_into('H',s,sidx,x)

def pack_uint8_t(s,sidx,x):
    return struct.pack_into('B',s,sidx,x)



















