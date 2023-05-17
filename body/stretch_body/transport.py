from __future__ import print_function
import serial
import time
import struct
import array as arr
import stretch_body.cobbs_framing as cobbs_framing
import copy
import fcntl
import logging
import math
import threading

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


RPC_V1_MAX_FRAMES = 18 #Required to support 1024 bytes
RPC_V1_PUSH_FRAME_FIRST_MORE= 201
RPC_V1_PUSH_FRAME_FIRST_ONLY= 202
RPC_V1_PUSH_FRAME_MORE= 203
RPC_V1_PUSH_FRAME_LAST=  204
RPC_V1_PUSH_ACK=  205
RPC_V1_PULL_FRAME_FIRST= 206
RPC_V1_PULL_FRAME_MORE= 207
RPC_V1_PULL_FRAME_ACK_MORE=  208
RPC_V1_PULL_FRAME_ACK_LAST= 209
RPC_V1_FRAME_DATA_MAX_BYTES=  58 #63 - 2 (CRC) - 1 (Cobbs Header) - 1 (FRAME CMD) - 1 (Packet Marker)
COBBS_FRAME_SIZE_V1= 63 #Was seeing issues when transmitting 64 bytes so limiting to 63. Issue resolved.

# //////////////////////////////  V0 Defines ///////////////////////////////////////////////////
RPC_V0_START_NEW_RPC= 100
RPC_V0_ACK_NEW_RPC= 101
RPC_V0_SEND_BLOCK_MORE=  102
RPC_V0_ACK_SEND_BLOCK_MORE=  103
RPC_V0_SEND_BLOCK_LAST=  104
RPC_V0_ACK_SEND_BLOCK_LAST=  105
RPC_V0_GET_BLOCK=  106
RPC_V0_ACK_GET_BLOCK_MORE=  107
RPC_V0_ACK_GET_BLOCK_LAST=  108
RPC_V0_BLOCK_SIZE= 32
COBBS_FRAME_SIZE_V0= 64

#//////////////////////////////  Shared Defines ///////////////////////////////////////////////////
RPC_DATA_MAX_BYTES=  1024
RPC_MAX_FRAME_SIZE= 64 #Arduino and Linux USB Uart has a 64 byte buffer. When frame is >64 have seen issues.

RPC_TRANSPORT_VERSION_0 = 0
RPC_TRANSPORT_VERSION_1 = 1

dbg_on = 0

class Transport():
    """
    Handle serial communication with Hello Devices
    """
    def __init__(self, usb, logger=logging.getLogger()):
        self.usb = usb
        self.logger = logger
        self.empty_payload=arr.array('B',[0] * (RPC_DATA_MAX_BYTES+1)) #RPC ID + 1024 bytes of data
        self.empty_frame = arr.array('B',[0]*RPC_MAX_FRAME_SIZE)
        self.status={'read_error':0,'write_error':0,'transactions':0}
        self.version = RPC_TRANSPORT_VERSION_0
        self.lock = threading.Lock()

    def startup(self):
        try:
            self.logger.debug('Starting TransportConnection on: ' + self.usb)
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

    def get_empty_payload(self): #Just a fast convience function to create a large array of 'B'
        return self.empty_payload[:]

    def get_empty_frame(self): #Just a fast convience function to create a large array of 'B'
        return self.empty_frame[:]

    def configure_version(self,firmware_version):
        """
        Starting with Stepper/Wacc/Pimu firmware v0.5.0 a faster version (V1) of the transport layer is supported
        Check here if can run the Transport in V1 mode
        """
        xl = firmware_version.split('.')
        if len(xl) != 4:
            raise Exception('Invalid firmware version len')
        device = xl[0]
        if not (device == 'Stepper' or device == 'Wacc' or device == 'Pimu'):
            raise Exception('Invalid device name ')
        major = int(xl[1][1:])
        minor = int(xl[2])
        if major==0 and minor<=4:
            self.version=RPC_TRANSPORT_VERSION_0
        else:
            self.version=RPC_TRANSPORT_VERSION_1

    def set_version(self,v):
        if v==0:
            self.version = RPC_TRANSPORT_VERSION_0
        if v==1:
            self.version=RPC_TRANSPORT_VERSION_1

    def do_pull_rpc(self,payload,reply_callback,exiting=False):
        """
        Do an RPC that pulls data from the device
        Parameters
        ----------
        payload: Array of type 'B' with length of RPC data to transmit
        reply_callback: Called after RPC data has been returned
        exiting: Cleanup if a final call during exit

        Returns
        -------
        None
        """
        with self.lock:
            self.do_rpc(push=False,payload=payload, reply_callback=reply_callback,exiting=exiting)

    def do_push_rpc(self, payload, reply_callback, exiting=False):
        """
        Do an RPC that pushes data to the device
        Parameters
        ----------
        payload: Array of type 'B' with length of RPC data to transmit
        reply_callback: Called after RPC data has been returned
        exiting: Cleanup if a final call during exit

        Returns
        -------
        None
        """
        with self.lock:
            self.do_rpc(push=True,payload=payload, reply_callback=reply_callback,exiting=exiting)

    def do_rpc(self,push,payload, reply_callback, exiting=False):
        if not self.ser:
            return
        self.status['transactions']+=1
        if exiting:
            time.sleep(0.1) #May have been a hard exit, give time for bad data to land, remove, do final RPC
            self.ser.reset_output_buffer()
            self.ser.reset_input_buffer()
        #This will block until all RPCs have been completed
        try:
            # Now run RPC calls
            if push:
                if self.version==RPC_TRANSPORT_VERSION_0:
                    self.do_transaction_v0(payload,reply_callback)
                elif self.version==RPC_TRANSPORT_VERSION_1:
                    self.do_push_transaction_v1(payload,reply_callback)
            else:
                if self.version == RPC_TRANSPORT_VERSION_0:
                    self.do_transaction_v0(payload,reply_callback)
                elif self.version == RPC_TRANSPORT_VERSION_1:
                    self.do_pull_transaction_v1(payload,reply_callback)
        except IOError as e:
            print("IOError({0}): {1}".format(e.errno, e.strerror))
            self.status['read_error']+=1
        except serial.SerialTimeoutException as e:
            self.status['write_error']+=1
            print("SerialException({0}): {1}".format(e.errno, e.strerror))


    def do_push_transaction_v1(self, rpc_data, rpc_callback):
        """
                Parameters
        ----------
        rpc_data: Buffer of data to transmit in RPC
        rpc_callback: Call back to process RPC reply data
        Returns: True/False if successful
        -------
        Push command data
        Take the rpc_data, break into 64 byte encoded frames, and write to serial.
        Recieve back acks for each frame sent

        RPC data looks like:
        rpc_data = [RPC_ID D0, D1,...] (Len 1024 max)

        The rpc_data is then grouped into one or more 59 byte frames.

        For rpc_data<=59 bytes, an RPC send is straigthforward:

        push_frame_0 = [RPC_PUSH_FRAME_LAST, D0,...DN, CRC1 CRC2] (len 62 max, N<=58)

        If the size of data sent is over 59 bytes, then multiple frames are used. If for example, 150 bytes are sent:

        frame_0 = [RPC_PUSH_FRAME_MORE, , D0,...D58, CRC1 CRC2] (len 62 max)
        frame_1 = [RPC_PUSH_FRAME_MORE, D59,...D117, CRC1 CRC2] (len 62 max)
        frame_2 = [RPC_PUSH_FRAME_LAST, D118,...D149, CRC1 CRC2] (len 62 max)

        Before transmission each frame is first Cobbs encoded as:

        [ OverheadByte 62_bytes_max_encoded DelimiterByte] (Len 64 max)

        A push transaction does not return status data back. It returns a RPC_PUSH_x_ACK to acknowledge that a frame was
        succesfully recieved. It also returns an RPC_ID_ACK for the callback to verify that the correct RPC call was completed.

        reply_frame_0 = [RPC_PUSH_ACK RPC_ID_ACK CRC1 CRC2]

        Finally, reply data is decoded and passed to the rpc_callback
        """

        n_frames = math.ceil(len(rpc_data) / RPC_V1_FRAME_DATA_MAX_BYTES)
        widx = 0
        dbg_buf = ''
        frame_buf_in = self.get_empty_frame()
        frame_buf_out = self.get_empty_frame()
        framer = cobbs_framing.CobbsFraming()
        try:
            for fid in range(n_frames):
                # Build the Nth frame and transmit
                if fid == 0 and n_frames == 1:
                    frame_buf_out[0] = RPC_V1_PUSH_FRAME_FIRST_ONLY
                elif fid == 0 and n_frames > 1:
                    frame_buf_out[0] = RPC_V1_PUSH_FRAME_FIRST_MORE
                elif fid == n_frames - 1:
                    frame_buf_out[0] = RPC_V1_PUSH_FRAME_LAST
                else:
                    frame_buf_out[0] = RPC_V1_PUSH_FRAME_MORE
                nb_frame = min(RPC_V1_FRAME_DATA_MAX_BYTES, len(rpc_data) - widx)
                frame_buf_out[1:nb_frame + 1] = rpc_data[widx:widx + nb_frame]
                widx = widx + nb_frame
                framer.sendFramedData(frame_buf_out, nb_frame + 1, self.ser)
                # Get Ack back
                frame_buf_in[0]=0 #Remove stale data
                crc, nr = framer.receiveFramedData(frame_buf_in, self.ser)
                self.__handle_push_ack_v1(crc,nr,dbg_buf,frame_buf_in[0])
                if frame_buf_in[0]!=RPC_V1_PUSH_ACK:
                    raise TransportError("Byte 0 not RPC_V1_PUSH_ACK")
                if fid == n_frames - 1:
                    rpc_callback(frame_buf_in[1:nr])
            return True
        except TransportError as e:
            if dbg_on:
                print('---- Debug Exception')
                print(dbg_buf)
            self.status['read_error'] += 1
            self.ser.reset_output_buffer()
            self.ser.reset_input_buffer()
            self.logger.error("TransportError: %s : %s" % (self.usb, str(e)))
        except serial.SerialTimeoutException as e:
            self.status['write_error'] += 1
            self.ser = None
            self.logger.error("SerialTimeoutException: %s : %s" % (self.usb, str(e)))
        except serial.SerialException as e:
            self.logger.error("SerialException: %s : %s" % (self.usb, str(e)))
            self.ser = None
        except TypeError as e:
            self.logger.error("TypeError: %s : %s" % (self.usb, str(e)))
            self.ser = None
        return False

    def __handle_push_ack_v1(self, crc,nr,dbg_buf,ack_code):
        """
        Utility function for code readability
        """
        if dbg_on:
            if nr:
                dbg_buf = dbg_buf + 'Framer rcvd on RPC_V1_PUSH_ACK CRC: ' + str(crc) + ' NR: ' + str(nr) + \
                          ' B0: ' + str(ack_code) + ' Expected B0: ' + str(RPC_V1_PUSH_ACK) + ':' + self.usb
            else:
                dbg_buf = dbg_buf + 'Framer rcvd 0 bytes on RPC_PUSH_ACK'
        if crc != 1:
            self.logger.error('Transport CRC Error on RPC_V1_PUSH_ACK {0} {1} {2}'.format(crc, nr, ack_code))
            raise TransportError
        if ack_code != RPC_V1_PUSH_ACK:
            self.logger.error('Transport RX Error on RPC_V1_PUSH_ACK {0} {1} {2}'.format(crc, nr, ack_code))
            raise TransportError

    def __handle_pull_ack_v1(self, crc,nr,dbg_buf,ack_code):
        """
        Utility function for code readability
        """
        if dbg_on:
            if nr:
                dbg_buf = dbg_buf + 'Framer rcvd on RPC_PULL_ACK CRC: ' + str(crc) + ' NR: ' + str(nr) + \
                          ' B0: ' + str(ack_code) + ' Expected B0: ' + str(RPC_V1_PULL_FRAME_ACK_MORE) +\
                          ' OR B0: ' + str(RPC_V1_PULL_FRAME_ACK_LAST) +':' + self.usb
            else:
                dbg_buf = dbg_buf + 'Framer rcvd 0 bytes on RPC_PULL_ACK'
        if crc != 1 :
            self.logger.error('Transport CRC Error on RPC_PUSH_ACK {0} {1} {2}'.format(crc, nr, ack_code))
            raise TransportError
        if ack_code != RPC_V1_PULL_FRAME_ACK_MORE and ack_code != RPC_V1_PULL_FRAME_ACK_LAST:
            self.logger.error('Transport RX Error on RPC_PUSH_ACK {0} {1} {2}'.format(crc, nr, ack_code))
            raise TransportError

    def do_pull_transaction_v1(self,rpc_data, rpc_callback):
        """
                Parameters
        ----------
        rpc_data: Buffer of data to transmit in RPC
        rpc_callback: Call back to process RPC reply data
        Returns: True/False if successful
        -------
        This pulls status data from the device. The frame layout is analogous to do_push_transaction.
        However, here we send down only a pull request (single frame with an RPC_ID).
        We get back one or more frames of status data which is then decoded and passed to the callback.
        """
        dbg_buf = ''
        frame_buf_in = self.get_empty_frame()
        frame_buf_out = self.get_empty_frame()
        framer = cobbs_framing.CobbsFraming()

        try:
            #First initiate a pull transaction
            frame_buf_out[0] = RPC_V1_PULL_FRAME_FIRST
            nb_frame = min(RPC_V1_FRAME_DATA_MAX_BYTES, len(rpc_data))
            frame_buf_out[1:nb_frame + 1] = rpc_data[0:nb_frame]
            framer.sendFramedData(frame_buf_out, nb_frame + 1, self.ser)

            reply = arr.array('B')

            #Next read out the N reply frames (up to 18, if no ACK_LAST, then error
            for i in range(RPC_V1_MAX_FRAMES):
                frame_buf_in[0] = 0  # Remove stale data
                crc, nr = framer.receiveFramedData(frame_buf_in, self.ser)
                self.__handle_pull_ack_v1(crc,nr,dbg_buf,frame_buf_in[0])
                reply = reply + frame_buf_in[1:nr]
                if frame_buf_in[0]==RPC_V1_PULL_FRAME_ACK_LAST:#No more frames to request
                    rpc_callback(reply)
                    return True
                elif frame_buf_in[0]==RPC_V1_PULL_FRAME_ACK_MORE: #More frames to request
                    frame_buf_in[0] = RPC_V1_PULL_FRAME_MORE
                    nb_frame = min(RPC_V1_FRAME_DATA_MAX_BYTES, len(rpc_data))
                    frame_buf_in[1:nb_frame + 1] = rpc_data[0:nb_frame]
                    framer.sendFramedData(frame_buf_in, nb_frame + 1, self.ser)
                else:
                    raise TransportError
            self.logger.error("In do_pull_transaction. Failed to get RPC_V1_PULL_FRAME_ACK_LAST")
            raise TransportError
        except TransportError as e:
            if dbg_on:
                print('---- Debug Exception')
                print(dbg_buf)
            self.status['read_error'] += 1
            self.ser.reset_output_buffer()
            self.ser.reset_input_buffer()
            self.logger.error("TransportError: %s : %s" % (self.usb, str(e)))
        except serial.SerialTimeoutException as e:
            self.status['write_error'] += 1
            self.ser = None
            self.logger.error("SerialTimeoutException: %s : %s" % (self.usb, str(e)))
        except serial.SerialException as e:
            self.logger.error("SerialException: %s : %s" % (self.usb, str(e)))
            self.ser = None
        except TypeError as e:
            self.logger.error("TypeError: %s : %s" % (self.usb, str(e)))
            self.ser = None
        return False



    def do_transaction_v0(self,rpc,rpc_callback): #Handle a single RPC transaction
        dbg_buf = ''
        buf_v0 = arr.array('B', [0] * (COBBS_FRAME_SIZE_V0))
        framer = cobbs_framing.CobbsFraming()

        try:
            if dbg_on:
                dbg_buf=dbg_buf+'--------------- New RPC -------------------------\n'
            ########## Initiate new RPC
            buf_v0[0]=RPC_V0_START_NEW_RPC
            framer.sendFramedData(buf_v0, 1, self.ser)
            if dbg_on:
                dbg_buf=dbg_buf+'Framer sent RPC_V0_START_NEW_RPC\n'
            crc, nr = framer.receiveFramedData(buf_v0, self.ser)
            if dbg_on:
                if nr:
                    dbg_buf=dbg_buf+'Framer rcvd on RPC_V0_ACK_NEW_RPC CRC: '+str(crc)+' NR: '+str(nr)+' B0: '+str(buf_v0[0])+' Expected B0: '+str(RPC_V0_ACK_NEW_RPC) +':'+self.usb
                else:
                    dbg_buf = dbg_buf +'Framer rcvd 0 bytes on RPC_V0_ACK_NEW_RPC'

            if crc!=1 or buf_v0[0] != RPC_V0_ACK_NEW_RPC:
                self.logger.error('Transport RX Error on RPC_V0_ACK_NEW_RPC {0} {1} {2}'.format(crc, nr, buf_v0[0]))
                raise TransportError
            #if dbg_on:
            #    print('New RPC initiated, len',len(rpc))
            ########### Send all blocks
            ntx=0
            while ntx<len(rpc):
                nb=min(len(rpc)-ntx,RPC_V0_BLOCK_SIZE) #Num bytes to send
                b = rpc[ntx:ntx + nb]
                ntx=ntx+nb
                if ntx==len(rpc):#Last block
                    buf_v0[0] = RPC_V0_SEND_BLOCK_LAST
                    buf_v0[1:len(b) + 1] = b
                    #if dbg_on:
                    #    print('Sending last block',ntx)
                    framer.sendFramedData(buf_v0, nb+1, self.ser)
                    if dbg_on:
                        dbg_buf = dbg_buf + 'Framer sent RPC_V0_SEND_BLOCK_LAST\n'
                    #if dbg_on:
                    #    print('Getting last block ack',ntx)

                    crc, nr = framer.receiveFramedData(buf_v0, self.ser)
                    if dbg_on:
                        if nr:
                            dbg_buf = dbg_buf + 'Framer rcvd on RPC_V0_SEND_BLOCK_LAST CRC: ' + str(crc) + ' NR: ' + str(nr) + ' B0: ' + str(buf_v0[0]) + ' Expected B0: ' + str(RPC_V0_ACK_SEND_BLOCK_LAST)+':'+self.usb+'\n'
                        else:
                            dbg_buf = dbg_buf + 'Framer rcvd 0 bytes on RPC_V0_SEND_BLOCK_LAST'
                    #if dbg_on:
                    #    print('Last block ack rcvd',ntx)
                    if crc!=1 or buf_v0[0]!=RPC_V0_ACK_SEND_BLOCK_LAST:
                        self.logger.error('Transport RX Error on RPC_V0_ACK_SEND_BLOCK_LAST {0} {1} {2}'.format(crc, nr, buf_v0[0]))
                        raise TransportError
                else:
                    buf_v0[0] = RPC_V0_SEND_BLOCK_MORE
                    buf_v0[1:len(b) + 1] = b
                    #if dbg_on:
                    #    print('Sending next block',ntx)
                    framer.sendFramedData(buf_v0, nb + 1, self.ser)
                    if dbg_on:
                        dbg_buf = dbg_buf + 'Framer sent RPC_V0_SEND_BLOCK_MORE\n'
                    #if dbg_on:
                    #    print('Sent next block',ntx)
                    crc, nr = framer.receiveFramedData(buf_v0, self.ser)
                    if dbg_on:
                        if nr:
                            dbg_buf = dbg_buf + 'Framer rcvd on RPC_V0_SEND_BLOCK_MORE CRC: ' + str(crc) + ' NR: ' + str(nr) + ' B0: ' + str(buf_v0[0]) + ' Expected B0: ' + str(RPC_V0_ACK_SEND_BLOCK_MORE)+':'+self.usb+'\n'
                        else:
                            dbg_buf = dbg_buf + 'Framer rcvd 0 bytes on RPC_V0_SEND_BLOCK_MORE'
                    if crc!=1 or buf_v0[0]!=RPC_V0_ACK_SEND_BLOCK_MORE:
                        self.logger.error('Transport RX Error on RPC_V0_ACK_SEND_BLOCK_MORE {0} {1} {2}'.format(crc, nr, buf_v0[0]))
                        raise TransportError
            ########### Receive all blocks
            reply = arr.array('B')
            #if dbg_on:
            #    print('Receiving RPC reply')
            while True:
                buf_v0[0] = RPC_V0_GET_BLOCK
                #if dbg_on:
                #    print('Block requested')
                framer.sendFramedData(buf_v0,1, self.ser)
                crc, nr = framer.receiveFramedData(buf_v0, self.ser)
                #if dbg_on:
                #    print('Block request success')
                if crc != 1 or not (buf_v0[0] == RPC_V0_ACK_GET_BLOCK_MORE or buf_v0[0] == RPC_V0_ACK_GET_BLOCK_LAST):
                    self.logger.error('Transport RX Error on RPC_V0_GET_BLOCK {0} {1} {2}'.format(crc, nr, buf_v0[0]))
                    raise TransportError
                reply=reply+buf_v0[1:nr]

                if buf_v0[0] == RPC_V0_ACK_GET_BLOCK_LAST:
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
            self.status['read_error'] += 1
            self.ser.reset_output_buffer()
            self.ser.reset_input_buffer()
            self.logger.error("TransportError: %s : %s" % (self.usb, str(e)))
        except serial.SerialTimeoutException as e:
            self.status['write_error'] += 1
            self.ser=None
            self.logger.error("SerialTimeoutException: %s : %s"%(self.usb, str(e)))
        except serial.SerialException as e:
            self.logger.error("SerialException: %s : %s"%(self.usb, str(e)))
            self.ser=None
        except TypeError as e:
            self.logger.error("TypeError: %s : %s" % (self.usb, str(e)))
            self.ser=None
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



















