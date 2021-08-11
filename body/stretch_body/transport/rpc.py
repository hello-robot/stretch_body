from __future__ import print_function
from transport import encoder
import time
import array
import fcntl
import serial
import logging

# methods
RPC_START_NEW_RPC =100
RPC_ACK_NEW_RPC= 101
RPC_SEND_BLOCK_MORE = 102
RPC_ACK_SEND_BLOCK_MORE = 103
RPC_SEND_BLOCK_LAST = 104
RPC_ACK_SEND_BLOCK_LAST = 105
RPC_GET_BLOCK = 106
RPC_ACK_GET_BLOCK_MORE = 107
RPC_ACK_GET_BLOCK_LAST = 108

# other
RPC_BLOCK_SIZE = 32
RPC_DATA_SIZE = 1024
READ_TIMEOUT = 0.2 # seconds
PACKET_MARKER = 0

bus = None
logger = logging.getLogger()


def start(usb):
    global bus, logger
    logger.warn('Starting transport on: ' + usb)
    try:
        bus = serial.Serial(usb, write_timeout=1.0)
        if bus.isOpen():
            try:
                fcntl.flock(bus.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            except IOError:
                logger.error('Port %s is busy. Check if another Stretch Body process is already running' % usb)
                bus.close()
                bus = None
    except serial.SerialException as e:
        logger.error("SerialException({0}): {1}".format(e.errno, e.strerror))
        bus = None


def stop():
    global bus, logger
    if bus is not None:
        logger.warn('Shutting down transport')
        bus.close()
        bus = None


def send(method, arguments_format, returns_format, arguments):
    global bus, logger
    if bus is None:
        return
    buffer = array.array('B', [0] * (RPC_BLOCK_SIZE*2))
    buffer[0] = RPC_START_NEW_RPC
    encoded_buffer = encoder.cobs_frame(data=buffer, size=1)
    bus.write(encoded_buffer)
    buffer, size = _receive_data()
    if buffer[0] != RPC_ACK_NEW_RPC:
        logger.error('Transport RX Error on RPC_ACK_NEW_RPC')
        return


def _receive_data():
    global bus, logger
    start = time.time()
    rx_buffer = []
    while (time.time() - start) < READ_TIMEOUT:
        nn = bus.inWaiting()
        waiting_bytes = bus.read(nn) if nn > 0 else []
        for byte in waiting_bytes:
            if type(byte) == str: # Py2 needs this, Py3 not
                byte = struct.unpack('B', byte)[0]
            if byte == PACKET_MARKER:
                buffer, size = encoder.cobs_deframe(encoded_data=rx_buffer)
                return buffer, size
            else:
                rx_buffer.append(byte)
