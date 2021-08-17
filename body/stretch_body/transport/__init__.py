from __future__ import print_function
import stretch_body.transport.rpc
import serial
import fcntl
import errno
import logging

bus = None
logger = logging.getLogger('transport')


def start(usb):
    global bus, logger
    logger.debug('Opening transport on: {0}'.format(usb))
    try:
        bus = serial.Serial(usb, write_timeout=1.0)
        fcntl.flock(bus.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except (serial.SerialException, IOError) as e:
        if e.errno == errno.ENOENT:
            logger.error('Device {0} does not exist. Check cable connections.'.format(usb))
        elif e.errno == errno.EAGAIN:
            logger.error('Device {0} busy. Close other Stretch Body instances.'.format(usb))
            bus.close()
        bus = None
    return bus is not None


def stop():
    global bus, logger
    if bus is not None:
        logger.debug('Closing transport on: {0}'.format(bus.port))
        bus.close()
        bus = None
