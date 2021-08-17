from __future__ import print_function
import stretch_body.transport.rpc
import serial
import fcntl
import errno
import logging

buses = {}
logger = logging.getLogger('transport')


def startup(usb):
    global buses, logger
    if usb in buses:
        logger.debug('Already opened transport on: {0}'.format(usb))
        return True

    logger.debug('Opening transport on: {0}'.format(usb))
    try:
        bus = serial.Serial(usb, write_timeout=1.0)
        fcntl.flock(bus.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        buses[usb] = bus
        return True
    except (serial.SerialException, IOError) as e:
        if e.errno == errno.ENOENT:
            logger.error('Device {0} does not exist. Check cable connections.'.format(usb))
        elif e.errno == errno.EAGAIN:
            logger.error('Device {0} busy. Close other Stretch Body instances.'.format(usb))
            bus.close()

    return False


def stop(usb):
    global buses, logger
    if usb not in buses:
        logger.debug('Transport not open on: {0}'.format(usb))
        return

    logger.debug('Closing transport on: {0}'.format(usb))
    buses[usb].close()
    buses.pop(usb)
