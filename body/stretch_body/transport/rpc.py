from __future__ import print_function
import stretch_body.transport as transport
from stretch_body.transport import cobs_encoder as encoder
import time
import array

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
RPC_ACK_ERROR = -1
READ_TIMEOUT = 0.2 # seconds
PACKET_MARKER = 0


def send(usb, call_type, reply_type, call_struct, reply_struct, call_dict, clear_buffers=False):
    if usb not in transport.buses:
        transport.logger.debug('Transport not open on: {0}'.format(usb))
        return False

    # discard previous rpcs
    if clear_buffers:
        time.sleep(0.1) # wait for old data to arrive
        transport.buses[usb].reset_output_buffer()
        transport.buses[usb].reset_input_buffer()

    # initiate a new RPC
    _transact(usb=usb, data=[RPC_START_NEW_RPC], expected_ack=RPC_ACK_NEW_RPC)
    # encoded_buffer = encoder.frame(data=[RPC_START_NEW_RPC])
    # transport.buses[usb].write(encoded_buffer)
    # buffer = _receive(usb)
    # if buffer[0] != RPC_ACK_NEW_RPC:
    #     transport.logger.error('Transport receive error on: RPC_ACK_NEW_RPC')
    #     return False

    # send call data in batches
    data = [call_type]
    for i in range(0, len(data), RPC_BLOCK_SIZE):
        data_block = data[i:i+RPC_BLOCK_SIZE]
        last_block = len(data) <= i + RPC_BLOCK_SIZE
        call_block_type = RPC_SEND_BLOCK_LAST if last_block else RPC_SEND_BLOCK_MORE
        reply_block_type = RPC_ACK_SEND_BLOCK_LAST if last_block else RPC_ACK_SEND_BLOCK_MORE

        encoded_buffer = encoder.frame(data=[call_block_type] + data_block)
        transport.buses[usb].write(encoded_buffer)
        buffer = _receive(usb)
        if buffer[0] != reply_block_type:
            transport.logger.error('Transport receive error on: RPC_ACK_SEND_BLOCK_MORE/LAST')
            return False

    # receive reply data in batches
    encoded_buffer = encoder.frame(data=[RPC_GET_BLOCK])
    transport.buses[usb].write(encoded_buffer)
    buffer = _receive(usb)
    if buffer[0] != RPC_ACK_GET_BLOCK_MORE and buffer[0] != RPC_ACK_GET_BLOCK_LAST:
        transport.logger.error('Transport receive error on: RPC_ACK_GET_BLOCK_MORE/LAST')
        return False
    received_data = buffer
    while buffer[0] == RPC_ACK_GET_BLOCK_MORE:
        encoded_buffer = encoder.frame(data=[RPC_GET_BLOCK])
        transport.buses[usb].write(encoded_buffer)
        buffer = _receive(usb)
        if buffer[0] != RPC_ACK_GET_BLOCK_MORE and buffer[0] != RPC_ACK_GET_BLOCK_LAST:
            transport.logger.error('Transport receive error on: RPC_ACK_GET_BLOCK_MORE/LAST')
            return False
        received_data += buffer

    return True


def _transact(usb, data, expected_ack):
    encoded_buffer = encoder.frame(data=data)
    transport.buses[usb].write(encoded_buffer)
    received_buffer = _receive(usb)
    if received_buffer[0] != expected_ack:
        transport.logger.error('Transport receive error on: {0}'.format(expected_ack))
        raise TransportError
    return received_buffer


def _receive(usb):
    start = time.time()
    rx_buffer = []
    while (time.time() - start) < READ_TIMEOUT:
        nn = transport.buses[usb].in_waiting
        waiting_bytes = transport.buses[usb].read(nn) if nn > 0 else []
        for byte in waiting_bytes:
            if type(byte) == str: # Py2 needs this, Py3 not
                byte = struct.unpack('B', byte)[0]
            if byte == PACKET_MARKER:
                buffer, size = encoder.deframe(encoded_data=rx_buffer)
                return buffer
            else:
                rx_buffer.append(byte)
    return [RPC_ACK_ERROR]
