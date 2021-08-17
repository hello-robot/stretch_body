from __future__ import print_function
import stretch_body.transport as transport
from stretch_body.transport import encoder
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
READ_TIMEOUT = 0.2 # seconds
PACKET_MARKER = 0


def send(method, arguments_format, returns_format, arguments):
    if transport.bus is None:
        return False
    buffer = array.array('B', [0] * (RPC_BLOCK_SIZE*2))
    buffer[0] = RPC_START_NEW_RPC
    encoded_buffer = encoder.cobs_frame(data=buffer, size=1)
    transport.bus.write(encoded_buffer)
    buffer, size = _receive_data()
    if buffer[0] != RPC_ACK_NEW_RPC:
        transport.logger.error('Transport RX Error on RPC_ACK_NEW_RPC')
        return False
    return True


def _receive_data():
    start = time.time()
    rx_buffer = []
    while (time.time() - start) < READ_TIMEOUT:
        nn = transport.bus.inWaiting()
        waiting_bytes = transport.bus.read(nn) if nn > 0 else []
        for byte in waiting_bytes:
            if type(byte) == str: # Py2 needs this, Py3 not
                byte = struct.unpack('B', byte)[0]
            if byte == PACKET_MARKER:
                buffer, size = encoder.cobs_deframe(encoded_data=rx_buffer)
                return buffer, size
            else:
                rx_buffer.append(byte)
