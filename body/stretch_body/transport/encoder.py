from __future__ import print_function
import time
import array
import struct

# other
RPC_BLOCK_SIZE = 32


def cobs_frame(data, size):
    crc = calculate_crc(data, size)
    data[size] = (crc >> 8) & 0xFF
    data[size + 1] = (crc) & 0xFF
    size += 2
    encoded_data = cobs_encode(data, size)
    encoded_data.append(0x00)
    return encoded_data


def cobs_deframe(encoded_data):
    crc1, data, size = cobs_decode(encoded_data, len(encoded_data))
    crc2 = calculate_crc(data, size)
    if crc1 != crc2:
        print("ERROR!!!!! CRC1 not equal CRC2")
        return False
    return data, size


def calculate_crc(data, size):
    crc = 0xFFFF
    for i in range(size):
        crc ^= data[i]
        for i in range(8):
            if ((crc & 1) != 0):
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    return crc


def cobs_encode(data, size):
    read_index = 0
    write_index = 1
    code_index = 0
    code = 1
    encode_buffer = [0] * (2 * size)
    while (read_index < size):
        if (data[read_index] == 0):
            encode_buffer[code_index] = code
            code = 1
            code_index = write_index
            write_index += 1
            read_index += 1
        else:
            encode_buffer[write_index] = data[read_index]
            read_index += 1
            write_index += 1
            code += 1
            if (code == 0xFF):
                encode_buffer[code_index] = code
                code = 1
                code_index = write_index
                write_index += 1
    encode_buffer[code_index] = code
    return encode_buffer[:write_index]


def cobs_decode(encoded_data, size):
    buffer = array.array('B', [0] * (RPC_BLOCK_SIZE*2))
    if size == 0:
        return (0, buffer, 0)

    read_index = 0
    write_index = 0
    code = 0
    while read_index < size:
        code = encoded_data[read_index]
        if (read_index + code > size and code != 1):
            return (0, buffer, 0)

        read_index += 1
        for i in range(1, code):
            buffer[write_index] = encoded_data[read_index]
            read_index += 1
            write_index += 1
        if (code != 0xFF and read_index != size):
            buffer[write_index] = 0
            write_index += 1
    crc = (buffer[write_index - 2] << 8) | buffer[write_index - 1]
    return (crc, buffer, write_index-2)
