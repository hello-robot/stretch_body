from __future__ import print_function
from array import array


#Based on COBS.h from
#https://github.com/bakercp/PacketSerial
#MIT License
#Copyright (c) 2017 Christopher Baker https://christopherbaker.net

class CobbsFraming():
    """
    Encoding for communications
    """
    def __init__(self):
        pass

    def encode_data(self, data):
        """
        Encode data (len nb bytes)
        Append CRC first
        Return buffer of encoded data
        """
        crc=self._calc_crc(data,len(data))
        data.append((crc>>8)&0xFF)
        data.append(crc & 0xFF)
        encoded_data=self._encode(data)
        encoded_data.append(0x00)
        return encoded_data

    def decode_data(self,data):
        """
        Decode data into decode buffer
        Check CRC
        Return crc ok, num bytes in decoded buffer
        """
        crc1, nr, decode_buffer = self._decode(data)
        crc2 = self._calc_crc(decode_buffer, nr)
        return crc1 == crc2, nr, decode_buffer

    # ######################################

    def _calc_crc(self, buf, nr): #Modbus CRC
        crc = 0xFFFF
        for i in range(nr):
            crc ^= buf[i]
            for i in range(8):
                if ((crc & 1) != 0):
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        return crc


    def _encode(self,data):
        """
        Cobbs encode the data buffer of nb bytes
        Return encoded data
        """
        nb=len(data)
        read_index = 0
        write_index = 1
        code_index = 0
        code = 1
        encode_buffer=array('B',[0]*2*nb)
        while (read_index < nb):
            if (data[read_index] == 0):
                encode_buffer[code_index] = code
                code = 1
                code_index = write_index
                write_index=write_index+1
                read_index=read_index+1
            else:
                encode_buffer[write_index]=data[read_index]
                read_index=read_index+1
                write_index=write_index+1
                code=code+1
                if (code == 0xFF):
                    encode_buffer[code_index] = code
                    code = 1
                    code_index = write_index
                    write_index=write_index+1
        encode_buffer[code_index] = code
        return encode_buffer[:write_index]

    def _decode(self, data):
        #return crc valid, num bytes in decode buffer, decoded data
        nb=len(data)
        if nb==0:
            return 0,0
        read_index=0
        write_index=0
        code =0
        decode_buffer=array('B',[0]*2*nb)
        while read_index < nb:
            code = data[read_index]
            if (read_index + code > nb and code != 1):
                return 0,0
            read_index=read_index+1
            for i in range(1,code):
                decode_buffer[write_index]=data[read_index]
                read_index=read_index+1
                write_index=write_index+1
            if (code != 0xFF and read_index != nb):
                decode_buffer[write_index]=0
                write_index=write_index+1
        crc = (decode_buffer[write_index- 2]<<8)|decode_buffer[write_index-1]
        return crc, write_index-2, decode_buffer[:write_index-2]