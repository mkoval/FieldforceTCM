#!/usr/bin/env python

import crcmod, serial, struct

class FieldforceTCM:
    kGetModInfo  = 1
    kModInfoResp = 2

    def __init__(self, path):
        self.fp = serial.Serial(
            path:     path,
            baudrate: 38400,
            bytesize: serial.EIGHTBITS,
            parity:   serial.PARITY_NONE,
            stopbits: serial.STOPBITS_ONE
        )
        # CRC-16 with generator polynomial X^16 + X^12 + X^5 + 1.
        self.crc = crcmod.mkCrcFun(0b10001000000100001, 0, False)

    def __enter__(self):
        return self

    def __exit__(self, type, value, traceback):
        self.close()
        return False

    def close(self):
        pass

    def _buildMessage(self, id, payload):
        count = len(payload) + 5
        head = struct.pack('>HB{0}s'.format(len(payload)), count, id, payload)
        tail = struct.pack('>H', self.crc(head))
        return head + tail

    def _sendMessage(self, id, payload):
        msg = self._buildMessage(id, payload)
        self.fp.write(msg)

    def getModInfo(self):
        request  = self._buildMessage(self.kGetModInfo, b'')
        response = struct.unpack(

with FieldforceTCM('/dev/ttyUSB0') as compass:
    msg = compass._getModInfo()
    print msg.encode('hex')
