#!/usr/bin/env python

import collections, crcmod, serial, struct, sys
from collections import namedtuple
from serial import Serial
from struct import Struct

class FieldforceTCM:
    Component = namedtuple('Component', [ 'name', 'struct' ])
    ModInfo   = namedtuple('ModInfo', [ 'Type', 'Revision' ])

    struct_uint8   = Struct('>B')
    struct_uint32  = Struct('>I')
    struct_float32 = Struct('>f')
    struct_boolean = Struct('>?')

    # Frame IDs
    kGetModInfo        = 1
    kModInfoResp       = 2
    kSetDataComponents = 3
    kGetData           = 4
    kDataResp          = 5
    kSetConfig         = 6
    kGetConfig         = 7
    kConfigResp        = 8
    kSave              = 9
    kStartCal          = 10
    kStopCal           = 11
    kSetParam          = 12
    kGetParam          = 13
    kParamResp         = 14
    kPowerDown         = 15
    kSaveDone          = 16
    kUserCalSampCount  = 17
    kUserCalScore      = 18
    kSetConfigDone     = 19
    kSetParamDone      = 20
    kStartIntervalMode = 21
    kStopIntervalMode  = 22
    kPowerUp           = 23
    kSetAcqParams      = 24

    # Component IDs
    components = {
        5:  Component('Heading',     struct_float32),
        7:  Component('Temperature', struct_float32),
        8:  Component('Distortion',  struct_boolean),
        9:  Component('CalStatus',   struct_boolean),
        21: Component('PAligned',    struct_float32),
        22: Component('RAligned',    struct_float32),
        23: Component('IZAligned',   struct_float32),
        24: Component('PAngle',      struct_float32),
        25: Component('RAngle',      struct_float32),
        27: Component('XAligned',    struct_float32),
        28: Component('YAligned',    struct_float32),
        29: Component('ZAligned',    struct_float32)
    }

    # Config IDs
    kDeclination         = 1
    kTrueNorth           = 2
    kBigEndian           = 6
    kMountingRef         = 10
    kUserCalNumPoints    = 12
    kUserCalAutoSampling = 13
    kBaudRate            = 14
    kMilOutput           = 15
    kDataCal             = 16
    kCoeffCopySet        = 18
    kAccelCoeffCopySet   = 19

    config = {
        1:  Component('Declination',         struct_float32),
        2:  Component('TrueNorth',           struct_boolean),
        6:  Component('BigEndian',           struct_boolean),
        10: Component('MountingRef',         struct_uint8),
        12: Component('UserCalNumPoints',    struct_uint32),
        13: Component('UserCalAutoSampling', struct_boolean),
        14: Component('BaudRate',            struct_uint8),
        15: Component('MilOutput',           struct_boolean),
        16: Component('DataCal',             struct_boolean),
        18: Component('CoeffCopySet',        struct_uint32),
        19: Component('AccelCoeffCopySet',   struct_uint32)
    }

    def __init__(self, path):
        self.fp = Serial(
            port     = path,
            baudrate = 38400,
            bytesize = serial.EIGHTBITS,
            parity   = serial.PARITY_NONE,
            stopbits = serial.STOPBITS_ONE
        )
        # CRC-16 with generator polynomial X^16 + X^12 + X^5 + 1.
        self.crc = crcmod.mkCrcFun(0b10001000000100001, 0, False)

    def __enter__(self):
        return self

    def __exit__(self, type, value, traceback):
        self.close()
        return False

    def close(self):
        self.fp.close()

    def _send(self, fmt):
        self.fp.write(fmt)

    def _sendStruct(self, fmt, *args):
        data = fmt.pack(*args)
        self.fp.write(data)

    def _recv(self, fmt):
        struct_fmt = fmt if type(fmt) == Struct else Struct(fmt)
        data = self.fp.read(struct_fmt.size)
        return struct_fmt.unpack(data)

    def _sendMessage(self, frame_id, payload):
        count = len(payload) + 5
        head = struct.pack('>HB{0}s'.format(len(payload)), count, frame_id, payload)
        tail = struct.pack('>H', self.crc(head))
        self._send(head + tail)

    def _recvMessage(self):
        (count, ) = self._recv('>H')
        payload_count = count - 5
        fid, payload, crc = self._recv('>B{0}sH'.format(payload_count))

        check = struct.pack('>HB{0}s'.format(payload_count), count, fid, payload)
        crc_check = self.crc(check)

        if crc == crc_check:
            return fid, payload
        else:
            raise IOError('CRC-16 checksum failed.')

    def _recvSpecificMessage(self, expected_frame_id):
        frame_id, data = self._recvMessage()

        if frame_id == expected_frame_id:
            return data
        else:
            raise IOError('Response has unexpected frame type.')

    def getModInfo(self):
        self._sendMessage(self.kGetModInfo, b'')
        payload = self._recvSpecificMessage(self.kModInfoResp)
        return self.ModInfo(*struct.unpack('>4s4s', payload))

    def getData(self):
        self._sendMessage(self.kGetData, b'')
        payload = self._recvSpecificMessage(self.kDataResp)

        (comp_count, ) = struct.unpack('>B', payload[0])
        comp_index = 0
        offset = 1
        data = dict()

        while comp_index < comp_count:
            (component_id, ) = struct.unpack('>B', payload[offset])
            component        = self.components[component_id]

            datum = payload[(offset + 1):(offset + component.struct.size + 1)]
            (value, ) = component.struct.unpack(datum)
            data[component.name] = value

            offset     += 1 + component.struct.size
            comp_index += 1

        return data

    def setConfig(self, config_id, value):
        payload_id    = self.struct_uint8.pack(config_id)
        payload_value = self.config[config_id].struct.pack(value)
        self._sendMessage(self.kSetConfig, payload_id + payload_value)
        self._recvSpecificMessage(self.kSetConfigDone)

    def getConfig(self, config_id):
        payload_id = self.struct_uint8.pack(config_id)
        self._sendMessage(self.kGetConfig, payload_id)

        response = self._recvSpecificMessage(self.kConfigResp)
        (response_id, ) = self.struct_uint8.unpack(response[0])

        if response_id == config_id:
            (value, ) = self.config[config_id].struct.unpack(response[1:])
            return value
        else:
            raise IOError('Response has unexpected configuration ID.')

def main():
    with FieldforceTCM('/dev/ttyUSB0') as compass:
        print 'ModInfo:     ', compass.getModInfo()
        print 'Data:        ', compass.getData()
        print 'Declination: ', compass.getConfig(compass.kDeclination)

if __name__ == '__main__':
    sys.exit(main())

# vim: set et sw=4 ts=4:
