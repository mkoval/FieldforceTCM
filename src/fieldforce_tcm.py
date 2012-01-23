#!/usr/bin/env python
"""
Copyright (c) 2012, Michael Koval
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

import collections, crcmod, serial, struct, sys
from collections import namedtuple
from serial import Serial
from struct import Struct

class FrameID:
    kGetModInfo         = 1
    kModInfoResp        = 2
    kSetDataComponents  = 3
    kGetData            = 4
    kDataResp           = 5
    kSetConfig          = 6
    kGetConfig          = 7
    kConfigResp         = 8
    kSave               = 9
    kStartCal           = 10
    kStopCal            = 11
    kSetParam           = 12
    kGetParam           = 13
    kParamResp          = 14
    kPowerDown          = 15
    kSaveDone           = 16
    kUserCalSampCount   = 17
    kUserCalScore       = 18
    kSetConfigDone      = 19
    kSetParamDone       = 20
    kStartIntervalMode  = 21
    kStopIntervalMode   = 22
    kPowerUp            = 23
    kSetAcqParams       = 24
    kGetAcqParams       = 25
    kAcqParamsDone      = 26
    kAcqParamsResp      = 27
    kPowerDownDone      = 28
    kFactoryUserCal     = 29
    kFactorUserCalDone  = 30
    kTakeUserCalSample  = 31
    kFactoryInclCal     = 36
    kFactoryInclCalDone = 37
    kSetMode            = 46
    kSetModeResp        = 47
    kSyncRead           = 49

class Component:
    kHeading     = 5
    kTemperature = 7
    kDistortion  = 8
    kCalStatus   = 9
    kPAligned    = 21
    kRAligned    = 22
    kIZAligned   = 23
    kPAngle      = 24
    kRAngle      = 25
    kXAligned    = 27
    kYAligned    = 28
    kZAligned    = 29

class Configuration:
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

class Orientation:
    kOrientationSTD0     = 1
    kOrientationXUP0     = 2
    kOrientationYUP0     = 3
    kOrientationSTD90    = 4
    kOrientationSTD180   = 5
    kOrientationSTD270   = 6
    kOrientationZDOWN0   = 7
    kOrientationXUP90    = 8
    kOrientationXUP180   = 9
    kOrientationXUP270   = 10
    kOrientationYUP90    = 11
    kOrientationYUP180   = 12
    kOrientationYUP270   = 13
    kOrientationZDOWN90  = 14
    kOrientationZDOWN180 = 15
    kOrientationZDOWN270 = 16

class Calibration:
    kFullRangeCalibration     = 10
    k2DCalibration            = 20
    kHardIronCalibration      = 30
    kLimitedTiltCalibraion    = 40
    kAccelCalibration         = 100
    kAccelMagneticCalibration = 110

class FieldforceTCM:
    Component = namedtuple('Component', [
        'name', 'struct'
    ])
    ModInfo   = namedtuple('ModInfo', [
        'Type', 'Revision'
    ])
    CalScores = namedtuple('CalScores', [
        'CalScore', 'CalParam2', 'AccelCalScore', 'DistError',
        'TiltError', 'TiltRange'
    ])
    AcqParams = namedtuple('AcqParams', [
        'PollingMode', 'FlushFilter', 'SensorAcqTime', 'IntervalRespTime'
    ])
    Datum     = namedtuple('Datum', [
        'Heading', 'Temperature', 'Distortion', 'CalStatus',
        'PAligned', 'RAligned', 'IZAligned',
        'PAngle', 'RAngle', 'KXAligned', 'KYAligned', 'KZAligned'
    ])

    struct_uint8   = Struct('>B')
    struct_uint16  = Struct('>H')
    struct_uint32  = Struct('>I')
    struct_float32 = Struct('>f')
    struct_boolean = Struct('>?')

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

    fir_defaults = {
        0:  [ ],
        4:  [ 4.6708657655334e-2, 4.5329134234467e-1,
              4.5329134234467e-1, 4.6708657655334e-2 ],
        8:  [ 1.9875512449729e-2, 6.4500864832660e-2,
              1.6637325898141e-1, 2.4925036373620e-1,
              2.4925036373620e-1, 1.6637325898141e-1,
              6.4500864832660e-2, 1.9875512449729e-2 ],
        16: [ 7.9724971069144e-3, 1.2710056429342e-2,
              2.5971390034516e-2, 4.6451949792704e-2,
              7.1024151197772e-2, 9.5354386848804e-2,
              1.1484431942626e-1, 1.2567124916369e-1,
              1.2567124916369e-1, 1.1484431942626e-1,
              9.5354386848804e-2, 7.1024151197772e-2,
              4.6451949792704e-2, 2.5971390034516e-2,
              1.2710056429342e-2, 7.9724971069144e-3 ],
        32: [ 1.4823725958818e-3, 2.0737124095482e-3,
              3.2757326624196e-3, 5.3097803863757e-3,
              8.3414139286254e-3, 1.2456836057785e-2,
              1.7646051430536e-2, 2.3794805168613e-2,
              3.0686505921968e-2, 3.8014333463472e-2,
              4.5402682509802e-2, 5.2436112653103e-2,
              5.8693165018301e-2, 6.3781858267530e-2,
              6.7373451424187e-2, 6.9231186101853e-2,
              6.9231186101853e-2, 6.7373451424187e-2,
              6.3781858267530e-2, 5.8693165018301e-2,
              5.2436112653103e-2, 4.5402682509802e-2,
              3.8014333463472e-2, 3.0686505921968e-2,
              2.3794805168613e-2, 1.7646051430536e-2,
              1.2456836057785e-2, 8.3414139286254e-3,
              5.3097803863757e-3, 3.2757326624196e-3,
              2.0737124095482e-3, 1.4823725958818e-3 ]
    }

    def __init__(self, path, baud):
        self.fp = Serial(
            port     = path,
            baudrate = baud,
            bytesize = serial.EIGHTBITS,
            parity   = serial.PARITY_NONE,
            stopbits = serial.STOPBITS_ONE
        )
        # CRC-16 with generator polynomial X^16 + X^12 + X^5 + 1.
        self.crc = crcmod.mkCrcFun(0b10001000000100001, 0, False)

    def close(self):
        self.fp.flush()
        self.fp.close()

    def _send(self, fmt):
        self.fp.write(fmt)

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
            raise IOError('Response has unexpected frame id: {0}.'.format(frame_id))

    def _createDatum(self, data):
        for component in self.Datum._fields:
            if component not in data.keys():
                data[component] = None
        return self.Datum(**data)

    def getModelInfo(self):
        """
        Query the module's type and firmware revision number.
        """
        self._sendMessage(FrameID.kGetModInfo, b'')
        payload = self._recvSpecificMessage(FrameID.kModInfoResp)
        return self.ModInfo(*struct.unpack('>4s4s', payload))

    def getData(self):
        """
        Query a single packet of data that containing the components specified
        by setDataComponents(). All other components are set to zero.
        """
        self._sendMessage(FrameID.kGetData, b'')
        payload = self._recvSpecificMessage(FrameID.kDataResp)

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

        return self._createDatum(data)

    def setConfig(self, config_id, value):
        """
        Sets a single configuration value based on the config_id, which must be
        one of the values in Configuration. Acceptable values deend upon the
        configuration option being set.
        """
        payload_id    = self.struct_uint8.pack(config_id)
        payload_value = self.config[config_id].struct.pack(value)
        self._sendMessage(FrameID.kSetConfig, payload_id + payload_value)
        self._recvSpecificMessage(FrameID.kSetConfigDone)

    def getConfig(self, config_id):
        """
        Get the value of a single configuration value based on config_id.
        """
        payload_id = self.struct_uint8.pack(config_id)
        self._sendMessage(FrameID.kGetConfig, payload_id)

        response = self._recvSpecificMessage(FrameID.kConfigResp)
        (response_id, ) = self.struct_uint8.unpack(response[0])

        if response_id == config_id:
            (value, ) = self.config[config_id].struct.unpack(response[1:])
            return value
        else:
            raise IOError('Response has unexpected configuration id: {0}.'
                           .format(response_id))

    def setFilter(self, count, values=None):
        """
        Configure the number of taps and weights of the on-board finite impulse
        response (FIR) filter. The number of taps must be zero (i.e. disabled),
        4, 8, 16, or 32. If values is omitted or is set to None, the weights
        default to PNI's recommended values. See the Fieldforce TCM User Manual
        for details.
        """
        assert count in [ 0, 4, 8, 16, 32 ]

        if values == None:
            values = self.fir_defaults[count]
        else:
            assert len(values) == count

        payload = struct.pack('>BBB{0}d'.format(count), 3, 1, count, *values)
        self._sendMessage(FrameID.kSetParam, payload)
        self._recvSpecificMessage(FrameID.kSetParamDone)

    def getFilter(self):
        """
        Gets the current finite impulse response (FIR) filter weights. See
        setFilter() for more information.
        """
        payload_request  = struct.pack('>BB', 3, 1)
        self._sendMessage(FrameID.kGetParam, payload_request)

        payload_response = self._recvSpecificMessage(FrameID.kParamResp)
        param_id, axis_id, count = struct.unpack('>BBB', payload_response[0:3])

        if param_id != 3:
            raise IOError('Expected param ID of 3, got {0}'.format(param_id))
        elif axis_id != 1:
            raise IOError('Expected axis ID of 1, got {0}'.format(axis_id))

        fir = struct.unpack('>{0}d'.format(count), payload_response[3:])
        return list(fir)

    def setDataComponents(self, components):
        """
        Specify which data components, specified as a list of component IDs,
        will be returned with each sample. An arbitrary number of IDs is
        supported.
        """
        count = len(components)
        payload_counts  = struct.pack('>B', count)
        payload_content = struct.pack('>{0}B'.format(count), *components)
        payload = payload_counts + payload_content
        self._sendMessage(FrameID.kSetDataComponents, payload)

    def setAcquisitionParams(self, mode, flush_filter, acq_time, resp_time):
        """
        Set the acquisition mode, including:
         - PollingMode: poll if true, push if false
         - FlushFilter: flush the FIR filter registers after each sample
         - SensorAcqTime: time between sample in seconds
         - IntervalRespTime: time delay between sending subsequent samples
        Even if polling is enabled here, it must be explicitly started using
        startStreaming().
        """
        payload = struct.pack('>BBff', mode, flush_filter, acq_time, resp_time)
        self._sendMessage(FrameID.kSetAcqParams, payload)
        self._recvSpecificMessage(FrameID.kAcqParamsDone)

    def getAcquisitionParams(self):
        """
        Gets the current acquisition mode. See setAcquisitionParams() for more
        information.
        """
        self._sendMessage(FrameID.kGetAcqParams, b"")
        payload  = self._recvSpecificMessage(FrameID.kAcqParamsResp)
        response = struct.unpack('>BBff', payload)
        return self.AcqParams(*response)

    def startStreaming(self):
        """
        Start streaming data. See setAcquisitionParams() for more information
        and use stopStreaming() when done. Streaming must be stopped before any
        other commands can be used.
        """
        self._sendMessage(FrameID.kStartIntervalMode, b'')

    def stopStreaming(self):
        """
        Stops streaming data; companion of startStreaming(). Streaming must be
        stopped before any other commands can be used.
        """
        self._sendMessage(FrameID.kStopIntervalMode, b'')
        self.fp.flushInput()

    def powerUp(self):
        """
        Power up the sensor after a powerDown(). This has undefined results if
        the sensor is already powered on.
        """
        self._send(b'\xFF')
        self._recvSpecificMessage(FrameID.kPowerUp)

    def powerDown(self):
        """
        Power down the sensor down.
        """
        self._sendMessage(FrameID.kPowerDown, b'')
        self._recvSpecificMessage(FrameID.kPowerDownDone)

    def save(self):
        """
        Write the current configuration to non-volatile memory. Note that this
        is the only command that writes to non-volatile memory, so it should be
        paired with any configuration options (e.g. calibration) that are
        intended to be persistant.
        """
        self._sendMessage(FrameID.kSave, b'')
        response = self._recvSpecificMessage(FrameID.kSaveDone)
        (code, ) = self.struct_uint16.unpack(response)

        if code != 0:
            raise IOError('Save failed with error code {0}.'.format(code))

    def startCalibration(self, mode):
        """
        Starts calibration. See the FieldForce TCM User Manual for details
        about the necessary setup for each calibration mode.
        """
        payload_mode = self.struct_uint32.pack(mode)
        self._sendMessage(FrameID.kStartCal, payload_mode)

    def getCalibrationStatus(self):
        """
        Blocks waiting for a calibration update from the sensor. This returns a
        tuple where the first elements is a boolean that indicates whether the
        calibration is complete.
        """
        while True:
            frame_id, message = self._recvMessage()

            # One UserCalSampCount message is generated for each recorded
            # sample. This continues until the calibration has converged or the
            # maximum number of points have been collected.
            if frame_id == FrameID.kUserCalSampCount:
                (sample_num, ) = self.struct_uint32.unpack(message)
                return (False, sample_num)
            # Calibration accuracy is reported in a single UserCalScore message
            # once calibration is complete.
            elif frame_id == FrameID.kUserCalScore:
                scores_raw = struct.unpack('>6f', message)
                scores     = self.CalScores(*scores_raw)
                return (True, scores)
            # Ignore data updates
            elif frame_id == FrameID.kDataResp:
                continue
            else:
                raise IOError('Unexpected frame id: {0}.'.format(frame_id))

# vim: set et sw=4 ts=4:
