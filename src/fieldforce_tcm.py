#!/usr/bin/env python
# vim: set fileencoding=utf-8 :
"""
Copyright (c) 2012, Michael Koval
Copyright (c) 2012, Cody Schafer <cpschafer --- gmail.com>
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

import collections, crcmod, serial, struct, sys, threading
from time import time as _time
from collections import namedtuple
from decorator import decorator
from serial import Serial
from struct import Struct

_crc_ccitt = crcmod.mkCrcFun(0b10001000000100001, 0, False)

def encode_frame(byteb):
    l = len(byteb) + 4
    pkt = bytearray()
    pkt.extend(struct.pack('>H', l))
    pkt.extend(byteb)
    crc = _crc_ccitt(bytes(pkt))
    pkt.extend(struct.pack('>H', crc))
    return pkt

def encode_command(frame_id, payload = b''):
    print 'encoding {0}'.format(frame_id)
    pkt = bytearray()
    pkt.append(frame_id)
    pkt.extend(payload)
    return encode_frame(pkt)

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
FrameID.__dict__['invert'] = dict([(v, k) for (k, v) in FrameID.__dict__.iteritems()])

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
    STD_0      = 1
    X_UP_0     = 2
    Y_UP_0     = 3
    STD_90     = 4
    STD_180    = 5
    STD_270    = 6
    Z_DOWN_0   = 7
    X_UP_90    = 8
    X_UP_180   = 9
    X_UP_270   = 10
    Y_UP_90    = 11
    Y_UP_180   = 12
    Y_UP_270   = 13
    Z_DOWN_90  = 14
    Z_DOWN_180 = 15
    Z_DOWN_270 = 16

class Calibration:
    kFullRangeCalibration     = 10
    k2DCalibration            = 20
    kHardIronCalibration      = 30
    kLimitedTiltCalibraion    = 40
    kAccelCalibration         = 100
    kAccelMagneticCalibration = 110

class _one_msg_stall:
    def __init__(self, *frame_ids):
        self.cond = threading.Condition()
        self.data = None
        self.frame_ids = frame_ids

    def cb(self):
        def real_cb(pkts):
            c = self.cond
            frame_ids = self.frame_ids

            c.acquire()
            if self.data != None:
                c.release()
                return True # we are no longer waiting
            c.release()

            for p in pkts:
                if p[0] in frame_ids:
                    c.acquire()
                    self.data = bytes(p)
                    c.notify()
                    c.release()
                    return True # remove us.
            return False # haven't got our packet yet.
        return real_cb

    def wait(self, timeout=None):
        c = self.cond
        c.acquire()
        if timeout != None:
            start_time = _time()
        while self.data == None:
            c.wait(timeout)
            # required due to http://bugs.python.org/issue1175933
            if timeout != None and (_time() - start_time) > timeout:
                c.release()
                return None
        it = self.data
        c.release()
        return it

class TimeoutException(Exception):
    def __init__(self, msg, time=None):
        Exception.__init__(self, msg)
        self.time=time

class FieldforceTCM:
    Component = namedtuple('Component', [
        'name', 'struct'
    ])
    ModInfo   = namedtuple('ModInfo', [
        'Type', 'Revision'
    ])
    CalScores = namedtuple('CalScores', [
        'MagCalScore', 'CalParam2', 'AccelCalScore', 'DistError',
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

    good_cal_score = CalScores('< 1', 'ignore (pni reserved)', '< 1',
            '< 1', '< 1', 'angle of tilt' )

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

    def reader(self):
        """
        factory which returns a callable object suitable (for example)
        to pass to threading.Thread(target=reader()).
        """
        def do_it():
            while True:
                self._wait_and_read_all()
                rdy_pkts = self._decode()
                if rdy_pkts:
                    self._notify_listeners(rdy_pkts)
        return do_it

    def _wait_and_read_all(self):
        s = self.fp
        b = self.recv_buf
        b.append(s.read())
        wait_ct = s.inWaiting()
        if wait_ct > 0:
            b.extend(s.read(wait_ct))

    def _notify_listeners(self, rdy_pkts):
        cbs = self.recv_cbs
        lock = self.cb_lock
        lock.acquire()
        x = []
        for i in range(0, len(cbs)):
            if cbs[i](rdy_pkts):
                # A callback returning true is removed.
                # XXX: Can't remove while iterating over it.
                x.append(i)
        for it in x:
            del cbs[it]
        lock.release()

    def _decode(self):
        """
        Given self.recv_buf and self.crc, attempts to decode a valid packet,
        advancing by a single byte on each decoding failure.
        """
        b = self.recv_buf
        crc_fn = self.crc
        decode_pos  = 0
        discard_amt = 0
        good_pos    = 0 # discard before this
        decode_len  = len(b)
        rdy_pkts = []

        # min packet = 2 (byte_count) + 1 (frame id) + 2 (crc) = 5
        #attempt_ct = 0
        #print decode_len
        while decode_len >= 5:
            #attempt_ct += 1
            #print '--decode attempt {0}'.format(attempt_ct)
            (byte_count, ) = struct.unpack('>H', bytes(b[decode_pos:decode_pos+2]))
            frame_size = byte_count - 4

            # max frame = 4092, min frame = 1
            if frame_size < 1 or frame_size > 4092:
                #print '-- fail 1 {0}'.format(frame_size)
                decode_pos += 1
                decode_len -= 1
                continue

            # not enough in buffer for this decoding
            if decode_len < byte_count:
                #print '-- fail 2'
                decode_pos += 1
                decode_len -= 1
                continue

            frame_pos = decode_pos + 2
            frame_id = b[frame_pos]

            # invalid frame id
            if frame_id not in FrameID.__dict__.itervalues():
                #print '-- fail 3'
                decode_pos += 1
                decode_len -= 1
                continue

            crc_pos   = frame_pos  + frame_size
            crc = b[crc_pos:crc_pos + 2]
            entire_pkt = b[decode_pos:frame_pos + frame_size + 2]

            # CRC failure
            crc_check = crc_fn(bytes(entire_pkt))
            if crc_check != 0:
                #print '-- fail 4'
                decode_pos += 1
                decode_len -= 1
                continue

            # valid packet? wow.
            rdy_pkts.append(b[frame_pos:frame_pos + frame_size])

            # number of invalid bytes discarded to make this work.
            discard_amt += decode_pos - good_pos

            # advance to right after the decoded packet.
            decode_pos += byte_count
            decode_len -= byte_count

            # the decode position that will be started from next time
            good_pos     = decode_pos

        # discard this packet from buffer. also discard everything prior.
        del b[0:good_pos]
        self.discard_stat += discard_amt

        #print 'decode:', repr(bytes(b))
        return rdy_pkts

    def remove_listener(self, r):
        c = self.recv_cbs
        l = self.cb_lock
        l.acquire()
        try:
            # FIXME: linear search.
            c.remove(r)
        except Exception:
            pass
        l.release()

    def add_listener(self, cb):
        c = self.recv_cbs
        l = self.cb_lock
        l.acquire()
        c.append(cb)
        l.release()
        return cb

    def _recvSpecificMessage(self, *expected_frame_id, **only_timeout):
        # XXX: Really, python?
        # 'def x(*a, b=2)' is not allowed.
        if 'timeout' not in only_timeout:
            timeout = 0.5
        else:
            timeout = only_timeout['timeout']
        s = _one_msg_stall(*expected_frame_id)
        t = self.add_listener(s.cb())

        r = s.wait(timeout)

        self.remove_listener(t)

        if (r == None):
            raise TimeoutException('Did not recv frame_id {0} within time limit.'.format(expected_frame_id), timeout)
        else:
            # XXX: change this when len(expected_frame_id) = 1?
            return (ord(r[0]), r[1:])

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
        self.recv_cbs = []
        self.cb_lock = threading.Lock()

        self.decode_pos = 0
        self.discard_stat = 0
        self.recv_buf = bytearray()

        self.read_th = rt = threading.Thread(target=self.reader())
        rt.daemon = True
        rt.start()

    def close(self):
        self.fp.flush()
        self.fp.close()

    def _send(self, fmt):
        self.fp.write(fmt)

    def _sendMessage(self, frame_id, payload=b''):
        count = len(payload) + 5
        head = struct.pack('>HB{0}s'.format(len(payload)), count, frame_id, payload)
        tail = struct.pack('>H', self.crc(head))
        self._send(head + tail)

    def _createDatum(self, data):
        for component in self.Datum._fields:
            if component not in data.keys():
                data[component] = None
        return self.Datum(**data)

    def getModelInfo(self):
        """
        Query the module's type and firmware revision number.
        """
        self._sendMessage(FrameID.kGetModInfo)
        (_, payload) = self._recvSpecificMessage(FrameID.kModInfoResp)
        return self.ModInfo(*struct.unpack('>4s4s', payload))
    
    def readData(self):
        """
        Read a single DataResp frame
        """
        (_, payload) = self._recvSpecificMessage(FrameID.kDataResp)

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

    def getData(self):
        """
        Query a single packet of data that containing the components specified
        by setDataComponents(). All other components are set to zero.
        """
        self._sendMessage(FrameID.kGetData)
        return self.readData()

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

    def setOrientation(self, orientation):
        """
        Set the orientation of the compass
        """
        self.setConfig(Configuration.kMountingRef, orientation)

    def takeUserCalSample(self):
        """
        Commands the module to take a sample during user calibration.
        """
        self._sendMessage(FrameID.kTakeUserCalSample)

    def getConfig(self, config_id):
        """
        Get the value of a single configuration value based on config_id.
        """
        payload_id = self.struct_uint8.pack(config_id)
        self._sendMessage(FrameID.kGetConfig, payload_id)

        (_, response) = self._recvSpecificMessage(FrameID.kConfigResp)
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

        (_, payload_response) = self._recvSpecificMessage(FrameID.kParamResp)
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
        self._sendMessage(FrameID.kGetAcqParams)
        (_, payload)  = self._recvSpecificMessage(FrameID.kAcqParamsResp)
        response = struct.unpack('>BBff', payload)
        return self.AcqParams(*response)

    def startStreaming(self):
        """
        Start streaming data. See setAcquisitionParams() for more information
        and use stopStreaming() when done. Streaming must be stopped before any
        other commands can be used.
        """
        self._sendMessage(FrameID.kStartIntervalMode)

    def stopAll(self):
        """
        Stop all modes which result in periodic messages
        """
        self.stopStreaming()
        self.stopCalibration()

    def stopStreaming(self):
        """
        Stops streaming data; companion of startStreaming(). Streaming must be
        stopped before any other commands can be used.
        """
        self._sendMessage(FrameID.kStopIntervalMode)
    
    def stopCalibration(self):
        """
        Stops calibration;
        """
        self._sendMessage(FrameID.kStopCal)

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
        self._sendMessage(FrameID.kPowerDown)
        self._recvSpecificMessage(FrameID.kPowerDownDone)

    def save(self):
        """
        Write the current configuration to non-volatile memory. Note that this
        is the only command that writes to non-volatile memory, so it should be
        paired with any configuration options (e.g. calibration) that are
        intended to be persistant.
        """
        self._sendMessage(FrameID.kSave)
        (_, response) = self._recvSpecificMessage(FrameID.kSaveDone)
        (code, ) = self.struct_uint16.unpack(response)

        if code != 0:
            raise IOError('Save failed with error code {0}.'.format(code))

    def startCalibration(self, mode=None):
        """
        Starts calibration. See the FieldForce TCM User Manual for details
        about the necessary setup for each calibration mode.
        When mode = None, the last calibration mode used is repeated.
        """
        if mode == None:
            self._sendMessage(FrameID.kStartCal)
        else:
            payload_mode = self.struct_uint32.pack(mode)
            self._sendMessage(FrameID.kStartCal, payload_mode)


    def getCalibrationStatus(self, timeout=15):
        """
        Blocks waiting for a calibration update from the sensor. This returns a
        tuple where the first elements is a boolean that indicates whether the
        calibration is complete.
        """
        frame_id, message = self._recvSpecificMessage(FrameID.kUserCalSampCount, FrameID.kUserCalScore, timeout=timeout)

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
        else:
            raise IOError('Unexpected frame id: {0}.'.format(frame_id))

# vim: set et sw=4 ts=4:
