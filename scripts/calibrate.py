#!/usr/bin/env python
# vim: set fileencoding=utf-8 :
from __future__ import print_function

"""
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
from sys import argv, exit, path as spath
from os.path import dirname

_us = dirname(__file__)
spath.insert(0, "{0}/../src".format(_us))
from fieldforce_tcm import *
spath.pop(0)

import termios
import threading
from event import *
from time import time as _time

_stdin = sys.stdin.fileno()
_termios_settings = None

def _term_save():
    global _termios_settings
    _termios_settings = termios.tcgetattr(_stdin)

def _term_restore():
    termios.tcsetattr(_stdin, termios.TCSANOW, _termios_settings)

def _term_echo(on):
    it = termios.tcgetattr(_stdin)
    if on:
        it[3] |= termios.ECHO
    else:
        it[3] &= ~ termios.ECHO
    termios.tcsetattr(_stdin, termios.TCSANOW, it)

def _term_buffer(on):
    it = termios.tcgetattr(_stdin)
    if on:
        it[3] |= termios.ICANON
    else:
        it[3] &= ~ termios.ICANON
    termios.tcsetattr(_stdin, termios.TCSANOW, it)

def init_for_calib(compass, auto, num_samples, calib_type):
    # Using the kSetParam command, set the number of tap filters to 32.
    compass.setFilter(32)
    # Using the kSetConfig command, set kUserCalAutoSampling. “False” is
    #  generally recommended, but “True” may be more convenient.
    compass.setConfig(Configuration.kUserCalAutoSampling, auto)
    # Using the kSetConfig command, set kCoeffCopySet (magnetometer
    #   calibration) and/or kAccelCoeffCopySet (accelerometer calibration).
    #   These fields allow the user to save multiple sets of calibration
    #   coefficients.  “0” is the default.
    compass.setConfig(Configuration.kCoeffCopySet, 1)
    compass.setConfig(Configuration.kAccelCoeffCopySet, 1)
    # Using the kSetConfig command again, set kUserCalNumPoints to the
    # appropriate number of calibration points. The number of calibration
    # points should be at least 12 for Full Range Calibration, Limited Tilt
    # Range Calibration and 2D Calibration; at least 6 for Hard Iron Only
    # Calibration; and at least 18 for Accel Only Calibration and

    #Accel and Mag Calibration.
    compass.setConfig(Configuration.kUserCalNumPoints, num_samples)
    # Initiate a calibration using the kStartCal command. Note that this
    #   command requires indentifying the type of calibration procedure
    #   (i.e. Full Range, 2D, etc.).

    compass.startCalibration(calib_type)
    print('calibration started')
    # Follow the appropriate calibration procedure discussed in Sections
    #   6.2.1 to 6.2.6. If kUserCalAutoSampling was set to “False”, then
    #   send a kTakeUserCalSample command when ready to take a calibration
    #   point.  If kUserCalAutoSampling was set to “True”, then look for
    #   kUserCalSampCount to confirm when a calibration point has been
    #   taken. During the calibration process, heading, pitch, and roll
    #   information will be output from the module, and this can be
    #   monitored using kDataResp.

def main():
    #if not pygame.mixer: print('Warning, sound disabled')
    if len(argv) < 2:
        exit('usage: {0} <serial port>'.format(argv[0]))

    fname   = argv[1]
    compass = FieldforceTCM(fname, 38400)
    event   = EventQueue()
    #pygame.mixer.init()
    #exit('Could not open {0}, {1}'.format(fname, e))

    COMPASS_IN_CALIB   = 0
    COMPASS_CALIB_DONE = 1
    KEYS               = 2

    def compass_reader():
        while True:
            try:
                done, data = compass.getCalibrationStatus()
                if done:
                    event.post(Event(COMPASS_CALIB_DONE, score = data))
                else:
                    event.post(Event(COMPASS_IN_CALIB, sample_num = data))
            except TimeoutException as e:
                # Timed out.
                #print('calib')
                pass
            except IOError as e:
                print(repr(e))

    cr = threading.Thread(target=compass_reader)
    cr.daemon = True
    cr.start()

    def keyboard_reader():
        _term_buffer(False)
        _term_echo(False)
        while True:
            event.post(Event(KEYS, key=sys.stdin.read(1)))
    kr = threading.Thread(target=keyboard_reader)
    kr.daemon = True
    kr.start()

    ver = compass.getModelInfo()
    print('Found Fieldforce TCM: {0}'.format(ver))

    num_samples = 12
    auto = False
    #calib_type = Calibration.kLimitedTiltCalibraion
    calib_type = Calibration.k2DCalibration

    compass.stopAll()
    init_for_calib(compass, auto, num_samples, calib_type)


    started_once = True
    running  = True
    in_calib = True
    sampling_done = False
    have_calib = False
    old_time = _time()
    while running:
        ev = event.wait()
        if ev.type == COMPASS_IN_CALIB:
            new_time = _time()
            print('Sample #{0} ({1} seconds)'.format(ev.sample_num, new_time - old_time))
            old_time = new_time
            in_calib = True
            if ev.sample_num == num_samples:
                sampling_done = True
                print('Calculating calibration... ')
        elif ev.type == COMPASS_CALIB_DONE:
            new_time = _time()
            if not sampling_done:
                print('unexpected calibration completion')
            print('Calibration complete: {0}'.format(repr(ev.score)))
            print('Calculated in {0} seconds'.format(new_time - old_time))
            old_time = new_time
            in_calib = False
            have_calib = True
        elif ev.type == KEYS:
            if   ev.key == 'Q' or ev.key == 'q':
                compass.stopCalibration()
                running = False
            elif ev.key == ' ' or ev.key == '\n':
                if   not auto and in_calib:
                    compass.takeUserCalSample()
                elif not in_calib:
                    if not started_once:
                        init_for_calib(compass, auto, num_samples)
                    print('Restarting calib')
                    compass.startCalibration()
            elif ev.key == 's' or ev.key == 'S':
                # if deciding to save, save
                if have_calib:
                    compass.save()
                    print('saving...')
                else:
                    print('no calibration avaliable to save')
            elif ev.key == 'a' or ev.key == 'A':
                if not in_calib:
                    auto = not auto
                    compass.setConfig(Configuration.kUserCalAutoSampling, auto)
                    if auto:
                        print('auto measurments ON')
                    else:
                        print('auto measurments OFF')
            elif ev.key == '?':
                print('lol.')
            else:
                print('unknown key = {0}'.format(repr(ev.key)))
        else:
            print('unknown event {0}'.format(ev))

# When the final calibration point is taken, the module will present the
#   calibration score using kUserCalScore.
# If the calibration is acceptable (see Section 7.4.17), save the calibration
#   coefficients using kSave.

if __name__ == '__main__':
    _term_save()
    try:
        main()
    finally:
        _term_restore()
