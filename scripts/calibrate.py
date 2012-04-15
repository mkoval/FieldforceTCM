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

from collections import deque
import threading
import pygame
from pygame.event import Event

class EventQueue:
    def __init__(self):
        self.cond = threading.Condition()
        self.q    = deque()

    def post(self, event):
        c = self.cond
        c.acquire()
        self.q.appendleft(event)
        c.notify()
        c.release()

        def wait(self, timeout=None):
            c = self.cond
            c.acquire()
            if timeout != None:
                start_time = _time()
            while not self.q:
                c.wait(timeout)
                # required due to http://bugs.python.org/issue1175933
                if timeout != None and (_time() - start_time) > timeout:
                    c.release()
                    return None
            it = self.q.pop()
            c.release()
            return it

def main():
	#if not pygame.mixer: print('Warning, sound disabled')
	if len(argv) < 2:
		exit('usage: {0} <serial port>'.format(argv[0]))

	fname   = argv[1]
	compass = FieldforceTCM(fname, 38400)
	event   = EventQueue()
	#pygame.mixer.init()
	#exit('Could not open {0}, {1}'.format(fname, e))

	# Using the kSetParam command, set the number of tap filters to 32.
	compass.setFilter(32)
	# Using the kSetConfig command, set kUserCalAutoSampling. “False” is
	#  generally recommended, but “True” may be more convenient.
	compass.setConfig(Configuration.kUserCalAutoSampling, False)
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
	compass.setConfig(Configuration.kUserCalNumPoints, 32)
	# Initiate a calibration using the kStartCal command. Note that this
	#   command requires indentifying the type of calibration procedure
	#   (i.e. Full Range, 2D, etc.).

	compass.startCalibration(Calibration.kLimitedTiltCalibraion)
	print('calibration started')
	# Follow the appropriate calibration procedure discussed in Sections
	#   6.2.1 to 6.2.6. If kUserCalAutoSampling was set to “False”, then
	#   send a kTakeUserCalSample command when ready to take a calibration
	#   point.  If kUserCalAutoSampling was set to “True”, then look for
	#   kUserCalSampCount to confirm when a calibration point has been
	#   taken. During the calibration process, heading, pitch, and roll
	#   information will be output from the module, and this can be
	#   monitored using kDataResp.

	COMPASS_IN_CALIB   = pygame.USEREVENT
	COMPASS_CALIB_DONE = COMPASS_IN_CALIB + 1
	KEYS               = COMPASS_CALIB_DONE + 1
	def compass_reader():
		while True:
			done, data = compass.getCalibrationStatus()
			if done:
				event.post(Event(COMPASS_IN_CALIB,   prog_data = data))
			else:
				event.post(Event(COMPASS_CALIB_DONE, cal_score = data))

	cr = threading.Thread(target=compass_reader)
	cr.daemon = True
	cr.start()


	def keyboard_reader():
		while True:
			event.post(Event(KEYS, key=sys.stdin.read(1)))
	kr = threading.Thread(target=keyboard_reader)
	kr.daemon = True
	kr.start()

	running  = True
	auto     = False
	in_calib = True
	save_q   = False
	while running:
		print('waiting for event')
		ev = event.wait()
		if   ev.type == QUIT:
			running = False
		elif ev.type == COMPASS_IN_CALIB:
			print(ev.prog_data)
		elif ev.type == COMPASS_CALIB_DONE:
			print(ev.cal_score)
		elif ev.type == KEYS:
			if   ev.key == 'Q' or ev.key == 'q':
				running = False
			elif ev.key == ' ':
				if   not auto and in_calib:
					compass.takeUserCalSample()
				elif not in_calib:
					compass.startCalibration()
			elif ev.key == 's' or ev.key == 'S':
				# if deciding to save, save
				if save_q:
					save_q = False
					compass.save()
			elif ev.key == 'a' or ev.key == 'A':
				auto = not auto
				compass.setConfig(Configuration.kUserCalAutoSampling, auto)
				if auto:
						print('auto measurments ON')
				else:
						print('auto measurments OFF')
		else:
			print('unknown event {0}'.format(ev))

# When the final calibration point is taken, the module will present the
#   calibration score using kUserCalScore.
# If the calibration is acceptable (see Section 7.4.17), save the calibration
#   coefficients using kSave.

if __name__ == '__main__':
    main()
