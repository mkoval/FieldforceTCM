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

from event import *

def main():
	if len(argv) != 2:
		print('usage: {0} <pty>'.format(argv[0]))
		exit()

	ff = FieldforceTCM(argv[1], 38400)
	eq = EventQueue()
	
	PKT = 0
	def pkt_cb(rdy_pkts):
		for pkt in rdy_pkts:
			eq.post(Event(PKT, data=pkt))
	ff.add_listener(pkt_cb)


	ff_config = dict()

	while True:
		ev = eq.wait()
		if ev.type == PKT:
			frame = ev.data
			frame_id = frame[0]
			payload  = frame[1:len(frame)]
			id_name = FrameID.invert[frame_id]
			print ('recved {0} ({1})'.format(id_name, frame_id))
			if frame_id == FrameID.kSetParam:
				pkt = encode_command(FrameID.kSetParamDone)
				ff._send(pkt)
			elif frame_id == FrameID.kSetConfig:
				ff._send(encode_command(FrameID.kSetConfigDone))
			elif frame_id == FrameID.kStartCal:
				pass
			else:
				raise IOError('''Don't know how to handle frame_id {0} ({1})'''.format(id_name, frame_id))
		else:
			print ('eh?')

	print('done.')

if __name__ == '__main__':
	main()
