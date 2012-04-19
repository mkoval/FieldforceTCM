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
from collections import deque
import threading

class Signal:
    def __init__(self):
        self.last  = None
        self.first = None

    def attach(self, connection):
        self.last.attach(connection)

class Event:
	def __init__(self, etype, **kws):
		self.type = etype
		self.__dict__.update(kws)

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
