#!/usr/bin/env python
"""
Copyright (c) 2012, Michael Koval
Copyright 2012, Cody Schafer <cpschafer --- gmail.com>
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

import roslib; roslib.load_manifest('fieldforce_tcm')
import rospy
import math

from fieldforce_tcm import Calibration, Component, Configuration, FieldforceTCM, Orientation, TimeoutException

def main():
    rospy.init_node('estimate_latency')

    path = rospy.get_param('~path')
    baud = rospy.get_param('~baud', 38400)
    samples = rospy.get_param('~samples', 100)

    compass = FieldforceTCM(path, baud)
    ver = compass.getModelInfo()
    rospy.loginfo('Found Fieldforce TCM: {0}'.format(ver))

    # Configure the compass in a realistic way.
    compass.stopAll()
    compass.setConfig(Configuration.kCoeffCopySet, 0)
    compass.setConfig(Configuration.kAccelCoeffCopySet, 0)

    compass.setDataComponents([
        Component.kHeading,
        Component.kPAngle,
        Component.kRAngle,
        Component.kDistortion,
        Component.kCalStatus
    ])

    # Estimate the latency by using blocking sample requests.
    latencies = list()
    try:
        for i in xrange(samples):
            before = rospy.get_rostime()
            compass.readData()
            after = rospy.get_rostime()
            latency = (after - before).to_sec()
            latencies.append(latency)
    except Exception as e:
        compass.close()
        raise e

    avg_latency = sum(latencies) / (2 * len(latencies))
    rospy.loginfo('Offset = {0} seconds'.format(avg_latency))

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

# vim: set et sw=4 ts=4:
