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
from geometry_msgs.msg import Quaternion, Vector3
from std_msgs.msg import Header
from sensor_msgs.msg import Imu
from tf import transformations

inf = float('+inf')
var = 0.034906585 ** 2

def start_compass(compass):
    #compass.setConfig(Configuration.kMountingRef, Orientation.Y_UP_180)
    compass.stopAll()
    compass.setDataComponents([
        Component.kHeading,
        Component.kPAngle,
        Component.kRAngle,
        Component.kDistortion,
        Component.kCalStatus
    ])

    compass.startStreaming()

def main():
    rospy.init_node('fieldforce_tcm')
    pub = rospy.Publisher('compass', Imu)

    path  = rospy.get_param('~path', '/dev/ttyUSB0')
    baud  = rospy.get_param('~baud', 38400)
    frame = rospy.get_param('~frame_id', '/base_link')
    cov   = rospy.get_param('~covariance', [
        inf, 0.0, 0.0,
        0.0, inf, 0.0,
        0.0, 0.0, var
    ])

    compass = FieldforceTCM(path, baud)
    start_compass(compass)

    warn_distortion  = False
    warn_calibration = False
    timeout_total = 0
    timeout_since_last = 0

    try:
        while True:
            try:
                datum = compass.readData(2)
            except TimeoutException as e:
                rospy.logwarn('Wait for data timed out. Total timeouts: {0}, timouts since last data: {1}'.format(timeout_total, timeout_since_last))
                timeout_total += 1
                timeout_since_last += 1
                start_compass(compass)
                continue
            timeout_since_last = 0
            now   = rospy.get_rostime()

            if datum.Distortion and not warn_distortion:
                rospy.logwarn('Magnometer has exceeded its linear range.')
                warn_distortion = True

            if not datum.CalStatus and not warn_calibration:
                rospy.logwarn('Compass is not calibrated.')
                warn_calibration = True

            ax = math.radians(datum.RAngle)
            ay = math.radians(datum.PAngle)
            az = math.radians(datum.Heading)
            quaternion = transformations.quaternion_from_euler(ax, ay, az)

            pub.publish(
                header = Header(stamp=now, frame_id=frame),
                orientation            = Quaternion(*quaternion),
                orientation_covariance = [ 0.0 ] * 9,
                angular_velocity            = Vector3(0, 0, 0),
                angular_velocity_covariance = [ -1, 0, 0, 0, 0, 0, 0, 0, 0 ],
                linear_acceleration            = Vector3(0, 0, 0),
                linear_acceleration_covariance = [ -1, 0, 0, 0, 0, 0, 0, 0, 0 ]

            )
    except Exception as e:
        compass.stopStreaming()
        compass.close()
        raise e

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

# vim: set et sw=4 ts=4:
