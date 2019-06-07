#!/usr/bin/env python
# -*- coding: UTF-8 -*-
# Copyright (C) 2019, Raffaello Bonghi <raffaello@rnext.it>
# All rights reserved
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
# CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING,
# BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
# OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
# EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from jtop import jtop


def wrapper(jetson):
    # Initialization ros pubblisher
    pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=1)
    # Set default rate jetson stats 2hz
    rate = rospy.Rate(2)
    # Extract board information
    board = jetson.board
    # Define Diagnostic array message
    # http://docs.ros.org/api/diagnostic_msgs/html/msg/DiagnosticStatus.html
    arr = DiagnosticArray()
    arr.status = [
        DiagnosticStatus(name='jetson_stats CPU', message='Jetson-stats CPU', hardware_id=board["board"]["name"]),
        DiagnosticStatus(name='jetson_stats GPU', message='jetson-stats GPU', hardware_id=board["board"]["name"])
    ]
    # Initialization Tegrastats
    while not rospy.is_shutdown():
        jtop_str = "jtop message %s" % rospy.get_time()
        rospy.loginfo(jtop_str)
        pub.publish(arr)
        rate.sleep()


if __name__ == '__main__':
    try:
        # Initialization ros node
        rospy.init_node('jtop', anonymous=True)
        # Run Tegrastats jetson
        with jtop() as jetson:
            wrapper(jetson)
    except rospy.ROSInterruptException:
        pass
