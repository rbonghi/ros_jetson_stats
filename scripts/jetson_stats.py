#!/usr/bin/env python
# -*- coding: UTF-8 -*-
# Copyright (C) 2020, Raffaello Bonghi <raffaello@rnext.it>
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
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from datetime import timedelta
import jtop
# Import Diagnostic status converters
from ros_jetson_stats.utils import (
    other_status,
    board_status,
    disk_status,
    cpu_status,
    fan_status,
    gpu_status,
    ram_status,
    swap_status,
    power_status,
    temp_status,
    emc_status)


def wrapper(jetson):
    # Load level options
    level_options = {
        rospy.get_param("~level/error", 60): DiagnosticStatus.ERROR,
        rospy.get_param("~level/warning", 40): DiagnosticStatus.WARN,
        rospy.get_param("~level/ok", 20): DiagnosticStatus.OK,
    }
    rospy.loginfo(level_options)
    # Initialization ros publisher
    pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=1)
    # Extract board information
    board = jetson.board
    # Define hardware name
    hardware = board["info"]["machine"]
    # Define Diagnostic array message
    # http://docs.ros.org/api/diagnostic_msgs/html/msg/DiagnosticStatus.html
    arr = DiagnosticArray()
    # Initialization jtop
    while not rospy.is_shutdown() and jetson.ok():
        # Add timestamp
        arr.header.stamp = rospy.Time.now()
        # Status board and board info
        arr.status = [other_status(hardware, jetson, jtop.__version__)]
        # Make diagnostic message for each cpu
        arr.status += [cpu_status(hardware, name, jetson.cpu[name]) for name in jetson.cpu]
        # Merge all other diagnostics
        arr.status += [gpu_status(hardware, jetson.gpu)]
        arr.status += [ram_status(hardware, jetson.ram, 'mem')]
        arr.status += [swap_status(hardware, jetson.swap, 'mem')]
        arr.status += [emc_status(hardware, jetson.emc, 'mem')]
        # Temperature
        arr.status += [temp_status(hardware, jetson.temperature, level_options)]
        # Read power
        total, power = jetson.power
        if power:
            arr.status += [power_status(hardware, total, power)]
        # Fan controller
        if jetson.fan is not None:
            arr.status += [fan_status(hardware, jetson.fan, 'board')]
        # Status board and board info
        arr.status += [board_status(hardware, board, 'board')]
        # Add disk status
        arr.status += [disk_status(hardware, jetson.disk, 'board')]
        # Update status jtop
        rospy.logdebug("jtop message %s" % rospy.get_time())
        pub.publish(arr)


if __name__ == '__main__':
    try:
        # Initialization ros node
        rospy.init_node('jtop_node')
        # Load default interval speed
        interval = rospy.get_param("~interval", 0.5)
        # Run jtop reader
        with jtop.jtop(interval=interval) as jetson:
            wrapper(jetson)
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass
# EOF
