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
from jtop import jtop
# Import Diagnostic status converters
from utils import (strfdelta,
                   board_status,
                   disk_status,
                   cpu_status,
                   fan_status,
                   gpu_status,
                   ram_status,
                   swap_status,
                   voltage_status,
                   temp_status,
                   emc_status)


def other_status(hardware, jetson):
    """
    Generic informations about jetson_clock and nvpmdel
    """
    values = []
    nvpmodel = jetson.nvpmodel
    text = ""
    if nvpmodel is not None:
        values += [KeyValue("NV Power" , "{num} - {name}".format(num=nvpmodel.num, name=nvpmodel.mode))]
        text += "NV Power[{num}] {name}".format(num=nvpmodel.num, name=nvpmodel.mode)
    jc = jetson.jetson_clocks
    if jc is not None:
        jc_status = jc.status
        if jc_status == "active":
            level = DiagnosticStatus.OK
        elif jc_status == "inactive":
            level = DiagnosticStatus.OK
        elif "ing" in jc_status:
            level = DiagnosticStatus.WARN
        else:
            level = DiagnosticStatus.ERROR
        # Show if JetsonClock is enabled or not
        values += [KeyValue("Jetson Clocks" , "{jc_status}".format(jc_status=jc_status))]
        values += [KeyValue("Enable" , "{enable}".format(enable=jc.enable))]
        text += " - JC {jc_status}".format(jc_status=jc_status)
    # Uptime
    uptime_string = strfdelta(timedelta(seconds=jetson.uptime), "{days} days {hours}:{minutes}:{seconds}")
    values += [KeyValue("Up Time" , "{time}".format(time=uptime_string))]
    # Jtop version
    values += [KeyValue("jtop" , "v{version}".format(version=jetson.version))]
    # Make board diagnostic status
    status = DiagnosticStatus(level=level,
                              name='jetson_stats',
                              message=text,
                              hardware_id=hardware,
                              values=values)
    return status

def wrapper(jetson):
    # Initialization ros pubblisher
    pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=1)
    # Set default rate jetson stats 2hz
    rate = rospy.Rate(2)
    # Extract board information
    board = jetson.board
    # Define hardware name
    hardware = board["board"]["Name"]
    # Define Diagnostic array message
    # http://docs.ros.org/api/diagnostic_msgs/html/msg/DiagnosticStatus.html
    arr = DiagnosticArray()
    # Initialization Tegrastats
    while not rospy.is_shutdown():
        # Save all stats
        stats = jetson.stats
        # Add timestamp
        arr.header.stamp = rospy.Time.now()
        # Status board and board info
        arr.status = [other_status(hardware, jetson)]
        # Make diagnostic message for each cpu
        arr.status += [cpu_status(hardware, cpu) for cpu in stats['CPU']]
        # Merge all other diagnostics
        if 'GR3D' in stats:
            arr.status += [gpu_status(hardware, stats['GR3D'])]
        if 'RAM' in stats:
            arr.status += [ram_status(hardware, stats['RAM'])]
        if 'SWAP' in stats:
            arr.status += [swap_status(hardware, stats['SWAP'])]
        if 'FAN' in stats:
            arr.status += [fan_status(hardware, stats['FAN'])]
        if 'VOLT' in stats:
            arr.status += [voltage_status(hardware, stats['VOLT'])]
        if 'TEMP' in stats:
            arr.status += [temp_status(hardware, stats['TEMP'])]
        if 'EMC' in stats:
            arr.status += [emc_status(hardware, stats['EMC'])]
        # Status board and board info
        arr.status += [board_status(hardware, board)]
        # Add disk status
        arr.status += [disk_status(hardware, jetson.disk)]
        # Update status jtop
        rospy.logdebug("jtop message %s" % rospy.get_time())
        pub.publish(arr)
        rate.sleep()


if __name__ == '__main__':
    try:
        # Initialization ros node
        rospy.init_node('jtop_node')
        # Run Tegrastats jetson
        with jtop() as jetson:
            wrapper(jetson)
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass
