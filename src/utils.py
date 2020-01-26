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


from diagnostic_msgs.msg import DiagnosticStatus, KeyValue


def cpu_status(hardware, cpu):
    # 'CPU': {'status': 'ON', 'frq': 204, 'name': 'CPU1', 'val': 8, 'governor': 'schedutil'}
    val = cpu['val']
    status = cpu['status']
    # Make Dianostic Status message with cpu info
    d_cpu = DiagnosticStatus(name='jetson_stats cpu {name}'.format(name=cpu['name']),
                             message='{val}% status {status}'.format(val=val, status=status),
                             hardware_id=hardware,
                             values=[KeyValue("Status", "{status}".format(status=cpu['status'])),
                                     KeyValue("Governor", "{governor}".format(governor=cpu['governor'])),
                                     KeyValue("Val", "{val}%".format(val=val)),
                                     KeyValue("Freq", "{frq}Mhz".format(frq=cpu['frq'])),
                                     ])
    return d_cpu


def gpu_status(harware, gpu):
    d_gpu = DiagnosticStatus(name='jetson_stats gpu',
                                message='{val}%'.format(val=gpu['val']),
                                hardware_id=hardware,
                                values=[KeyValue('Val', '{val}%'.format(val=gpu['val']))])
    return d_gpu


def fan_status(hardware, fan):
    # 'FAN': {'status': 'ON', 'ctrl': True, 'cap': 255, 'tpwm': 0, 'step': 100, 'cpwm': 0}
    if 'cpwm' in fan:
        if 'ctrl' in fan:
            ctrl = "Ta" if fan.get("ctrl", False) else "Tm"
        else:
            ctrl = "T"
        label = "{ctrl}={target: >3}%".format(ctrl=ctrl, target=fan.get("tpwm", 0))
        value = fan.get('cpwm', 0)
    else:
        label = ''
        value = fan.get('tpwm', 0)
    # Make fan diagnostic status
    d_fan = DiagnosticStatus(name='jetson_stats fan',
                        message='{speed}% {label}'.format(speed=value, label=label),
                        hardware_id=hardware,
                        values=[KeyValue("Status", "{status}".format(status=fan['status'])),
                                KeyValue("Temp control", "{ctrl}".format(ctrl=fan['ctrl'])),
                                KeyValue("Capacity", "{cap}%".format(cap=fan['cap'])),
                                ])
    return d_fan

# 'EMC': {'val': 0}, 
# 'TEMP': {u'AO': 40.0, u'PMIC': 100.0, u'iwlwifi': 36.0, u'thermal': 31.0, u'GPU': 31.0, u'PLL': 28.5, u'CPU': 31.0},
# 'RAM': {'use': 1325, 'unit': u'M', 'tot': 3964, 'lfb': {'nblock': 411, 'unit': u'M', 'size': 4}},
# 'VOLT': {'POM_5V_CPU': {'avg': 712, 'cur': 212}, 'POM_5V_IN': {'avg': 1891, 'cur': 1271}, 'POM_5V_GPU': {'avg': 31, 'cur': 0}},
# 'SWAP': {'cached': {'unit': u'M', 'size': 0}, 'use': 0, 'unit': u'M', 'tot': 1982},

# EOF
