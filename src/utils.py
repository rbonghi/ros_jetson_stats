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
    """
    Decode a cpu stats

    Fields:
     - 'status': 'ON'
     - 'frq': 204
     - 'name': 'CPU1'
     - 'val': 8
     - 'governor': 'schedutil'
    """
    val = cpu['val']
    status = cpu['status']
    # Make Dianostic Status message with cpu info
    d_cpu = DiagnosticStatus(name='jetson_stats cpu {name}'.format(name=cpu['name']),
                             message='status={status} {val}%'.format(val=val, status=status),
                             hardware_id=hardware,
                             values=[KeyValue("Status", "{status}".format(status=cpu['status'])),
                                     KeyValue("Governor", "{governor}".format(governor=cpu['governor'])),
                                     KeyValue("Val", "{val}%".format(val=val)),
                                     KeyValue("Freq", "{frq}Mhz".format(frq=cpu['frq'])),
                                     ])
    return d_cpu


def gpu_status(hardware, gpu):
    """
    Decode and build a diagnostic status message

    Fields:
     - 'val': 10
    """
    d_gpu = DiagnosticStatus(name='jetson_stats gpu',
                             message='{val}%'.format(val=gpu['val']),
                             hardware_id=hardware,
                             values=[KeyValue('Val', '{val}%'.format(val=gpu['val']))])
    return d_gpu


def fan_status(hardware, fan):
    """
    Fan speed and type of control

    Fields:
     - 'status': 'ON'
     - 'ctrl': True
     - 'cap': 255
     - 'tpwm': 0
     - 'step': 100
     - 'cpwm': 0
    """
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
                             message='speed={speed}% {label}'.format(speed=value, label=label),
                             hardware_id=hardware,
                             values=[KeyValue("Status", "{status}".format(status=fan['status'])),
                                     KeyValue("Temp control", "{ctrl}".format(ctrl=fan['ctrl'])),
                                     KeyValue("Capacity", "{cap}%".format(cap=fan['cap'])),
                                     ])
    return d_fan


def ram_status(hardware, ram):
    """
    Make a RAM diagnostic status message

    Fields:
        - 'use': 1325
        - 'unit': 'M'
        - 'tot': 3964
        - 'lfb': 
            - 'nblock': 411
            - 'unit': 'M'
            - 'size': 4
    """
    lfb_status = ram['lfb']
    # value = int(ram['use'] / float(ram['tot']) * 100.0)
    unit_name = 'G'  # TODO improve with check unit status
    percent = "{use:2.1f}{unit}/{tot:2.1f}{unit}B".format(use=ram['use'] / 1000.0,
                                                          unit=unit_name,
                                                          tot=ram['tot'] / 1000.0),
    # Make ram diagnostic status
    d_ram = DiagnosticStatus(name='jetson_stats ram',
                             message='{percent} (lfb {nblock}x{size}{unit}B)'.format(percent=percent,
                                                                                     nblock=lfb_status['nblock'],
                                                                                     size=lfb_status['size'],
                                                                                     unit=lfb_status['unit']),
                             hardware_id=hardware,
                             values=[KeyValue("Use", "{use:2.1f}{unit}B".format(use=ram['use'] / 1000.0, unit=unit_name)),
                                     KeyValue("Total", "{tot:2.1f}{unit}B".format(tot=ram['tot'] / 1000.0, unit=unit_name)),
                                     KeyValue("lfb", "{nblock}x{size}{unit}B".format(nblock=lfb_status['nblock'],
                                                                                     size=lfb_status['size'],
                                                                                     unit=lfb_status['unit'])),
                                     ])
    return d_ram

def swap_status(hardware, swap):
    """
    Make a swap diagnostic message

    Fields:
        - 'use': 0
        - 'unit': 'M'
        - 'tot': 1982
        - 'cached':
            - 'unit': 'M'
            - 'size': 0
    """
    swap_cached = swap.get('cached', {})
    if swap.get('tot', 0) < 1000:
        unit = swap['unit']
        divider = 1.0
    if swap.get('tot', 0) > 1000:
        if 'k' == swap['unit']:
            unit = 'M'
        elif 'M' == swap['unit']:
            unit = 'G'
        divider = 1000.0
    # Make swap diagnostic status
    d_swap = DiagnosticStatus(name='jetson_stats swap',
                              message='{use}{unit}B/{tot}{unit}B (cached {size}{unit}B)'.format(
                                                                                use=swap.get('use', 0) / divider,
                                                                                tot=swap.get('tot', 0) / divider,
                                                                                size=swap_cached.get('size', '0'),
                                                                                unit=swap_cached.get('unit', '')),
                              hardware_id=hardware,
                              values=[KeyValue("Use", "{use:2.1f}{unit}B".format(use=swap['use'] / divider, unit=unit)),
                                      KeyValue("Total", "{tot:2.1f}{unit}B".format(tot=swap['tot'] / divider, unit=unit)),
                                      KeyValue("Cached", "{size}{unit}B".format(size=swap_cached.get('size', '0'),
                                                                                unit=swap_cached.get('unit', ''))),
                                      ])
    return d_swap


def voltage_status(hardware, volt):
    """
    Make a volt diagnostic message

    Fields:
     - 'POM_5V_CPU': {'avg': 712, 'cur': 212}
     - 'POM_5V_IN': {'avg': 1891, 'cur': 1271}
     - 'POM_5V_GPU': {'avg': 31, 'cur': 0}}
    """
    values = []
    tot = {'cur': 0, 'avg': 0}
    for key, value in volt.items():
        values += [KeyValue(key, "curr={curr}mW avg={avg}mW".format(curr=int(value['cur']), avg=int(value['avg'])))]
        tot['cur'] += value['cur']
        tot['avg'] += value['avg']
    # Make voltage diagnostic status
    d_volt = DiagnosticStatus(name='jetson_stats volt',
                              message='curr={curr}mW avg={avg}mW'.format(curr=int(tot['cur']), avg=int(tot['avg'])),
                              hardware_id=hardware,
                              values=values)
    return d_volt


def temp_status(hardware, temp):
    """
    Make a temperature diagnostic message

    Fields:
     - 'AO': 40.0
     - 'PMIC': 100.0
     - 'iwlwifi': 36.0
     - 'thermal': 31.0
     - 'GPU': 31.0
     - 'PLL': 28.5
     - 'CPU': 31.0
    """
    values = []
    for key, value in temp.items():
        values += [KeyValue(key, "{value:8.2f}C".format(curr=value))]
    # Make temperature diagnostic status
    d_temp = DiagnosticStatus(name='jetson_stats temp',
                              message='temp n={n_temp}'.format(n_temp=len(temp)),
                              hardware_id=hardware,
                              values=values)
    return d_temp

def emc_status(hardware, emc):
    """
    Make a EMC diagnostic message

    Fields:
     - 'val': 0
    """
    # Make EMC diagnostic status
    d_emc = DiagnosticStatus(name='jetson_stats emc',
                              message='{val}%'.format(val=emc['val']),
                              hardware_id=hardware)
    return d_emc
# EOF
