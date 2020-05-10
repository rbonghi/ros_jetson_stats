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


from copy import deepcopy
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue


def size_min(num, divider=1.0, n=0, start=''):
    if num >= divider * 1000.0:
        n += 1
        divider *= 1000.0
        return size_min(num, divider, n, start)
    else:
        vect = ['', 'k', 'M', 'G', 'T']
        idx = vect.index(start)
        return round(num / divider, 1), divider, vect[n + idx]


def strfdelta(tdelta, fmt):
    """ Print delta time
        - https://stackoverflow.com/questions/8906926/formatting-python-timedelta-objects
    """
    d = {"days": tdelta.days}
    d["hours"], rem = divmod(tdelta.seconds, 3600)
    d["minutes"], d["seconds"] = divmod(rem, 60)
    return fmt.format(**d)


def board_status(hardware, board, dgtype):
    """
    Board information and libraries installed
    """
    values = []
    for key, value in board['board'].items():
        values += [KeyValue(key, "{value}".format(value=value))]
    for key, value in board['libraries'].items():
        values += [KeyValue("lib " + key, "{value}".format(value=value))]
    # Make board diagnostic status
    d_board = DiagnosticStatus(name='jetson_stats {type} config'.format(type=dgtype),
                              message='Jetpack {jetpack}'.format(jetpack=board['info']['Jetpack']),
                              hardware_id=hardware,
                              values=values)
    return d_board


def disk_status(hardware, disk, dgtype):
    """
    Status disk
    """
    value=int(float(disk['used']) / float(disk['total']) * 100.0)
    if value >= 90:
        level = DiagnosticStatus.ERROR
    elif value >= 70:
        level = DiagnosticStatus.WARN
    else:
        level = DiagnosticStatus.OK
    # Make board diagnostic status
    d_board = DiagnosticStatus(level=level,
                               name='jetson_stats {type} disk'.format(type=dgtype),
                               message="{0:2.1f}GB/{1:2.1f}GB".format(disk['used'], disk['total']),
                               hardware_id=hardware,
                               values=[
                                   KeyValue("Used", "{used:2.1f}GB".format(used=disk['used'])),
                                   KeyValue("Total", "{total:2.1f}GB".format(total=disk['total'])),
                               ])
    return d_board


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
    status = cpu['status']
    if status == 'ON':
        # read value
        val = cpu['val']
        message = '{status} - {val}%'.format(val=val, status=status)
        # Make Dianostic Status message with cpu info
        values = [KeyValue("Status", "{status}".format(status=status)),
                KeyValue("Val", "{val}%".format(val=val)),
                KeyValue("Freq", "{frq}Mhz".format(frq=cpu['frq']))]
        if 'governor' in cpu: 
            values += [KeyValue("Governor", "{governor}".format(governor=cpu['governor']))]
    else:
        values = [] 
        message = '{status}'.format(status=status)
    # Make Diagnostic message
    d_cpu = DiagnosticStatus(name='jetson_stats cpu {name}'.format(name=cpu['name']),
                             message=message,
                             hardware_id=hardware,
                             values=values)
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


def fan_status(hardware, fan, dgtype):
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
    d_fan = DiagnosticStatus(name='jetson_stats {type} fan'.format(type=dgtype),
                             message='speed={speed}% {label}'.format(speed=value, label=label),
                             hardware_id=hardware,
                             values=[KeyValue("Status", "{status}".format(status=fan['status'])),
                                     KeyValue("Temp control", "{ctrl}".format(ctrl=fan['ctrl'])),
                                     KeyValue("Capacity", "{cap}%".format(cap=fan['cap'])),
                                     ])
    return d_fan


def ram_status(hardware, ram, dgtype):
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
    tot_ram, divider, unit_name = size_min(ram.get('tot', 0), start=ram.get('unit', 'M'))
    # Make ram diagnostic status
    d_ram = DiagnosticStatus(name='jetson_stats {type} ram'.format(type=dgtype),
                             message='{use:2.1f}{unit_ram}/{tot:2.1f}{unit_ram}B (lfb {nblock}x{size}{unit}B)'.format(
                                                                        use=ram['use'] / divider,
                                                                        unit_ram=unit_name,
                                                                        tot=tot_ram,
                                                                        nblock=lfb_status['nblock'],
                                                                        size=lfb_status['size'],
                                                                        unit=lfb_status['unit']),
                             hardware_id=hardware,
                             values=[KeyValue("Use", "{use:2.1f}{unit}B".format(use=ram['use'] / divider, unit=unit_name)),
                                     KeyValue("Total", "{tot:2.1f}{unit}B".format(tot=ram['tot'] / divider, unit=unit_name)),
                                     KeyValue("lfb", "{nblock}x{size}{unit}B".format(nblock=lfb_status['nblock'],
                                                                                     size=lfb_status['size'],
                                                                                     unit=lfb_status['unit'])),
                                     ])
    return d_ram

def swap_status(hardware, swap, dgtype):
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
    tot_swap, divider, unit = size_min(swap.get('tot', 0), start=swap.get('unit', 'k'))
    message = '{use}{unit_swap}B/{tot}{unit}B (cached {size}{unit}B)'.format(use=swap.get('use', 0) / divider,
                                                                             tot=tot_swap,
                                                                             unit_swap=unit,
                                                                             size=swap_cached.get('size', '0'),
                                                                             unit=swap_cached.get('unit', ''))
    # Make swap diagnostic status
    d_swap = DiagnosticStatus(name='jetson_stats {type} swap'.format(type=dgtype),
                              message=message,
                              hardware_id=hardware,
                              values=[KeyValue("Use", "{use:2.1f}{unit}B".format(use=swap['use'] / divider, unit=unit)),
                                      KeyValue("Total", "{tot:2.1f}{unit}B".format(tot=swap['tot'] / divider, unit=unit)),
                                      KeyValue("Cached", "{size}{unit}B".format(size=swap_cached.get('size', '0'),
                                                                                unit=swap_cached.get('unit', ''))),
                                      ])
    return d_swap


def power_status(hardware, power):
    """
    Make a Power diagnostic message

    Fields:
     - 'POM_5V_CPU': {'avg': 712, 'cur': 212}
     - 'POM_5V_IN': {'avg': 1891, 'cur': 1271}
     - 'POM_5V_GPU': {'avg': 31, 'cur': 0}}
    """
    values = []
    total_name = ""
    for val in power:
        if "_IN" in val:
            total_name = val
            break
    # https://forums.developer.nvidia.com/t/xavier-jetson-total-power-consumption/81016
    if total_name:
        total = power[total_name]
        del power[total_name]
    else:
        total = {'cur': 0, 'avg': 0}
    # Make list power
    for watt in sorted(power):
        value = power[watt]
        watt_name = watt.replace("VDD_", "").replace("POM_", "").replace("_", " ")
        values += [KeyValue(watt_name, "curr={curr}mW avg={avg}mW".format(curr=int(value['cur']), avg=int(value['avg'])))]
        if not total_name:
            total['cur'] += value['cur']
            total['avg'] += value['avg']
    # Make voltage diagnostic status
    d_volt = DiagnosticStatus(name='jetson_stats power',
                              message='curr={curr}mW avg={avg}mW'.format(curr=int(total['cur']), avg=int(total['avg'])),
                              hardware_id=hardware,
                              values=values)
    return d_volt


def temp_status(hardware, temp, level_options):
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
    level = DiagnosticStatus.OK
    list_options = sorted(level_options.keys(), reverse=True)
    max_temp = 20
    # Remove from list PMIC temperature
    if 'PMIC' in temp:
        del temp['PMIC']
    for key, value in temp.items():
        values += [KeyValue(key, "{value:8.2f}C".format(value=value))]
        if value > max_temp:
            # Add last high temperature
            max_temp = value
    # Make status message
    for th in list_options:
        if max_temp >= th:
            level = level_options[th]
            break

    if level is not DiagnosticStatus.OK:
        max_temp_names = []
        # List off names
        for key, value in temp.items():
            if value >= th:
                # Store name
                max_temp_names += [key]
        # Write a message
        message = '[' + ', '.join(max_temp_names) + '] are more than 40 C'
    else:
        message = '{n_temp} temperatures reads'.format(n_temp=len(temp))
    # Make temperature diagnostic status
    d_temp = DiagnosticStatus(level=level,
                              name='jetson_stats temp',
                              message=message,
                              hardware_id=hardware,
                              values=values)
    return d_temp

def emc_status(hardware, emc, dgtype):
    """
    Make a EMC diagnostic message

    Fields:
     - 'val': 0
    """
    # Make EMC diagnostic status
    d_emc = DiagnosticStatus(name='jetson_stats {type} emc'.format(type=dgtype),
                              message='{val}%'.format(val=emc['val']),
                              hardware_id=hardware)
    return d_emc
# EOF
