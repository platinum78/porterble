#!/usr/bin/python3

"""
********************************************************************************
* Filename      : Device Identifier
* Author        : Susung Park
* Description   : Device identifier for identification of multiple Arduinos.
* Version       : Initial release; 18 Jul 2019
********************************************************************************
"""

import os, sys

def identify_arduinos(keyword):
    device_list = os.popen("ls /dev | grep " + keyword).read().strip().split('\n')
    device_dict = {}
    
    for dev in device_list:
        dev_id = os.popen("udevadm info --query=all --name=" + dev + " | grep ID_SERIAL_SHORT").read()
        dev_id = dev_id[dev_id.index("=")+1:].strip()
        device_dict[dev_id] = "/dev/" + dev
    
    return device_dict