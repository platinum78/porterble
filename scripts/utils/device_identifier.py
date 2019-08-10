#!/usr/bin/python

"""
********************************************************************************
* Filename      : Device Identifier
* Author        : Susung Park
* Description   : Device identifier for identification of multiple Arduinos.
* Version       : Initial release; 18 Jul 2019
********************************************************************************
"""

import os, sys
import rospy

def query_arduino(dev_keyword, id_keyword, arduino_id):
    device_list = os.popen("ls /dev | grep " + dev_keyword).read().strip().split('\n')
    
    for dev in device_list:
        dev_id = os.popen("udevadm info --query=all --name=" + dev + " | grep " + id_keyword).read()
        dev_id = dev_id[dev_id.index("=")+1:].strip()
        if dev_id == arduino_id:
            return "/dev/" + dev
    else:
        rospy.logerr("Arduino with ID [%s] not found." % arduino_id)
        raise RuntimeError("Device not found.")

def query_camera(dev_keyword, id_keyword, camera_id):
    device_list = os.popen("ls /dev | grep " + dev_keyword).read().strip().split('\n')
    
    for dev in device_list:
        dev_id = os.popen("udevadm info --name=" + dev + " | grep " + id_keyword).read()
        dev_id = dev_id[dev_id.index('=')+1:].strip()
        if dev_id == camera_id:
            return int(dev[5:])