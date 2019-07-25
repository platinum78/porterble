#!/usr/bin/python

"""
********************************************************************************
* Filename      : Torch Interface Node
* Author        : Susung Park
* Description   : Accepts services and interacts with torch Arduino.
* Version       : On development...
********************************************************************************
"""

import os, sys, time, struct
import numpy as np
from serial.serialutil import SerialException
from scripts.utils.serial_handler import SerialHandler

class Lightings:
    LIGHT_FRONT = 0b10000000
    LIGHT_REAR = 0b01000000
    INDICATOR_LEFT = 0b00100000
    INDICATOR_RIGHT = 0b00010000
    INDICATOR_ALERT = 0b00001000

    def __init__(self, config_dict, debug=False):
        self.ser = SerialHandler(config_dict=config_dict, debug=True)
        msg = self.ser.readline()
        print(msg)
        if msg != '\x01':
            raise SerialException("System not properly set.")
        
    def set_lights(self, code=0):
        msg = struct.pack("<B", code) + b'\n'
        self.ser.writebytes(msg)



import os, sys, struct
import rospy
from porterble.srv import *

def lightings_service_handler(request):
    request.

    LightingsResponse(True)

if __name__ == "__main__":
    lightings = Lightings(config_dict, debug=True)