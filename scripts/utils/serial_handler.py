#!/usr/bin/python

"""
********************************************************************************
* Filename      : Serial Handler
* Author        : Susung Park
* Description   : Serial data handler for communication with Arduino
* Version       : Initial release; 07 Jul 2019
********************************************************************************
"""

import os, sys, time
import rospy
from serial import Serial
from serial.serialutil import SerialException

class SerialHandler(object):
    def __init__(self, port_name, baudrate):
        self.debug = False
        self.simulation = False

        self.port_name = port_name
        self.baudrate = baudrate

        # Try to open serial port; 
        try:
            rospy.loginfo("Opening serial port \"" + port_name + "\".")
            self.ser = Serial(self.port_name, baudrate=baudrate)
            time.sleep(2)
            rospy.loginfo("Opened serial port \"" + port_name + "\".")
        except SerialException:
            self.simulation = True
            rospy.logerr("Failed to open serial port \"" + port_name + "\". Operating in simulation mode.")