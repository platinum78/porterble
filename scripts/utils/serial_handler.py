#!/usr/bin/python3

"""
********************************************************************************
* Filename      : Serial Handler
* Author        : Susung Park
* Description   : Serial data handler for communication with Arduino
* Version       : Initial release; 07 Jul 2019
********************************************************************************
"""

import rospy
from serial import Serial
from serial.serialutil import SerialException

class SerialHandler:
    def __init__(self, config_dict, debug=False, simulation=False):
        self.debug = False
        self.simulation = False

        serial_portname = config_dict["port_name"]
        baudrate = config_dict["baudrate"]

        # Try to open serial port; 
        try:
            self.ser = Serial(serial_portname, baudrate=baudrate)
            self.ser.close()
            self.ser.open()
            rospy.loginfo("Opened serial port \"" + serial_portname + "\".")
        except SerialException:
            self.simulation = True
            rospy.logerr("Failed to open serial port \"" + serial_portname + "\". Operating in simulation mode.")
        
        if self.debug: rospy.logwarn("Serial handler object created at" + str(id(self)) + ".")

    
    def readline(self, timeout=None):
        msg = ""
        char_buf = self.ser.read().decode()
        while char_buf != '\n':
            msg += char_buf
            char_buf = self.ser.read().decode()
        if self.debug: rospy.loginfo("SERIAL INBOUND: " + msg)
        return msg
    
    def readbytes(self, timeout=None):
        msg = bytes(0)
        char_buf = self.ser.read()
        while char_buf != '\n':
            msg += char_buf
            char_buf = self.ser.read()
        if self.debug: rospy.loginfo("SERIAL INBOUND: " + msg)
        return msg
    
    def writeline(self, msg, ends_with='\n'):
        if type(msg) != str:
            rospy.logerr("Message should be given in str-type.")
            raise TypeError("Message should be given in string-type.")
        
        if msg[-1] != ends_with:
            msg += ends_with
        
        msg_bytes = msg.encode()
        self.ser.write(msg_bytes)
        if self.debug: rospy.loginfo("SERIAL OUTBOUND: " + msg)
    
    def writebytes(self, msg, ends_with='\n'):
        if type(msg) != bytes:
            raise TypeError("Message should be given in bytes-type.")
        if msg[-1] != ord(ends_with):
            msg += bytes(ends_with)
        
        self.ser.write(msg)