#!/usr/bin/python

"""
********************************************************************************
* Filename      : Watchdog Interface Node
* Author        : Susung Park
* Description   : Watchdog (Arduino) interface node, for sensor data.
* Version       : Initial release; 07 Jul 2019
********************************************************************************
"""

import rospy
from serial import Serial
from serial.serialutil import SerialException