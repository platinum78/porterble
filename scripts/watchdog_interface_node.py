#!/usr/bin/python

"""
********************************************************************************
* Filename      : Watchdog Interface Node
* Author        : Susung Park
* Description   : Watchdog (Arduino) interface node, for sensor data.
* Version       : Initial release; 07 Jul 2019
  
  watchdog_interface_node constantly communicates with Watchdog Arduino,
  receives sonar range data, and publishes it onto '/ranges' topic.
********************************************************************************
"""

import sys
import numpy as np
import struct

import rospy
from std_msgs.msg import Float64MultiArray
from porterble.msg import SonarRange

from utils.serial_handler import SerialHandler
from utils.device_identifier import query_arduino


class WatchdogSerialHandler(SerialHandler):
    def __init__(self, port_name, baudrate):
        super(WatchdogSerialHandler, self).__init__(port_name, baudrate)
        self.range = np.zeros(7)
    
    def require_data(self):
        byte_string = "\x04\n".encode()
        self.ser.write(byte_string)
    
    def receive_data(self):
        byte_string = self.ser.readline()
        try:
            self.range = list(struct.unpack("HHHHHcc", byte_string))
            return self.range
        except struct.error:
            rospy.logerr("Serial error. Publishing null values.")
            return [0] * 5


class WatchdogInterfaceNode:
    def __init__(self):
        # Initialize as ROS node, and advertise topic.
        rospy.init_node("watchdog_interface_node", sys.argv)
        self.publisher = rospy.Publisher("/range", SonarRange, queue_size=1000)
        
        # Initialize serial handler for watchdog Arduino.
        dev_keyword = rospy.get_param("/communication/arduino/dev_keyword")
        id_keyword = rospy.get_param("/communication/arduino/id_keyword")
        arduino_id = rospy.get_param("/communication/arduino/watchdog/id")
        rospy.loginfo(arduino_id)
        self.port_name = query_arduino(dev_keyword, id_keyword, arduino_id)
        self.baudrate = rospy.get_param("/communication/arduino/watchdog/baudrate")
        self.serial = WatchdogSerialHandler(self.port_name, self.baudrate)
        rospy.loginfo("Established serial connection with Watchdog Arduino.")

        # Prepare storages for measurement values.
        self.range = [0] * 5
        
    def publish_range(self):
        self.serial.require_data()
        self.range = self.serial.receive_data()[:5]
        msg = SonarRange()
        msg.range = self.range
        rospy.loginfo(self.range)
        self.publisher.publish(msg)
    
    def exec_loop(self, frequency=8):
        loop_rate = rospy.Rate(frequency)

        while not rospy.is_shutdown():
            self.publish_range()
            loop_rate.sleep()

def main():
    interface = WatchdogInterfaceNode()
    interface.exec_loop(frequency=8)

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass