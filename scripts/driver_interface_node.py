#!/usr/bin/python

"""
********************************************************************************
* Filename      : Driver Interface Node
* Author        : Susung Park
* Description   : Interface with driver Arduino.
* Version       : On development; 08 AUG 2019

  driver_interface_node subscribes to the topic '/cmd_vel', computes the
  velocities of each wheel using 'QuadMecanumKinematics' class, writes them
  via serial, receives actual encoder values, and publishes them onto
  '/wheels_vel' topic.
********************************************************************************
"""

import os, sys, struct
from serial.serialutil import SerialException

import rospy
from geometry_msgs.msg import Pose2D
from porterble.msg import QuadMecanumWheel

from utils.serial_handler import SerialHandler
from utils.device_identifier import query_arduino

from controls.kinematics import QuadMecanumKinematics

class DriverSerialHandler(SerialHandler):
    def __init__(self, port_name, baudrate):
        super(DriverSerialHandler, self).__init__(port_name, baudrate)
    
    def write_velocity(self, v1, v2, v3, v4):
        byte_string = struct.pack("hhhhc", int(v1), int(v2), int(v3), int(v4), '\n')
        self.ser.write(byte_string)

    def read_response(self):
        byte_string = self.ser.readline()
        data = struct.unpack("Iiiiicc", byte_string)
        return data[:-2]

class DriverInterfaceNode:
    def __init__(self, arg):
        # Initialize node as ROS node.
        rospy.init_node("driver_interface_node", arg)

        # Find and open Arduino.
        dev_keyword = rospy.get_param("/communication/arduino/dev_keyword")
        id_keyword = rospy.get_param("/communication/arduino/id_keyword")
        arduino_id = rospy.get_param("/communication/arduino/driver/id")
        self.port_name = query_arduino(dev_keyword, id_keyword, arduino_id)
        rospy.loginfo(self.port_name)
        self.baudrate = rospy.get_param("/communication/arduino/driver/baudrate")
        # try:
        self.serial = DriverSerialHandler(self.port_name, self.baudrate)
        rospy.loginfo("Established serial connection with Driver Arduino.")
        # except:
        #     rospy.logerr("Failed to establish serial connection with Driver Arduino.")

        # Initialize quad-mecanum-wheel-kinematics solver.
        kinematics_config_dict = rospy.get_param("/kinematics/wheel_info")
        self.quad_mecanum_kinematics = QuadMecanumKinematics(kinematics_config_dict)

        # Create publisher and subscriber.
        self.publisher = rospy.Publisher("/wheels_vel", QuadMecanumWheel, queue_size=1000)
        self.subscriber = rospy.Subscriber("/cmd_vel", Pose2D, queue_size=1000, callback=self.cmd_vel_callback)

        # Declare variables to store encoder values and velocities.
        self.time_curr = self.time_prev = 0
        self.t1_curr = self.t2_curr = self.t3_curr = self.t4_curr = 0
        self.t1_prev = self.t2_prev = self.t3_prev = self.t4_prev = 0
        self.v1_target = self.v2_target = self.v3_target = self.v4_target = 0
        self.v1_sysout = self.v2_sysout = self.v3_sysout = self.v4_sysout = 0

        rospy.loginfo("(driver_interface_node) Driver-Arduino interface node is ready.")
        
    def cmd_vel_callback(self, msg):
        """
        Arg 'msg' is a Pose2D-type.
        """
        # Write veocity to Arduino.
        self.v1_target, self.v2_target, self.v3_target, self.v4_target = self.quad_mecanum_kinematics.compute_wheel_vel(msg)
        # print(self.v1_target, self.v2_target, self.v3_target, self.v4_target)
        # rospy.loginfo("%6d, %6d, %6d, %6d" % (self.v1_target, self.v2_target, self.v3_target, self.v4_target))
        self.serial.write_velocity(self.v1_target, self.v2_target, self.v3_target, self.v4_target)

        # # Receive encoder position and calculate rotational velocity of each wheel.
        # self.time_curr, self.t1_curr, self.t2_curr, self.t3_curr, self.t4_curr = self.serial.read_response()
        # self.v1_sysout = (self.t1_curr - self.t1_prev) / (self.time_curr - self.time_prev)
        # self.v2_sysout = (self.t2_curr - self.t2_prev) / (self.time_curr - self.time_prev)
        # self.v3_sysout = (self.t3_curr - self.t3_prev) / (self.time_curr - self.time_prev)
        # self.v4_sysout = (self.t4_curr - self.t4_prev) / (self.time_curr - self.time_prev)

        # # Publish received angles to topic.
        # msg = QuadMecanumWheel()
        # msg.time_diff = self.time_curr - self.time_prev
        # msg.wheel1_val = self.t1_curr
        # msg.wheel2_val = self.t2_curr
        # msg.wheel3_val = self.t3_curr
        # msg.wheel4_val = self.t4_curr
        # self.publisher.publish(msg)
    
    def exec_loop(self, frequency=10):
        loop_rate = rospy.Rate(frequency)
        while not rospy.is_shutdown():
            rospy.spin()


def main(arg):
    # Get parameters to seek Arduino.
    node = DriverInterfaceNode(arg)
    node.exec_loop(16)
    
if __name__ == "__main__":
    try:
        main(sys.argv)
    except rospy.ROSInterruptException:
        pass