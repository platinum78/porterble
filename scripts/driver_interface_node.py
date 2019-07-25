import os, sys, struct
from serial.serialutil import SerialException

import rospy
from geometry_msgs.msg import Pose2D
from porterble.msg import QuadMecanumWheel

from utils.serial_handler import SerialHandler
from utils.device_identifier import identify_arduinos, query_arduino

from controls.kinematics import QuadMecanumKinematics

class DriverSerialHandler(SerialHandler):
    def __init__(self, port_name, baudrate):
        super(DriverSerialHandler, self).__init__(port_name, baudrate)
    
    def write_velocity(self, v1, v2, v3, v4):
        byte_string = struct.pack("hhhhc", int(v1), int(v2), int(v3), int(v4), '\n')
        self.ser.write(byte_string)

    def read_response(self):
        byte_string = self.ser.readline()
        data = struct.unpack("Ihhhhcc", byte_string)
        return data[:-2]

class DriverInterface:
    def __init__(self):
        # Initialize node as ROS node, and create subscriber.
        rospy.init_node("driver_interface_node", arg)
        self.publisher = rospy.Publisher("/vehicle_vel", Pose2D, queue_size=1000)
        self.subscriber = rospy.Subscriber("/cmd_vel", Pose2D, queue_size=1000, callback=self.cmd_vel_callback)

        # Find and open Arduino.
        seek_keyword = rospy.get_param("/communication/arduino/seek_keyword")
        arduino_id = rospy.get_param("/communication/arduino/driver/id")
        self.port_name = query_arduino(seek_keyword, arduino_id)
        self.baudrate = rospy.get_param("/communication/arduino/driver/baudrate")
        self.serial = DriverSerialHandler(port_name, baudrate)
        rospy.loginfo("Established serial connection with Driver Arduino.")

        # Initialize quad-mecanum-wheel-kinematics solver.
        kinematics_config_dict = rospy.get_param("/kinematics/wheel_info")
        self.quad_mecanum_kinematics = QuadMecanumKinematics(kinematics_config_dict)

    def cmd_vel_callback(self, msg):
        """
        Arg 'msg' is a Pose2D-type.
        """
        self.v1_request, self.v2_request, self.v3_request, self.v4_request = self.quad_mecanum_kinematics.compute_wheel_vel(msg)
    
    def write_velocity(self):
        msg = QuadMecanumWheel()
        self.serial.write_velocity(self.v1_request, self.v2_request, self.v3_request, self.v4_request)
        msg.time_diff, msg.wheel1_val, msg.wheel2_val, msg.wheel3_val, msg.wheel4_val = self.serial.read_response()
        self.publisher.publish(msg)
    
    def exec_loop(self, frequency=10):
        loop_rate = rospy.Rate(frequency)


def main(arg):
    # Get parameters to seek Arduino.

    while not rospy.is_shutdown():
        pass
    
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass