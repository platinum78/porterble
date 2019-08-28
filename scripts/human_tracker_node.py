#!/usr/bin/python

"""
********************************************************************************
* Filename      : Human Tracker Node
* Author        : Susung Park
* Description   : Captures image, requests detection, and receives result.
* Runs On       : Raspberry Pi
* Cooperators   : [human_detection_server]
* Version       : On development...

  This node will run on Raspberry Pi, capturing image from USB camera and then
  transmitting over ROS service to [human_detection_server].
********************************************************************************
"""

import numpy as np
import cv2
import time

import rospy, rospkg
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from geometry_msgs.msg import Pose2D

rospack = rospkg.RosPack()
PKG_DIR = rospack.get_path("porterble")

from peripherals.camera_handler import MarkerCameraHandler
from utils.device_identifier import query_camera
from perception.aruco_detector import TrackingArucoMarkerDetector
from utils.mck_noise_attenuator import *
from utils.polynomial_regression import *
from porterble.srv import *
from porterble.msg import *
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse


class HumanTrackerNode(object):
    def __init__(self):
        # Initialize as ROS node.
        rospy.init_node("human_tracker_node")

        # Find camera paramters from roscore.
        
        dev_keyword = rospy.get_param("/communication/camera/dev_keyword")
        id_keyword = rospy.get_param("/communication/camera/id_keyword")
        camera_id = rospy.get_param("/communication/camera/human_tracker/id")
        img_width = rospy.get_param("/communication/camera/human_tracker/img_width")
        img_height = rospy.get_param("/communication/camera/human_tracker/img_height")
        angle_horizontal, angle_vertical = rospy.get_param("/human_tracking/camera/view_angle")
        marker_types = rospy.get_param("/human_tracking/marker_types")

        # Open camera handler.
        camera_idx = query_camera(dev_keyword, id_keyword, camera_id)
        self.camera_handler = MarkerCameraHandler(camera_idx, img_width, img_height, angle_horizontal, angle_vertical)
        self.aruco_detector = TrackingArucoMarkerDetector(self.camera_handler, marker_types)
        rospy.loginfo("(human_tracker_node) Human tracking camera is initialized with index %d" % camera_idx)

        # Publish tracking data, when tracking object is set.
        self.human_position_publisher = rospy.Publisher("/human_position", Pose2D, queue_size=100)

        # Subscribe to /ranges topic.
        self.ranges_subscriber = rospy.Subscriber("/ranges", SonarRange, self.ranges_callback)
        
        self.trigger_service = rospy.Service("/human_tracker_trigger_service", SetBool, self.trigger_service_handler)
        self.active = False

        # self.mck_noise_attenuator = MCKNoiseAttenuator()
        time.sleep(2)
    
    def ranges_callback(self, msg):
        self.ranges = msg.range
    
    def trigger_service_handler(self, request):
        self.active = request.data
        return SetBoolResponse(True, "")
    
    def publish_human_tracker_result(self):
        """
        Recognizes markers from image and publishes the result.
        """
        if self.active:
            try:
                frame = self.camera_handler.get_frame()
                theta, phi = self.aruco_detector.detect_markers()
                msg = Pose2D(0.2, 0, theta)
                rospy.loginfo("(human_tracker_node) Current angle: %f" % theta)
                self.human_position_publisher.publish(msg)
            except RuntimeError:
                rospy.logerr("(human_tracker_node) No marker detected. Publishing null values.")
                self.human_position_publisher.publish(Pose2D(0, 0, 0))

    def exec_loop(self, frequency=10):
        rospy.loginfo("Human tracker node started.")
        loop_rate = rospy.Rate(frequency)
        while not rospy.is_shutdown():
            self.publish_human_tracker_result()
            loop_rate.sleep()

if __name__ == "__main__":
    try:
        node = HumanTrackerNode()
        node.exec_loop(frequency=4)
    except rospy.ROSInterruptException:
        pass
