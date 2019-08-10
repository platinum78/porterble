#!/usr/bin/python

import numpy as np
import cv2

import rospy
from sensor_msgs.msg import Image
import cv_bridge

from peripherals.camera_handler import CameraHandler
from utils.device_identifier import query_camera
from porterble.srv import *


class HumanTrackingCameraHandler(CameraHandler):
    def __init__(self, camera_idx, img_width, img_height):
        super(HumanTrackingCameraHandler, self).__init__(camera_idx, img_width, img_height)


class ImageTransmitNode(object):
    def __init__(self):
        # Initialize as ROS node.
        rospy.init_node("image_transmit_node")

        # Find camera paramters from roscore and open camera handler.
        dev_keyword = rospy.get_param("/communication/camera/dev_keyword")
        id_keyword = rospy.get_param("/communication/camera/id_keyword")
        camera_id = rospy.get_param("/communication/camera/human_tracker/id")
        camera_idx = query_camera(dev_keyword, id_keyword, camera_id)
        img_width = rospy.get_param("/communication/camera/human_tracker/img_width")
        img_height = rospy.get_param("/communication/camera/human_tracker/img_height")
        self.camera_handler = HumanTrackingCameraHandler(camera_idx, img_width, img_height)

        # Find image parameters from roscore.
        self.img_width = rospy.get_param("/communication/camera/human_tracker/img_width")
        self.img_height = rospy.get_param("/communication/camera/human_tracker/img_height")

        # Establish and advertise topic for sending images.
        rospy.wait_for_service("human_tracker_service")
        self.human_tracker_service = rospy.ServiceProxy("human_tracker_service", HumanTracking)
        self.bridge = cv_bridge.CvBridge()

        rospy.loginfo("Image transmit node started.")
    
    def request_detection(self):
        frame = self.camera_handler.get_frame()
        cv2.imshow("window", frame)
        key = cv2.waitKey(1)
        msg = self.bridge.cv2_to_imgmsg(frame, encoding="rgb8")
        response = self.human_tracker_service(msg)
        return response.boxes
    
    def exec_loop(self, frequency=2):
        loop_rate = rospy.Rate(frequency)
        while not rospy.is_shutdown():
            response = self.request_detection()
            rospy.loginfo("Responses: \n")
            print(response)
            loop_rate.sleep()

if __name__ == "__main__":
    try:
        image_transmit_node = ImageTransmitNode()
        image_transmit_node.exec_loop(frequency=5)
    except rospy.ROSInterruptException:
        pass
