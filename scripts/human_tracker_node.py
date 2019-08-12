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

import rospy, rospkg
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
import cv_bridge

rospack = rospkg.RosPack()
PKG_DIR = rospack.get_path("porterble")

from peripherals.camera_handler import CameraHandler
from utils.device_identifier import query_camera
from porterble.msg import BoundingBox
from porterble.srv import *
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse


class HumanTrackingCameraHandler(CameraHandler):
    def __init__(self, camera_idx, img_width, img_height):
        super(HumanTrackingCameraHandler, self).__init__(camera_idx, img_width, img_height)


class HumanTrackerNode(object):
    def __init__(self):
        # Initialize as ROS node.
        rospy.init_node("human_tracker_node")

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

        # Get connection to service.
        rospy.wait_for_service("/human_detection_service")
        rospy.wait_for_service("/get_detection_image_service")
        self.human_detection_service = rospy.ServiceProxy("/human_detection_service", HumanTracking)
        self.get_detection_image_service = rospy.ServiceProxy("/get_detection_image_service", DetectionImage)
        self.bridge = cv_bridge.CvBridge()

        # Establish service to get image request.
        self.get_bbox_image_service = rospy.Service("/get_bbox_image_service", HumanTracking, self.get_bbox_image_handler)
        self.set_tracking_obj_service = rospy.Service("/set_tracking_obj_service", SetTrackingObj, self.set_tracking_obj_handler)
        
        # Publish tracking data, when tracking object is set.
        self.publisher = rospy.Publisher("/human_position", Point, queue_size=100)

        # Set a storage for currently-tracking object.
        # This will act as a trigger switch, which will block while it is None.
        self.tracking_obj = None
        
        rospy.loginfo("Image transmit node started.")
    
    def human_tracker_handler(self, request):
        frame = self.camera_handler.get_frame()
        cv2.imshow("window", frame)
        key = cv2.waitKey(1)
        msg = self.bridge.cv2_to_imgmsg(frame)
        response = self.human_tracker_service(msg)
        return response.boxes

    def get_bbox_image_handler(self, request):
        """
        Caller: [http_watcher_node]
        """
        frame = self.camera_handler.get_frame()
        msg = self.bridge.cv2_to_imgmsg(frame)
        response = self.get_detection_image_service(msg)
        image = self.bridge.imgmsg_to_cv2(response.detected_image)
        cv2.imwrite(PKG_DIR + "/.temp/detected_image.png", image)
        self.bbox_list = response.boxes
        return HumanTrackingResponse(response.boxes)

    def set_tracking_obj_handler(self, request):
        """
        Caller: [http_watcher_node]
        """
        obj_idx = request.obj_idx - 1
        self.tracking_obj = request.obj_idx - 1
        return SetTrackingObjResponse(True)

    def exec_loop(self, frequency=2):
        loop_rate = rospy.Rate(frequency)
        while not rospy.is_shutdown():
            if self.tracking_obj is not None:
                # Code for calculating human position.
                self.publisher.publish(msg)
                loop_rate.sleep()

if __name__ == "__main__":
    try:
        node = HumanTrackerNode()
        node.exec_loop(frequency=5)
    except rospy.ROSInterruptException:
        pass
