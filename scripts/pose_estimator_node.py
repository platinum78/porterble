#!/usr/bin/python

"""
********************************************************************************
* Filename      : Aruco Marker Detector Node
* Author        : Susung Park
* Description   : Aruco marker detector for pose estimation.
* Version       : Beta Version 08 AUG 2019

  pose_estimator_server receives requests from other nodes to estimate current
  pose of the cart. It directly receives image from camera, extracts 
********************************************************************************
"""

import os, sys
import numpy as np

import rospy
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from geometry_msgs.msg import Pose2D
from porterble.srv import *

from utils.device_identifier import query_camera
from peripherals.camera_handler import MarkerCameraHandler
from perception.aruco_detector import DockingArucoMarkerDetector
from perception.pose_estimator import MarkerPoseEstimator


class DockingPoseEstimatorNode:
    def __init__(self):
        # Initialize ROS node and advertise topics.
        rospy.init_node("docking_pose_estimator_node")

        # Get parameters from ROS parameter server.
        rospy.loginfo("(pose_estimator_node) Loading recognition parameters...")
        self.view_angle = rospy.get_param("/pose_estimation/camera/view_angle")
        self.img_width = rospy.get_param("/communication/camera/marker_tracker/img_width")
        self.img_height = rospy.get_param("/communication/camera/marker_tracker/img_height")
        self.left_marker = rospy.get_param("/pose_estimation/marker_types/left")
        self.middle_marker = rospy.get_param("/pose_estimation/marker_types/middle")
        self.right_marker = rospy.get_param("/pose_estimation/marker_types/right")
        self.x2, self.y2 = rospy.get_param("/pose_estimation/marker_pose/marker2")
        self.x3, self.y3 = rospy.get_param("/pose_estimation/marker_pose/marker3")
        self.docking_x, self.docking_y = rospy.get_param("/pose_estimation/docking_pose")
        
        self.marker2_pose = Pose2D(self.x2, self.y2, 0)
        self.marker3_pose = Pose2D(self.x3, self.y3, 0)
        self.docking_pose = Pose2D(self.docking_x, self.docking_y, 0)
        
        rospy.loginfo("(pose_estimator_node) Loading camera interface parameters...")
        dev_keyword = rospy.get_param("/communication/camera/dev_keyword")
        id_keyword = rospy.get_param("/communication/camera/id_keyword")
        camera_id = rospy.get_param("/communication/camera/marker_tracker/id")
        camera_idx = query_camera(dev_keyword, id_keyword, camera_id)
        
        # Make trigger service.
        self.active = False
        self.trigger_service = rospy.Service("/pose_estimator_trigger_service", SetBool, self.trigger_service_handler)

        # Create handlers and detectors objects.
        self.camera_handler = MarkerCameraHandler(camera_idx, self.img_width, self.img_height, self.view_angle[0], self.view_angle[1])
        self.aruco_detector = DockingArucoMarkerDetector(self.camera_handler, self.left_marker, self.middle_marker, self.right_marker)
        self.pose_estimator = MarkerPoseEstimator(self.aruco_detector, self.marker2_pose, self.marker3_pose, self.docking_pose)

        # Finally, advertise service.
        self.publisher = rospy.Publisher("/docking_error", Pose2D, queue_size=1000)

    # Service handler.
    def trigger_service_handler(self, request):
        rospy.loginfo("(pose_estimator_node) Changed mode")
        self.pose_estimator.single_marker_mode = False
        self.active = request.data
        return SetBoolResponse(True, "")
    
    def publish_docking_error(self):
        if self.active:
            try:
                docking_error = self.pose_estimator.compute_docking_error()
                self.publisher.publish(docking_error)
            except RuntimeError:
                rospy.logerr("(pose_estimator_node) Improper results. Publishing null values.")
                self.publisher.publish(Pose2D(0, 0, 0))
    
    # Serve until shutdown.
    def exec_loop(self, frequency=10):
        self.loop_rate = rospy.Rate(frequency)
        rospy.loginfo("(pose_estimator_node) Docking pose estimator node up.")
        while not rospy.is_shutdown():
            self.publish_docking_error()
            self.loop_rate.sleep()


if __name__ == "__main__":
    try:
        node = DockingPoseEstimatorNode()
        node.exec_loop()
    except Exception, e:
        rospy.logerr(e)
        pass