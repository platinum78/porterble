#!/usr/bin/python

"""
********************************************************************************
* Filename      : Aruco Marker Detector Node
* Author        : Susung Park
* Description   : Aruco marker detector for pose estimation.
* Version       : On development...
********************************************************************************
"""

import os, sys
import numpy as np

import rospy
from geometry_msgs.msg import Pose2D
from porterble.srv import *


from peripherals.camera_handler import CameraHandler
from perception.aruco_detector import ArucoMarkerDetector
from perception.pose_estimator import MarkerPoseEstimator

class MarkerCameraHandler(CameraHandler):
    def __init__(self, camera_idx, img_width, img_height, angle_horizontal, angle_vertical):
        super(MarkerCameraHandler, self).__init__(camera_idx, img_width, img_height)
        self.alpha = angle_horizontal / 2
        self.beta = angle_vertical / 2
        rospy.loginfo("Initialized camera handler.")
    
    def pixel_to_spherical_angle(self, col, row):
        width, height = self.resolution
        y = width / 2 - col
        z = height / 2 - row

        d1 = width / (2 * np.tan(self.alpha))
        d2 = height / (2 * np.tan(self.beta))

        d = (d1 + d2) / 2

        theta = np.arctan(y / d)
        phi = np.arctan(z / d * np.cos(theta))

        return theta, phi

class PoseEstimatorService:
    def __init__(self):
        # Initialize ROS node and advertise topics.
        rospy.init_node("pose_by_marker_server")

        # Get parameters from ROS parameter server.
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

        # Create handlers and detectors objects.
        self.camera_handler = MarkerCameraHandler(1, self.img_width, self.img_height, self.view_angle[0], self.view_angle[1])
        self.aruco_detector = ArucoMarkerDetector(self.camera_handler, self.left_marker, self.middle_marker, self.right_marker)
        self.pose_estimator = MarkerPoseEstimator(self.aruco_detector, self.marker2_pose, self.marker3_pose, self.docking_pose)

        # Finally, advertise service.
        s = rospy.Service("pose_by_marker_service", PoseByMarker, self.service_handler)

    # Service handler.    
    def service_handler(self, request):
        docking_error = self.pose_estimator.compute_docking_error()
        return PoseByMarkerResponse(docking_error)
    
    # Serve until shutdown.
    def serve(self):
        rospy.loginfo("Pose estimator service ready.")
        rospy.spin()


if __name__ == "__main__":
    try:
        service = PoseEstimatorService()
        service.serve()
    except rospy.ROSInterruptException:
        pass