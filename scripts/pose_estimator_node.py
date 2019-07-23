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
import rospy
from geometry_msgs.msg import Pose2D
from peripherals.camera_handler import CameraHandler
from perception.aruco_detector import ArucoMarkerDetector
from perception.pose_estimator import MarkerPoseEstimator

def main():
    # Initialize ROS node and advertise topics.
    pose_publisher = rospy.Publisher("docking_error", Pose2D, queue_size=1000)
    rospy.init_node("pose_estimator_node")
    loop_rate = rospy.Rate(4)

    # Get parameters from ROS parameter server.
    view_angle = rospy.get_param("/pose_estimation/camera/view_angle")
    left_marker = rospy.get_param("/pose_estimation/marker_types/left")
    middle_marker = rospy.get_param("/pose_estimation/marker_types/middle")
    right_marker = rospy.get_param("/pose_estimation/marker_types/right")
    
    x2, y2 = rospy.get_param("/pose_estimation/marker_pose/marker2")
    x3, y3 = rospy.get_param("/pose_estimation/marker_pose/marker3")
    docking_x, docking_y = rospy.get_param("/pose_estimation/docking_pose")
    marker2_pose = Pose2D(x2, y2, 0)
    marker3_pose = Pose2D(x3, y3, 0)
    docking_pose = Pose2D(docking_x, docking_y, 0)

    # Create handlers and detectors objects.
    camera_handler = CameraHandler(view_angle[0], view_angle[1])
    aruco_detector = ArucoMarkerDetector(camera_handler, left_marker, middle_marker, right_marker)
    pose_estimator = MarkerPoseEstimator(aruco_detector, marker2_pose, marker3_pose, docking_pose)

    # Compute and publish pose.
    while not rospy.is_shutdown():
        docking_error = pose_estimator.compute_docking_error()
        pose_publisher.publish(docking_error)
        loop_rate.sleep()
    
    return 0

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass