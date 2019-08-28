#!/usr/bin/python

"""
********************************************************************************
* Filename      : Marker Pose Detector
* Author        : Susung Park
* Description   : Detects 2-D pose of robot, given three angles of each marker.
* Version       : Initial release; 07 Jul 2019
********************************************************************************
"""

import os, sys
import numpy as np

import rospy
from geometry_msgs.msg import Pose2D

class MarkerPoseEstimator:
    def __init__(self, aruco_detector, marker2, marker3, docking_pose):
        # Get marker and docking positions.
        self.marker2 = marker2
        self.marker3 = marker3
        self.docking_pose = docking_pose

        # Origin of camera.
        self.offset = Pose2D(0, 0, 0)

        # Initialize Aruco marker detector.
        self.detector = aruco_detector
        
        self.single_marker_mode = False

    def estimate_pose(self):
        return_val = self.detector.detect_markers(self.single_marker_mode)
        if return_val[0] == 3:
            _, t1, _, t2, _, t3, _ = self.detector.detect_markers()
            x2, y2 = self.marker2.x, self.marker2.y
            x3, y3 = self.marker3.x, self.marker3.y
            alpha2 = t2 - np.arctan2(y2, x2)
            alpha3 = t3 - np.arctan2(y3, x3)
            A2 = np.sqrt(x2**2 + y2**2) / np.sin(t2 - t1)
            A3 = np.sqrt(x3**2 + y3**2) / np.sin(t3 - t1)
            tan_theta = (A3 * np.sin(alpha3) - A2 * np.sin(alpha2)) / (A3 * np.cos(alpha3) - A2 * np.cos(alpha2))
            theta = np.arctan(tan_theta)
            omega = -theta
            if omega < 0:
                omega += np.pi

            d1 = (y3 * np.cos(omega + t3) - x3 * np.sin(omega + t3)) / np.sin(t3 - t1)
            d1_ = (y2 * np.cos(omega + t2) - x2 * np.sin(omega + t2)) / np.sin(t2 - t1)
            d2 = (y2 * np.cos(omega + t1) - x2 * np.sin(omega + t1)) / np.sin(t2 - t1)
            d3 = (y3 * np.cos(omega + t1) - x3 * np.sin(omega + t1)) / np.sin(t3 - t1)

            # Verify numerical stability by comparing d1 and d1_.
            if abs(d1 - d1_) > 0.0001:
                rospy.logerr("Solution is numerically unstable. This iteration will be neglected.")
                raise RuntimeError("Solution is numerically unstable. This iteration will be neglected.")
            else:
                mat_A = np.array([[ 2 * x2, 2 * y2 ],
                                [ 2 * x3, 2 * y3 ]])
                vec_b = np.array([[ d1**2 - d2**2 + x2**2 + y2**2 ],
                                [ d1**2 - d3**2 + x3**2 + y3**2 ]])
                vec_x = np.matmul(np.linalg.inv(mat_A), vec_b)
                x = vec_x[0, 0]
                y = vec_x[1, 0]
                
                self.offset.x = x
                self.offset.y = y
                self.offset.theta = omega
        elif return_val[0] == 1:
            self.single_marker_mode = True
            _, t2l, t2c, t2r, _, rot = return_val
            if t2c < -0.05:
                self.offset.x = -1 + self.docking_pose.x
            elif t2c > 0.05:
                self.offset.x = 1 + self.docking_pose.x
            else:
                self.offset.x = self.docking_pose.x
            self.offset.y = -1
            self.offset.theta = rot
        
    def compute_docking_error(self):
        try:
            self.estimate_pose()
            dx = self.docking_pose.x - self.offset.x
            dy = self.docking_pose.y - self.offset.y
            dt = np.pi / 2 - self.offset.theta
            return Pose2D(dx, dy, dt)
        except Exception, e:
            rospy.logerr(e)
            rospy.logerr("(%s) Perception error! Publishing null value." % rospy.get_name())
            return Pose2D(0, 0, 0)
    
    def __str__(self):
        return "Offset: [%.6f, %.6f], Rotation: %.6f" % (self.offset.x, self.offset.y, self.offset.t * 180 / np.pi)
