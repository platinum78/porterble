#!/usr/bin/python

"""
********************************************************************************
* Filename      : Serial Handler
* Author        : Susung Park
* Description   : Serial data handler for communication with Arduino
* Version       : Initial release; 07 Jul 2019
********************************************************************************
"""

import cv2
from cv2 import aruco
import numpy as np

import rospy

class TrackingArucoMarkerDetector:
    def __init__(self, camera_handler, marker_types):
        self.found = False
        self.marker_types = marker_types
        self.camera = camera_handler

        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        self.parameters =  aruco.DetectorParameters_create()

    def detect_markers(self):
        frame = self.camera.get_frame()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
        if ids is not None:
            for idx in range(len(ids)):
                if ids[idx] in self.marker_types:
                    self.marker_centerpoint = np.int32(np.average(corners[idx][0], axis=0))
                    self.marker_detected = True
                    break
        else:
            raise RuntimeError("No marker is detected.")
        
        marker_theta, marker_phi = self.camera.pixel_to_spherical_angle(self.marker_centerpoint[0], self.marker_centerpoint[1])

        return marker_theta, marker_phi

class DockingArucoMarkerDetector:
    def __init__(self, camera_handler, marker1_type, marker2_type, marker3_type):
        # Set the type of marker to use for each position.
        self.marker1 = marker1_type
        self.marker2 = marker2_type
        self.marker3 = marker3_type

        # Create objects to save centerpoints of each marker.
        self.marker1_centerpoint = None
        self.marker2_centerpoint = None
        self.marker3_centerpoint = None

        # Flags to display if each marker is detected.
        self.marker1_detected = False
        self.marker2_detected = False
        self.marker3_detected = False

        # Corners of each marker.
        self.marker1_corner = None
        self.marker2_corner = None
        self.marker3_corner = None

        # Preset Aruco marker detection parameters.
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        self.parameters =  aruco.DetectorParameters_create()

        # Initialize camera handler.
        self.camera = camera_handler
        
    def detect_markers(self, single_marker_mode=False):
        frame = self.camera.get_frame()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
        if ids is not None:
            self.marker1_detected = self.marker2_detected = self.marker3_detected = False
            for idx in range(len(ids)):
                marker = ids[idx][0]
                if marker == self.marker1:
                    self.marker1_centerpoint = np.int32(np.average(corners[idx][0], axis=0))
                    self.marker1_corner = corners[idx]
                    self.marker1_detected = True
                elif marker == self.marker2:
                    self.marker2_centerpoint = np.int32(np.average(corners[idx][0], axis=0))
                    self.marker2_corner = corners[idx]
                    self.marker2_detected = True
                elif marker == self.marker3:
                    self.marker3_centerpoint = np.int32(np.average(corners[idx][0], axis=0))
                    self.marker3_corner = corners[idx]
                    self.marker3_detected = True
                else:
                    rospy.logerr("Aruco marker detected, but not valid. This input is ignored...")
                    continue
                
            if self.marker1_detected and self.marker2_detected and self.marker3_detected and single_marker_mode == False:
                marker_cnt = 3
                marker1_theta, marker1_phi = self.camera.pixel_to_spherical_angle(self.marker1_centerpoint[0], self.marker1_centerpoint[1])
                marker2_theta, marker2_phi = self.camera.pixel_to_spherical_angle(self.marker2_centerpoint[0], self.marker2_centerpoint[1])
                marker3_theta, marker3_phi = self.camera.pixel_to_spherical_angle(self.marker3_centerpoint[0], self.marker3_centerpoint[1])
                return marker_cnt, marker1_theta, marker1_phi, marker2_theta, marker2_phi, marker3_theta, marker3_phi
            else:
                if self.marker2_detected:
                    rospy.loginfo("Single marker mode.")
                    marker_cnt = 1
                    left_height = np.abs(self.marker2_corner[0, 3, 1] - self.marker2_corner[0, 0, 1])
                    right_height = np.abs(self.marker2_corner[0, 2, 1] - self.marker2_corner[0, 1, 1])
                    avg_height = (left_height + right_height) / 2
                    left_pos = (self.marker2_corner[0, 3, 0] + self.marker2_corner[0, 0, 0]) / 2
                    left_avg = (self.marker2_corner[0, 3, 1] + self.marker2_corner[0, 0, 1]) / 2
                    right_pos = (self.marker2_corner[0, 1, 0] + self.marker2_corner[0, 2, 0]) / 2
                    right_avg = (self.marker2_corner[0, 1, 1] + self.marker2_corner[0, 2, 1]) / 2

                    marker2_center, marker2_phi = self.camera.pixel_to_spherical_angle(self.marker2_centerpoint[0], self.marker2_centerpoint[1])
                    marker2_left, _ = self.camera.pixel_to_spherical_angle(left_pos, left_avg)
                    marker2_right, _ = self.camera.pixel_to_spherical_angle(right_pos, right_avg)
                    print(left_height, right_height)
                    if left_height > right_height:
                        marker2_rotation = -1 + np.pi / 2
                    elif right_height > left_height:
                        marker2_rotation = 1 + np.pi / 2
                    else:
                        marker2_rotation = np.pi / 2
                    return marker_cnt, marker2_left, marker2_center, marker2_right, marker2_phi, marker2_rotation
                else:
                    rospy.logerr("Incorrect inputs. This iteration will be neglected.")
                    raise RuntimeError("Incorrect inputs. This iteration will be neglected.")
        else:
            rospy.logerr("No marker is detected.")
            raise RuntimeError("No marker is detected.")
        
