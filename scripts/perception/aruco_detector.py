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

class ArucoMarkerDetector:
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

        # Preset Aruco marker detection parameters.
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        self.parameters =  aruco.DetectorParameters_create()

        # Initialize camera handler.
        self.camera = camera_handler
        
    def detect_markers(self):
        frame = self.camera.get_frame()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
        if ids is not None:
            self.marker1_detected = self.marker2_detected = self.marker3_detected = False
            for idx in range(len(ids)):
                marker = ids[idx][0]
                if marker == self.marker1:
                    self.marker1_centerpoint = np.int32(np.average(corners[idx][0], axis=0))
                    self.marker1_detected = True
                elif marker == self.marker2:
                    self.marker2_centerpoint = np.int32(np.average(corners[idx][0], axis=0))
                    self.marker2_detected = True
                elif marker == self.marker3:
                    self.marker3_centerpoint = np.int32(np.average(corners[idx][0], axis=0))
                    self.marker3_detected = True
                else:
                    # rospy.logerr("Aruco marker detected, but not valid. This input is ignored...")
                    pass
                
            if self.marker1_detected and self.marker2_detected and self.marker3_detected:
                rospy.loginfo("All markers properly detected.")
            else:
                rospy.logerr("Incorrect inputs. This iteration will be neglected.")
                raise RuntimeError("Incorrect inputs. This iteration will be neglected.")
                # cv2.waitKey(1)
        else:
            rospy.logerr("No marker is detected.")
            raise RuntimeError("No marker is detected.")
        
        marker1_theta, marker1_phi = self.camera.pixel_to_spherical_angle(self.marker1_centerpoint[0], self.marker1_centerpoint[1])
        marker2_theta, marker2_phi = self.camera.pixel_to_spherical_angle(self.marker2_centerpoint[0], self.marker2_centerpoint[1])
        marker3_theta, marker3_phi = self.camera.pixel_to_spherical_angle(self.marker3_centerpoint[0], self.marker3_centerpoint[1])

        return marker1_theta, marker1_phi, marker2_theta, marker2_phi, marker3_theta, marker3_phi