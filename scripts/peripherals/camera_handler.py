#!/usr/bin/python

"""
********************************************************************************
* Filename      : Camera Handler
* Author        : Susung Park
* Description   : Camera handler.
* Version       : Beta Version; 08 AUG 2019
********************************************************************************
"""

import os, sys, threading, time
import cv2
from cv2 import aruco
import numpy as np

import rospy

class CameraHandler(object):
    def __init__(self, camera_idx, img_width, img_height):
        self.camera = cv2.VideoCapture(camera_idx)
        self.camera.set(3, img_width)
        self.camera.set(4, img_height)
        self.resolution = [img_width, img_height]
    
    def get_frame(self):
        ret, frame = self.camera.read()
        rospy.loginfo("Read frame")
        return frame

if __name__ == "__main__":
    camera = MarkerCameraHandler(1.1927, 0.7053)
    camera.start_stream()