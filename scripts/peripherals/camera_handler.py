#!/usr/bin/python3

"""
********************************************************************************
* Filename      : Camera Handler
* Author        : Susung Park
* Description   : Camera handler.
* Version       : On development...
********************************************************************************
"""

import os, sys, threading, time
import cv2
from cv2 import aruco
import numpy as np

class CameraHandler(object):
    def __init__(self, camera_idx):
        self.alpha = angle_horizontal / 2
        self.beta = angle_vertical / 2
        self.camera = cv2.VideoCapture(camera_idx)
        self.resolution = [int(self.camera.get(3)), int(self.camera.get(4))]
    
    def get_frame(self):
        ret, frame = self.camera.read()
        return frame

if __name__ == "__main__":
    camera = MarkerCameraHandler(1.1927, 0.7053)
    camera.start_stream()