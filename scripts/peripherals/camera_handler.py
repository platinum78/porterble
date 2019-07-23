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

class CameraHandler:
    def __init__(self, angle_horizontal, angle_vertical):
        self.alpha = angle_horizontal / 2
        self.beta = angle_vertical / 2
        self.camera = cv2.VideoCapture(0)
        self.resolution = [int(self.camera.get(3)), int(self.camera.get(4))]
    
    def get_frame(self):
        ret, frame = self.camera.read()
        return frame
    
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

if __name__ == "__main__":
    camera = CameraHandler(1.1927, 0.7053)
    camera.start_stream()