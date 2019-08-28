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

class ImageBuffer:
    def __init__(self):
        self.image = None
    
    def set_image(self, image):
        self.image = image

class CameraHandler(object):
    def __init__(self, camera_idx, img_width, img_height):
        self.camera = cv2.VideoCapture(camera_idx)
        # rospy.loginfo(self.camera.get(cv2.CAP_PROP_FPS))
        # self.camera.set(cv2.CAP_PROP_FPS, 15)
        # self.camera.set(cv2.CAP_PROP_BUFFERSIZE, 2)
        self.camera.set(3, img_width)
        self.camera.set(4, img_height)
        self.resolution = [img_width, img_height]

        self.event = threading.Event()
        self.image_buffer = ImageBuffer()
        
        self.thread = threading.Thread(target=self.stream_camera, args=(self.image_buffer,))
        self.thread.daemon = True
        self.thread.start()
        self.start_stream()
    
    def stream_camera(self, img_buffer):
        while True:
            self.event.wait()
            while self.event.is_set():
                _, frame = self.camera.read()
                img_buffer.set_image(frame)
                cv2.waitKey(1)

    def start_stream(self):
        self.event.set()
    
    def stop_stream(self):
        self.event.clear()
    
    def get_frame(self):
        return self.image_buffer.image

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