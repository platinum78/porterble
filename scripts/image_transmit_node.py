#!/usr/bin/python

import numpy as np
import cv2

import rospy
from sensor_msgs.msg import Image

from peripherals.camera_handler import CameraHandler
from utils.device_identifier import query_camera


class HumanTrackingCameraHandler(CameraHandler):
    def __init__(self, camera_idx):
        super(HumanTrackingCameraHandler, self).__init__(camera_idx)
    
    def get_resized_frame(self, width, height):
        frame = cv2.resize(self.get_frame(), [width, height])
        return frame


class ImageTransmitNode(object):
    def __init__(self):
        # Initialize as ROS node.
        rospy.init_node("image_transmit_node")

        # Find camera paramters from roscore and open camera handler.
        dev_keyword = rospy.get_param("/communication/camera/dev_keyword")
        id_keyword = rospy.get_param("/communication/camera/id_keyword")
        camera_id = rospy.get_param("/communication/camera/human_tracker/id")
        camera_idx = query_camera(dev_keyword, camera_id)
        self.camera_handler = HumanTrackingCameraHandler(camera_idx)

        # Find image parameters from roscore.
        self.img_width = rospy.get_param("/communication/camera/human_tracker/img_width")
        self.img_height = rospy.get_param("/communication/camera/human_tracker/img_height")

        # Establish and advertise topic for sending images.
        publisher = rospy.Publisher("/human_tracking_image", Image, queue_size=10)
    
    def publish_image(self):
        frame = self.camera_handler.get_resized_frame(self.img_width, self.img_height)
        msg = Image()
        width, height = frame.shape
        msg.data = np.reshape(frame, [1, -1])
    
    def exec_loop(self, frequency=2):
        loop_rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            self.publish_image()
            loop_rate.sleep()
    
def main():
    image_transmit_node = ImageTransmitNode()
    image_transmit_node.exec_loop(frequency=2)

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
