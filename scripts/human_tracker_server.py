#!/usr/bin/python

import numpy as np

import rospy
import cv_bridge
from porterble.srv import *
from porterble.msg import BoundingBox

from perception.detection_api import *

class HumanTrackerServer:
    def __init__(self):
        self.odapi = DetectorAPI("/home/susung/ros_workspace/src/porterble/scripts/perception/faster_rcnn_inception_v2_coco_2018_01_28/frozen_inference_graph.pb")
        rospy.init_node("human_tracker_server")
        self.service = rospy.Service("human_tracker_service", HumanTracking, self.human_tracker_handler)
        self.bridge = cv_bridge.CvBridge()
        self.detection_queue = []
        rospy.loginfo("Human tracker service ready.")
    
    def human_tracker_handler(self, request):
        image = self.bridge.imgmsg_to_cv2(request.image, desired_encoding="rgb8")
        
        threshold = 0.7
        boxes, scores, classes, num = self.odapi.processFrame(image)
        response = HumanTrackingResponse()

        rospy.loginfo("%d objects detected." % len(boxes))
        print(classes)

        for i in range(len(boxes)):
            # Class 1 represents human
            if classes[i] == 1 and scores[i] > threshold:
                box = boxes[i]
                bounding_box = BoundingBox()
                bounding_box.tl.row, bounding_box.tl.col = box[0], box[1]
                bounding_box.dr.row, bounding_box.dr.col = box[2], box[3]
                self.detection_queue.append(bounding_box)
                if len(self.detection_queue) == 
                response.boxes.append(bounding_box)
        
        return response

    def exec_loop(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        server = HumanTrackerServer()
        server.exec_loop()
    except rospy.ROSInterruptException:
        pass