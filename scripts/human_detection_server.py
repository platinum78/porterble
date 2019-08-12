#!/usr/bin/python

"""
********************************************************************************
* Filename      : Human Detection Server
* Author        : Susung Park
* Description   : Detects humans, triggered by service requests.
* Runs on       : External Workstation
* Version       : On development...

  This node will run on external Linux server, receiving camera frames from
  requests, detecting humans from frames, and then returning bounding boxes
  to source node.
********************************************************************************
"""

import numpy as np

import rospy, rospkg
import cv_bridge
from porterble.srv import *
from porterble.msg import BoundingBox

from perception.detection_api import *

rospack = rospkg.RosPack()
PKG_DIR = rospack.get_path("porterble")

class HumanTrackerServer:
    def __init__(self):
        # Initiate node and detector API.
        rospy.init_node("human_detection_server")
        self.odapi = DetectorAPI(PKG_DIR + "/scripts/perception/faster_rcnn_inception_v2_coco_2018_01_28/frozen_inference_graph.pb")

        # Establish services.
        self.human_tracker_service = rospy.Service("/human_detection_service", HumanTracking, self.human_tracker_handler)
        self.get_detection_image_service = rospy.Service("/get_detection_image_service", DetectionImage, self.get_detection_image_handler)

        self.bridge = cv_bridge.CvBridge()
        self.detection_queue = []
        rospy.loginfo("Human tracker service ready.")
    
    def human_tracker_handler(self, request):
        image = self.bridge.imgmsg_to_cv2(request.image)
        
        threshold = 0.7
        boxes, scores, classes, num = self.odapi.processFrame(image)
        rospy.loginfo("%d objects detected." % len(classes))
        response = HumanTrackingResponse()

        for i in range(len(boxes)):
            # Class 1 represents human
            if classes[i] == 1 and scores[i] > threshold:
                box = boxes[i]
                bounding_box = BoundingBox()
                bounding_box.tl.row, bounding_box.tl.col = box[0], box[1]
                bounding_box.dr.row, bounding_box.dr.col = box[2], box[3]
                self.detection_queue.append(bounding_box)
                if len(self.detection_queue) == 11:
                    self.detection_queue.pop(0)
                response.boxes.append(bounding_box)
        
        return response
    
    def get_detection_image_handler(self, request):
        image = self.bridge.imgmsg_to_cv2(request.original_image)
        boxes, scores, classes, num = self.odapi.processFrame(image)
        threshold = 0.7
        image_idx = 0
        response = DetectionImageResponse()

        for i in range(len(boxes)):
            # Class 1 represents human
            if classes[i] == 1 and scores[i] > threshold:
                image_idx += 1
                box = boxes[i]
                bounding_box = BoundingBox()
                bounding_box.tl.row, bounding_box.tl.col = box[0], box[1]
                bounding_box.dr.row, bounding_box.dr.col = box[2], box[3]
                response.boxes.append(bounding_box)
                cv2.rectangle(image,(box[1],box[0]),(box[3],box[2]),(0,255,0),5)
                cv2.putText(image, str(image_idx), (box[1]+10,box[2]-15), cv2.FONT_HERSHEY_COMPLEX, 3, (0,255,0),5)
        
        msg = self.bridge.cv2_to_imgmsg(image)
        response.detected_image = msg
        return response
        
    def exec_loop(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        server = HumanTrackerServer()
        server.exec_loop()
    except rospy.ROSInterruptException:
        pass