#!/usr/bin/python

"""
********************************************************************************
* Filename      : Master Control Node
* Author        : Susung Park
* Description   : The master node that determines everyting about Porterble.
* Version       : On development; 08 AUG 2019
********************************************************************************
"""

import rospy, rospkg
from geometry_msgs.msg import Pose2D
from porterble.srv import *

rospack = rospkg.RosPack()
PKG_DIR = rospack.get_path("porterble")

MANUAL =    0
FOLLOW =    1
DOCKING =   2

class MasterControlNode(object):
    def __init__(self):
        rospy.init_node("master_control_node")
        self.change_mode_service = rospy.Service("/change_mode_service", ChangeMode, self.change_mode_service_handler)
        # State variables.
        self.operation_mode = MANUAL
        rospy.loginfo("Master control node started.")
    
    def change_mode_service_handler(self, request):
        self.runmode = request.mode
        rospy.loginfo("Received request " + str(self.runmode))
        response = ChangeModeResponse(ack=True)
        return response

    def exec_loop(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        node = MasterControlNode()
        node.exec_loop()
    except:
        pass