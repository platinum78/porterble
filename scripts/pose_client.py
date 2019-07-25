#!/usr/bin/python

import os, sys, time

import rospy
from porterble.srv import *

if __name__ == "__main__":
    try:
        rospy.wait_for_service("pose_by_marker_service")
        pose_by_marker = rospy.ServiceProxy("pose_by_marker_service", PoseByMarker)

        while not rospy.is_shutdown():
            pose = pose_by_marker()
            rospy.loginfo(pose)
            time.sleep(1)
            
    except rospy.ROSInterruptException:
        pass