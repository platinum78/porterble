#!/usr/bin/python

"""
********************************************************************************
* Filename      : HTTP Watcher Node
* Author        : Susung Park
* Description   : Listens for user input on runmode.
* Version       : On development...
********************************************************************************
"""

import os
import BaseHTTPServer

import rospy, rospkg
from geometry_msgs.msg import Pose2D
from porterble.srv import *
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse

rospack = rospkg.RosPack()
PKG_DIR = rospack.get_path("porterble")

MANUAL      = 0
FOLLOW      = 1
DOCKING     = 2

def make_server_handler(service_dict):
    class ServerHandler(BaseHTTPServer.BaseHTTPRequestHandler):
        change_mode_service = service_dict["change_mode_service"]
        set_manual_vel_service = service_dict["set_manual_vel_service"]

        def do_GET(self):
            rospy.loginfo("Inbound request.")
            if "changeMode" in self.path:
                if "manual" in self.path:
                    self.send_response(200)
                    self.end_headers()
                    self.change_mode_service(MANUAL)
                elif "follow" in self.path:
                    self.send_response(200)
                    self.end_headers()
                    self.change_mode_service(FOLLOW)
                elif "docking" in self.path:
                    self.send_response(200)
                    self.end_headers()
                    self.change_mode_service(DOCKING)
            elif "manualMove" in self.path:
                if "stop" in self.path:
                    rospy.loginfo("(http_watcher_node) STOP request")
                    self.set_manual_vel_service(Pose2D(0, 0, 0))
                    self.send_response(200)
                    self.end_headers()
                else:
                    speed = float(self.path[self.path.index("speed") + 6 : -1])
                    velocity = None
                    rospy.loginfo("(http_watcher_node) speed: %d" % speed)
                    if "front" in self.path:
                        rospy.loginfo("(http_watcher_node) FRONT request")
                        velocity = Pose2D(speed / 1000, 0, 0)
                    elif "right" in self.path:
                        rospy.loginfo("(http_watcher_node) RIGHT request")
                        velocity = Pose2D(0, speed / 1000, 0)
                    elif "left" in self.path:
                        rospy.loginfo("(http_watcher_node) LEFT request")
                        velocity = Pose2D(0, -speed / 1000, 0)
                    elif "rear" in self.path:
                        rospy.loginfo("(http_watcher_node) REAR request")
                        velocity = Pose2D(-speed / 1000, 0, 0)
                    elif "CCW" in self.path:
                        rospy.loginfo("(http_watcher_node) CCW request")
                        velocity = Pose2D(0, 0, speed / 1000)
                    elif "CW" in self.path:
                        rospy.loginfo("(http_watcher_node) CW request")
                        velocity = Pose2D(0, 0, -speed / 1000)
                    self.set_manual_vel_service(velocity)
                    self.send_response(200)
                    self.end_headers()
    return ServerHandler
        

class HTTPWatcherNode(object):
    def __init__(self):
        # Initialize node as ROS node.
        rospy.init_node("http_watcher_node")

        # Get server paramters from ROS param server.
        self.port = rospy.get_param("/communication/server/port")
        self.ip = rospy.get_param("/communication/server/ip")
        rospy.loginfo(rospy.get_caller_id() + "Server IP: " + self.ip + ", Port: " + str(self.port))
        
        # Get connection to services.
        rospy.wait_for_service("/change_mode_service")
        rospy.wait_for_service("/set_manual_vel_service")
        self.change_mode_service = rospy.ServiceProxy("/change_mode_service", ChangeMode)
        self.set_manual_vel_service = rospy.ServiceProxy("/set_manual_vel_service", SetManualVel)
        service_dict = {
            "change_mode_service": self.change_mode_service,
            "set_manual_vel_service": self.set_manual_vel_service
        }
        server_class = make_server_handler(service_dict)
        self.httpd = BaseHTTPServer.HTTPServer((self.ip, self.port), server_class)
        rospy.on_shutdown(self.stop)
        
    def start(self):
        rospy.loginfo("(http_watcher_node) Started HTTP server.")
        self.httpd.serve_forever()
    
    def stop(self):
        rospy.loginfo("(http_watcher_node) Shutting down HTTP server.")
        self.httpd.server_close()
        self.httpd.shutdown()

if __name__ == "__main__":
    try:
        rospy.loginfo("(http_watcher_node) Staring HTTP watcher node...")
        node = HTTPWatcherNode()
        node.start()
    except rospy.ROSInterruptException:
        pass
