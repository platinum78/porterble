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
        get_bbox_image_service = service_dict["get_bbox_image_service"]
        set_tracking_obj_service = service_dict["set_tracking_obj_service"]

        def do_GET(self):
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
            elif "getTrackImage" in self.path:
                response = self.get_bbox_image_service()
                rospy.loginfo(len(response.boxes))
                self.send_response(200)
                self.send_header("Content-type", "image/png")
                self.send_header("Detection-count", str(len(response.boxes)))
                self.end_headers()
                image_io = open(PKG_DIR + "/.temp/detected_image.png")
                image_binary = image_io.read()
                self.wfile.write(image_binary)
        
        def do_POST(self):
            if "setDetectionObj" in self.path:
                content_length = self.headers.getheaders("content-length")
                length = int(content_length[0]) if content_length else 0
                selection = int(self.rfile.read(length))
                rospy.loginfo("User has selected %d." % selection)
            
    return ServerHandler
        

class HTTPWatcherNode(object):
    def __init__(self):
        # Initialize node as ROS node.
        rospy.init_node("http_watcher_node")
        rospy.loginfo(rospy.get_caller_id() + os.getcwd())

        # Get server paramters from ROS param server.
        self.port = rospy.get_param("/communication/server/port")
        self.ip = rospy.get_param("/communication/server/ip")
        rospy.loginfo("Server IP: " + self.ip + ", Port: " + str(self.port))
        
        # Get connection to services.
        rospy.wait_for_service("/change_mode_service")
        rospy.wait_for_service("/get_bbox_image_service")
        rospy.wait_for_service("/set_tracking_obj_service")
        self.change_mode_service = rospy.ServiceProxy("/change_mode_service", ChangeMode)
        self.get_bbox_image_service = rospy.ServiceProxy("/get_bbox_image_service", HumanTracking)
        self.set_tracking_obj_service = rospy.ServiceProxy("/set_tracking_obj_service", SetTrackingObj)
        service_dict = {
            "change_mode_service": self.change_mode_service,
            "get_bbox_image_service": self.get_bbox_image_service,
            "set_tracking_obj_service": self.set_tracking_obj_service
        }
        server_class = make_server_handler(service_dict)
        self.httpd = BaseHTTPServer.HTTPServer((self.ip, self.port), server_class)
        
        rospy.on_shutdown(self.stop)
        
    def start(self):
        rospy.loginfo("Started HTTP server.")
        self.httpd.serve_forever()
    
    def stop(self):
        rospy.loginfo("Shutting down HTTP server.")
        self.httpd.shutdown()

if __name__ == "__main__":
    try:
        node = HTTPWatcherNode()
        node.start()
    except rospy.ROSInterruptException:
        pass
