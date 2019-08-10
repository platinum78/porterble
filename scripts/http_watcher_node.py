#!/usr/bin/python

"""
********************************************************************************
* Filename      : HTTP Watcher Node
* Author        : Susung Park
* Description   : Listens for user input on runmode.
* Version       : On development...
********************************************************************************
"""

import BaseHTTPServer

import rospy
from porterble.srv import ChangeMode

MANUAL      = 0
FOLLOW      = 1
DOCKING     = 2

def make_server_handler(service_proxy):
    class ServerHandler(BaseHTTPServer.BaseHTTPRequestHandler):
        change_mode_service = service_proxy

        def do_GET(self):
            if "changeMode" in self.path:
                if "manual" in self.path:
                    self.change_mode_service(MANUAL)
                elif "follow" in self.path:
                    self.change_mode_service(FOLLOW)
                elif "docking" in self.path:
                    self.change_mode_service(DOCKING)
    return ServerHandler
        

class HTTPWatcherNode(object):
    def __init__(self):
        rospy.init_node("http_watcher_node")

        self.port = rospy.get_param("/communication/server/port")
        self.ip = rospy.get_param("/communication/server/ip")
        rospy.loginfo("Server IP: " + self.ip + ", Port: " + str(self.port))
        
        rospy.wait_for_service("/change_mode_service")
        self.change_mode_service = rospy.ServiceProxy("/change_mode_service", ChangeMode)
        server_class = make_server_handler(self.change_mode_service)
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
