#!/usr/bin/python

"""
********************************************************************************
* Filename      : Master Control Node
* Author        : Susung Park
* Description   : The master node that determines everyting about Porterble.
* Version       : On development; 08 AUG 2019
********************************************************************************
"""

import numpy as np

import rospy, rospkg
from std_srvs.srv import *
from geometry_msgs.msg import Pose2D
from porterble.srv import *

from controls.kinematics import KinematicControllerPID

rospack = rospkg.RosPack()

PKG_DIR = rospack.get_path("porterble")

MANUAL =    0
FOLLOW =    1
DOCKING =   2

class MasterControlNode(object):
    def __init__(self):
        # Initialize process as ROS node.
        rospy.init_node("master_control_node")
        rospy.loginfo("(master_control_node) Starting master control node...")
        
        # State variables.
        self.operation_mode = MANUAL
        self.operation_mode_prev = MANUAL
        self.docking_phase = 0
        
        # Initialize kinematics controller.
        # try:
        #     self.kinematics_controller = KinematicControllerPID((1, 0, 0), (1, 0, 0))
        #     rospy.loginfo("(master_control_node) Spawned kinematic PID controller.")
        # except Exception, e:
        #     rospy.logerr(e)
        #     rospy.logerr("(master_control_node) Failed to spawn PID controller!")
        
        self.human_position = Pose2D(0, 0, 0)
        self.docking_error = Pose2D(0, 0, 0)
        self.manual_velocity = Pose2D(0, 0, 0)
        self.cmd_vel = Pose2D(0, 0, 0)
        
        # Initialize publishers.
        rospy.loginfo("(master_control_node) Establishing publishers...")
        self.cmd_vel_publisher = rospy.Publisher("/cmd_vel", Pose2D, queue_size=1000)
        rospy.loginfo("(master_control_node) Established publishers.")
        
        # Subscribe to topics.
        rospy.loginfo("(master_control_node) Subscribing to topics...")
        self.manual_op_vel_subscriber = rospy.Subscriber("/manual_op_vel", Pose2D, self.manual_op_vel_callback)
        self.human_position_subscriber = rospy.Subscriber("/human_position", Pose2D, self.human_position_callback)
        self.docking_error_subscriber = rospy.Subscriber("/docking_error", Pose2D, self.docking_error_callback)
        rospy.loginfo("(master_control_node) Subscribed to topics.")
        
        # Open services.
        self.change_mode_service = rospy.Service("/change_mode_service", ChangeMode, self.change_mode_service_handler)
        self.set_manual_vel_service = rospy.Service("/set_manual_vel_service", SetManualVel, self.set_manual_vel_handler)

        self.docking_on_translation = False
        self.docking_on_rotation = False

        # Connect to services.
        rospy.loginfo("(master_control_node) Waiting for services...")
        rospy.wait_for_service("/human_tracker_trigger_service")
        rospy.wait_for_service("/pose_estimator_trigger_service")
        self.human_tracker_trigger_service = rospy.ServiceProxy("/human_tracker_trigger_service", SetBool)
        self.pose_estimator_trigger_service = rospy.ServiceProxy("/pose_estimator_trigger_service", SetBool)
        rospy.loginfo("(master_control_node) Connected to services.")
        
    def manual_op_vel_callback(self, msg):
        if self.operation_mode == MANUAL:
            self.cmd_vel_publisher.publish(msg)
        
    def human_position_callback(self, msg):
        self.human_position = Pose2D(msg.x, msg.y, msg.theta)
    
    def docking_error_callback(self, msg):
        self.docking_error = Pose2D(msg.x, msg.y, msg.theta)
    
    def change_mode_service_handler(self, request):
        self.operation_mode = request.mode
        if request.mode == MANUAL:
            rospy.loginfo("(master_control_node) Change to MANUAL OPERATION mode.")
        elif request.mode == FOLLOW:
            rospy.loginfo("(master_control_node) Change to FOLLOWING mode.")
        elif request.mode == DOCKING:
            rospy.loginfo("(master_control_node) Change to DOCKING mode.")
        response = ChangeModeResponse(ack=True)
        return response
    
    def set_manual_vel_handler(self, request):
        self.manual_velocity = request.velocity
        rospy.loginfo("(master_control_node) Velocity set to: (%f, %f, %f)" % (self.manual_velocity.x, self.manual_velocity.y, self.manual_velocity.theta))
        return SetManualVelResponse(True)
    
    def manual_control(self):
        if self.operation_mode == MANUAL:
            self.cmd_vel_publisher.publish(self.manual_velocity)
            # pass

    def follow_control(self):
        # cmd_vel = self.kinematics_controller.exec_pid(self.human_position)
        # self.cmd_vel_publisher.publish(cmd_vel)
        msg = self.human_position
        self.cmd_vel_publisher.publish(msg)

    def docking_control(self, frequency=10, docking_err_thres=0.05):
        # Pause if docking error is exactly zero, which implies an error.
        if self.docking_error == Pose2D(0, 0, 0):
            self.cmd_vel_publisher.publish(Pose2D(0, 0, 0))
        else:
            # Phase 1: Rotate until aligned to marker.
            if self.docking_phase == 1:
                rospy.loginfo("(master_control_node) docking: phase 1")
                path_radius = np.sqrt(self.docking_error.x**2 + self.docking_error.y**2)
                if self.docking_error.x > docking_err_thres:
                    rospy.loginfo("(master_control_node) Rotate CCW")
                    self.cmd_vel_publisher.publish(Pose2D(0.15, 0, 0.15 / path_radius))
                elif self.docking_error.x < -docking_err_thres:
                    rospy.loginfo("(master_control_node) Rotate CW")
                    self.cmd_vel_publisher.publish(Pose2D(-0.15, 0, -0.15 / path_radius))
                else:
                    self.docking_phase += 1

            # Phase 2: Rotate until aligned to markers.
            elif self.docking_phase == 2:
                rospy.loginfo("(master_control_node) docking: phase 2")
                if self.docking_error.theta > docking_err_thres:
                    rospy.loginfo("(master_control_node) Rotate CCW")
                    self.cmd_vel_publisher.publish(Pose2D(0, 0, 0.3))
                elif self.docking_error.y < -docking_err_thres:
                    rospy.loginfo("(master_control_node) Rotate CW")
                    self.cmd_vel_publisher.publish(Pose2D(0, 0, -0.3))
                else:
                    self.docking_phase += 1
            
            # Phase 3: Translate until docked.
            elif self.docking_phase == 3:
                rospy.loginfo("(master_control_node) docking: phase 3")
                if self.docking_error.theta > docking_err_thres:
                    rospy.loginfo("(master_control_node) Path correction - CCW")
                    self.docking_on_rotation = True
                    self.cmd_vel_publisher.publish(Pose2D(0, 0.15, 0.3))
                elif self.docking_error.theta < -docking_err_thres:
                    rospy.loginfo("(master_control_node) Path correction - CW")
                    self.docking_on_rotation = True
                    self.cmd_vel_publisher.publish(Pose2D(0, 0.15, -0.3))
                else:
                    rospy.loginfo("(master_control_node) Rotational path correction done.")
                    self.docking_on_rotation = False
                
                if self.docking_on_rotation == False:
                    if self.docking_error.x > docking_err_thres:
                        rospy.loginfo("(master_control_node) Path correction - Forward")
                        self.cmd_vel_publisher.publish(Pose2D(0.15, 0.15, 0))
                    elif self.docking_error.x < -docking_err_thres:
                        rospy.loginfo("(master_control_node) Path correction - Backward")
                        self.cmd_vel_publisher.publish(Pose2D(-0.15, 0.15, 0))
                    else:
                        rospy.loginfo("(master_control_node) Translational path correction done.")
                
                if self.docking_on_rotation == False:
                    rospy.loginfo("master_control_node")
                    self.cmd_vel_publisher.publish(Pose2D(0, 0.2, 0))
            
            elif self.docking_phase > 3:
                self.cmd_vel_publisher.publish(Pose2D(0, 0, 0))
    
    def departing_control(self):
        pass

    def exec_loop(self, frequency=10):
        self.loop_rate = rospy.Rate(frequency)
        while not rospy.is_shutdown():
            # Calibrate if operating mode has changed.
            if self.operation_mode != self.operation_mode_prev:
                if self.operation_mode == MANUAL:
                    rospy.loginfo("(master_control_node) Changed to MANUAL mode.")
                    self.cmd_vel_publisher.publish(Pose2D(0, 0, 0))
                    self.human_tracker_trigger_service(False)
                    self.pose_estimator_trigger_service(False)
                elif self.operation_mode == FOLLOW:
                    rospy.loginfo("(master_control_node) Changed to follow.")
                    self.human_tracker_trigger_service(True)
                    self.pose_estimator_trigger_service(False)
                    rospy.wait_for_message("/human_position", Pose2D)
                elif self.operation_mode == DOCKING:
                    rospy.loginfo("(master_control_node) Changed to DOCKING mode.")
                    self.docking_phase = 1
                    self.human_tracker_trigger_service(False)
                    self.pose_estimator_trigger_service(True)
                    rospy.wait_for_message("/docking_error", Pose2D)

            if self.operation_mode == MANUAL:
                self.manual_control()
            elif self.operation_mode == FOLLOW:
                self.follow_control()
            elif self.operation_mode == DOCKING:
                self.docking_control()
                pass

            self.operation_mode_prev = self.operation_mode
            self.loop_rate.sleep()

if __name__ == "__main__":
    try:
        node = MasterControlNode()
        node.exec_loop()
    except Exception, e:
        rospy.logerr(e)
        pass