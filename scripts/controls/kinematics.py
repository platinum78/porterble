#!/usr/bin/python3

"""
********************************************************************************
* Filename      : Kinematic Controller
* Author        : Susung Park
* Description   : Kinematics controller for autonomous vehicle.
* Version       : On development...
********************************************************************************
"""

import rospy
from geometry_msgs.msg import Pose2D
import time, threading
import numpy as np
from .controllers import PIDController

class MecanumWheel:
    def __init__(self, wheel_info_dict):
        self.origin_x, self.origin_y = wheel_info_dict["origin"]
        self.radius = wheel_info_dict["radius"]
        self.wheel_orientation = wheel_info_dict["wheel_orientation"]
        self.roller_orientation = wheel_info_dict["roller_orientation"]
        self.alpha = None
        self.beta = None
        self.gamma = None
        self.offset_dist = None

class QuadMecanumKinematics:
    def __init__(self, config_dict):
        wheel_info_dict = config_dict["wheel_info"]

        # Load wheel dimension and allocation information from configuration dict.
        self.wheel_1 = MecanumWheel(wheel_info_dict["wheel_1"])
        self.wheel_2 = MecanumWheel(wheel_info_dict["wheel_2"])
        self.wheel_3 = MecanumWheel(wheel_info_dict["wheel_3"])
        self.wheel_4 = MecanumWheel(wheel_info_dict["wheel_4"])
    
    def compute_wheel_vel(self, vel):
        """
        Wheel velocity calculator.
        Velocity should be given in Pose2D object.
        """
        wheels = [self.wheel_1, self.wheel_2, self.wheel_3, self.wheel_4]
        velocity = []
        vx, vy, omega = vel.x, vel.y, vel.theta

        for wheel in wheels:
            a = wheel.alpha
            b = wheel.beta
            g = wheel.gamma
            L = wheel.offset_dist
            R = wheel.radius
            velocity.append((-vx - vy * np.tan(a + b + g) - L * omega * np.sin(b + g) / np.cos(a + b + g)) / (R * np.sin(g) / np.cos(a + b + g)))
        
        return velocity

class KinematicControllerPID:
    def __init__(self, translation_gains, rotation_gains):
        """
        Kinematic controller deals only with kinematic constraints.
        This controller acts as if the system is immediately responsive.
        Arguments 'pose_gains' and 'velocity_gains' should be given in
        length-of-three tuples.
        """
        # Get PID gains.
        self.translation_pid = PIDController(translation_gains[0], translation_gains[1], translation_gains[2])
        self.rotation_pid = PIDController(rotation_gains[0], rotation_gains[1], rotation_gains[2])

        # Kinematic state variables.
        self.pose_curr = Pose2D(0, 0, 0)
        self.pose_prev = Pose2D(0, 0, 0)
        self.pose_diff = Pose2D(0, 0, 0)
        self.velocity_curr = Pose2D(0, 0, 0)
        self.velocity_prev = Pose2D(0, 0, 0)
        self.velocity_diff = Pose2D(0, 0, 0)

        # Kinematic target variables.
        self.pose_target = Pose2D(0, 0, 0)
        self.velocity_target = Pose2D(0, 0, 0)

        # Timestamps.
        self.timestamp_curr = 0.0
        self.timestamp_prev = 0.0
        self.timestamp_diff = 0.0

        # Error-related variables.
        self.error_integral = Pose2D(0, 0, 0)
        self.error_differential = Pose2D(0, 0, 0)
    
    def set

    def pid_controller(self, pose_target=None, velocity_target=None):
        if pose_target is not None:
            self.pose_target