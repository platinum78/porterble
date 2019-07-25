#!/usr/bin/python3

"""
********************************************************************************
* Filename      : Kinematic Controller
* Author        : Susung Park
* Description   : Kinematics controller for autonomous vehicle.
* Version       : On development...
********************************************************************************
"""

import os, sys, time
import numpy as np

class Controller:
    def __init__(self):
        self.target = None
        self.sysout = None
        self.output = None
    
    def set_target(self, val):
        self.target = val
    
    def set_sysout(self, val):
        self.sysout = val

class PIDController(Controller):
    def __init__(self, kp, ki, kd):
        super().__init__()

        # PID gains.
        self.kp = kp
        self.ki = ki
        self.kd = kd

        # Error values.
        self.error_curr = 0
        self.error_prev = 0
        self.error_integral = 0
        self.error_derivative = 0
        
        # Timesteps.
        self.timestamp_curr = 0
        self.timestamp_prev = 0
        self.timestamp_diff = 0
    
    def exec_free_pid(self):
        # Calculate time difference.
        self.timestamp_curr = time.time()
        self.timestamp_diff = self.timestamp_curr - self.timestamp_prev

        # Calculate errors.
        self.error_curr = self.target - self.sysout
        self.error_integral += (self.error_curr + self.error_prev) * self.timestamp_diff / 2
        self.error_derivative = (self.error_curr - self.error_prev) / self.timestamp_diff

        # Calculate PID output
        self.output = self.error_curr * self.kp + self.error_integral * self.ki + self.error_derivative * self.kd
        
        # Update timestamp.
        self.timestamp_prev = self.timestamp_curr

        # Return PID output value.
        return self.output
    
    def exec_clipped_pid(self, clip_min, clip_max):
        pid_output = self.exec_free_pid()
        if pid_output < clip_min:
            pid_output = clip_min
        elif pid_output > clip_max:
            pid_output = clip_max
        self.output = pid_output
        return self.output