#!/usr/bin/python

"""
********************************************************************************
* Filename      : MCK Noise Attenuator
* Author        : Susung Park
* Description   : Attenuates sensor noise, using MCK system mockup.
* Version       : Version 1; 11 AUG 2019
********************************************************************************
"""

import numpy as np
import time

class MCKNoiseAttenuator:
    def __init__(self, m, c, k, init_val, anomaly_criterion=0.3):
        self.m, self.c, self.k = m, c, k
        self.init_val = init_val

        self.x, self.v, self.a = init_val, 0, 0

        self.timestamp = time.time()
    
    def attenuate_noise(self, val, dt=0):
        if dt == 0:
            tc = time.time()
            dt = tc - self.timestamp
        else:
            dt = dt
        self.a = (val - self.k * self.x - self.c * self.v) / self.m
        self.v = self.v + self.a * dt
        self.x = self.x + self.a * dt**2 / 2
        return self.x