#!/usr/bin/python

"""
********************************************************************************
* Filename      : Human Tracker Node
* Author        : Susung Park
* Description   : Captures image, requests detection, and receives result.
* Runs On       : Raspberry Pi
* Cooperators   : [human_detection_server]
* Version       : On development...

  This node will run on Raspberry Pi, capturing image from USB camera and then
  transmitting over ROS service to [human_detection_server].
********************************************************************************
"""