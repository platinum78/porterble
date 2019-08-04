#!/usr/bin/python

import os, sys
import curses, time, threading

import rospy
from geometry_msgs.msg import Pose2D


class KeyboardControllerNode(object):
    def __init__(self):
        rospy.init_node("keyboard_controller_node")
        self.publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1000)
        self.sigterm = threading.Event()
        self.cmd_obj = Pose2D()
        self.init_screen()
    
    def init_screen(self):
        self.stdscr = curses.initscr()
        curses.curs_set(0)
        curses.cbreak()
        curses.noecho()
        self.stdscr.keypad(1)
        self.pad = curses.newpad(24, 78)
        self.pad.refresh(0, 0, 0, 1, 24, 78)

    def clear_screen(self):
        curses.curs_set(1)
        curses.nocbreak()
        curses.echo()
        self.stdscr.keypad(0)
        curses.endwin()
    
    def exec_loop(self, rate=10):
        self.thread = threading.Thread(target=self.publisher_thread, args=(self.publisher, self.cmd_obj, rate, self.sigterm))
        self.thread.start()

        while not rospy.is_shutdown():
            c = curses.getch()
            if c == curses.KEY_UP:
                self.cmd_obj.x += 0.1
                for x in range(78):
    
    def shutdown_callback(self):
        self.sigterm.set()
        self.thread.join()
        self.clear_screen()
    
    def publisher_thread(self, publisher, cmd_obj, rate, sigterm):
        loop_rate = rospy.Rate(rate)
        while not sigterm.is_set():
            msg = Pose2D()
            msg.x = cmd_obj.x
            msg.y = cmd_obj.y
            msg.theta = cmd_obj.theta
            publisher.publish(msg_theta)
            loop_rate.sleep()


display = """
==============================================================================
                        PORTERBLE KEYBOARD CONTROLLER                         
==============================================================================

  This is a keyboard-based controller to maneuver the Porterble Smart Cart.   
  
  Arrow Keys: Linear movement.
  Page Up/Down Keys: Angular movement.

              +------------------------+------------------------+
              |    Linear Velocity     |    Angular Velocity    |
              +------------------------+------------------------+
              |                m/s     |                rad/s   |
              +------------------------+------------------------+             

==============================================================================
"""















==============================================================================
"""

