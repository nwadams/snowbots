#!/usr/bin/env python
"""
Curses-based furious state monitor node.
"""

# Configuration constants
NODE_NAME = "keyboard_js_sim"
PUBLISH_TOPIC = "joy"
LOOP_FREQ = 2 # Hz

# Standard library imports
import curses, curses.wrapper

# ROS imports
import roslib
roslib.load_manifest("sb_joystick")
import rospy

# furious package imports
from joy.msg import Joy


def main(screen):
  screen.nodelay(1)  # make getch() a non-blocking call
  pub = rospy.Publisher(PUBLISH_TOPIC, Joy)
  rospy.init_node(NODE_NAME)
  rate = rospy.Rate(LOOP_FREQ)

  while not rospy.is_shutdown():
    keypress = screen.getch()
    if keypress > -1:
      keyval = chr(keypress)
      if keyval == "q":  # quit
        rospy.signal_shutdown("User requested shutdown.")
      else:
        button = int(keyval)
        if button < 10:
          msg = Joy()
          msg.buttons = [0 for x in range(10)]
          msg.buttons[button] = 1
          pub.publish(msg)
    rate.sleep() 



if __name__ == "__main__":
  curses.wrapper(main)
  rospy.loginfo("Shutting down...")
