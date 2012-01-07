#!/usr/bin/env python

"""
Reads input from a joystick and issues servo commands to the furiousDriver
node.

REQUIRED PACKAGES:
  joystick
  python-pygame

@author Ian Phillips
"""

NODE_NAME = "joystickCommander"
PUBLISH_TOPIC = "servoCommand"

import roslib
roslib.load_manifest("furious")
import rospy

from furious.msg import ServoCommand
import pygame_joystick as pj


def shutdown(retval):
  pj.quit()
  exit(retval)

def main():

  # See if there are joysticks attached to control the robot.
  stick = None
  sticks = pj.detect_joysticks()
  if len(sticks) > 0:
    stick = sticks[0]
    rospy.loginfo("Joystick detected.  Calibrating...")
    stick.calibrate()
  else:
    rospy.logfatal("Error: no joysticks detected.")
    shutdown(1)

  pub = rospy.Publisher(PUBLISH_TOPIC, ServoCommand)
  rospy.init_node(NODE_NAME)

  ## Execute the main loop.
  prev_steer = 0
  prev_throt = 0
  rospy.loginfo("Ready to accept joystick commands.")
  while not rospy.is_shutdown():
    event = stick.await_event()
    if pj.is_motion_event(event):
      # Get steering value from joystick
      steer = stick.getX()
      # Exponential response is easier to control
      steer = int(steer * abs(steer) * 100)
      # If the steering value has changed, publish the new one
      if steer != prev_steer:
        cmd = ServoCommand(servo="steering", value=steer)
        pub.publish(cmd)
        prev_throt = throttle

      # Get throttle value from joystick
      throttle = stick.getT() * 100
      # If the throttle value has changed, publish the new one
      if throttle != prev_throt:
        cmd = ServoCommand(servo="throttle", value=throttle)
        pub.publish(cmd)
        prev_throt = throttle
      rospy.loginfo("steering %i;   throttle %i" % ( steer, throttle ))

    elif pj.is_button_event(event):
      # Any button shuts the node down
      rospy.loginfo("Shutting down")
      shutdown(0)




if __name__ == "__main__":
  main()
