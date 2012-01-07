#!/usr/bin/env python

"""
SUMMARY:
  Infrared rangefinder-based obstacle avoidance.

USAGE:
  %s -p PORT [-c CONFIG_FILE] [-t THROTTLE]

COMMAND-LINE OPTIONS:
  -p PORT          Connect to the robot on port PORT
  -c CONFIG_FILE   Read settings in from file CONFIG_FILE 
     (default: robot.cfg.example)
  -t THROTTLE      Set max throttle to THROTTLE
"""

NODE_NAME = "irCommander"
PUBLISH_TOPIC = "servoCommand"
SUBSCRIBE_TOPIC = "furiousState"

## Standard library imports
import signal
import sys
import time

## ROS imports
import roslib
roslib.load_manifest("furious")
import rospy

## Homemade library imports
from furious.msg import ServoCommand, FuriousState
from util import clamp


# Constants
THROTTLE = 30
ESC_THROTTLE = -40
ESC_DIST = 0.25
KP = 256
IR_FRONT = 1
IR_RIGHT = 0
IR_LEFT = 2
# Valid range of IRs
MIN_RANGE = 0.1
MAX_RANGE = 0.8

def on_update(msg):
  """
  Callback function to handle the arrival of a new FuriousState message.
  """
  global pub
  ri = msg.ir[IR_RIGHT]
  le = msg.ir[IR_LEFT]
  fr = msg.ir[IR_FRONT]

  global dist_r, dist_l, dist_f
  if ri > MIN_RANGE and ri < MAX_RANGE:
    dist_r = ri
  if le > MIN_RANGE and le < MAX_RANGE:
    dist_l = le
  if fr > MIN_RANGE and fr < MAX_RANGE:
    dist_f = fr

  steer = int(clamp( KP * (dist_r - dist_l), -100, 100 ))
  global throttle
  throt = throttle

  global escape
  if escape > 5:
    throt = 0
    steer = 0
    escape -= 1
    rospy.loginfo("Pre-escape %i" % escape)
    
  elif escape > 0:
    throt = ESC_THROTTLE
    steer = 0
    escape -= 1
    rospy.loginfo("Escape %i" % escape)

  elif dist_f < ESC_DIST:
    escape = 8
    throt = 0
    steer = 0
    rospy.loginfo("Danger! Danger!")

  else:
    rospy.loginfo("%.2f, %.2f, %4i,   %.2f, %4i" % (
      dist_l, dist_r, steer, 
      dist_f, throt ) )

  pub.publish( ServoCommand(servo="steering", value=steer) )
  pub.publish( ServoCommand(servo="throttle", value=throt) )


def main():
  rospy.loginfo("Starting up...")

  ## Load options
  global throttle
  throttle = THROTTLE
  args = sys.argv[1:]
  while len(args) > 0:
    opt = args.pop(0)
    if opt == "-t":
      throttle = clamp( int(args.pop(0)), -100, 100)

  ## Set up ROS publisher & subscriber
  global pub
  pub = rospy.Publisher(PUBLISH_TOPIC, ServoCommand)
  rospy.Subscriber(SUBSCRIBE_TOPIC, FuriousState, on_update)
  rospy.init_node(NODE_NAME)

  ## Initialization
  global dist_r, dist_l, dist_f
  global escape
  dist_r = dist_l = dist_f = MAX_RANGE
  escape = 0

  rospy.loginfo("dist_l, dist_r, steer,   dist_f, throt")
  
  rospy.spin()

  ## Shutdown
  rospy.loginfo("Shutting down normally...")



if __name__ == "__main__":
  main()
