#!/usr/bin/env python
"""
ROS node for controlling a Furious microcontroller-based robot.
"""

# Configuration constants
NODE_NAME = "furiousDriver"
PUBLISH_TOPIC = "furiousState"
SUBSCRIBE_TOPIC = "servoCommand"
LOOP_FREQ = 20.0 # Hz

import os
#import threading

import roslib
roslib.load_manifest("furious")
import rospy

# Import from this package
from furious.driver import FuriousDriver
from furious.msg import FuriousState, ServoCommand
import furiousSimulator


def main():
  # Process command line arguments.
  args = rospy.myargv()[1:]
  port = None
  config_file = None
  while len(args) > 0:
    arg = args.pop(0)
    if arg == "-p":  # port
      port = args.pop(0)
    else:
      config_file = arg

  # Make sure config file is provided
  if not config_file:
    rospy.logfatal("ERROR: missing required param 'config_file'")
    exit(1)
  if not os.path.exists(config_file):
    rospy.logfatal("ERROR: config file %s not found")
    exit(1)

  # Instantiate driver object
  driver = FuriousDriver(config_file)

  # Attempt to detect the Furious board or simulator
  if not port:
    rospy.logwarn("No port specified.")
    if rospy.has_param(furiousSimulator.PARAM_NAME):
      port = rospy.get_param(furiousSimulator.PARAM_NAME)
      rospy.loginfo("Simulator detected: using port %s" % port)
  if not port:
    rospy.loginfo("Scanning for Furious board...")
    port = driver.probe()
  if not port:
    rospy.logfatal("ERROR: unable to detect Furious board.")
    exit(1)

  driver.connect(port)
  rospy.loginfo("Connected to furious driver on port %s" % port)

  # Establish publishers, subscribers, and loop frequency
  pub = rospy.Publisher(PUBLISH_TOPIC, FuriousState)
  rospy.Subscriber(SUBSCRIBE_TOPIC, ServoCommand, driver.on_servo_command)
  rospy.init_node(NODE_NAME)
  rate = rospy.Rate(LOOP_FREQ)

  # Begin main loop
  msg = FuriousState()
  while driver.is_ready() and (not rospy.is_shutdown()):
    sent = driver.refresh()
    rospy.loginfo("Sent <%s> at time %.2f" % (sent, rospy.get_time() % 1000))
    msg = driver.get_state(msg)
    pub.publish(msg)
    rate.sleep()
  rospy.loginfo("Shutting down.")
  sent = driver.disconnect()
  rospy.loginfo("Sent <%s> for shutdown" % sent)



if __name__ == "__main__":
  main()


