#!/usr/bin/env python

NODE_NAME = "setSpeed"
PUBLISH_TOPIC = "driveSpeed"

import readline

import roslib
roslib.load_manifest("oscar")
import rospy
from std_msgs.msg import Float64


def main():
  pub = rospy.Publisher(PUBLISH_TOPIC, Float64)
  rospy.init_node(NODE_NAME)
  val = raw_input("Enter a speed: ")
  while (val != "q") and (not rospy.is_shutdown()):
    speed = float(val)
    msg = Float64()
    msg.data = speed
    pub.publish(msg)
    val = raw_input("Enter a speed: ")

if __name__ == "__main__":
  main()
