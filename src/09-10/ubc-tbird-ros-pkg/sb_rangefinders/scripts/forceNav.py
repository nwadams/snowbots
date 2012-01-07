#!/usr/bin/env python

NODE_NAME = "forceNav"
SCAN_TOPIC = "base_scan"
OUTPUT_TOPIC = "forceNavVector"

import math

IGNORE_ANGLE = math.pi / 1.0

import roslib
roslib.load_manifest("sb_rangefinders")
import rospy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3


class ForceNavNode(object):
  def __init__(self):
    self.pub = rospy.Publisher(OUTPUT_TOPIC, Vector3)
    rospy.init_node(NODE_NAME)
    rospy.Subscriber(SCAN_TOPIC, LaserScan, self.on_scan)

  def on_scan(self, msg):
    num_rays = len(msg.ranges)
    x_total = 0.0
    y_total = 0.0
    valid_rays = 0
    for ray in range(num_rays):
      angle = msg.angle_min + (ray * msg.angle_increment)
      dist = msg.ranges[ray]
      # If no object within range, go to next ray 
      #if dist > msg.range_max:
      #  continue
      #if dist < msg.range_min:
      #  continue

      # Ignore things that aren't in front of us
      if angle < -IGNORE_ANGLE:
        continue
      if angle > IGNORE_ANGLE:
        continue

      # Objects emit repellant forces, which are inversely 
      # proportional to distance.
      force = -1.0 / dist
      x_total += force * math.cos(angle)
      y_total += force * math.sin(angle)
      valid_rays += 1

    if valid_rays <= 0:
      rospy.logerr("No valid rays found!")
      return
    vec = Vector3()
    vec.x = x_total / valid_rays
    vec.y = y_total / valid_rays 
    vec.z = 0.0
    rospy.loginfo("%i valid rays (%.2f, %.2f)", valid_rays, vec.x, vec.y)
    self.pub.publish(vec)

def main():
  node = ForceNavNode()
  while not rospy.is_shutdown():
    rospy.spin()
  rospy.loginfo("Shutting down %s", NODE_NAME)

if __name__ == "__main__":
  main()
