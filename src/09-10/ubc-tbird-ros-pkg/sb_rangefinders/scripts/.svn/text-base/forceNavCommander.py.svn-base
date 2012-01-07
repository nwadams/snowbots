#!/usr/bin/env python

NODE_NAME = "forceNavCommander"
INPUT_TOPIC = "forceNavVector"
SERVO_TOPIC = "drive"

# TUNABLE CONSTANTS
KP_TURN = 6.0
KP_DRIVE = 0.3
FWD_FORCE = 2.10
MIN_FWD = 0.20
MAX_FWD = 0.45
ESCAPE_FWD = 0.05
ESCAPE_SPEED = -0.5
ESCAPE_DURATION = 1.5
STOP_DURATION = 1.0
MAX_TURN = 1.0

import roslib
roslib.load_manifest("sb_rangefinders")
import rospy

from geometry_msgs.msg import Vector3
from sb_msgs.msg import CarCommand
from sb_util.range import clamp


class ForceNavCommander(object):
  def __init__(self):
    self.timeout = 0.0

  def subscribe(self):
    self.pub = rospy.Publisher(SERVO_TOPIC, CarCommand)
    rospy.init_node(NODE_NAME)
    rospy.Subscriber(INPUT_TOPIC, Vector3, self.on_input)
    self.timeout = rospy.get_time()

  def on_input(self, msg):
    throttle = steering = 0.0
    forward = msg.x + FWD_FORCE
    turn = msg.y
    if self.timeout > rospy.get_time() + STOP_DURATION + ESCAPE_DURATION:
      pass # stop and wait
    elif self.timeout > rospy.get_time() + STOP_DURATION:
      throttle = ESCAPE_SPEED
      steering = 0.0
    elif self.timeout > rospy.get_time():
      pass # stop and wait
    elif forward < ESCAPE_FWD:
      throttle = -0.01
      steering = 0.0
      self.timeout = rospy.get_time() + STOP_DURATION + \
        ESCAPE_DURATION + STOP_DURATION
    else:
      throttle = clamp(forward * KP_DRIVE, MIN_FWD, MAX_FWD)
      steering = clamp(turn * KP_TURN, -MAX_TURN, MAX_TURN)
    rospy.loginfo("Fwd %5.2f => %5.2f, turn %5.2f => %5.2f", 
        forward, throttle, turn, steering)
    self.publish(throttle, steering)

  def publish(self, throttle, steering):
    msg = CarCommand()
    msg.throttle = throttle
    msg.steering = steering
    self.pub.publish(msg)

  def shutdown(self):
    self.publish(0.0, 0.0)  # stop servos


def main():
  rospy.loginfo("Starting %s", NODE_NAME)
  cmdr = ForceNavCommander()
  rospy.on_shutdown(cmdr.shutdown)
  cmdr.subscribe()
  while not rospy.is_shutdown():
    rospy.spin()
  rospy.loginfo("Shutting down %s", NODE_NAME)

if __name__ == "__main__":
  main()
