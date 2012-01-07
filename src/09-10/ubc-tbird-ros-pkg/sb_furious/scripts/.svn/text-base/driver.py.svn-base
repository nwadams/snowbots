#!/usr/bin/env python
"""
ROS node for controlling a Furious microcontroller.
Subscribes to
=============
  - sb_msgs/ServoCommand on the "servo" topic
  - sb_msgs/CarCommand on the "drive" topic
  - sb_msgs/TurretCommand on the "turret" topic

Publishes
=========
  - sb_msgs/RobotState on the "furious_state" topic

@author: Ian Phillips <ianfp@freeshell.org>
"""

# Configuration constants
NODE_NAME = "furiousDriver"
STATE_TOPIC = "robot_state"
SERVO_TOPIC = "servo"
CAR_TOPIC = "drive"
TURRET_TOPIC = "turret"
LOOP_FREQ = 20.0 # Hz

# Standard library imports
from optparse import OptionParser
import os
import socket

# ROS imports/setup
import roslib
roslib.load_manifest("sb_furious")
import rospy

# Snowbots stack imports
from sb_config import find_config_file
from sb_furious import FuriousBoard, Servo
from sb_msgs.msg import RobotState, ServoCommand, CarCommand, TurretCommand
import simulator  # importing from a script is cheating a bit, but...


class FuriousDriverNode(object):
  def __init__(self, name):
    rospy.loginfo("Starting %s", name)
    self.name = name
    self.board = FuriousBoard()
    self.throttle = Servo("throttle")
    self.steering = Servo("steering")
    self.pan = Servo("pan")
    self.tilt = Servo("tilt")

  def load_config(self, cmd_line_arg):
    cfg = find_config_file(cmd_line_arg)
    self.throttle.load_config(cfg)
    self.steering.load_config(cfg)
    self.pan.load_config(cfg)
    self.tilt.load_config(cfg)

  def detect_port(self, port):
    if not port:
      rospy.logwarn("No port specified.")
      try:
        if rospy.has_param(simulator.PARAM_NAME):
          port = rospy.get_param(simulator.PARAM_NAME)
          rospy.loginfo("Simulator detected: using port %s", port)
      except socket.error:
        rospy.logfatal("Unable to connnect to ROS master.  Is it running?")
        exit(1)
    if not port:
      rospy.loginfo("Scanning for Furious board...")
      port = self.board.probe()
    if not port:
      rospy.logfatal("ERROR: unable to detect Furious board.")
      exit(1)
    return port

  def connect(self, port):
    if self.board.is_valid_port(port):
      self.board.open(port)
      rospy.loginfo("Connected to furious board on port %s", port)
      # sweep the servos
#      self.board.servos[self.throttle.get_id()] = \
#        self.throttle.get_raw_value(-1.0)
#      sent = self.board.update()
#      rospy.sleep(0.1)

      rospy.loginfo("Booting ESC, please wait...")
      sent = self.stop_servos()
      rospy.loginfo("sent <%s> for init", sent)
      rospy.sleep(5)
      sent = self.stop_servos()
      rospy.loginfo("sent <%s> for init stop", sent)
      rospy.sleep(0.1)
      rospy.loginfo("Ready.")

#      rospy.sleep(0.1)
    else:
      rospy.logfatal("'%s' is not a valid port", port)
      exit(1)

  def init_ros(self, debug):
    """
    Establishes publishers, subscribers, and loop frequency, and
    calls rospy.init_node().
    """
    log_level = rospy.INFO
    if debug:
      log_level = rospy.DEBUG
    self.pub = rospy.Publisher(STATE_TOPIC, RobotState)
    rospy.init_node(self.name, log_level=log_level)
    rospy.Subscriber(SERVO_TOPIC, ServoCommand, self.on_servo_command)
    rospy.Subscriber(CAR_TOPIC, CarCommand, self.on_car_command)
    rospy.Subscriber(TURRET_TOPIC, TurretCommand, self.on_turret_command)
    self.timer = rospy.Rate(LOOP_FREQ)

  def on_servo_command(self, msg):
    self.board.servos[msg.id] = msg.pwm

  def on_car_command(self, msg):
    self.set_throttle(msg.throttle)
    self.set_steering(msg.steering)

  def on_turret_command(self, msg):
    self.board.servos[self.pan.get_id()] = \
        self.pan.get_raw_value(msg.pan)
    self.board.servos[self.tilt.get_id()] = \
        self.tilt.get_raw_value(msg.tilt)

  def set_throttle(self, val):
    self.board.servos[self.throttle.get_id()] = \
        self.throttle.get_raw_value(val)

  def set_steering(self, val):
    self.board.servos[self.steering.get_id()] = \
        self.steering.get_raw_value(val)

  def stop_servos(self):
    """
    Immediately returns all servos to their center positions.
    """
    self.set_throttle(0.0)
    self.set_steering(0.0)
    self.board.servos[self.pan.get_id()] = \
        self.pan.get_raw_value(0.0)
    self.board.servos[self.tilt.get_id()] = \
        self.tilt.get_raw_value(0.0)
    return self.board.update()

  def run(self):
    msg = self.init_state_msg()
    while self.board.is_ready() and (not rospy.is_shutdown()):
      try:
        sent = self.board.update()
        time = rospy.get_time() % 1000
        rospy.loginfo("Sent <%s> at time %.2f", sent, time)
        msg = self.update_state_msg(msg)
        self.pub.publish(msg)
        self.timer.sleep()
      except rospy.ROSInterruptException:
        rospy.logwarn("Caught ROSInterruptException")

  def init_state_msg(self):
    msg = RobotState()
    msg.portname = self.board.get_port_name()
    msg.num_analog = FuriousBoard.NUM_ANALOG
    msg.analog = [ 0 for x in range(msg.num_analog) ]
    msg.num_batteries = FuriousBoard.NUM_BATTERIES
    msg.battery_names = [ "motor", "logic" ]
    msg.batteries = [ 0 for x in range(msg.num_batteries) ]
    return msg

  def update_state_msg(self, msg):
    for i in range(FuriousBoard.NUM_ANALOG):
      msg.analog[i] = self.board.get_analog(i)
    msg.batteries[0] = self.board.get_motor_battery()
    msg.batteries[1] = self.board.get_logic_battery()
    msg.odometer = self.board.get_odometer()
    return msg

  def shutdown(self):
    rospy.loginfo("Shutting down %s", self.name)
    #sent = self.stop_servos()
    #rospy.logdebug("Sent <%s> for shutdown" % sent)
    sent = self.board.reset()
    rospy.loginfo("Sent <%s> for shutdown" % sent)
    self.board.close()

###  end FuriousDriverNode  ###


def main():
  parser = OptionParser()
  parser.add_option("--debug", "-d", action="store_true")
  parser.add_option("--port", "-p")
  parser.add_option("--config-file", "-c")
  options, args = parser.parse_args(rospy.myargv())
  driver = FuriousDriverNode(NODE_NAME)
  driver.load_config(options.config_file)
  port = driver.detect_port(options.port)
  driver.connect(port)
  driver.init_ros(options.debug)
  driver.run()
  driver.shutdown()



if __name__ == "__main__":
  main()


