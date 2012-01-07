#!/usr/bin/env python

"""
Does localization based on FuriousState and ServoCommand messages (see the
furious package) and displays a belief about where the robot is.

Usage
=====
  %s MAPFILE

  Arguments
  ---------
    MAPFILE - the name of the text file containing the "map" of the course

@author: Ian Phillips
"""

# Configuration constants
NODE_NAME = "mapNode"
SERVO_TOPIC = "servoCommand"
STATE_TOPIC = "furiousState"
LOOP_FREQ = 2 # Hz
STEER_MIX = 0.3

import curses
import os

import roslib
roslib.load_manifest("furious")
import rospy

from furious.msg import FuriousState, ServoCommand
from localization import CourseMap, CourseMapError

class MapNode(object):
  def __init__(self, screen):
    self.screen = screen
    self.screen.nodelay(1)  # make screen.getch() non-blocking
    self.curr_segment = None
    self.prev_segment = None
    self.steer = 0
    self.odometer = 0.0
    self.lap_start = 0.0
    self.compass = 0

  def load_mapfile(self, filename):
    self.map = CourseMap(filename)

  def on_servo_update(self, msg):
    """
    Callback function that handles receipt of a ServoCommand message.
    @param msg: the ServoCommand message
    @type msg: furious.msg.ServoCommand
    """
    if msg.servo == "steering":
      # Use exponential smoothing to smooth out steering values, which
      # often jitter considerably.
      self.steer = int( (msg.value * STEER_MIX) + 
          (self.steer * (1.0 - STEER_MIX)) )

  def on_state_update(self, msg):
    """
    Callback function that handles receipt of a FuriousState message.
    @param msg: the FuriousState message
    @type msg: furious.msg.FuriousState
    """
    self.odometer = msg.odometer
    # TODO: once we actually get some compasses for our robots, then
    # self.compass here.

  def step(self):
    """
    Calculates where the robot is and updates the display.  This method
    is called once on every iteration.
    """
    # How far are we into the current lap?
    lap_distance = self.odometer - self.lap_start
    # What is the probability distribution of our location across all
    # map segments?
    dist = self.map.segment_dist(lap_distance, self.steer, self.compass)
    # Which segment are we most likely to be in?
    most_likely = dist.index( max(dist) )
    # Have we moved into a new segment?
    if self.curr_segment != most_likely:
      self.prev_segment = self.curr_segment
      self.curr_segment = most_likely
      # Have we started a new lap?
      if self.curr_segment == 0:
        self.lap_start = self.odometer
    self.draw(dist)

    # Check for user commands
    keypress = self.screen.getch()
    if keypress == ord("q"):   # quit
      rospy.signal_shutdown("User requested shutdown.")

  def draw(self, dist):
    """
    Redraws the screen using the information given.
    @param dist: the probability distribution of the robot's location
    @type dist: list of floats
    """
    s = self.screen
    row = 0
    s.clear()
    s.addstr( row, 0, "===  Oscar map system  ===")
    row += 2
    s.addstr( row, 0, "Odometer:  %.2f" % self.odometer)
    row += 1
    s.addstr( row, 0, "Lap start: %.2f" % self.lap_start)
    row += 1
    s.addstr( row, 0, "Steering:  %4i" % self.steer)
    row += 1
    s.addstr( row, 0, "Compass:   %4i" % self.compass)
    row += 2
    s.addstr( row, 0, "Probability distribution:")
    row += 1
    section = 0
    for p in dist:
      # Show the probability of each section, plus a histogram
      bar = "*" * int(p / 0.05)
      s.addstr( row, 0, "Section %i @ %.2f: %s" % (section, p, bar))
      row += 1
      section += 1
    row += 1
    s.addstr( row, 0, "Commands:" )
    row +=1
    s.addstr( row, 2, "(q)uit")
    s.refresh()

###  end MapNode  ###



def main(stdscr):
  # Initialize
  node = MapNode(stdscr)

  rospy.Subscriber(STATE_TOPIC, FuriousState, node.on_state_update)
  rospy.Subscriber(SERVO_TOPIC, ServoCommand, node.on_servo_update)
  rospy.init_node(NODE_NAME)

  rate = rospy.Rate(LOOP_FREQ)

  # Process command-line args
  mapfile_name = None
  args = rospy.myargv()[1:]
  while len(args) > 0:
    arg = args.pop(0)
    mapfile_name = os.path.abspath(arg)

  if not mapfile_name:
    return "Missing required argument MAPFILE".append(
        __doc__ % os.path.basename(rospy.myargv()[0]) )

  try:
    node.load_mapfile(mapfile_name)
  except CourseMapError, e:
    return "Error: %s" % e

  # Run
  while not rospy.is_shutdown():
    node.step()
    rate.sleep()



if __name__ == "__main__":
  err = curses.wrapper(main)
  if err:
    rospy.logfatal(err)
  rospy.loginfo("Shutting down...")
