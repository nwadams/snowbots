#!/usr/bin/env python
"""
Curses-based battery monitor node.
"""

# Configuration constants
NODE_NAME = "battery_monitor"
SUBSCRIBE_TOPIC = "robot_state"
LOOP_FREQ = 2 # Hz

# Standard library imports
from ConfigParser import RawConfigParser
import curses, curses.wrapper

# ROS imports
import roslib
roslib.load_manifest("sb_util")
import rospy

# sb package imports
from sb_config import find_config_file
from sb_msgs.msg import RobotState
from sb_util.range import saturate

class Battery(object):
  """
  Represents a battery and defines its analog port and charge limits (eg,
  full, empty).
  """
  def __init__(self, empty=120, full=150):
    """
    Constructor.
    @param empty: the raw signal value that indicates empty battery
    @param full: the raw signal value that indicates full battery
    """
    self.BATT_SIZE = 50   # How big the battery appears onscreen, in chars
    self.FULL_CHAR = "+"  # Char that represents fullness
    self.EMPTY_CHAR = "." # Char that represents emptyness

    self.empty = empty
    self.full = full
    # A value on None for self.charge means no data has been received yet
    self.charge = None

  def get_level(self):
    """
    @return: the current charge of the battery as a percentage.
    """
    if self.charge is None:
      return None
    return saturate(self.charge, self.empty, self.full, 0, 100)

  def draw(self):
    """
    @return: a goofy little ASCII-art battery picture displaying the 
      given level of charge.
    """
    # Convert the raw level into a percentage
    level = self.get_level()
    if level is None:
      return "Awaiting data..."
    # saturate() should ensure that the battery level is valid
    assert (level >= 0 and level <= 100), "Invalid battery level %i" % level

    fullLen = int(self.BATT_SIZE * level / 100)
    emptyLen =    self.BATT_SIZE - fullLen
    return "()%s%s) %3i%%" % ( self.EMPTY_CHAR * emptyLen,
      self.FULL_CHAR * fullLen, level )

###  end Battery  ###


class BatteryMonitor(object):
  """
  Contains the important state and functions for this script.
  """

  def __init__(self, screen, cfg):
    """
    Constructor.  
    @param screen: a curses window object on which the battery
      levels will be drawn.
    @param cfg: a RawConfigParser object that contains the battery
      configuration information.
    """
    self.batteries = {}
    for section in cfg.sections():
      parts = section.split(".")
      if parts[0] == "battery":
        name = parts[1]
        empty = cfg.getint(section, "empty")
        full = cfg.getint(section, "full")
        self.batteries[name] = Battery(empty, full)

    # Constants
    self.BATT_LOW = 30    # What battery level is considered low?

    # Set up display things
    self.screen = screen
    self.sound_on = True   # generate alert sound when batteries are low
    # Set up key handling
    self.screen.nodelay(1)  # make getch() a non-blocking call
    self.update_screen()
  
  def on_update(self, msg):
    """
    A callback function that handles the arrival of a new RobotState
    message.
    @param msg: a RobotState message
    @type msg: sb_msgs.msg.RobotState
    """
    for i in range(msg.num_batteries):
      name = msg.battery_names[i]
      charge = msg.batteries[i]
      self.batteries[name].charge = charge

  def step(self):
    """
    This method is called on every iteration of the main loop.  It 
    updates the screen and checks for user input.
    """
    self.update_screen()

    # Play alert sound if levels are low
    if self.sound_on:
      for battery in self.batteries.values():
        if battery.get_level() < self.BATT_LOW:
          curses.beep()
          break

    # Check for user commands
    keypress = self.screen.getch()
    if keypress == ord("q"):   # quit
      rospy.signal_shutdown("User requested shutdown.")
    elif keypress == ord("s"):  # toggle sound
      self.sound_on = not self.sound_on
  
  def update_screen(self):
    """
    Redraws the screen.
    """
    self.screen.clear()
    ypos = 0
    self.screen.addstr( ypos, 0, "===  Battery Monitor  ===")
    ypos += 2
    for name, battery in self.batteries.items():
      self.screen.addstr( ypos, 0, "%s battery:" % name)
      ypos += 1
      self.screen.addstr( ypos, 2, battery.draw() )
      ypos += 2

    self.screen.addstr( ypos, 0, "Commands:")
    ypos += 1
    change_sound = "on"
    if self.sound_on:
      change_sound = "off"
    self.screen.addstr( ypos, 2, "(s)ound %s" % change_sound)
    ypos += 1
    self.screen.addstr( ypos, 2, "(q)uit")
    ypos += 1
    self.screen.refresh()


###  end BatteryMonitor  ###



def main(stdscr):
  args = rospy.myargv()[1:]
  cfgfile = find_config_file(args)
  if not cfgfile:
    rospy.logfatal("Could not locate config file.")
    exit(1)
  cfg = RawConfigParser()
  cfg.read(cfgfile)
  bm = BatteryMonitor(stdscr, cfg)
  rospy.init_node(NODE_NAME)
  rospy.Subscriber(SUBSCRIBE_TOPIC, RobotState, bm.on_update)
  rate = rospy.Rate(LOOP_FREQ)

  while not rospy.is_shutdown():
    bm.step()
    rate.sleep()



if __name__ == "__main__":
  try:
    curses.wrapper(main)
  except rospy.ROSInterruptException:
    pass
  rospy.loginfo("Shutting down...")
