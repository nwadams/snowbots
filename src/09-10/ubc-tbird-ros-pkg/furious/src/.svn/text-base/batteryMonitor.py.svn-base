#!/usr/bin/env python
"""
Curses-based battery monitor node.
"""

# Configuration constants
NODE_NAME = "batteryMonitor"
SUBSCRIBE_TOPIC = "furiousState"
LOOP_FREQ = 2 # Hz

# Standard library imports
import curses, curses.wrapper

# ROS imports
import roslib
roslib.load_manifest("furious")
import rospy

# furious package imports
from furious.msg import FuriousState


class BatteryMonitor(object):
  """
  Contains the important state and functions for this script.
  """

  def __init__(self, screen):
    """
    Constructor.  Takes a curses window object on which the battery
    levels will be drawn.
    """
    # Initialize battery levels to None, which means "no info received yet"
    self.logic_batt = None
    self.motor_batt = None

    # Constants
    self.BATT_LOW = 30    # What battery level is considered low?
    self.BATT_SIZE = 50   # How big the battery appears onscreen, in chars
    self.FULL_CHAR = "+"  # Char that represents fullness
    self.EMPTY_CHAR = "." # Char that represents emptyness

    # Set up display things
    self.screen = screen
    self.sound_on = True   # generate alert sound when batteries are low
    # Set up key handling
    self.screen.nodelay(1)  # make getch() a non-blocking call
    self.update_screen()
  
  def on_update(self, msg):
    """
    A callback function that handles the arrival of a new FuriousState
    message.
    @param msg: a FuriousState message
    @type msg: furious.msg.FuriousState
    """
    self.logic_batt = msg.logic_battery
    self.motor_batt = msg.motor_battery

  def step(self):
    """
    This method is called on every iteration of the main loop.  It 
    updates the screen and checks for user input.
    """
    self.update_screen()

    # Play alert sound if levels are low
    if self.sound_on:
      if self.logic_batt < self.BATT_LOW or \
         self.motor_batt < self.BATT_LOW:
        curses.beep()

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
    self.screen.addstr( 0, 0, "===  Furious Battery Monitor  ===")
    self.screen.addstr( 2, 0, "Logic battery:")
    self.screen.addstr( 3, 2, self.draw_battery(self.logic_batt))
    self.screen.addstr( 5, 0, "Motor battery:")
    self.screen.addstr( 6, 2, self.draw_battery(self.motor_batt))
    self.screen.addstr( 8, 0, "Commands:")
    self.screen.addstr( 9, 2, "(s)ound on/off")
    self.screen.addstr(10, 2, "(q)uit")
    self.screen.refresh()

  def draw_battery(self, level):
    """
    Returns a goofy little ASCII-art battery picture displaying the 
    given level of charge.
    """
    if level is None:
      return "Awaiting data..."
    if level < 0 or level > 100:
      return "Warning: invalid battery level %i" % level
    fullLen = int(self.BATT_SIZE * level / 100)
    emptyLen =    self.BATT_SIZE - fullLen
    return "()%s%s) %3i%%" % ( self.EMPTY_CHAR * emptyLen,
      self.FULL_CHAR * fullLen, level )

###  end BatteryMonitor  ###



def main(stdscr):
  bm = BatteryMonitor(stdscr)
  rospy.Subscriber(SUBSCRIBE_TOPIC, FuriousState, bm.on_update)
  rospy.init_node(NODE_NAME)
  rate = rospy.Rate(LOOP_FREQ)

  while not rospy.is_shutdown():
    bm.step()
    rate.sleep()



if __name__ == "__main__":
  curses.wrapper(main)
  rospy.loginfo("Shutting down...")
