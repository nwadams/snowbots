#!/usr/bin/env python
"""
Curses-based furious state monitor node.
"""

# Configuration constants
NODE_NAME = "monitor"
SUBSCRIBE_TOPIC = "robot_state"

# Standard library imports
import curses, curses.wrapper

# ROS imports
import roslib
roslib.load_manifest("sb_util")
import rospy

# furious package imports
from sb_msgs.msg import RobotState


class FuriousMonitor(object):
  """
  Contains the important state and functions for this script.
  """

  def __init__(self, screen):
    """
    Constructor.  Takes a curses window object on which the battery
    levels will be drawn.
    """

    # Set up display things
    self.screen = screen
    # Set up key handling
    self.screen.nodelay(1)  # make getch() a non-blocking call
  
  def on_update(self, msg):
    """
    A callback function that handles the arrival of a new FuriousState
    message.
    @param msg: a RobotState message
    @type msg: sb_mgs.msg.RobotState
    """
    row = 0
    self.screen.clear()
    self.screen.addstr( row, 0, "===  Robot State Monitor  ===")
    row += 2
    
    self.screen.addstr( row, 0, "Connected on port %s" % msg.portname)
    row +=1
    
    # Display IR and Analog
    for i in range(msg.num_analog):
      self.screen.addstr( 
        row, 0, "Analog %i: %.2f" % (i, msg.analog[i]) )
      row += 1
    
    for i in range(msg.num_batteries):
      self.screen.addstr( row, 0, 
        "%s battery: %i" % (msg.battery_names[i], msg.batteries[i]) )
      row += 1

    self.screen.addstr( row, 0, 
      "Odometer: %i ticks" % msg.odometer )
    row += 2
    
    self.screen.addstr( row, 0, "Commands:")
    row += 1
    self.screen.addstr( row, 2, "(q)uit")
    row += 1

    # Check for user commands
    keypress = self.screen.getch()
    if keypress == ord("q"):   # quit
      rospy.signal_shutdown("User requested shutdown.")

###  end FuriousMonitor  ###



def main(stdscr):
  m = FuriousMonitor(stdscr)
  rospy.Subscriber(SUBSCRIBE_TOPIC, RobotState, m.on_update)
  rospy.init_node(NODE_NAME)

  while not rospy.is_shutdown():
    rospy.spin()



if __name__ == "__main__":
  curses.wrapper(main)
  rospy.loginfo("Shutting down...")
