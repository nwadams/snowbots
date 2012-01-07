#!/usr/bin/env python
"""
Simulates a Furious board that the driver.py node can talk to if
there is no actual board connected to the computer.  This node does 
not publish or subscribe to any topics; all it does it open a pseudo-
tty that the driver.py node can connect to.  However, it does write 
the name of the pseudo-tty port that it is listening on to the 
"furiousSimPort" ROS parameter so that driver.py can automatically 
detect and connect to the simulator.

@author: Ian Phillips <ianfp@freeshell.org>
"""

# Configuration constants
NODE_NAME = "furiousSimulator"
PARAM_NAME = "furiousSimPort"

# Standard library imports
import random
import sys

# ROS imports and initialization
import roslib
roslib.load_manifest("sb_furious")
import rospy

# Imports from this package
from sb_furious import FuriousSimulator


def shutdown(retval):
  """
  We need to delete the parameter that the simulator sets, or else other
  nodes might still think this node is running.  (Maybe there's a better
  way to do this...)
  """
  rospy.loginfo("Shutting down...")
  rospy.delete_param(PARAM_NAME)
  exit(retval)


def main():
  """ 
  This is the main program that runs the simulator. 
  """

  rospy.init_node(NODE_NAME)

  # Run the simulator
  rospy.loginfo("Initializing simulator.")
  sim = FuriousSimulator()
  sim.set_debugger(sys.stdout)
  sim.open_pty()
  port = sim.slave_ttyname()
  rospy.set_param(PARAM_NAME, port)
  rospy.loginfo("Furious simulator listening on %s." % port)
  sim.connect()
  logic_batt = 150
  motor_batt = 150
  while not rospy.is_shutdown():
    # make up some analog values
    for x in range(6):
      sim.set_analog(x, random.randint(0, 1023))
    sim.set_odometer(random.randint(0, 3))
    # deplete the batteries slowly
    logic_batt -= random.randint(0,101) // 100
    motor_batt -= random.randint(0,101) // 100
    sim.set_logic_battery(logic_batt)
    sim.set_motor_battery(motor_batt)
    # Set the sonar value to 10 times the id, for debugging purposes
    sim.set_sonar_value(sim.get_sonar_request() * 10)

    # send/receive
    sim.step()

  rospy.loginfo("Disconnecting...")
  sim.disconnect()
  shutdown(0)



# Start the simulator if this script is the one being run
if __name__ == "__main__":
  try:
    main()
  except OSError, e:
    rospy.logerr("Lost connection to tbrprobe: %s" % e)
    shutdown(1)
