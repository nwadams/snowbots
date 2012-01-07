#!/usr/bin/env python
"""
Simulates a Furious board that furiousDriver.py can talk to if
there is no actual board connected to the computer.  Unlike 
furiousSimulator.py, this one does not depend on ROS at all.  The
downside of this is that furiousDriver.py cannot detect this
simulator automatically.

Usage:
  ./simulatorLite.py [-d]
Command-line arguments:
  -d   print debugging info to STDOUT

@author: Ian Phillips
"""

import random
import signal
import sys

from furious.simulator import FuriousSim

def interrupt_handler(signum, frame):
  global running
  running = False
  print " Caught interrupt signal."


def main():
  """
  This is the main program that runs the simulator.
  """
  # Assign signal handlers
  signal.signal(signal.SIGINT, interrupt_handler)
  global running
  running = True

  print "Starting simulator."
  sim = FuriousSim()

  # Process command line arguments
  arguments = sys.argv[1:]  # shallow copy
  while len(arguments) > 0:
    arg = arguments.pop(0).strip()
    if arg == "-d":
      # Enable debugging output
      sim.set_debugger(sys.stdout)
    else:
      # Print usage and exit
      print __doc__
      exit(1)

  # Run the simulator
  sim.open_pty()
  print "Listening on %s." % sim.slave_ttyname()
  sim.connect()
  logic_batt = 150
  motor_batt = 150
  try:
    while running and sim.is_ready():
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

  except OSError, e:
    print >>sys.stderr, "Lost connection to driver: %s" % e

  print "Shutting down..."
  sim.disconnect()



# Start the simulator if this script is the one being run
if __name__ == "__main__":
  main()

