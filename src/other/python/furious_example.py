#!/usr/bin/env python
"""
This script demonstrates the usage of the furious.devices and
furious.config modules.

USAGE:
  %s [-f CONFIG_FILE] [-p PORT]

COMMAND-LINE ARGUMENTS:
  -f CONFIG_FILE - load configuration settings from file CONFIG_FILE
  -p PORT - connect to the Furious board attached to port PORT

"""

import sys
import time

from furious.devices import FuriousRobot
from furious.config import configure_robot


def main():
  print "FuriousRobot test script"
  print "========================"

  # Initialize cmd-line params to default values
  port = None
  scan_ports = ["/dev/ttyACM%i" % i for i in xrange(9, -1, -1)]
  config_file = None

  # Process command line arguments
  args = sys.argv[1:]
  while len(args) > 0:
    flag = args.pop(0)
    if flag == "-p":
      # port
      port = args.pop(0)
    elif flag == "-f":
      # config file
      config_file = args.pop(0)

  # Instantiate and configure the robot
  robot = FuriousRobot()
  # To where (if anywhere) should the robot write debug messages?
  robot.set_debugger(sys.stdout)
  if config_file:
    print "Reading robot configuration from %s" % config_file
    configure_robot(robot, config_file)
  else:
    print "Manually configuring the robot."
    robot.add_servo("steering", 6, 200, 250, 300)
    robot.add_servo("throttle", 7, 200, 250, 300)
    robot.add_sonars(11, 12, 13)

  # Configure the port that the Furious board is connected to.
  if not port:
    print >>sys.stderr, "No port specified.  Probing..."
    # If no port is specified at the command line, then scan typical ports.
    port = robot.probe(scan_ports)
  if not port:
    print >>sys.stderr, "Unable to find a valid Furious device.  Exiting..."
    exit(1)
  print "Connecting to port %s." % port
  robot.connect(port)

  # Initialize servo values.
  steer = 0
  throt = 0
  # Run for a fixed amount of time.
  for i in range(20):
    robot.refresh()
    print "Distance: %.2f" % robot.get_odometer()

    # Print the voltage for each analog input and the distance
    # that voltage would correspond to if interpreted as an
    # infrared rangefinder.
    for i in range(6):
      print "Analog %i: %.2f volts    IR: %.2f meters" % (
          i, robot.get_analog(i), robot.get_ir(i) )

    print "Sonars:  %.2f  %.2f  %.2f" % (
        robot.get_sonar(11),
        robot.get_sonar(12),
        robot.get_sonar(13) )

    robot.set_servo("steering", steer)
    robot.set_servo("throttle", throt)
    # Change the servo values some arbitrary amount.
    steer += 5
    throt += 1
    time.sleep(1.0/20.0)

  print " Exiting..."
  robot.stop()
  robot.disconnect()


if __name__ == "__main__":
  main()
