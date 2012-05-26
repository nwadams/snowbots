#!/usr/bin/env python

"""
A robot client that simply polls all sonar sensors and prints the data 
to screen.

Usage:
  %s -p PORT [optional arguments]
Required command-line arguments:
  -p PORT        - connect to robot on port PORT
Optional command-line arguments:
  -c CONFIG_FILE - load settings from CONFIG_FILE
  -o OUTPUT_FILE - write output to OUTPUT_FILE
  -b             - enable joystick calibration
  -t THROTTLE    - set throttle to THROTTLE
"""

import signal
import sys
import time

import joystick   # drive the robot with joystick while collecting data!
from furious.devices import FuriousRobot 
from furious.config import configure_robot

## Static constants
# Number of main loop iterations per second
LOOP_FREQ = 20.0
# The ids of the IR rangefinders
IR_RIGHT = 0
IR_FRONT = 1
IR_LEFT  = 2
# The ids of the sonar rangefinders
SONAR_LEFT = 11
SONAR_RIGHT = 12

## Global variables
# Whether the main loop is running
running = True  


def interrupt_handler(signum, frame):
  print "\nCaught interrupt signal.  Shutting down..."
  global running
  running = False


def usage(msg):
  print >>sys.stderr, "Error: %s", msg
  print >>sys.stderr, __doc__ % sys.argv[0]
  exit(1)

def main():
  global running

  # Assign signal handlers
  signal.signal(signal.SIGINT, interrupt_handler)

  # Process command-line args
  port = None
  config_file = "robot.cfg.example"
  output_file = sys.stdout
  calibrate = False
  throttle = 30
  args = sys.argv[1:]
  while len(args) > 0:
    opt = args.pop(0)
    if opt == "-c":
      config_file = args.pop(0)
    elif opt == "-o":
      output_file = open(args.pop(0), "w")
    elif opt == "-b":
      calibrate = True
    elif opt == "-t":
      throttle = int(args.pop(0))
    elif opt == "-p":
      port = args.pop(0)
    else:
      usage("Invalid argument %s" % opt)

  if not port:
    usage("Required argument PORT not specified")

  # Instantiate a robot and load config settings
  robot = FuriousRobot()
  robot.connect(port)
  configure_robot(robot, config_file)

  if not robot.is_ready():
    print "Unable to connect to robot.  Exiting..."
    exit(1)
  else:
    robot.stop()
    print "Client is ready"

  # See if there are joysticks attached to control the robot.
  stick = None
  sticks = joystick.detect_joysticks()
  if len(sticks) > 0:
    stick = sticks[0]
    print "Joystick detected.  Calibrating..."
    if calibrate:
      stick.calibrate()
  else:
    print "Warning: no joysticks detected."

  ## Execute the main loop.
  print "Beginning main loop"
  print >>output_file, "irleft, irfront, irright, sonarleft, sonarright"
  while running:

    # Get driving commands from the joystick, if there is one.
    if stick:
      event = stick.await_event()
      if joystick.is_motion_event(event):
        turn = stick.getX()
        # Exponential response is easier to control
        turn = int(turn * abs(turn) * 100)
        throttle = 40
        robot.set_servo("steering", turn)
        robot.set_servo("throttle", throttle)
    else:
      robot.set_servo("throttle", 42)
      robot.set_servo("steering", 0)

    # Send the driving commands and get the latest sensor info 
    # from the robot.
    robot.refresh()

    # Print sensor info
    print >>output_file, "%.2f, %.2f, %.2f, %.2f, %.2f" % (
        robot.get_ir(IR_LEFT),
        robot.get_ir(IR_FRONT),
        robot.get_ir(IR_RIGHT),
        robot.get_sonar(SONAR_LEFT),
        robot.get_sonar(SONAR_RIGHT) )

    # sleep until the next iteration
    time.sleep(1.0 / LOOP_FREQ)
    
  # Main loop is over -- clean up
  robot.stop()
  robot.disconnect()
  if output_file != sys.stdout:
    output_file.close()


if __name__ == "__main__":
  main()
