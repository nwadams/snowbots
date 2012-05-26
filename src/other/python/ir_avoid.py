#!/usr/bin/env python

"""
SUMMARY:
  Infrared rangefinder-based obstacle avoidance.

USAGE:
  %s -p PORT [-c CONFIG_FILE] [-t THROTTLE]

COMMAND-LINE OPTIONS:
  -p PORT          Connect to the robot on port PORT
  -c CONFIG_FILE   Read settings in from file CONFIG_FILE 
     (default: robot.cfg.example)
  -t THROTTLE      Set max throttle to THROTTLE
"""

## Standard library imports
import signal
import sys
import time

## Homemade library imports
from furious.devices import FuriousRobot
from furious.config import configure_robot
from robolib import PidControl, clamp

# Global variables
running = True

# Constants
THROTTLE = 30
ESC_THROTTLE = -40
ESC_DIST = 0.25
KP = 256
KI = 0
KD = 0
LOOP_FREQ = 10.0
IR_FRONT = 1
IR_RIGHT = 0
IR_LEFT = 2
# Valid range of IRs
MIN_RANGE = 0.1
MAX_RANGE = 0.8

def main():
  print "Starting up..."
  # Assign signal handlers
  signal.signal(signal.SIGINT, interrupt_handler)

  ## Load options
  cfgfile = "robot.cfg.example"
  throttle = THROTTLE
  port = None
  args = sys.argv[1:]
  while len(args) > 0:
    opt = args.pop(0)
    if opt == "-c":
      cfgfile = args.pop(0)
    elif opt == "-t":
      throttle = clamp( int(args.pop(0)), -100, 100)
    elif opt == "-p":
      port = args.pop(0)

  if not port:
    print >>sys.stderr, "Missing required param PORT"
    print >>sys.stderr, __doc__ % sys.argv[0]
    exit(1)

  ## Initialization
  robot = FuriousRobot()
  configure_robot(robot, cfgfile)
  robot.connect(port)
  robot.stop()
  time.sleep(1)
  robot.stop()

  dist_r = dist_l = dist_f = MAX_RANGE
  pid = PidControl(kp=KP, ki=KI, kd=KD)
  escape = 0

  print "odom,   dist_l, dist_r, steer,   dist_f, throt,   log, mot"
  global running
  while running:
    ri = robot.get_ir(IR_RIGHT)
    le = robot.get_ir(IR_LEFT)
    fr = robot.get_ir(IR_FRONT)
    if ri > MIN_RANGE and ri < MAX_RANGE:
      dist_r = ri
    if le > MIN_RANGE and le < MAX_RANGE:
      dist_l = le
    if fr > MIN_RANGE and fr < MAX_RANGE:
      dist_f = fr
      
    odom = robot.get_odometer()

    steer = int(clamp(pid.getOutput(dist_r, dist_l), -100, 100))
    throt = throttle

    if escape > 5:
      throt = 0
      steer = 0
      escape -= 1
      print "Pre-escape %i" % escape
    elif escape > 0:
      throt = ESC_THROTTLE
      steer = 0
      escape -= 1
      print "Escape %i" % escape

    elif dist_f < ESC_DIST:
      escape = 8
      throt = 0
      steer = 0
      print "Danger! Danger!"

    else:
      print "%.2f,   %.2f, %.2f, %4i,   %.2f, %4i,   %3i%%, %3i%%" % (
        odom,
        dist_l, dist_r, steer, 
        dist_f, throt, 
        robot.get_logic_battery(), robot.get_motor_battery() )

    robot.set_servo("steering", steer)
    robot.set_servo("throttle", throt)
    robot.refresh()

    time.sleep(1.0 / LOOP_FREQ)

  ## Shutdown
  print "Shutting down normally..."
  robot.stop()



def interrupt_handler(signum, frame):
  global running
  print "\nCaught interrupt signal."
  running = False


if __name__ == "__main__":
  main()
