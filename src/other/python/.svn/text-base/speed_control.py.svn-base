#!/usr/bin/env python
"""
Test script for speed control.
Usage:
  %s SPEED
Command line arguments:
  SPEED       The target speed of the robot in meters/second
  -kp
  -ki
  -kd
"""
import sys, time
from tbrclient import tbrclient
from robolib import clamp, SpeedControl

LOOP_PERIOD = 0.1

KP_DEFAULT = 64
KI_DEFAULT = 0
KD_DEFAULT = 0


# Process command line args
args = sys.argv[1:]
if len(args) < 1:
  usage("Error: missing required param SPEED.")
target_vel = float(args.pop(0))
kp = KP_DEFAULT
ki = KI_DEFAULT
kd = KD_DEFAULT
while len(args) > 0:
  opt = args.pop(0)
  if opt == "-kp":
    kp = int(args.pop(0))
  elif opt == "-ki":
    ki = int(args.pop(0))
  elif opt == "-kd":
    kd = int(args.pop(0))
  else:
    print "Unknown argument %s" % opt
    exit(1)


# Start things up
client = tbrclient()
print "connecting... "
client.initialize(1225, "127.0.0.1")

ai = SpeedControl(kp=kp, ki=ki, kd=kd)
ai.setObserver(sys.stdout)

if client.ready():
  try:
    while True: 
      cur_dist = client.getOdometerDistance()
      throt = ai.calculateThrottle(target_vel, cur_dist)
      client.setThrottle(throt)
      time.sleep(LOOP_PERIOD)
  except KeyboardInterrupt:
    print " Shutting down..."
    client.setThrottle(0)
else:
  print "Error: unable to connect to tbrprobe."
client.setThrottle(0)
client.finalize()
  
