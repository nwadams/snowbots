#! /usr/bin/env python

"""
Very rough script for sending and receiving test data from tbrprobe.
"""
import sys, os, time
import readline
from tbrclient import tbrclient

client = tbrclient()
client.initialize(1225, "127.0.0.1")

print "tbrclient test script"
print "====================="

while client.ready():
  print "Sonar NW: %.2f   N: %.2f   NE: %.2f   S: %.2f" % (
    client.getSonarNW(),
    client.getSonarN(),
    client.getSonarNE(),
    client.getSonarS() )
  print "Distance: %.2f    Velocity: %.2f" % (
    client.getOdometerDistance(), 
    client.getOdometerVelocity() )
  print "Throttle: %4i     Steering:  %4i" %  (
    client.getThrottle(), 
    client.getSteering() )
  print "Pan:      %4i     Tilt:      %4i" %  (
    client.getPan(), 
    client.getTilt() )

  try:
    command = None
    value = 0
    inputs = raw_input("Enter a command (h for help): ").split()
    if len(inputs) > 1:
      command, value = tuple(inputs)
      value = int(value)
    elif len(inputs) == 1:
      command = inputs[0]

    if command == "h":
      print """
  h       show this help
  q       quit
  t VAL   throttle
  s VAL   steering
  i VAL   tilt
  p VAL   pan
"""
    elif command == "q":
      exit()
    elif command == "t":
      client.setThrottle(value)
    elif command == "s":
      client.setSteering(value)
    elif command == "i":
      tilt = client.getTilt()
      client.setTurret(tilt, value)
    elif command == "p":
      pan = client.getPan()
      client.setTurret(value, pan)
  except KeyboardInterrupt:
    break
  

print " Exiting..."
client.setThrottle(0)
client.setSteering(0)
client.finalize()
