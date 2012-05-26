#!/usr/bin/env python

"""
SUMMARY:
Wrapper around pygame's joytick support.

REQUIRED PACKAGES:
  jscalibrator
  joystick
  python-pygame
Run jscalibrator if programs using this library fail to detect your joystick.
"""

import pygame
import sys, time
from robolib import interpolate

X_AXIS = 0
Y_AXIS = 1
T_AXIS = 2
UNSUPPORTED_AXIS = -1


def detect_joysticks():
  """
  Factory function that detects joysticks attached to the computer and returns a
  list of JoystickController objects.  Call this first!
  """
  print "Detecting joysticks.  Please wait..."
  pygame.joystick.init()
  pygame.display.init()
  time.sleep(1)  # pygame wierdness
  sticks = []
  num_sticks = pygame.joystick.get_count()
  if num_sticks < 1:
    print "No joysticks detected."
  else:
    "%i joysticks found:" % num_sticks
  
  for i in range(0, num_sticks):
    js = pygame.joystick.Joystick(i)
    js.init()
    print "Adding joystick '%s'" % js.get_name()
    sticks.append(JoystickController(js))
    
  return sticks


def quit():
  pygame.quit()


class JoystickController(object):
  """
  Encapsulates and simplifies pygame's joystick API.  Represents a single 
  joystick.
  """
  def __init__(self, joystick):
    self.observer = sys.stdout
    self.joystick = joystick
    self.x_range = JoystickRange()
    self.y_range = JoystickRange()
    self.t_range = JoystickRange()


  def set_observer(self, obs):
    self.observer = obs

  
  def debug(self, msg):
    if self.observer:
      print >>self.observer, msg


  def calibrate(self):
    print "Beginning calibration.  Please wait..."
    self.x_range.low = self.get_val("stick", "leftmost", X_AXIS)
    print self.x_range.low
    self.x_range.high = self.get_val("stick", "rightmost", X_AXIS)
    print self.x_range.high
    self.y_range.low = self.get_val("stick", "reverse", Y_AXIS)
    print self.y_range.low
    self.y_range.high = self.get_val("stick", "forward", Y_AXIS)
    print self.y_range.high
    self.t_range.high = self.get_val("throttle", "highest", T_AXIS)
    print self.t_range.high
    self.t_range.low = self.get_val("throttle", "lowest", T_AXIS)
    print self.t_range.low
    print "Calibration finished."


  def get_val(self, control, position, axis):
    print "Move the %s to its %s position and press any button." % (control, position)
    event = None
    value = None
    pygame.event.clear()
    while not is_button_event(event):
      event = pygame.event.wait()
      if is_motion_event(event) and get_event_axis(event) == axis:
        # Get the raw value, not the cleaned up value from getX or getY
        value = self.joystick.get_axis(axis)
    return value
      
    
  def scale_axis_value(self, value, axis):
    """
    Scales the given value for the given axis to a number from -1 to 1 for
    the stick, and 0 to 1 for the throttle.
    """
    range = None
    low, high = -1.0, 1.0
    if axis == X_AXIS:
      range = self.x_range
    elif axis == Y_AXIS:
      range = self.y_range
    elif axis == T_AXIS:
      range = self.t_range
      low = 0.0
    return interpolate(value, range.low, range.high, low, high)


  def await_event(self):
    return pygame.event.wait()


  def getX(self):
    return self.scale_axis_value(self.joystick.get_axis(X_AXIS), X_AXIS)


  def getY(self):
    return self.scale_axis_value(self.joystick.get_axis(Y_AXIS), Y_AXIS)

  def getT(self):
    return self.scale_axis_value(self.joystick.get_axis(T_AXIS), T_AXIS)


  def getXY(self):
    """
    Returns a tuple of the joystick's current X and Y values.
    """
    return (self.getX(), self.getY())



class JoystickRange(object):
  """
  Helper class that defines the range of values returned by a joystick axis.
  """
  def __init__(self):
    ## Default values; calibration will override these.
    self.low = 0.0
    self.high = 1.0



##  Utility functions  ##

def is_motion_event(event):
  return event and event.type and event.type == pygame.JOYAXISMOTION


def is_button_event(event):
  return event and event.type and event.type == pygame.JOYBUTTONDOWN


def get_event_axis(event):
  if is_motion_event(event):
    return event.dict["axis"]
  return UNSUPPORTED_AXIS


def get_event_button(event):
  if is_button_event(event):
    return event.dict["button"]
  return None
