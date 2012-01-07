"""
Basic utility functions.

@author: Ian Phillips
"""

def clamp(val, low=0, high=1.0):
  """ 
  Returns val if val is between low and high; otherwise, returns low or high.
  """
  if val > high: return high
  elif val < low: return low
  else: return val
  

def interpolate(val, old_low, old_high, new_low, new_high):
  """
  Generates a new value between new_low and new_high that corresponds to val's
  position between old_low and old_high.
  """
  slope = float(new_high - new_low) / float(old_high - old_low)
  intercept = new_low - ( slope * old_low )  # b = y - mx
  return (slope * val) + intercept           # y = mx + b


def saturate(val, old_low, old_high, new_low, new_high):
  """
  Same as L{interpolate()}, but clamps the return value between new_low
  and new_high.
  """
  return clamp( interpolate(val, old_low, old_high, new_low, new_high),
      new_low, new_high )
