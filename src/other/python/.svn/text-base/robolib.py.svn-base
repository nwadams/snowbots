"""
Library of classes and functions that are generally useful for robotics
(I hope).

Author: Ian Phillips
"""

import time


## Useful constants
THROTTLE_MAX = 100
THROTTLE_MIN = 14  # The minimum throttle required to move at all

## Tunable constants ##
THROTTLE_LOW = 15
THROTTLE_HIGH = 50
FULL_ACCEL = 3.0
SPEED_DIFF = 3.0

THROTTLE_MIX = 0.5
VELOCITY_MIX = 0.5


class SpeedControl(object):
  """
  Uses PID control to calculate what the throttle of the robot should be.
  """
  def __init__(self, kp, ki=1, kd=0, per=0.5):
    self.pid_control = PidControl(kp, ki, kd)
    self.sample_period = per
    self.avg_velocity = 0
    self.prev_throttle = THROTTLE_MIN
    self.prev_distance = None
    self.prev_timestamp = None
    self.observer = None

    
  def calculateThrottle(self, target_velocity, current_distance):
    """
    Takes the desired target velocity and the current distance travelled
    and returns a new throttle value required to reach the target.
    """

    ## Special case: if the target speed is zero, that's easy to do.
    if target_velocity == 0:
      return 0

    if self.prev_distance is None:
      # Bootstrap
      self.prev_distance = current_distance
      self.prev_timestamp = time.time()
      return self.prev_throttle

    current_timestamp = time.time()
    delta_dist = current_distance - self.prev_distance
    delta_time = current_timestamp - self.prev_timestamp

    if delta_time < self.sample_period:
      # Wait until enough time has elapsed
      return self.prev_throttle

    current_velocity = delta_dist / delta_time

    ## Smooth out input changes using exponential average
    current_velocity = (current_velocity * VELOCITY_MIX) + (
      self.avg_velocity * (1.0 - VELOCITY_MIX))

    ## Calculate the new throttle setting
    throttle = int( clamp( 
      self.pid_control.getOutput(target_velocity, current_velocity),
      -THROTTLE_MAX, THROTTLE_MAX ) )

    ## Smooth out throttle changes
    throttle = int( (throttle * THROTTLE_MIX) + 
      (self.prev_throttle * (1.0 - THROTTLE_MIX)) )

    self.debug("%.2f,%.2f,%.2f,%i" % (
      current_distance, target_velocity, current_velocity, throttle))

    ## Save old values for next time
    self.prev_throttle = throttle
    self.prev_distance = current_distance
    self.prev_timestamp = current_timestamp
    self.avg_velocity = current_velocity

    return throttle


  def setObserver(self, obs):
    """
    Takes a file (eg stdout) to which debugging messages will be printed.
    """
    self.observer = obs
    # Print column headings for the debug output in calculateThrottle().
    self.debug("distance,target,current,throttle")

  def debug(self, msg):
    """
    Prints a debugging message to the observer file, if any.
    """
    if self.observer:
      print >>self.observer, msg



class FuzzySpeedControl(object):
  """
  Uses fuzzy logic to determine what the speed of the robot should be.
  """

  def calculateThrottle(self, target_velocity, current_velocity):

    ## Special case: if the target speed is zero, that's easy to do.
    if target_velocity <= 0:
      return 0
    
    ## Generate fuzzy values for velocity
    error = abs(target_velocity - current_velocity)
    speed_ok = clamp( 1 - ( error / SPEED_DIFF) )
    too_fast = clamp( (current_velocity - target_velocity) / SPEED_DIFF )
    too_slow = clamp( (target_velocity - current_velocity) / SPEED_DIFF )
    
    debug("speed_ok: %.2f   too_fast: %.2f   too_slow: %.2f" % 
      (speed_ok, too_fast, too_slow))

    ## Generate fuzzy values for acceleration.
    no_accel = clamp( (FULL_ACCEL - abs(current_accel)) / FULL_ACCEL )
    accel_fwd = clamp( current_accel / FULL_ACCEL )
    accel_rev = clamp( current_accel / (-FULL_ACCEL))

    debug("no_accel: %.2f   accel_fwd: %.2f   accel_rev: %.2f" %
      (no_accel, accel_fwd, accel_rev))

    ## Calculate degree-of-belief (dob) that we should speed up, slow down, or
    ## carry on.
    dob_speed_up = max( min(accel_rev, too_slow),
                        min(accel_rev, speed_ok),
                        min(no_accel,  too_slow) )
    dob_slow_down = max( min(accel_fwd, speed_ok),
                         min(accel_fwd, too_fast),
                         min(no_accel, too_fast) )
    dob_no_change = max( min(accel_fwd, too_slow),
                         min(no_accel,  speed_ok),
                         min(accel_rev, too_fast) )
    debug("DOB up: %.2f   down: %.2f   nc: %.2f" %
      (dob_speed_up, dob_slow_down, dob_no_change) )
    
    ## Calculate weighted average for each value.
    speed_up  = dob_speed_up * THROTTLE_HIGH
    slow_down = dob_slow_down * THROTTLE_LOW
    no_change = dob_no_change * current_throttle
    debug("    up: %.2f   down: %.2f   nc: %.2f" %
      (speed_up, slow_down, no_change) )
    
    ## De-fuzzify
    return int( clamp( 
      (speed_up + slow_down + no_change) / (too_slow + too_fast + speed_ok),
      THROTTLE_MIN, THROTTLE_MAX ) )



class SonarBot(object):
  def __init__(self, pv=0.01, mv=0.05, fmix=0.3):
    self.sl  = SonarFilter(pv=pv, mv=mv)
    self.sfl = SonarFilter(pv=pv, mv=mv)
    self.sfr = SonarFilter(pv=pv, mv=mv)
    self.sr  = SonarFilter(pv=pv, mv=mv)
    self.front_mix = fmix

  def getReadings(self, client):
    l  = self.sl.filter( client.getSonarNW() )
    fl = self.sfl.filter( client.getSonarN() )
    fr = self.sfr.filter( client.getSonarS() )
    r  = self.sr.filter( client.getSonarNE() )

    front = (0.5 * fl) + (0.5 * fr)
    left  = (self.front_mix * fl) + ((1 - self.front_mix) * l)
    right = (self.front_mix * fr) + ((1 - self.front_mix) * r)
    return (front, left, right)

  def getDangers(self, client):
    (f, l, r) = self.getReadings(client)
    df = self.danger(f)
    dl = self.danger(l)
    dr = self.danger(r)
    return (df, dl, dr)

  def danger(self, dist):
    if dist == 0: return -1
    return 1 / (dist ** 2)



class SonarFilter(object):
  def __init__(self, pv=0.01, mv=0.05, min=0.05, max=3.0):
    self.kf = KalmanFilter(pv=pv, mv=mv)
    self.min_range = min
    self.max_range = max
    self.prev_reading = 0.0

  def filter(self, reading):
    if reading < self.min_range or reading > self.max_range:
      return self.prev_reading
    self.prev_reading = self.kf.getEstimate(reading)
    return self.prev_reading




class KalmanFilter(object):
  """
  A Kalman filter.
  """
 
  def __init__(self, pv, mv):
    """
    The constructor takes two parameters:
      pv - the process variance (how quickly the true value changes)
      mv - the estimate of measurement variance (how noisy measurements are)
    """
    self.estimate_pre = 0.0  # pre-evidence estimate of actual value
    self.estimate = 0.0      # post-evidence estimate of actual value
    self.error_pre = 1.0     # pre-evidence error estimate
    self.error = 0.0         # post-evidence error estimate
    self.gain = 0.0          # gain or blending factor
    self.proc_var = pv       # process variance
    self.meas_var = mv       # estimate of measurement variance

  def setPV(self, pv):
    """
    Changes the process variance (which describes how quickly the true
    value changes).
    """
    self.proc_var = pv
  
  def setMV(self, mv):
    """
    Changes the measurement variance (which describes how noisy the
    observations are).
    """ 
    self.meas_var = mv

  def getEstimate(self, observation):
    """
    Takes an observation and estimates the true value based on previous
    observations.
    """
    # current pre-evidence estimate is previous post-evidence estimate
    self.estimate_pre = self.estimate
    # current pre-ev error is previous post-ev error plus process variance
    self.error_pre = self.error + self.proc_var

    # Update values using observations
    self.gain = self.error_pre / ( self.error_pre + self.meas_var )
    self.estimate = self.estimate_pre + ( self.gain * 
      (observation - self.estimate_pre) )
    self.error = (1 - self.gain) * self.error_pre
    return self.estimate



class PidControl(object):
  """
  A PID controller.
  """
  
  def __init__(self, kp, ki=0, kd=0):
    """
    Constructor parameters are:
      kp - the proportional constant
      ki - the integral constant (optional; default 0)
      kd - the derivative constant (optional; default 0)
    """
    self.kp = kp
    self.ki = ki
    self.kd = kd
    self.prev_error = 0
    self.total_error = 0
    
  def getOutput(self, target, current):
    """
    Takes an target value and a current value and calculates an output 
    required to reach the target.
    """
    error = target - current
    delta_error = error - self.prev_error
    output = (self.kp * error) + (self.ki * self.total_error) + \
      (self.kd * delta_error)

    # Save current values for next time
    self.prev_error = error
    self.total_error += error

    return output



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
  return new_low + ( ( 
    float(abs(val - old_low)) / float(abs(old_high - old_low)) ) *
    ( new_high - new_low ) )

