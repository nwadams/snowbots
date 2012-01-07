#!/usr/bin/env python
"""
The process that collects info from all other Oscar processes and
makes the final decision on where the robot should go.
"""
# Configuration constants
LOOP_FREQ = 10.0 # Hz
CAR_TOPIC = "drive"
VELOCITY_TOPIC = "cmd_vel"
RANGER_TOPIC = "forceNavVector"
VISION_TOPIC = "visual_force_nav"
STOPSIGN_TOPIC = "stopsign"
ESTOP_TOPIC = "joy"
NODE_NAME = "oscar_commander"


from ConfigParser import RawConfigParser, NoOptionError
import roslib 
roslib.load_manifest('sb_commander')
import rospy
from sb_config import find_config_file
from sb_msgs.msg import CarCommand
from sb_util.range import saturate
from geometry_msgs.msg import Vector3, Twist
from std_msgs.msg import Float64
from joy.msg import Joy # joystick for e-stop
import time

class Recommendation(object):
  def __init__(self, throttle, steering):
    self.throt = throttle
    self.steer = steering

  def __str__(self):
    return "<%.2f, %.2f>" % (self.throt, self.steer)


class EStop(object):
  GO = 0
  STOP = 1
  
  def __init__(self, now):
    # State variables
    self.status = EStop.STOP
    self.ignore_until = now
    self.count_until = now

    # Configuration settings
    self.button = 2
    self.ignore_duration = 0.5
    self.count_duration = 3.0

  def load_config(self, filename):
    """
    Loads configuration settings from the file whose name is given.
    It will read the section called "oscar.estop".
    @param filename: the name of the config file to be read
    """
    cfg = RawConfigParser();
    cfg.read(filename)
    section = "oscar.estop"
    self.button = cfg.getint(section, "button")
    self.ignore_duration = cfg.getfloat(section, "ignore_duration")
    self.count_duration = cfg.getfloat(section, "count_duration")

  def on_update(self, msg, now):
    if self.ignore_until > now:
      return
    if msg.buttons[self.button]:
      if self.status == EStop.GO:
        self.status = EStop.STOP
        self.ignore_until = now + self.ignore_duration
      elif self.status == EStop.STOP:
        self.status = EStop.GO
        self.ignore_until = now + self.ignore_duration
        self.count_until = now + self.count_duration



class Stopsign(object):
  NO = 0
  YES = 1
  STOPPED = 2
  STOP_DURATION = 3.0 # seconds

   
  def __init__(self, now):
    # State variables
    self.status = Stopsign.NO
    self.stop_until = now

    # Configuration settings
    self.quantity = 0.505

  def load_config(self, filename):
    """
    Loads configuration settings from the file whose name is given.
    It will read the section called "oscar.stopsign".
    @param filename: the name of the config file to be read
    """
    cfg = RawConfigParser();
    cfg.read(filename)
    section = "oscar.stopsign"
    self.quantity = cfg.getfloat(section, "quantity")

  def on_update(self, msg, now):
    if self.status == Stopsign.YES:
      if msg.data < self.quantity:
        self.status = Stopsign.STOPPED
        self.stop_until = now + Stopsign.STOP_DURATION

    elif self.status == Stopsign.STOPPED:
      if self.stop_until < now:
        self.status = Stopsign.NO

    elif self.status == Stopsign.NO:
      if msg.data > self.quantity:
        self.status = Stopsign.YES


class Ranger(object):
  # Status constants
  GO = 0
  PANIC = 1
  ESCAPE = 2
  REGROUP = 3

  def __init__(self, now):
    # State variables
    self.status = Ranger.GO
    self.panic_until = now
    self.escape_until = now
    self.regroup_until = now
    self.rec = None

    # Configuration settings
    self.kp_steering = 2.0 # circuit: 4.0
    self.kp_steering_rev = -2.0
    self.panic_threshold = -2.00 # circuit: -2.0
    self.escape_speed = -0.4
    self.smoothing_factor = 0.3

    # Configuration constants
    self.x_low = -1.0
    self.x_high = -0.5
    self.panic_duration = 1.0
    self.escape_duration = 1.5
    self.regroup_duration = 1.0

  def load_config(self, filename):
    """
    Loads configuration settings from the file whose name is given.
    It will read the section called "oscar.ranger".
    @param filename: the name of the config file to be read
    """
    cfg = RawConfigParser();
    cfg.read(filename)
    section = "oscar.ranger"
    self.kp_steering = cfg.getfloat(section, "kp_steering")
    self.kp_steering_rev = cfg.getfloat(section, "kp_steering_rev")
    self.panic_threshold = cfg.getfloat(section, "panic_threshold")
    self.escape_speed = cfg.getfloat(section, "escape_speed")
    self.smoothing_factor = cfg.getfloat(section, "smoothing_factor")

  def on_update(self, msg, now):
    forward = msg.x
    turn = msg.y

    if self.status == Ranger.PANIC:
      self.rec = Recommendation(0.0, 0.0)
      if self.panic_until < now:
        self.status = Ranger.ESCAPE
        self.escape_until = now + self.escape_duration

    elif self.status == Ranger.ESCAPE:
      self.rec = Recommendation(self.escape_speed,
                                turn * self.kp_steering_rev)
      if self.escape_until < now:
        self.status = Ranger.REGROUP
        self.regroup_until = now + self.regroup_duration

    elif self.status == Ranger.REGROUP:
      self.rec = Recommendation(0.0, 0.0)
      if self.regroup_until < now:
        self.status = Ranger.GO

    elif forward < self.panic_threshold:
      self.rec = Recommendation(0.0, 0.0)
      self.status = Ranger.PANIC
      self.panic_until = now + self.panic_duration

    else:
      new_throttle = saturate(forward, self.x_low,
                                       self.x_high,
                                       0.01, 1.0)
      new_steering = turn * self.kp_steering
      if self.rec:
        if abs(new_steering) < abs(self.rec.steer):
          new_steering = self.smooth(new_steering, self.rec.steer)
      self.rec = Recommendation(new_throttle, new_steering)

  def smooth(self, new, old):
    return (self.smoothing_factor * new) + \
           ((1.0 - self.smoothing_factor) * old)
  
  def get_recommendation(self):
    return self.rec


class Vision(object):

  def __init__(self, now):
    # State variables
    self.rec = None

    # Configuration constants
    self.x_low = 80
    self.x_high = 0

    # Configuration settings
    self.kp_steering = -0.10

  def load_config(self, filename):
    """
    Loads configuration settings from the file whose name is given.
    It will read the section called "oscar.vision".
    @param filename: the name of the config file to be read
    """
    cfg = RawConfigParser();
    cfg.read(filename)
    section = "oscar.vision"
    self.kp_steering = cfg.getfloat(section, "kp_steering")

  def on_update(self, msg, now):
    throttle = saturate(msg.x, 
                        self.x_low, self.x_high,
                        0.01, 1.0)
    steering = msg.y * self.kp_steering
    self.rec = Recommendation(throttle, steering)

  def get_recommendation(self):
    return self.rec



class OscarCommander(object):

  def __init__(self, now):
    # Components
    self.vision = Vision(now)
    self.ranger = Ranger(now)
    self.stopsign = Stopsign(now)
    self.estop = EStop(now)
    self.debugger = None

    # Configuration settings
    self.obey_ranger = 0.3 # circuit: 0.2
    self.min_throttle = 0.52 # circuit: 0.52
    self.max_throttle = 0.55 # circuit: 0.55

  def load_config(self, filename):
    """
    Loads configuration settings from the file whose name is given.
    It will read the section called "oscar.commander".
    @param filename: the name of the config file to be read
    """
    cfg = RawConfigParser();
    cfg.read(filename)
    section = "oscar.commander"
    self.obey_ranger = cfg.getfloat(section, "obey_ranger")
    self.min_throttle = cfg.getfloat(section, "min_throttle")
    self.max_throttle = cfg.getfloat(section, "max_throttle")

    # load configuration of sub-components
    self.vision.load_config(filename)
    self.ranger.load_config(filename)
    self.stopsign.load_config(filename)
    self.estop.load_config(filename)

  def debug(self, msg):
    """
    Writes debug messages to the debugger function.
    @param msg: the debug message to write
    """
    if self.debugger:
      self.debugger(msg)

  def get_command(self, now):
    """
    Queries the status of the subcomponents and returns the 
    final Recommendation object that sets the throttle and 
    steering.
    @return: Recommendation
    """
    rec = self._get_command_core(now)
    if rec.throt > 0.0:
      rec.throt = saturate(rec.throt, 
                           0.01, 1.0,
                           self.min_throttle, self.max_throttle)
    return rec

  def _get_command_core(self, now):
    if self.estop.status == EStop.STOP:
      self.debug("Emergency stop engaged")
      return Recommendation(0.0, 0.0)
      
    if self.estop.count_until > now:
      self.debug("Estop countdown")
      return Recommendation(0.0, 0.0)

    if self.stopsign.status == Stopsign.STOPPED:
      self.debug("Stopsign detected")
      return Recommendation(0.0, 0.0)

    rng_rec = self.ranger.get_recommendation()
    vis_rec = self.vision.get_recommendation()
    if rng_rec:
      if self.ranger.status != Ranger.GO:
        self.debug("Ranger indicates emergency")
        return rng_rec
      elif abs(rng_rec.steer) > self.obey_ranger:
        self.debug("Ranger says %s" % rng_rec)
        return rng_rec
      elif vis_rec:
        self.debug("Vision says %s" % vis_rec)
        return vis_rec
      else:
        self.debug("Ranger says %s" % rng_rec)
        return rng_rec
    elif vis_rec:
      self.debug("Vision says %s" % vis_rec)
      return vis_rec
    else:
      return Recommendation(0.0, 0.0)



def get_time():
  return rospy.get_time()


def main():
  def vision_callback(msg):
    commander.vision.on_update(msg, get_time())

  def stopsign_callback(msg):
    commander.stopsign.on_update(msg, get_time())

  def ranger_callback(msg):
    commander.ranger.on_update(msg, get_time())

  def estop_callback(msg):
    commander.estop.on_update(msg, get_time())

  rospy.loginfo("Starting %s", NODE_NAME)
  car_pub = rospy.Publisher(CAR_TOPIC, CarCommand)
  vel_pub = rospy.Publisher(VELOCITY_TOPIC, Twist)
  rospy.init_node(NODE_NAME)
  commander = OscarCommander(get_time())
  commander.debugger = rospy.loginfo
  cfgfile = find_config_file(rospy.myargv()[1:])
  if cfgfile:
    rospy.loginfo("Loading config file %s", cfgfile)
    commander.load_config(cfgfile)
  else:
    rospy.logfatal("No config file found")
    exit(1)

  rospy.Subscriber(VISION_TOPIC, Vector3, vision_callback)
  rospy.Subscriber(STOPSIGN_TOPIC, Float64, stopsign_callback)
  rospy.Subscriber(RANGER_TOPIC, Vector3, ranger_callback)
  rospy.Subscriber(ESTOP_TOPIC, Joy, estop_callback)

  rate = rospy.Rate(LOOP_FREQ)
  
  while not rospy.is_shutdown():
    rec = commander.get_command(get_time())
    car_pub.publish(throttle = rec.throt, steering = rec.steer)
    #vel_msg = Twist()
    #vel_msg.linear.x = rec.throt
    #vel_msg.angular.z = rec.steer
    #vel_pub.publish(vel_msg)
    rate.sleep()
    
  rospy.loginfo("Stopping %s", NODE_NAME)
  


if __name__ == "__main__":
  main()

