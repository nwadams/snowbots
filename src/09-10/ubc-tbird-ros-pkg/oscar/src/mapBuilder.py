#!/usr/bin/env python


NODE_NAME = "setSpeed"
STATE_TOPIC = "furiousState"
SERVO_TOPIC = "servoCommand"
LOOP_RATE = 5.0
THROTTLE = 30

import pickle

import roslib
roslib.load_manifest("furious")
import rospy

from furious.msg import FuriousState, ServoCommand

class Reading(object):
  def __init__(self):
    self.steer_total = 0
    self.ticks = 0
    
  def value(self):
    if self.ticks > 0:
      return int(self.steer_total / self.ticks)
    else:
      # readings with no ticks render a special null value
      return None


class CourseMap(object):
  def __init__(self, unit=0.1, points=[]):
    self.points = points 
    self.dist_unit = unit

  def take_reading(self, dist, steer):
    waypoint = int( round( dist / self.dist_unit) )
    while len(self.points) <= waypoint:
      self.points.append( Reading() )
    self.points[waypoint].steer_total += steer
    self.points[waypoint].ticks += 1
  
  def get_reading(self, dist):
    waypoint = int( round( dist / self.dist_unit) )
    while waypoint >= len(self.points):
      # Reduce waypoint so that the system runs laps
      waypoint = waypoint - len(self.points)
    else:
      return self.points[waypoint].value()
  
  def save(self, filename):
    file = open(filename, "w")
    for i in range(len(self.points)):
      reading = self.points[i]
      if reading.value() is not None:
        file.write("%.3f,%i\n" % (i * self.dist_unit, reading.value()))
    file.close()


def restore(filename):
  file = open(filename)
  map = CourseMap()
  for line in file:
    data = line.strip().split(",")
    dist = float(data[0])
    steer = int(data[1])
    map.take_reading(dist, steer)
  file.close()
  return map



class MapBuilder(object):
  def __init__(self, mapfile):
    self.mapfile = mapfile
    self.map = CourseMap()
    self.lap_start = -1
    self.dist = 0

  def on_servo_command(self, msg):
    if msg.servo == "steering":
      self.map.take_reading(self.dist, msg.value)
      rospy.loginfo("got steering %i at distance %.2f", msg.value, self.dist)

  def on_state_update(self, msg):
    if self.lap_start == -1:
      self.lap_start = msg.odometer
      return
    self.dist = msg.odometer

  def shutdown(self):
    rospy.loginfo("Saving map data to %s", self.mapfile)
    self.map.save(self.mapfile)



class MapRunner(object):
  def __init__(self, mapfile):
    self.dist = 0.0
    self.lap_start = 0.0
    self.map = restore(mapfile)
  
  def on_state_update(self, msg):
    if self.lap_start == 0.0:
      self.lap_start = msg.odometer
    else:
      self.dist = msg.odometer - self.lap_start
  
  def run(self):
    pub = rospy.Publisher(SERVO_TOPIC, ServoCommand)
    rospy.Subscriber(STATE_TOPIC, FuriousState, self.on_state_update)
    rospy.init_node(NODE_NAME)
    timer = rospy.Rate(LOOP_RATE)

    steer_msg = ServoCommand()
    steer_msg.servo = "steering"
    steer_msg.value = 0
    throt_msg = ServoCommand()
    throt_msg.servo = "throttle"
    throt_msg.value = THROTTLE

    while not rospy.is_shutdown():
      if self.dist > 0.0:   
        reading = self.map.get_reading(self.dist)
        if reading is not None:
          steer_msg.value = reading
          print "At distance %.2f, steering is %i" % (self.dist, reading)
        pub.publish(steer_msg)
        pub.publish(throt_msg)
        
      timer.sleep()
    throt_msg.value = 0
    pub.publish(throt_msg)


def main():
  rospy.loginfo("Starting up...")

  # command line args
  mapfile = None
  runmap = False
  args = rospy.myargv()[1:]
  while len(args) > 0:
    opt = args.pop(0)
    if opt == "-r":
      runmap = True
    else:
      mapfile = opt

  if not mapfile:
    rospy.logfatal("Missing param 'mapfile'")
    exit(1)

  if runmap:
    rospy.loginfo("Running file %s" % mapfile)
    runner = MapRunner(mapfile)
    runner.run()
  else:
    mapper = MapBuilder(mapfile)
    rospy.Subscriber(SERVO_TOPIC, ServoCommand, mapper.on_servo_command)
    rospy.Subscriber(STATE_TOPIC, FuriousState, mapper.on_state_update)
    rospy.init_node(NODE_NAME)

    rospy.loginfo("Ready.")
    while not rospy.is_shutdown():
      rospy.spin()

    mapper.shutdown()

  print "Shutting down..."
  


if __name__ == "__main__":
  main()
