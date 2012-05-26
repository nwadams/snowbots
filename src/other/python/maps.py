#!/usr/bin/env python

import pickle, signal, sys, time
import joystick
from tbrclient import tbrclient

LOOP_PERIOD = 0.1
MAPFILE = "jsmap.txt"
MAX_THROTTLE = 35

running = True
client = tbrclient()

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
    pickle.dump(self.points, file)


def restore(filename):
  file = open(filename)
  points = pickle.load(file)
  return CourseMap(points=points)



class MapSegment(object):
  """
  Describes part of a course by giving the length in meters and the turn value
  a robot should use to navigate it.
  """
  def __init__(self, length, turn):
    self.length = length
    self.turn = turn


def loadSegmentMap(filename):
  """
  A 'segment map' is a high-level map that describes a course as a list of
  segments.  Each segment contains the length of that segment in meters, and
  the direction the robot should turn during that segment (hard right = 100,
  hard left = -100).  See MapSegment above.
  """
  map = []
  file = open(filename, "r")
  for line in file:
    parts = line.split(",")
    l = float(parts[0])
    t = int(parts[1])
    map.append( MapSegment(l, t) )
  file.close()
  return map


def build_map(mapfile):
  sticks = joystick.detect_joysticks()
  if len(sticks) < 1:
    print "No joysticks found.  Exiting..."
    shutdown(1)

  jsctrl = sticks[0]
  jsctrl.set_observer(sys.stdout)
  jsctrl.calibrate()
  dist = 0
  map = CourseMap()

  global running, client
  running = client.ready()
  lap_start = client.getOdometerDistance()
  while running:
    print "Awaiting input..."
    event = jsctrl.await_event()
    if joystick.is_motion_event(event):
      x = jsctrl.getX()
      t = jsctrl.getT()
      print "(%.2f, %.2f)" % (x, t)
      steer = int(100 * x * abs(x))
      throt = int(MAX_THROTTLE * t)
      client.setSteering(steer)
      client.setThrottle(throt)
      dist = client.getOdometerDistance() - lap_start
      print "dist %.2f  steer %i" % (dist, steer)
      map.take_reading(dist, steer)
    elif joystick.is_button_event(event):
      if joystick.get_event_button(event) == 0:
        lap_start = client.getOdometerDistance()  # new lap
      else:
        running = False  # Exit
  map.save(mapfile)


def run_map(mapfile):
  map = restore(mapfile)
  global running, client
  running = client.ready()
  steer = 0

  lap_start = client.getOdometerDistance()
  client.setThrottle(MAX_THROTTLE)
  while running:
    dist = client.getOdometerDistance() - lap_start
    reading = map.get_reading(dist)
    if reading is not None:
      steer = reading
    else:
      reading = 0.0
    print "At distance %.2f, steering is %i" % (dist, steer)
    client.setSteering(steer)
    time.sleep(LOOP_PERIOD)


def interrupt_handler(signum, frame):
  print "\nCaught interrupt signal."
  shutdown()


def shutdown(ret_val=0):
  global running, client
  running = False
  client.setThrottle(0)
  client.setSteering(0)
  exit(ret_val)

def main():
  print "Starting up..."
  # Assign signal handlers
  signal.signal(signal.SIGINT, interrupt_handler)

  client.initialize(1225, "127.0.0.1")

  mapfile = MAPFILE
  function = build_map

  # command line args
  args = sys.argv[1:]
  while len(args) > 0:
    opt = args.pop(0)
    if opt == "r":
      function = run_map
    elif opt == "b":
      pass
    else:
      mapfile = opt

  function(mapfile)


if __name__ == "__main__":
  main()
