"""
Provides the CourseMap class for representing a course or track that 
a robot might have to navigate around.  A CourseMap instance is 
initialized using a text file that provides a very simplistic 
description of a course as a list of "course segments."  The CourseMap
can be queried via its segment_dist() method, which returns a 
probability distribution that estimates which segment the robot is in.

@author: Ian Phillips
"""

import math
import os


class CourseMapError(Exception):
  """
  Base exception class for this library.
  """
  pass


class CourseMap(object):
  """
  Represents a course or track that a robot might race around.
  """

  def __init__(self, filename):
    """
    Constructor.
    @param filename: the name of the "map" file that contains a description
    of the course.  The map file is just a text file with one segment
    of the course per line.  See oscar/maps/ for examples.
    """
    self.segments = []  # list of CourseMapSegment objects
    self.length = 0.0   # the length of this course, in meters
    mapfile = open(filename)
    linenum = 0
    for line in mapfile:
      linenum += 1
      # Remove comments
      comment_begin = line.find("#")
      if comment_begin > -1:  # There's a comment
        line = line[:comment_begin]  # slice it out

      # Skip empty lines
      line = line.strip()
      if len(line) == 0:
        continue

      # Split line into parts and process them.
      parts = line.split(",")
      if len(parts) != 3:
        raise CourseMapError("""
Line %i in file %s is invalid.  Correct format is:
  length, steering, heading  # optional comment
""" % (linenum, os.path.basename(filename)) )
      length = direction = heading = None
      try:
        length = float( parts[0].strip() )
        direction = int( parts[1].strip() )
        heading = int( parts[2].strip() )
      except ValueError:
        raise CourseMapError("""
Line %i in file %s contains an invalid parameter type.  Correct types are:
  float, int, int
""" % (linenum, os.path.basename(filename)) )

      # Append a segment of the course map to the list of segments.
      self.segments.append(
          CourseMapSegment(self, self.length, length, direction, heading) )
      # Increment the total length of this course by the length of the
      # current segment.
      self.length += length

    mapfile.close()
    

  def segment_dist(self, odom, steer, compass):
    """
    Returns a probability distribution as a list.  Each item in the list
    is a probability (between 0.0 and 1.0) that the robot is in the 
    corresponding course segment.  For example, if your course map file
    contains four segments, then this method will return a list with
    four floating point values, one for each segment.

    @param odom: the robot's current odometer reading
    @type odom: float
    @param steer: the robot's current steering value
    @type steer: int
    @param compass: the robot's current heading (in degrees)
    @type compass: int
    @return: a list of probability values, one for each segment of the
      course
    """
    # get non-normalized probability distribution
    dist = [ seg.probability(odom, steer, compass) for seg in self.segments ]
    # normalize it and return that
    total = sum(dist)
    if total == 0:
      # TODO: This really shouldn't happen.  Does it happen because of the
      # crappy probability model I'm using?
      return [ 0.0 for prob in dist ]
    else:
      return [ prob / total for prob in dist ]



class CourseMapSegment(object):
  """
  A helper class to CourseMap that represents a single section or 
  "segment" of the course.  You shouldn't normally have to use this 
  class directly; just use CourseMap unless you know what you're doing.
  """

  SD_STEER = 50
  """ The standard deviation of the steering probability curve. """
  SD_COMPASS = 60
  """ The standard deviation of the compass probability curve. """
  SD_ODOM_FACTOR = 0.8
  """ The length of a segment is multiplied by this to determine the
      standard deviation of the odometry probability curve. """

  def __init__(self, map_, start, len_, dir, hdg):
    """
    Constructor.  As a user of this library, you should never have to
    instantiate CourseMapSegment instances; just use the CourseMap
    class.

    @param map_: a reference to the CourseMap instance to which this
      segment belongs
    @type map_: localization.CourseMap
    @param start: the distance of this segment from the beginning of 
      the course, in meters
    @type start: float
    @param len_: the length of this segment, in meters
    @type len_: float
    @param dir: the average steering direction for this segment
      (-100 means full left, 0 means straight, 100 means full right)
    @type dir: int
    @param hdg: the compass heading of this segment, in degrees
    @type hdg: int
    """
    self.map = map_
    self.start = start
    self.length = len_
    self.direction = dir
    self.heading = hdg

  def probability(self, odom, steer, compass):
    """
    Returns a probability (between 0.0 and 1.0) that the robot is in
    this segment, given the method arguments.

    @param odom: the robot's current odometer reading
    @type odom: float
    @param steer: the robot's current steering value
    @type steer: int
    @param compass: the robot's current compass reading, in degrees
    @type compass: int
    @return: the probability that the robot is in this segment
    """
    p_seg = self.length / self.map.length
    return p_seg * self.p_steer(steer) * self.p_odom(odom) * \
        self.p_compass(compass)

  def p_steer(self, steer):
    return gauss(steer, self.direction, CourseMapSegment.SD_STEER )

  def p_odom(self, odom):
    # ASSUMPTION: the course is a circuit.  This means that we
    # use the same "modulo" logic that is used in p_compass() below!
    mid = self.start + (self.length / 2.0)
    diff = abs(odom - mid)
    while diff > ( self.map.length / 2.0):
      diff = abs(self.map.length - diff)
    stddev = self.length * CourseMapSegment.SD_ODOM_FACTOR
    return gauss(diff, 0.0, stddev)

  def p_compass(self, compass):
    # Calculate how far off from the target heading we are.
    CIRCLE = 360
    diff = abs(compass - self.heading)
    while diff > (CIRCLE / 2):
      diff = abs(CIRCLE - diff)
    return gauss(diff, 0, CourseMapSegment.SD_COMPASS)



def gauss(x, mean, stddev):
  """
  Calculates the Gaussian probability of a value given a mean
  and standard deviation.

  @param x: the value whose probability will be calculated
  @param mean: the mean of the Gaussian (ie, normal) distribution
  @param stddev: the standard deviation of the distribution
  @return: the probability of x given the mean and std. deviation
  """
  mean = float(mean)
  var = float(stddev)**2
  coefficient = 1.0 / ( math.sqrt( var * 2.0 * math.pi ) )
  exponent = -((x - mean)**2) / (2.0 * var)
  return coefficient * math.exp(exponent)

