#!/usr/bin/env python

import math
import os
import sys
import time

import cv

DEBUG = True
START_ROW = 100
STOP_ROW = 100
FRAMERATE = 10.0

class ImgPoint(object):
  pass

class Vector(object):
  def __init__(self, x, y):
    self.x = x
    self.y = y

  def __str__(self):
    return "<%.2f, %.2f>" % (self.x, self.y)

class DebugWindow(object):
  def __init__(self):
    cv.NamedWindow("input")
    cv.NamedWindow("output")

  def create_image(self, size):
    self.img = cv.CreateImage(size, cv.IPL_DEPTH_8U, 1)

  def draw_circle(self, row, col, thickness=5):
    cv.Circle(self.img, (col, row), 20, (255, 0, 0), thickness)

  def draw_point(self, row, col, val):
    self.img[row, col] = val

  def draw_vector(self, vector):
    Y_FACTOR = 10
    X_FACTOR = 2
    center = ( int(self.img.width / 2), int(self.img.height / 2))
    end = ( center[0] + int(vector.y * Y_FACTOR), 
            center[1] + int(vector.x * X_FACTOR) )
    color = (255, 0, 0)
    cv.Line(self.img, center, end, color)

  def pause(self, input, seconds):
    cv.ShowImage("input", input)
    cv.ShowImage("output", self.img)
    cv.WaitKey( int(seconds * 1000) )

class DebugsOff(object):
  def __init__(self):
    pass

  def create_image(self, size):
    pass

  def draw_circle(self, col, row):
    pass

  def draw_point(self, row, col, val):
    pass

  def draw_vector(self, vector):
    pass

  def pause(self, input, seconds):
    time.sleep(seconds)

def main():
  global debugger
  if DEBUG:
    debugger = DebugWindow()
  else:
    debugger = DebugsOff()

  if len(sys.argv) > 1:
    filename = sys.argv[1]
    path, extension = os.path.splitext(filename)
    if is_image(extension):
      do_image(filename)
    else:
      capture = cv.CaptureFromFile(filename)
      do_video(capture)
  else:
    capture = cv.CaptureFromCAM(-1)
    do_video(capture)
  cv.DestroyAllWindows()


def is_image(ext):
  return ext in ["jpg", "jpeg", "gif"]

def do_video(capture):
  global debugger
  fps = cv.GetCaptureProperty(capture, cv.CV_CAP_PROP_FPS)
  print "framerate is ", fps
  waitframe = 1.0 / fps
  if waitframe < 0:
    waitframe = 1.0 / FRAMERATE
  print "waitframe is ", waitframe
  frame = cv.QueryFrame(capture)
  hsv = cv.CreateImage(cv.GetSize(frame), cv.IPL_DEPTH_8U, 3)
  debugger.create_image(cv.GetSize(frame))
  last_time = time.time()
  while frame:
    do_frame(frame, hsv)
    frame = cv.QueryFrame(capture)
    cur_time = time.time()
    frame_time = cur_time - last_time
    last_time = cur_time
    print "frame time is ", frame_time
    wait_time = waitframe - frame_time
    print "wait_time is ", wait_time
    if wait_time < 0:
      wait_time = 0.001
    debugger.pause(frame, wait_time)
  debugger.pause(frame, 0)

def do_image(imgfile):
  img = cv.LoadImage(imgfile)
  do_frame(img)
  debugger.pause(img, 0)

def do_frame(img, hsv):
  cv.CvtColor(img, hsv, cv.CV_RGB2HSV)
  one_ch = cv.CreateImage(cv.GetSize(img), cv.IPL_DEPTH_8U, 1)
  for channel in (2,):
    cv.SetImageCOI(hsv, channel)
    cv.Copy(hsv, one_ch)
    do_channel(one_ch)


def do_channel(channel):
  global debugger
  debugger.create_image(cv.GetSize(channel))
  base_row = channel.height - START_ROW
  base_col = int(channel.width / 2)
  base_val = channel[base_row, base_col]
  x_total = y_total = 0.0
  row = base_row
  row_min = 0
  while row > STOP_ROW:
    row_min = max(row_min, 
                  channel[row, base_col] - base_val)
    debugger.draw_point(row, base_col, row_min)

    col_min = row_min
    col = base_col - 1
    while col > 0:
      col_min = max(col_min, channel[row, col] - base_val)
      x_pos = base_row - row
      y_pos = base_col - col
      dist = math.sqrt( (x_pos * x_pos) + (y_pos * y_pos) )
      force = col_min / dist
      debugger.draw_point(row, col, col_min)
      x_total += force * x_pos
      y_total += force * y_pos
      col = col - 1

    col_min = row_min
    col = base_col + 1
    while col < channel.width:
      col_min = max(col_min, channel[row, col] - base_val)
      x_pos = base_row - row
      y_pos = base_col - col
      dist = math.sqrt( (x_pos * x_pos) + (y_pos * y_pos) )
      force = col_min / dist
      debugger.draw_point(row, col, col_min)
      x_total += force * x_pos
      y_total += force * y_pos
      col = col + 1

    row = row - 1
  num_pixels = channel.height * channel.width
  force_vector = Vector(x_total / num_pixels, y_total / num_pixels)
  print "force vector is %s" % force_vector
  debugger.draw_vector(force_vector)
  return force_vector
      

if __name__ == '__main__':
  main()
