#!/usr/bin/env python
# this code should recieve servo's steer and throttle and change glider
# from GUI, it does not publish to any of the node for now...

LOOP_FREQ = 10.0 # Hz
LIDAR_NAV_TOPIC = "lidar_nav"
NODE_NAME = "BlizzardCommander"

# Lidar Variables
lidar_direction = 0
lidar_state = 1
lidar_distance = 0
last_state = 0


car_throttle =0
car_steer = 0 

brake = 0
i = 0
eStop = 1
eStop_last = 0
throt_d =0

import roslib; roslib.load_manifest('sb_commander')
import rospy
from sb_msgs.msg import LidarNav
import sys
from PyQt4 import QtGui
from PyQt4 import QtCore


def Lidar_callback(message):
  global lidar_direction
  global lidar_distance
  global lidar_state
  lidar_direction = message.direction
  lidar_distance = message.distance
  lidar_state = message.confidence

def throttle():
  global brake
  global i
  global lidar_state
  global last_state
  global throt_d
  if (last_state == 3):
    brake = 5
  
  if (lidar_state == 3):
  
    if abs(lidar_direction) < 5 :
      throt = 0.08 + throt_d #adjust to go faster
        
    else :
      throt = 0.07 + throt_d #adjust to go faster (not recommended to adjust too high)
  
  elif (brake) :
    if(lidar_state == 2):      
      throt = -0.35
    else :
      throt = -0.50

    brake = brake - 1
    
  elif (lidar_state == 2) :
    if abs(lidar_direction) < 5 :
      throt = 0.07
    else :
      if (i%2 == 0):
        throt = 0.10

      else :
        throt = 0.04
    
    i=i+1

  else :
    if (i%2 == 0):
      throt = 0.08
    else :
      throt = 0.02

    i=i+1
    
  return throt
  
def steering() :
  if (lidar_state == 3) :
    if abs(lidar_direction) < 20:
      steer = lidar_direction/400.0
    else :
      steer = lidar_direction/200.0
      
  elif (lidar_state == 2) :
    if abs(lidar_direction) < 20:
      steer = lidar_direction/100.0    
    else :
      steer = lidar_direction/50.0
      
  else :
    if (abs(lidar_direction) < 20) :
      if (lidar_direction > 0 ):        
        steer = 1.0/4
      else :
        steer = -1.0/4
    else :
      if (lidar_direction > 0):
        steer = 1.0
      else : 
        steer = -1.0
        
  return steer


class Slider(QtGui.QWidget):
    def __init__(self, parent=None):
        QtGui.QWidget.__init__(self, parent)
        rospy.init_node(NODE_NAME)
        rate = rospy.Rate(LOOP_FREQ)
        rospy.loginfo("ready to test GUI")


        self.setGeometry(300, 300, 250, 150)
        self.setWindowTitle('Slider')

        self.slider1 = QtGui.QSlider(QtCore.Qt.Horizontal, self)
        self.slider1.setFocusPolicy(QtCore.Qt.NoFocus)
        self.slider1.setGeometry(30, 40, 100, 30)
        self.connect(self.slider1, QtCore.SIGNAL('valueChanged(int)'), self.changeValue)


        self.slider2 = QtGui.QSlider(QtCore.Qt.Horizontal, self)
        self.slider2.setFocusPolicy(QtCore.Qt.NoFocus)
        self.slider2.setGeometry(30, 60, 100, 30)
        self.slider2.setValue(50)
        self.connect(self.slider2, QtCore.SIGNAL('valueChanged(int)'), self.changeValue)
        
        if not rospy.is_shutdown():
            rospy.Subscriber(LIDAR_NAV_TOPIC, LidarNav, Lidar_callback)
            self.changeValue
   
    def changeValue(self, value):
        pos = throttle()
        pos1 = steering()

        print ('throttle %i'%pos)
        print ('steering %i'%pos1)

        car_throttle = (pos + 1) * 50
        car_steer = (pos1 + 1) * 50

        self.slider1.setValue(car_throttle)
        self.slider2.setValue(car_steer)

        



if __name__ == "__main__":
    app = QtGui.QApplication(sys.argv)
    icon = Slider()
    icon.show()
    app.exec_()
