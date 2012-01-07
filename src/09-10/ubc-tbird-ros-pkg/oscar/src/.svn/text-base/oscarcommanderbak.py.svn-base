#!/usr/bin/env python
"""
The process that collects info from all other Oscar processes and
makes the final decision on where the robot should go.
"""
eStop = False
vision_direction = 0
vision_confidence = 0
vision_distance = 0

# Configuration constants
LOOP_FREQ = 10.0 # Hz
NODE_NAME = "oscarCommander"
PUBLISH_TOPIC = "CarCommand"
VISION_TOPIC = "vision_navigation"
E_STOP_TOPIC = "eStop"

import roslib
roslib.load_manifest('furious')
import rospy
from furious.msg import ServoCommand
from oscar.msg import path
from std_msgs.msg import Bool
import time

def Vision_callback(message):
  global vision_direction
  global vision_distance
  global vision_confidence
  vision_direction = message.direction
  vision_distance = message.distance
  vision_confidence = message.confidence
  

def eStop_callback(message):
  global eStop
  eStop = message.data

def theDriver():
  global eStop
  global vision_direction
  global vision_distance
  global vision_confidence
  pub = rospy.Publisher(PUBLISH_TOPIC, CarCommand)
  rospy.Subscriber(VISION_TOPIC, path, Vision_callback)
  rospy.Subscriber(E_STOP_TOPIC, Bool, eStop_callback)
  rospy.init_node(NODE_NAME)

  rate = rospy.Rate(LOOP_FREQ)
  
  steer = 0.0
  steer_d = 5
  throt = 0.0
  throt_d = 5
  
  time.sleep(3) #wait 3 seconds
  
  while not rospy.is_shutdown():
    rospy.loginfo("sending steering=%i, throttle=%i" % (steer, throt) )
    pub.publish(throttle=throt, steering=steering)
   

    # Wireless E-stop 
    if eStop:
      throt = 0 
      steering = 0
    
    elif vision_confidence:
      steering = vision_direction/90
      
      if abs(steer) < 0.2 :
        steering = 2*steering
        throt = 0.06
        
      else :
        throt = 0.05
        if steering > 0:
          steering = 1
        else :
          steering = -1
         
    else :
      throt = 0.0

    rate.sleep()

  # Stop robot when the program is killed
  pub.publish(throttle = throt, steer = steering)



if __name__ == "__main__":
  theDriver()
