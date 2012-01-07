#!/usr/bin/env python
"""
The process that collects info from all other Blizzard processes and
makes the final decision on where the robot should go.
"""
# Configuration constants
LOOP_FREQ = 10.0 # Hz
CAR_PUBLISH_TOPIC = "car_command"
TURRET_PUBLISH_TOPIC = "TurretCommand"
LIDAR_NAV_TOPIC = "lidar_nav"
VISION_NAV_TOPIC = "Vision_Nav"
NODE_NAME = "BlizzardCommander"

# Vision Variables
vision_direction = 0
vision_confidence = 0
vision_distance = 0

# Lidar Variables
lidar_direction = 0
lidar_state = 1
lidar_distance = 0
last_state = 0

odometer_start = 0
odometer_count = 0
odometer_state = 0
odometer_last = 0
av_speed_last = 0
odometer = 0

brake = 0
i = 0
go_state = 0
  
traffic_light_state = 0
stop_sign_state = 0

import roslib 
roslib.load_manifest('sb_commander')
import rospy
from sb_msgs.msg import CarCommand
from sb_msgs.msg import TurretCommand
from sb_msgs.msg import VisionNav
from sb_msgs.msg import LidarNav
from sb_msgs.msg import RobotState
from std_msgs.msg import Bool
from std_msgs.msg import Float64
import time

def Vision_callback(message):
  global vision_direction
  global vision_distance
  global vision_confidence
  vision_direction = message.direction
  vision_distance = message.distance
  vision_confidence = message.confidence
  
def Lidar_callback(message):
  global lidar_direction
  global lidar_distance
  global lidar_state
  lidar_direction = message.direction
  lidar_distance = message.distance
  lidar_state = message.confidence
  
def RobotState_callback(message):
  global odometer_last
  global av_speed_last
  global odometer_state
  global odometer_start
  global odometer
  global odometer_count
  odometer = message.odometer
  odometer_diff = odometer - odometer_last
  if (odometer_diff == 0) :
    odometer_state = 2
    return
   
  av_speed_last = (odometer_last)*0.45/100
  current_speed = odometer_diff*0.45/100 #meters per second
  odometer_last = odometer
  odometer_count = odometer_count + 1
  
  if (current_speed < av_speed_last * 0.80 ): #speed theshold
    odometer_state = 1
  else :
    odometer_state = 0
    
def Stop_sign_callback(message):
  global stop_sign_state
  if (message.data >= 0.6):
    stop_sign_state = 1
  else :
    stop_sign_state = 0
  
def Traffic_light_callback(message):
  global traffic_light_state
  traffic_light_state
  
def go_callback(message):
  global go_state
  go_state = message.data
  
def throttle():
  global brake
  global i
  global lidar_state
  global last_state
  if (last_state == 3):
    brake = 3
  
  if (lidar_state == 3):
  
    if abs(lidar_direction) < 10 :
      throt = 0.10 + throt_d #adjust to go faster
        
    else :
      throt = 0.08 + throt_d #adjust to go faster (not recommended to adjust too high)
  
  elif (brake) :
    throttle = -0.25
    brake = brake - 1
    
  elif (lidar_state == 2) :
    if abs(lidar_direction) < 10 :
      throt = 0.09
    else :
      throt = 0.07
      
  else :
    if (i%2 == 0):
      throt = 0.08
    else :
      throt = 0.00

    i=i+1
    
  return throt
  
def steering() :
  if (lidar_state == 3) :
    if abs(lidar_direction) < 10:
      steer = lidar_direction/400.0
    else :
      steer = lidar_direction/200.0
      
  elif (lidar_state == 2) :
    if abs(lidar_direction) < 10:
      steer = lidar_direction/200.0    
    else :
      steer = lidar_direction/100.0
      
  else :
    if (abs(lidar_direction) < 10) :
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

def theDriver():
  global lidar_state
  global last_state
  global go_state
  car_pub = rospy.Publisher(CAR_PUBLISH_TOPIC, CarCommand)
  turret_pub = rospy.Publisher(TURRET_PUBLISH_TOPIC, TurretCommand)
  rospy.Subscriber(VISION_NAV_TOPIC, VisionNav, Vision_callback)
  rospy.Subscriber(LIDAR_NAV_TOPIC, LidarNav, Lidar_callback)
  
  rospy.Subscriber("RobotState", RobotState, RobotState_callback)
  rospy.Subscriber("traffic_light",Bool, Traffic_light_callback)
  rospy.Subscriber("stop_sign_state", Float64, Stop_sign_callback)
  rospy.Subscriber("go_button", Bool, go_callback)
  
  rospy.init_node(NODE_NAME)

  rate = rospy.Rate(LOOP_FREQ)
  
  steer = 0.0
  steer_d = 5
  throt = 0.0
  throt_d = 0
  
  last_state = 0

  rospy.loginfo("ready to go")
  #while not go_state :
   # x = 0
  rospy.loginfo("going")
    
  time.sleep(3) #wait 3 seconds
  car_pub.publish(0,0)
  
  state_count = 0
  odometer_start = odometer
  
  while not rospy.is_shutdown():
    if vision_confidence:
      steer = -vision_direction/200.0
      
      if abs(vision_direction) < 45 :
        steer = steer
        throt = 0.06
        
      else :
        throt = 0.05
        if steer > 0:
          steer = 1
        else :
          steer = -1
         
    else :
      throt = 0.0
    throt = 0
    rospy.loginfo("sending steering=%f, throttle=%f" % (steer, throt) )
    #car_pub.publish(0,0) #for testing
    car_pub.publish(throttle=throt, steering=steer)
    last_state = lidar_state
    
    rate.sleep()
    

  # car_pub.publish(throttle = 0, steering = 0)
  # Stop robot when the program is killed
  car_pub.publish(throttle = 0, steering = 0)
  rospy.loginfo("killing code")
  



if __name__ == "__main__":
  theDriver()
