#!/usr/bin/env python

# Configuration constants
LOOP_FREQ = 10.0 # Hz
NODE_NAME = "wiimote_controller"

x_axis = 0
y_axis = 0
eStop = 0

import roslib; roslib.load_manifest('sb_joystick')
import rospy
from wiimote.msg import LEDControl
from wiimote.msg import TimedSwitch
from wiimote.msg import State
from geometry_msgs.msg import Vector3, Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool

def navigation_callback(message):
  global x_axis
  global y_axis
  x_axis = (message.axes[0]*message.axes[0])
  y_axis = message.axes[1]*message.axes[1]
  if (message.axes[1] < 0):
    y_axis = -y_axis
  if (message.axes[0] > 0):
    x_axis = -x_axis

def state_callback(message):
  global eStop
  if (message.buttons[4]) :
    eStop = 1
  if (message.buttons[0] and message.buttons[1]) :
    eStop = 0

def talker():
  global y_axis
  global x_axis

  rospy.init_node(NODE_NAME)

  led_pub = rospy.Publisher('/wiimote/leds', LEDControl)
  car_pub = rospy.Publisher("cmd_vel", Twist)
  
  rospy.Subscriber('/wiimote/nunchuk', Joy, navigation_callback)
  rospy.Subscriber('/wiimote/state', State, state_callback)
  
  onSwitch       = TimedSwitch(switch_mode=TimedSwitch.ON)
  offSwitch      = TimedSwitch(switch_mode=TimedSwitch.OFF)
  noChangeSwitch = TimedSwitch(switch_mode=TimedSwitch.NO_CHANGE)

  msg = LEDControl(timed_switch_array=[onSwitch, offSwitch, offSwitch, offSwitch])
  rospy.logdebug("Msg: " + str(msg))
    
  
  rate = rospy.Rate(LOOP_FREQ)              

  while not rospy.is_shutdown():

    if(eStop) :
      y_axis = 0
      x_axis = 0

    print(y_axis, x_axis, eStop)

    vel_msg = Twist()
    vel_msg.linear.x = y_axis
    vel_msg.angular.z = x_axis
    car_pub.publish(vel_msg)
    led_pub.publish(msg)    
    rate.sleep()

if __name__ == '__main__':
  talker()
