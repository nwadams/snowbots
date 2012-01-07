#!/usr/bin/env python

# Configuration constants
LOOP_FREQ = 10.0 # Hz
NODE_NAME = "wiimote_controller"

x_axis = 1
y_axis = 0

import roslib; roslib.load_manifest('sb_joystick')
import rospy
from wiimote.msg import LEDControl
from wiimote.msg import TimedSwitch
from wiimote.msg import State
from sb_msgs.msg import CarCommand
from joy.msg import Joy

def navigation_callback(message):
  global x_axis
  global y_axis
  x_axis = (message.axes[0]*message.axes[0])
  y_axis = message.axes[1]*message.axes[1]
  if (message.axes[1] < 0):
    y_axis = -y_axis
  if (message.axes[0] > 0):
    x_axis = -x_axis

def talker():

  rospy.init_node(NODE_NAME)

  led_pub = rospy.Publisher('/wiimote/leds', LEDControl)
  car_pub = rospy.Publisher("cmd_vel", Twist)
  
  rospy.Subscriber('/wiimote/nunchuk', Joy, navigation_callback)
  
  onSwitch       = TimedSwitch(switch_mode=TimedSwitch.ON)
  offSwitch      = TimedSwitch(switch_mode=TimedSwitch.OFF)
  noChangeSwitch = TimedSwitch(switch_mode=TimedSwitch.NO_CHANGE)

  msg = LEDControl(timed_switch_array=[onSwitch, offSwitch, offSwitch, offSwitch])
  rospy.logdebug("Msg: " + str(msg))
    
  
  rate = rospy.Rate(LOOP_FREQ)              

  while not rospy.is_shutdown():
    print(x_axis, y_axis)
    vel_msg = Twist()
    vel_msg.linear.x = x_axis
    vel_msg.angular.z = y_axis
    car_pub.publish(vel_msg)
    led_pub.publish(msg)    
    rate.sleep()

if __name__ == '__main__':
  talker()
