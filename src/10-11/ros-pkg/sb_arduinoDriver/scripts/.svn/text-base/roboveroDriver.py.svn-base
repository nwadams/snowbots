#!/usr/bin/env python

"""
Snowbot driver for python
basicly a translation of arduinoDriver into python for the robovero
"""

import os, sys
home = os.path.expanduser("~")
sys.path.append(home + "/robovero/python")

import roslib 
roslib.load_manifest("sb_arduinoDriver")
import rospy
from sb_msgs.msg import RobotState
from sb_msgs.msg import LidarNav
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3, Twist
import time
import math

#robovero imports
from robovero.LPC17xx import LPC_PWM1
from robovero.lpc17xx_pwm import PWM_TC_MODE_OPT, \
					PWM_MATCH_UPDATE_OPT, PWM_MATCHCFG_Type, \
					PWM_MatchUpdate, PWM_ConfigMatch, PWM_ChannelCmd, \
					PWM_ResetCounter, PWM_CounterCmd, PWM_Cmd
from robovero.extras import roboveroConfig, initMatch
from robovero.lpc_types import FunctionalState

from robovero.arduino import analogWrite, PWM1, PWM2

# Configuration constants
LOOP_FREQ = 20.0 # Hz
#CAR_PUBLISH_TOPIC = "car_command"
CAR_PUBLISH_TOPIC = "cmd_vel"
TURRET_PUBLISH_TOPIC = "TurretCommand"
LIDAR_NAV_TOPIC = "lidar_nav"
VISION_NAV_TOPIC = "Vision_Nav"
ESTOP_TOPIC = "eStop"
NODE_NAME = "RoboveroDriver"

throttle = 90
steering = 90

def initPulse(channel, pulse_width):
	initMatch(channel, pulse_width)
	
def initPeriod(period):
	initMatch(0, period)

def initPWM1():
	"""Set up PWM to output a 1.5ms pulse at 50Hz.
	"""

	# Set the period to 20000us = 20ms = 50Hz
	initPeriod(20000)

	# Set the pulse width to 1.5ms
	initPulse(1, 1500)
	
	PWM_ChannelCmd(LPC_PWM1, 1, FunctionalState.ENABLE)
	PWM_ResetCounter(LPC_PWM1)
	PWM_CounterCmd(LPC_PWM1, FunctionalState.ENABLE)
	PWM_Cmd(LPC_PWM1, FunctionalState.ENABLE)

def initPWM2():
	"""Set up PWM to output a 1.5ms pulse at 50Hz.
	"""

	# Set the period to 20000us = 20ms = 50Hz
	initPeriod(20000)

	# Set the pulse width to 1.5ms
	initPulse(2, 1500)
	
	PWM_ChannelCmd(LPC_PWM1, 2, FunctionalState.ENABLE)
	PWM_ResetCounter(LPC_PWM1)
	PWM_CounterCmd(LPC_PWM1, FunctionalState.ENABLE)
	PWM_Cmd(LPC_PWM1, FunctionalState.ENABLE)

def getServoValue(angle):
	return 1050 + (angle*750/180)

def carCommandCallback(message):
	global throttle
	global steering
	throttle = message.linear.x * 90 + 90
	steering = -message.angular.z * 60 + 90

def theDriver():
	global throttle
	global steering
	rospy.init_node(NODE_NAME)
	rate = rospy.Rate(LOOP_FREQ)

	rospy.Subscriber(CAR_PUBLISH_TOPIC, Twist, carCommandCallback)

	roboveroConfig()
	initPWM1()
	initPWM2()

	time.sleep(3)

	while not rospy.is_shutdown():
		#main loop
		PWM_MatchUpdate(LPC_PWM1, 1, getServoValue(steering), PWM_MATCH_UPDATE_OPT.PWM_MATCH_UPDATE_NOW)

		PWM_MatchUpdate(LPC_PWM1, 2, getServoValue(throttle), PWM_MATCH_UPDATE_OPT.PWM_MATCH_UPDATE_NOW)

		rospy.loginfo("throttle=%f, steering=%f" % (throttle, steering) )
		rate.sleep()

#handles killing to set robot to stop
def set_exit_handler(func):
	import signal
	signal.signal(signal.SIGTERM, func)

if __name__ == "__main__":
	def on_exit(sig, func=None):
		print "Driver Killed, setting throttle and steering to 0"
		PWM_MatchUpdate(
			LPC_PWM1, 1, getServoValue(90), 
			PWM_MATCH_UPDATE_OPT.PWM_MATCH_UPDATE_NOW
			)

		PWM_MatchUpdate(
			LPC_PWM2, 2, getServoValue(90), 
			PWM_MATCH_UPDATE_OPT.PWM_MATCH_UPDATE_NOW
			)
	set_exit_handler(on_exit)

	theDriver()

