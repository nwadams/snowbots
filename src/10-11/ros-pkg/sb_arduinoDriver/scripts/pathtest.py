#!/usr/bin/env python
"""
Snowbot driver for python
basicly a translation of arduinoDriver into python for the robovero
"""
import os, sys

#sys.path.append('/home/nwadams/robovero/python')

import roslib 
roslib.load_manifest('sb_arduinoDriver')
import rospy
from sb_msgs.msg import RobotState
from sb_msgs.msg import LidarNav
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3, Twist
import time
import math

from robovero.LPC17xx import LPC_PWM1, LPC_PWM2
from robovero.lpc17xx_pwm import PWM_TIMER_PRESCALE_OPT, PWM_TC_MODE_OPT, \
					PWM_MATCH_UPDATE_OPT, PWM_TIMERCFG_Type, PWM_MATCHCFG_Type, \
					PWM_Init, PWM_MatchUpdate, PWM_ConfigMatch, PWM_ChannelCmd, \
					PWM_ResetCounter, PWM_CounterCmd, PWM_Cmd
from robovero.extras import roboveroConfig
from robovero.lpc_types import FunctionalState

if __name__ == "__main__":
	print "there"
	print str(sys.path)
