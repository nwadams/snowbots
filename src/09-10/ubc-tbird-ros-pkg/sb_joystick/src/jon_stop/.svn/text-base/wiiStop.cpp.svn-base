/*
Koko Yu: based on Nick's eStop.cpp
Last Edited , 2010
simple eStop, open for possibility of a wireless joystick.
*/

#include <cstdlib>
#include <unistd.h>
#include <iostream>
#include <stdio.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include "wiimote.h"

using namespace std;
	
int main (int argc, char** argv)
{
	
	bool eStop = 0;
	
	ros::init(argc, argv, "eStop");
	ros::NodeHandle n;
	ros::Publisher chatter_pub = n.advertise<std_msgs::Bool>("eStop", 1);
	ros::Rate loop_rate(100);	
	
	std_msgs::Bool msg;
	msg.data = eStop;

	WiiDriver remote;
	bool good = remote.Init();
	
	chatter_pub.publish(msg);
	ROS_INFO("let's go");

	while(ros::ok() && good ) 
	{
		eStop = remote.IsPressed();
		if( eStop ){
			msg.data = eStop;
			chatter_pub.publish(msg);
			ROS_INFO("I published [%d]", eStop);
		}
		loop_rate.sleep();
	}

	return 0;
}
