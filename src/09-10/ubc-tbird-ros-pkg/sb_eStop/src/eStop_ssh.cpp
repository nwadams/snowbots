/*
Nick Adams
Last Edited May 4, 2010
A simple ros node for an emergency stop.
the get_eStop function can be replaced with a function that receives instructions from a wireless or bluetooth device to wirelessly stop the vehicle
*/

#include <cstdlib>
#include <iostream>
#include <stdio.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

using namespace std;

bool get_eStop(bool eStop)
{	
	char c;
	cin >> c; 
	return !eStop;
}
	
int main (int argc, char** argv)
{
	
	bool eStop = 1;
	
	ros::init(argc, argv, "eStop");
	ros::NodeHandle n;
	ros::Publisher chatter_pub = n.advertise<std_msgs::Bool>("eStop", 1);
	ros::Rate loop_rate(10);

	ROS_INFO("e-stop ready");

	if(ros::ok()) 
	{
		eStop = get_eStop(eStop);	
		
		std_msgs::Bool msg;
		msg.data = eStop;
		chatter_pub.publish(msg);
		ROS_INFO("I published [%d]", eStop);
		eStop = get_eStop(eStop);	
		
		msg.data = eStop;
		chatter_pub.publish(msg);
		ROS_INFO("I published [%d]", eStop);
	}

	return 0;
}
