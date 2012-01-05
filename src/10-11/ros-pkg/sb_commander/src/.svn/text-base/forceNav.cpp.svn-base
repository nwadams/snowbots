/*
Force Nav 2.0
Date : Aug 17th, 2011
Author : Mark

Version : 1.0.0.1

Reuse/Modify Ian's ForceNav code

Each laser point from LIDAR will emit a force towards the vehicle.
Steer and throttle adjusting the net force.

*/



#include <math.h>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>

using namespace ros;
using namespace std;

//ros related constants
static const string NODE_NAME = "";
static const string SUBSCRIBE_TOPIC = "base_scan";
static const string PUBLISH_TOPIC = "cmd_vel";
static int LOOP_FREQ = 30;
geometry_msgs::Vector3 vec;

//forceNav related constatns
static const double IGNORE_ANGLE = 3.1415265;


//call back function
void callback(const sensor_msgs::LaserScanConstPtr& msg_ptr)
{
	int num_rays = msg_ptr->ranges.size();
	double x_total = 0.0;
	double y_total = 0.0;
	int valid_rays = 0;
	for(int i =0; i < num_rays;i++)
	{
		float angle = msg_ptr->angle_min + i*(msg_ptr->angle_increment);
		float dist = msg_ptr->ranges[i];
		if(angle < -IGNORE_ANGLE)
		{
			continue;
		}
		if(angle > IGNORE_ANGLE)
		{
			continue;
		}
		if(dist == msg_ptr->range_max)
		{
			continue;
		}
		float force = -1.0/dist;
		x_total += force * cos(angle);
		y_total += force * sin(angle);
		valid_rays++;		
	}
	if(valid_rays <= 0)
	{
		ROS_FATAL("No valid rays found");
		return;
	}


	vec.x = -3* x_total / valid_rays;
	vec.y = 5* y_total / valid_rays;
	vec.z = 0;
	
}

geometry_msgs::Twist twist_converter(geometry_msgs::Vector3 vec)
{
	geometry_msgs::Twist twist;
	geometry_msgs::Vector3 Linear;
	geometry_msgs::Vector3 Angular;

	Linear.x = vec.x;
	Linear.y = 0;
	Linear.z = 0;

	Angular.x = 0;
	Angular.y = 0;
	Angular.z = vec.y;

	twist.linear = Linear;
	twist.angular = Angular;

	return twist;
	
}



int main (int argc, char** argv)
{
	init(argc, argv,NODE_NAME);
	NodeHandle n;
	

	Subscriber lidar_state = n.subscribe(SUBSCRIBE_TOPIC,20,callback);
	
	Publisher car_pub = n.advertise<geometry_msgs::Twist>(PUBLISH_TOPIC,1);
	Rate loop_rate(LOOP_FREQ);
	ROS_INFO("ready to go");
	
	ROS_INFO("going");
	while(ros::ok())
	{
		geometry_msgs::Twist twistMsg = twist_converter(vec);
		ROS_INFO("the twist vector is %f , %f", &twistMsg.linear.x, &twistMsg.angular.z); 
		car_pub.publish(twistMsg);
	  ros::spinOnce();
    loop_rate.sleep();	
  }
  ROS_INFO("shutting down node");
  
  return 0;
}
