#include <stdio.h>
#include <iostream>
#include <math.h>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>	/* see: www.ros.org/doc/api/sensor_msgs/html/msg/LaserScan.html */

using namespace ros;

const std::string NODE_NAME        = "irAnalyzer";
const std::string SUBSCRIBE_TOPIC  = "base_scan";
const int MSG_QUEUE_SIZE = 20;		// # of possible messeges in buffer before deleting the old ones

void irCallBack(const sensor_msgs::LaserScanConstPtr&);

int main(int argc, char** argv)
{	
	ROS_INFO("Starting %s ...", NODE_NAME.c_str());	
	init(argc, argv, NODE_NAME);
	NodeHandle n;
	Subscriber dataSub = n.subscribe(SUBSCRIBE_TOPIC, MSG_QUEUE_SIZE, irCallBack);
	
	while (ok())
		spin();

	ROS_INFO("Sutting down %s!", NODE_NAME.c_str());
}

/* this method is called whenever a new messegae is arrived */
void irCallBack(const sensor_msgs::LaserScanConstPtr& laserScanMsg)
{
	//Variable assignment for ease of use	
	double irRightAngle = laserScanMsg->angle_min;	//angle of IR sensor to right (rad)
	double irLeftAngle  = laserScanMsg->angle_max;	//angle of IR sensor to left  (rad)
//	double irMax 	    = laserScanMsg->range_max;	//maximum acceptable IR value
//	double irMin 	    = laserScanMsg->range_min;	//minimum acceptable IR value

	double irFront = laserScanMsg->ranges[1];
	double irRight = laserScanMsg->ranges[0] * cos(irRightAngle);	//Distance from Right
	double irLeft  = laserScanMsg->ranges[2] * cos(irLeftAngle);	//Distance from Left

	ROS_INFO("angle_min: %f\nangle_max: %f\n", irRightAngle, irLeftAngle);
	ROS_INFO("I received a new messege!\nfront: %f\nright: %f\nleft: %f\n", irFront, irRight, irLeft);
}
