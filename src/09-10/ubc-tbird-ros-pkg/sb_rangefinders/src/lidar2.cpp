#include <stdio.h>
#include <iostream>
#include <math.h>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>	/* see: www.ros.org/doc/api/sensor_msgs/html/msg/LaserScan.html */
#include <geometry_msgs/Twist.h>	/* see: www.ros.org/doc/api/geometry_msgs/html/msg/Twist.html */
#include <geometry_msgs/Vector3.h>	/* see: www.ros.org/doc/api/geometry_msgs/html/msg/Vector3.html */

#include <sb_rangefinders/Rangefinder.h>

/* Creating a Rangefinder pointer */
sb_rangefinders::Rangefinder* lidar_ptr;

/* ROS-related Constants */
const std::string NODE_NAME        = "lidar";
const std::string SUBSCRIBE_TOPIC  = "base_scan";
const std::string PUBLISH_TOPIC    = "cmd_vel";
const int MSG_QUEUE_SIZE = 20;	// # of possible messeges in buffer before deleting the old ones

const int NUM_RAYS = 665;	//Number of lidar-rays emmitted

/* Function Declaration */
void irCallBack(const sensor_msgs::LaserScanConstPtr&);

using namespace ros;

//Global Variables
Publisher dataPub;
double steeringFwd = 0;
double steeringBck = 0;

int main(int argc, char** argv)
{	
   ROS_INFO("Starting %s ...", NODE_NAME.c_str());	
   init(argc, argv, NODE_NAME);
   NodeHandle n;
   Subscriber dataSub = n.subscribe(SUBSCRIBE_TOPIC, MSG_QUEUE_SIZE, irCallBack);
   dataPub = n.advertise<geometry_msgs::Twist>(PUBLISH_TOPIC, MSG_QUEUE_SIZE);

   while (ok())
       spin();

   ROS_INFO("Shutting down %s!", NODE_NAME.c_str());
   delete lidar_ptr;
}

/* this method is called whenever a new messegae is arrived */
void irCallBack(const sensor_msgs::LaserScanConstPtr& laserScanMsg)
{
	//Variables
	double throttle = 0.5;

	//Creating an instance of the Twist message
	geometry_msgs::Twist twist;

	//Variable assignment for ease of use	
//	double irRightAngle = laserScanMsg->angle_min;	//angle of IR sensor to right (rad)
//	double irLeftAngle  = laserScanMsg->angle_max;	//angle of IR sensor to left  (rad)
	double irMax 	    = laserScanMsg->range_max;	//maximum acceptable IR value (meters)
	double irMin 	    = laserScanMsg->range_min;	//minimum acceptable IR value (meters)

	double safeZone = 1.5;
	double dangerZone = 1.0;

	double irFront = laserScanMsg->ranges[NUM_RAYS/2];
	double irSide = laserScanMsg->ranges[NUM_RAYS];
//	double irLeft  = laserScanMsg->ranges[NUM_RAYS];

	/* if too close to collide with an object */	
	if (irFront < dangerZone)
	{
		throttle = 0;	//Stop!

		std::cout << "Ooops, that was close!" << std::endl;

		/* Backing up */
		steeringBck = steeringFwd * (-1);
		throttle = -0.5;
		dataPub.publish(lidar_ptr->vectorCalculator(throttle, steeringBck));
		usleep(1000000);
	}
	else
	{
		/* if the IR value is reliable (i.e. within the acceptable range) */
		if (irSide > irMin && irSide < irMax)
		{
			/* and if too close to the right side */
			if (irSide < dangerZone)
			{
				//turn right accordingly (relative to distance from obsticales)
				throttle = 0.5;
				//the sqrt function is used to make the turns smooth when relatively
				//far from objects and sharp when close to them
				steeringFwd = sqrt((dangerZone - irSide)/dangerZone) * (-1);
				//publish throttle and steering through a Twsit message (on topic cmd_vel)
				dataPub.publish(lidar_ptr->vectorCalculator(throttle, steeringFwd));
				std::cout << "Turn left by: " << steeringFwd << " rads" << std::endl;
			}
			if (irSide > safeZone)
			{				
				//turn left accordingly (relative to distance from obsticales)
				throttle = 0.5;
				//the sqrt function is used to make the turns smooth when relatively
				//far from objects and sharp when close to them
				steeringFwd = sqrt(-(safeZone - irSide)/irSide) * (1);
				//publish throttle and steering through a Twsit message (on topic cmd_vel)
				dataPub.publish(lidar_ptr->vectorCalculator(throttle, steeringFwd));
				std::cout << "Turn right by: " << steeringFwd << " rads" << std::endl;
			}
		}
	}
	dataPub.publish(lidar_ptr->vectorCalculator(throttle, steeringFwd));
}
