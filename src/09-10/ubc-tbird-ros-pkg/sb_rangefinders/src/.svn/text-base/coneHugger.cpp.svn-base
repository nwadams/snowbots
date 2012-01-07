/** ConeHugger ROS Node
 *
 * This node analyzes the messeges of type "LaserScan" upon receiving them
 * and publishes a messege of type "IrAdvisor" containing suggested throtle
 * and steering values for the commandor to decide what to do
 * 
 * Date: 13/5/2010
 * Modified: 17/5/2010
 * @author: Navid Fattahi <navid.fattahi@gmail.com>
 */

#include <stdio.h>
#include <iostream>
#include <math.h>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>	/* see: www.ros.org/doc/api/sensor_msgs/html/msg/LaserScan.html */
#include <geometry_msgs/Twist.h>	/* see: www.ros.org/doc/api/geometry_msgs/html/msg/Twist.html */
#include <geometry_msgs/Vector3.h>	/* see: www.ros.org/doc/api/geometry_msgs/html/msg/Vector3.html */

using namespace ros;

const std::string NODE_NAME        = "irAnalyzer";
const std::string SUBSCRIBE_TOPIC  = "base_scan";
const std::string PUBLISH_TOPIC    = "cmd_vel";
const int MSG_QUEUE_SIZE = 20;		// # of possible messeges in buffer before deleting the old ones

//Global Variables
Publisher dataPub;
double steeringFwd;
double steeringBck;

//Function Declarance
void irCallBack(const sensor_msgs::LaserScanConstPtr&);
geometry_msgs::Twist vectorCalculator(double throttle, double steering);

int main(int argc, char** argv)
{	
	/* Initializing this node */	
	ROS_INFO("Starting %s ...", NODE_NAME.c_str());	
	init(argc, argv, NODE_NAME);

	/* Creating a subscriber and publisher */	
	NodeHandle n;
	Subscriber dataSub = n.subscribe(SUBSCRIBE_TOPIC, MSG_QUEUE_SIZE, irCallBack);
	dataPub = n.advertise<geometry_msgs::Twist>(PUBLISH_TOPIC, MSG_QUEUE_SIZE);

	while (ok())
		spin();

	ROS_INFO("Shutting down %s!", NODE_NAME.c_str());
}

/* this method is called whenever a new messegae is arrived */
void irCallBack(const sensor_msgs::LaserScanConstPtr& laserScanMsg)
{
	//Variables
	double throttle;

	//Creating an instance of the Twist message
	geometry_msgs::Twist twist;

	//Variable assignment for ease of use	
//	double irRightAngle = laserScanMsg->angle_min;	//angle of IR sensor to right (rad)
//	double irLeftAngle  = laserScanMsg->angle_max;	//angle of IR sensor to left  (rad)
	double irMax 	    = laserScanMsg->range_max;	//maximum acceptable IR value (meters)
	double irMin 	    = laserScanMsg->range_min;	//minimum acceptable IR value (meters)

	double irFront = laserScanMsg->ranges[1];
	double irRight = laserScanMsg->ranges[0];
	double irLeft  = laserScanMsg->ranges[2];

	double dangerZone = 1.0;

	/* if too close to collide with an object */	
	if (irFront < dangerZone)
	{
		throttle = 0;	//Stop!

		std::cout << "Ooops, that was close!" << std::endl;

		/* Backing up */
		steeringBck = steeringFwd * (-1);
		throttle = -0.5;
		dataPub.publish(vectorCalculator(throttle, steeringBck));
		usleep(1000000);
	}
	else
	{
		/* if the IR value is reliable (i.e. within the acceptable range) */
		if (irRight > irMin && irRight < irMax)
		{
			/* and if too close to the right side */
			if (irRight < dangerZone)
			{				
				//turn left accordingly (relative to distance from obsticales)
				throttle = 0.5;
				//the sqrt function is used to make the turns smooth when relatively
				//far from objects and sharp when close to them
				steeringFwd = sqrt((dangerZone - irRight)/dangerZone) * (1);
				//publish throttle and steering through a Twsit message (on topic cmd_vel)
				dataPub.publish(vectorCalculator(throttle, steeringFwd));
				std::cout << "Turn left by: " << steeringFwd << " rads" << std::endl;
			}
		}
		/* if the IR value is reliable (i.e. within the acceptable range) */
		if (irLeft > irMin && irLeft < irMax)
		{
			/* and if too close to the left side */
			if (irLeft < dangerZone)
			{
				//turn right accordingly (relative to distance)
				throttle = 0.5;
				steeringFwd = sqrt((dangerZone - irLeft)/dangerZone) * (-1);
				dataPub.publish(vectorCalculator(throttle, steeringFwd));
				std::cout << "Turn right by: " << steeringFwd << " rads" << std::endl;
			}
		}

	}
	dataPub.publish(vectorCalculator(throttle, steeringFwd));
}

/* This method is used to convert the trottle and steering values to a
   twist object. We use this twist object as a message to publish throttle
   and steering values into the cmd_vel topic. */
geometry_msgs::Twist vectorCalculator(double throttle, double steering)
{
	//Creating instances of Twist and Vector3 classes	
	geometry_msgs::Twist twistReturn;
	geometry_msgs::Vector3 linearVel;
	geometry_msgs::Vector3 angularVel;
	
	//Since the robot can move in only one direction (x-axis),
	//we set the other two to zero	
	linearVel.x = throttle;
	linearVel.y = 0;
	linearVel.z = 0;

	//Since the robot can turn around z-axis only,
	//we set the other two to zero
	angularVel.x = 0;
	angularVel.y = 0;
	angularVel.z = steering;

	twistReturn.linear = linearVel;
	twistReturn.angular = angularVel;

	return twistReturn;
}
