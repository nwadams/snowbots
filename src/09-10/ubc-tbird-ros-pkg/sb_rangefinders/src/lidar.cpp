#include <stdio.h>
#include <iostream>
#include <math.h>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>	/* see: www.ros.org/doc/api/sensor_msgs/html/msg/LaserScan.html */
#include <geometry_msgs/Twist.h>	/* see: www.ros.org/doc/api/geometry_msgs/html/msg/Twist.html */
#include <geometry_msgs/Vector3.h>	/* see: www.ros.org/doc/api/geometry_msgs/html/msg/Vector3.html */

using namespace ros;

/* Global Variables */
Publisher dataPub;

/* ROS-Related Constants */
const std::string NODE_NAME        = "lidar";
const std::string SUBSCRIBE_TOPIC  = "base_scan";
const std::string PUBLISH_TOPIC    = "cmd_vel";
const int MSG_QUEUE_SIZE = 20;	   // # of possible messeges in buffer before deleting the old ones

/* Usufel Constants */
const int NUM_RAYS = 665;

/* Function Declarations */
void irCallBack(const sensor_msgs::LaserScanConstPtr&);
geometry_msgs::Twist vectorCalculator(double throttle, double steering);
double sharpenSteering(double steering);

int main(int argc, char** argv)
{
  /* Starting and initializing this node */	
  ROS_INFO("Starting %s ...", NODE_NAME.c_str());	
  init(argc, argv, NODE_NAME);	
  NodeHandle n;

  /* Defining subscriber and publisher */
  Subscriber dataSub = n.subscribe(SUBSCRIBE_TOPIC, MSG_QUEUE_SIZE, irCallBack);
  dataPub = n.advertise<geometry_msgs::Twist>(PUBLISH_TOPIC, MSG_QUEUE_SIZE);
	
  /* Main loop */
  while (ok())
	spin();

  /* Shuttiing down this node */
  ROS_INFO("Shutting down %s!", NODE_NAME.c_str());
}

/* this method is called whenever a new messegae is arrived */
void irCallBack(const sensor_msgs::LaserScanConstPtr& laserScanMsg)
{
  //Variable assignment for ease of use	
  double angle_min = laserScanMsg->angle_min;	//angle of IR sensor to right (rad)
  double angle_max  = laserScanMsg->angle_max;	//angle of IR sensor to left  (rad)
  double angle_increment = laserScanMsg->angle_increment;
//double irMax 	    = laserScanMsg->range_max;	//maximum acceptable IR value
//double irMin 	    = laserScanMsg->range_min;	//minimum acceptable IR value

  double fxcomponents[NUM_RAYS];	//f(x): component of rays along x-axis
  double xcomponents[NUM_RAYS];		// x  : component of rays along y-axis
  double dx[NUM_RAYS];			// dx : x(i) - x(i-1)

  /* Finding x and y-component of rays and assigning them into arrays*/
  for(int i = 0; i < NUM_RAYS; i++)
  {
	if (i < NUM_RAYS/2)
	{
	  //For angles to the right of the robot (i.e. 4th quadrant)	
	  fxcomponents[i] = laserScanMsg->ranges[i] * cos(angle_min + (i * angle_increment));
	   xcomponents[i] = laserScanMsg->ranges[i] * sin(angle_min + (i * angle_increment));
	}
	else
	{
	  //For angles to the left of the robot (i.e. 1st quadrant)
	  fxcomponents[i] = laserScanMsg->ranges[i] * cos((i-NUM_RAYS/2) * angle_increment);
	   xcomponents[i] = laserScanMsg->ranges[i] * sin((i-NUM_RAYS/2) * angle_increment);
	}
  }

  /* Calculating the area enclosed by the rays */
  double area = 0;
  for(int i = 0; i < NUM_RAYS; i++)
  {
	if ( i == 0)
	{
	  dx[i] = 0;//abs(xcomponents[i] - irMax);
	}
	else
	{
	  if (((xcomponents[i] > 0) & (xcomponents[i-1] >= 0)) || ((xcomponents[i] < 0) & (xcomponents[i-1] <= 0)))
		dx[i] = abs(xcomponents[i] - xcomponents[i-1]);
	}
	area += fxcomponents[i] * dx[i];	
  }

  //Finding x-comp of center of mass
  double Gx = 0;
  for (int i = 0; i < NUM_RAYS; i++)
  {
	Gx += (xcomponents[i]*fxcomponents[i]*dx[i])/area;
  }

  //Finding y-comp of center of mass
  double Gy = 0;
  for (int i = 0; i < NUM_RAYS; i++)
  {
	Gy += (fxcomponents[i]*fxcomponents[i]*dx[i])/area/2;
  }

  //Its x/y because our graph is converted	
  double theta = atan(Gx/Gy);
  double steering = sharpenSteering(theta);

  dataPub.publish(vectorCalculator(0.5, steering));

  double irFront = laserScanMsg->ranges[332];
  double irRight = laserScanMsg->ranges[0];
  double irLeft  = laserScanMsg->ranges[665];

	ROS_INFO("area: %f\n", area);
	ROS_INFO("Gx: %f\n", Gx);
	ROS_INFO("Gy: %f\n", Gy);
	ROS_INFO("theta: %f\n", theta);
	ROS_INFO("angle_min: %f\nangle_max: %f\n%f\n", angle_min, angle_max, angle_increment);
	ROS_INFO("I received a new messege!\nfront: %f\nright: %f\nleft: %f\n", irFront, irRight, irLeft);
}

double sharpenSteering(double steering)
{
  double newSteering;

  if (steering >= 0)
	newSteering = sqrt(steering);
  else
	newSteering = sqrt(-steering)*(-1);

  return newSteering;
}

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
