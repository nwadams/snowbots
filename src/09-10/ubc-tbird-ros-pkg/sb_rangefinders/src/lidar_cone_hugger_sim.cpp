/** 
 * lidar_cone_hugger ROS node
 * UBC Thunderbird Snowbots Team 2010
 *
 * Description:
 *   This ROS node gets the information of a lidar sensor via a "LaserScan" 
 *   message and sends the steering and throttle info to the commander node.
 *   The algorithem uses the concept of "center of mass" to accurately
 *   determine where the robot should head next.
 *
 * Usage:
 *   In order to be able to work with the simulator (satge), this node should
 *   subscribe to the "base_scan" topic; and if it is to work with the LIDAR,
 *   it should subscribe to the "scan" topic. Uncomment the one that you want
 *   to work with. And also if working with the simulator, change the value of
 *   the SIMULATOR_ON sonstant to 1 (true). 
 *
 * @author: Navid Fattahi
 * Summer 2010
 **/

#include <stdio.h>
#include <iostream>
#include <math.h>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>	/* see: www.ros.org/doc/api/sensor_msgs/html/msg/LaserScan.html */
#include <geometry_msgs/Twist.h>	/* see: www.ros.org/doc/api/geometry_msgs/html/msg/Twist.html */
#include <geometry_msgs/Vector3.h>	/* see: www.ros.org/doc/api/geometry_msgs/html/msg/Vector3.html */

#include <sb_rangefinders/Rangefinder.h>

// Change according to your usage
#define SIMULATOR_ON 1

/* Creating a Rangefinder pointer */
sb_rangefinders::Rangefinder* lidar_ptr;

/* ROS-related Constants */
const std::string NODE_NAME        = "lidar";
const std::string SUBSCRIBE_TOPIC  = "base_scan";	//Works with Stage
//const std::string SUBSCRIBE_TOPIC  = "scan";		//Works with hardware
const std::string PUBLISH_TOPIC    = "cmd_vel";
const int MSG_QUEUE_SIZE = 20;	// # of possible messeges in buffer before deleting the old ones

/* Global Constants */
const int NUM_RAYS = 511;	//Number of lidar-rays emmitted
const double FIELD = 3.00;  	//Maximum radius for creating the imaginary wall (meters)
const double MAX_DISTANCE = 2.99;
const int MAX_STEERING = 30.0;	//Degrees

/* Function Declaration */
void irCallBack(const sensor_msgs::LaserScanConstPtr&);
double calculateSteering(double theta);

using namespace ros;

Publisher dataPub;

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
   //Variable assignment for ease of use	
   float angle_min = laserScanMsg->angle_min;	//angle of IR sensor to right (rad)
   float angle_max  = laserScanMsg->angle_max;	//angle of IR sensor to left  (rad)
   float angle_increment = laserScanMsg->angle_increment;
// double irMax 	    = laserScanMsg->range_max;	//maximum acceptable IR value
// double irMin 	    = laserScanMsg->range_min;	//minimum acceptable IR value

// std::cout << "# rays: " << (angle_max-angle_min)/(angle_increment) << std::endl;

   /* Creating an imaginary wall passing through the cones*/
   int point1 = 0;
   int point2 = 0;
   double myRanges[NUM_RAYS];

   for (int i=0; i < NUM_RAYS; i++)
   {
      // Copying the distances into myRanges array
      myRanges[i] = laserScanMsg->ranges[i];

      if(myRanges[i] < 0.01 && myRanges[i] > -0.01)
        myRanges[i] = MAX_DISTANCE;
/*
      if ( i==7 )
         std::cout << "myRanges[7]: " << myRanges[7] << std::endl;
      if ( i==NUM_RAYS/2)
         std::cout << "myRanges[NUM_RAYS/2]: " << myRanges[NUM_RAYS/2] << std::endl;
      if ( i==NUM_RAYS-7)
         std::cout << "myRanges[NUM_RAYS-7]: " << myRanges[NUM_RAYS-7] << std::endl;
*/

      // If there is an empty space between the cones
      if(myRanges[i] < FIELD || i == NUM_RAYS-1)
      {
         point2 = i;
         for(int j = point2-1; point1 < j; j--)
         {
            myRanges[j] = (myRanges[point1]+myRanges[point2])/2;
         }
         point1 = point2+1;
      }
   }


   double fxcomponents[NUM_RAYS];
   double xcomponents[NUM_RAYS];
   double dx[NUM_RAYS];

   for(int i = 0; i < NUM_RAYS; i++)
   {
      if (i < NUM_RAYS/2)
      {
         //Geometry and Calculus skills!	
         fxcomponents[i] = myRanges[i] * cos((angle_min+7*angle_increment) + (i * angle_increment));
         xcomponents[i] = myRanges[i] * sin((angle_min+7*angle_increment) + (i * angle_increment));
      }
      else
      {
         fxcomponents[i] = myRanges[i] * cos((i-NUM_RAYS/2) * angle_increment);
         xcomponents[i] = myRanges[i] * sin((i-NUM_RAYS/2) * angle_increment);
      }
   }

   /* Calculating the area enclosed by the rays */
   double area = 0;
   for(int i = 0; i < NUM_RAYS; i++)
   {
      if ( i == 0)
         dx[i] = 0;
      else
      {
         if (((xcomponents[i] > 0) & (xcomponents[i-1] >= 0)) || ((xcomponents[i] < 0) & (xcomponents[i-1] <= 0)))
	    dx[i] = abs(xcomponents[i] - xcomponents[i-1]);
         else
	    dx[i] = abs(xcomponents[i] + xcomponents[i-1]);
      }

      area += (fxcomponents[i] * dx[i]);	
   }

   /* Finding x-comp of center of mass */
   double Gx = 0;
   for (int i = 0; i < NUM_RAYS; i++)
   {
      Gx += ((xcomponents[i]*fxcomponents[i]*dx[i])/area);
   }

   /* Finding y-comp of center of mass */
   double Gy = 0;
   for (int i = 0; i < NUM_RAYS; i++)
   {
      Gy += ((fxcomponents[i]*fxcomponents[i]*dx[i])/area/2);
   }

   /*Finding the angle from the center to center of mass */
   double theta = atan(Gy/Gx);
   double thetaDeg = theta*180/M_PI;

   double steering = calculateSteering(theta);

   /* Reverse the steering if using hardware*/
   // This is done because the sensor is physically up-side-down
   if (SIMULATOR_ON==0)
   	steering = -steering;

   std::cout << "Init steering: " << steering << std::endl;
   double steeringBck;
   double throttle = 0.5;

   steering = lidar_ptr->sharpenSteering(steering);

   /* Backing up */
   for (int i=NUM_RAYS*9/20; i < NUM_RAYS*11/20; i++)
   {
      if (myRanges[i] < 0.5)
      {
         steeringBck = -steering;
         throttle = -0.5;
         dataPub.publish(lidar_ptr->vectorCalculator(throttle, steeringBck));
         throttle = 0;
         dataPub.publish(lidar_ptr->vectorCalculator(throttle, steeringBck));
         throttle = -0.5;
         dataPub.publish(lidar_ptr->vectorCalculator(throttle, steeringBck));
         ROS_INFO("I Published: throttle - %f, steering - %f", throttle, steering);
         usleep(100000);
         break;
      }
   }

   dataPub.publish(lidar_ptr->vectorCalculator(throttle, steering));
   ROS_INFO("I Published: throttle - %f, steering - %f", throttle, steering);

   double irFront = myRanges[NUM_RAYS/2];//laserScanMsg->ranges[NUM_RAYS/2];
   double irRight = myRanges[7];
   double irLeft  = myRanges[NUM_RAYS-7];

   /* Some useful stuff to look after */
   ROS_INFO("I received a new messege!\nfront: %f\nright: %f\nleft: %f\n", irFront, irRight, irLeft);
   std::cout << "area: " << area << std::endl;
   std::cout << "Center of mass (x,y): (" << Gx << ", " << Gy << ")" << std::endl;
   std::cout << "theta (Gy/Gx): " << theta << std::endl;
   std::cout << "thetaDEG: " << thetaDeg << std::endl;
   std::cout << "Throttle: " << throttle << ", Steering: " << steering << std::endl;
   std::cout << "*****************************\n" << std::endl;
}

double calculateSteering(double theta)
{
   double thetaDegrees = theta*180/M_PI;
   double steering;

   if(thetaDegrees > MAX_STEERING)
     steering = 1.0;
   else
     steering = thetaDegrees/MAX_STEERING;

   return steering;
}
