/*
Navigation 1.4

Code used during Robot Racing 2011 Competition

Name: Edward Han
Date: July 23, 2011

*/

#include <iostream>
#include <stdlib.h>
#include <string>
#include <math.h>
#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/LaserScan.h>
#include "sb_msgs/RobotState.h"
#include "sb_msgs/CarCommand.h"
#include <sb_msgs/LidarNav.h>
#include "ConeLocation.h"
#include "Path.h"

// blizzard driver rosrun sb_arduinoDriver bliz(tab)
// rosrun hok(tab) hok(tab)
// Function Signatures

// steering = 1 is left in stage so steering = -1 is right

geometry_msgs::Twist twistConvertor(sb_msgs::CarCommand car_msg);

using namespace ros;
using namespace std;

// Global Constants
static const int SECOND = 1000000; //1 million us	

// ROS-related Constants
static const string NODE_NAME          = "magical_commander";
static const string CAR_PUBLISH_TOPIC  = "cmd_vel";
static const string IR_SUBSCRIBE_TOPIC = "base_scan"; //scan for real world, base_scan for stage
static const string VISION_SUBSCRIBE_TOPIC = "traffic_light";
//static const string LIDAR_SUBSCRIBE_TOPIC = "lidar_nav";
static const int LOOP_FREQ             = 30; // Hz

double my_data[550];
int my_count = 0;
double throttle = 0;
double steering = 0; // 1 is right, 30 shown on arduino driver (real world), -1 is left

ConeLocation data;
Path path;
bool stopSignFlag = false;
bool redLightFlag = false;

void Stop()
{
	throttle = 0;
	steering = 0;
		
}

void vision_callback(const std_msgs::Float64ConstPtr& float64Msg)
{
  double confidence;
  confidence = float64Msg->data;
  cout << "the confidence is: "<<confidence << endl;
}


//call back for lidar directions
void ir_state_callback(const sensor_msgs::LaserScanConstPtr& msg_ptr)
{
   data.GetConstants(msg_ptr->range_max, msg_ptr->range_min, msg_ptr->angle_max, msg_ptr->angle_min, msg_ptr->angle_increment);		


	for (int i = 0; i < msg_ptr->ranges.size(); i++) 
	{
	   data.my_data[i] = msg_ptr->ranges[i];
	} 
	
	data.GetDistanceAngle();
	data.IdentifyPoint();
	data.SeparatePoint(); 
	
	path.FindClosestPoint(data.left, data.front, data.right); // left, front, right
	path.DeterminePath();

	throttle = path.car.throttle;
	steering = path.car.steering; 

	if (my_count == 2)
	{
		//data.PrintConeStage();
		//data.PrintConstants();
		//path.PrintClosestPointStage();
		//data.PrintConstants();
		path.PrintClosestPoint();
		my_count = 0;
	}
	
	data.Clear();
	path.Clear();
	my_count++;
}

sb_msgs::CarCommand driver()
{
    sb_msgs::CarCommand car_msg;

    //uncomment the two lines below and populate car_msg with data 
    car_msg.throttle = throttle; //Throtle;
    car_msg.steering = steering;//Steering;
     
    // geometry_msgs::Twist twist_msg = twistConvertor(car_msg);

    return car_msg;
}


int main( int argc, char** argv )
{
 
    //ros initialization
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle n;
    
    //subscribes to IR topic to receive data from arduino
    //lidar_class my_lidar_class;

    ros::Subscriber IR_state = n.subscribe(IR_SUBSCRIBE_TOPIC, 20, ir_state_callback);
  //  ros::Subscriber Lidar_instructions = n.subscribe(LIDAR_SUBSCRIBE_TOPIC, 3, &lidar_class::callback,&my_lidar_class);
    ros::Subscriber Vision = n.subscribe(VISION_SUBSCRIBE_TOPIC, 20, vision_callback);

    ros::Publisher car_pub = n.advertise<geometry_msgs::Twist>(CAR_PUBLISH_TOPIC, 1);
	
		

    //controls how fast it goes
    ros::Rate loop_rate(LOOP_FREQ);

    ROS_INFO("ready to go");
    usleep(3*SECOND);
       
    ROS_INFO("going");   
    
    while(ros::ok())
    {
	
        //driver is navigation function
    	
        //publshing data to robot
       // ROS_INFO("sending throttle=%f, steering=%f", twist_msg.linear.x, twist_msg.angular.z);

	//sb_msgs::CarCommand car_msg = my_lidar_class.get_car_msg();
	sb_msgs::CarCommand car_msg = driver();
        geometry_msgs::Twist twist_msg = twistConvertor(car_msg);
        car_pub.publish(twist_msg);
        
        
        //checking callbacks and sleeping to match loop rate
        ros::spinOnce();
        loop_rate.sleep();	
    }
    ROS_INFO("shutting down node");
  
    return 0;
}

geometry_msgs::Twist twistConvertor(sb_msgs::CarCommand car_msg)
  {
      //Creating instances of Twist and Vector3 classes	
      geometry_msgs::Twist twistReturn;
      geometry_msgs::Vector3 linearVel;
      geometry_msgs::Vector3 angularVel;

      //Since the robot can move in only one direction (x-axis),
      //we set the other two to zero	
      linearVel.x = car_msg.throttle;
     // cout<<"throttle is "<<car_msg.throttle<<endl;
      linearVel.y = 0;
      linearVel.z = 0;

      //Since the robot can turn around z-axis only,
      //we set the other two to zero
      angularVel.x = 0;
      angularVel.y = 0;
      angularVel.z = car_msg.steering;

      twistReturn.linear = linearVel;
      twistReturn.angular = angularVel;

      return twistReturn;
  }
