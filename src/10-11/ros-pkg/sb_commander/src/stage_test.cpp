#include <iostream>
#include <stdlib.h>
#include <string>
#include <math.h>
#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/LaserScan.h>
#include "sb_msgs/RobotState.h"
#include "sb_msgs/CarCommand.h"


// Function Signatures
geometry_msgs::Twist twistConvertor(sb_msgs::CarCommand car_msg);

using namespace std;

// Global Constants
static const int SECOND = 1000000; //1 million us

// ROS-related Constants
static const string NODE_NAME          = "tutorial_commander";
static const string CAR_PUBLISH_TOPIC  = "cmd_vel";
static const string IR_SUBSCRIBE_TOPIC = "base_scan";
static const int LOOP_FREQ             = 30; // Hz

//IR values, approximately at -45, -22.5, 0, 22.5 and 45 respectively
class IR
{
  //distance from vehicle in m
  public:
    double W;
    double NW;
    double N;
    double NE;
    double E;
};

//global variable for IR data
IR state;

//call back for new IR data
void ir_state_callback(const sensor_msgs::LaserScanConstPtr& msg_ptr)
{


        double min_angle = msg_ptr->angle_min;
	double max_angle = msg_ptr->angle_max;
	double d_angle = msg_ptr->angle_increment;
	
	int num_vals = ((max_angle - min_angle) / d_angle);

	state.N = msg_ptr->ranges[num_vals/2]; 
	
}

/* use the IR data to navigate robot. car_msg.steering and car_msg.throttle values
 * they are from -1 <_ x <_ 1. 
 * throttle: -1 is full reverse, 1 is full forward, 0 is stopped
 * steering: -1 is full right, 1 is full left, 0 is straight
 * Will need to use the ros time libraries for timed events
 ex: ros::Time now = ros::Time::now();
  if(now-begin > 3) (returns true if now is more than three seconds after begin)
  
  Using sleep and manually publishing events to the driver would be simpler than
  using time for the simple examples but for more complex and real world situations
  using a time counter instead of a delay is much simpler,
  
 */

geometry_msgs::Twist driver()
{
    sb_msgs::CarCommand car_msg;

	float Steering = 0;
	float Throtle;
	

	if(state.N > 1.0)
	{
		Steering = 0;
		Throtle = 1.0;
	} 
	else 
	{
		Throtle = 0.5;
		Steering = 1;
	} 

    //uncomment the two lines below and populate car_msg with data 
    car_msg.throttle = Throtle;
    car_msg.steering = Steering;
     
    geometry_msgs::Twist twist_msg = twistConvertor(car_msg);
    return twist_msg;
}

/////////////////////////////////////////////////////////////////////////////////////////

int main( int argc, char** argv )
{
    //ros initialization
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle n;
    
    //subscribes to IR topic to receive data from arduino
    ros::Subscriber IR_state = n.subscribe(IR_SUBSCRIBE_TOPIC, 20, ir_state_callback);
    
    //publisher to publish new data back to arduino
    ros::Publisher car_pub = n.advertise<geometry_msgs::Twist>(CAR_PUBLISH_TOPIC, 1);

    //controls how fast it goes
    ros::Rate loop_rate(LOOP_FREQ);

    ROS_INFO("ready to go");
    usleep(3*SECOND);
       
    ROS_INFO("going");   
    
    while(ros::ok())
    {
        //driver is navigation function
        geometry_msgs::Twist twist_msg = driver();
    
        //publshing data to robot
       // ROS_INFO("sending throttle=%f, steering=%f", twist_msg.linear.x, twist_msg.angular.z);
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
