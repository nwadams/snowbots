/* 
 File: tutorial.cpp
 Created by: Nick Adams
 Last editied: September 11, 2010

 This is a tutorial commander node to quickly get up and running when joining 
 snowbots. The goal of this tutorial is to learn about the ROS framework while 
 making it easy to have a drivable robot. A basic level of C/C++ or java 
 knowledge is suggested for this tutorial. A python version also available. 
 
 Tutorial objectives:
 1. Go in a straight line for a pre-determined amount of time and then stop
 (ex. go forward for 3 seconds then stop)

 2. Turn Left and/or Right for a pre-determined amount of time and then stop
 (ex. turn half left for 2 seconds, then full right for 1 and then stop)

 3. Go forward and backwards (straight and curves)

 4. Go forward at a constant speed until the front IR (N) gets within 40 cm. 
 Stop when the front IR reads less than 40 cm. 

 5. Go forward at a contstant speed until the front IR (N) gets within 40 cm.
 Stop and then go backwards for a second or so. The idea of this is so when you
 get stuck you can backup to correct yourself and keep going. 

 6. Use the other 4 IR sensors to turn the robot away from obstacles on the 
 sides. The goal is to maintain the robot in the middle of a road. NOTE: This 
 part is the hardest and will probably take the most time. After completing this
 goal you will have a basic navigatino algorithm for a robot that can navigate 
 around a basic circuit course by itself without crashing.

 After some people get working robots we will have a little friendly competition
 between everyone to inspire ourselves to make better navigation algorithms. 
 But everyone should work together to try and beat Nick. 

 After completing this tutorial a good place to begin is to setup an environment 
 on your own computer or use one of the snowbots computers and begin following
 the ROS beginner tutorials at ROS.org.
 
 Setting up an environment can be difficult but I have written a script to do 
 most of the work. You will need to install Ubuntu linux 10.04 and download the 
 repository. Ask us for help with either of these if you have never done this 
 before.

*/

#include <iostream>
#include <stdlib.h>
#include <string>
#include <math.h>

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
	std::cout << "state.W = " << std::endl;    
	state.W = msg_ptr->ranges[4];
	
	std::cout << "state.W = " << state.W << std::endl;
  state.NW = msg_ptr->ranges[3];
  
  std::cout << "state.NW = " << state.NW << std::endl;
  state.N = msg_ptr->ranges[2];
  
  std::cout << "state.N = " << state.N << std::endl;
  state.NE = msg_ptr->ranges[1];
  
  std::cout << "state.NE = " << state.NE << std::endl;
  state.E = msg_ptr->ranges[0];
  
  std::cout << "state.E = " << state.E << std::endl;
	
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
geometry_msgs::Twist driver(ros::Time begin, double arg[])
{
    sb_msgs::CarCommand car_msg;

    // Begin writing here
	double SteeringFactor = arg[0]*pow(state.E,arg[1]) + arg[2]*pow(state.NE,arg[3]) -  arg[2]*pow(state.NW,arg[3]) - arg[0]*pow(state.W,arg[1]);
	double ThrotleFactor = arg[4]*pow(state.N,arg[5]) + arg[6]*pow(state.NE,arg[7]) + arg[6]*pow(state.NW,arg[7]) + arg[8]*pow(state.E,arg[9]) + arg[8]*pow(state.W,arg[9]);
	double Throtle = fabs(ThrotleFactor/(arg[10]*pow(state.E,arg[11]) + arg[12]*pow(state.NE,arg[13]) -  arg[12]*pow(state.NW,arg[13]) - arg[10]*pow(state.W,arg[11])));
	double Steering = SteeringFactor/(arg[14]*pow(state.N,arg[15]) + arg[16]*pow(state.NE,arg[17]) + arg[16]*pow(state.NW,arg[17]) + arg[18]*pow(state.E,arg[19]) + arg[18]*pow(state.W,arg[19]));
    // End writing

    //uncomment the two lines below and populate car_msg with data 
    car_msg.throttle = Throtle;
    car_msg.steering = Steering;

    ros::Time now = ros::Time::now();
    if(now.toSec() - begin.toSec() < 3)
    {
      car_msg.throttle = 0;
      car_msg.steering = 0;    
    }   
     
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
    
    //set start time
    ros::Time begin = ros::Time::now();
     
	double parameters[]  = {0.4,1,0.1,1,0.4,1,0.2,1,0.1,1,2,1,1,13,1,2,1,1,1}; 
    for (int i = 0; i < argc; i++)
    {
		parameters[i] = atof(argv[i+1]);
    }
    while(ros::ok())
    {
        //driver is navigation function
        geometry_msgs::Twist twist_msg = driver(begin, parameters);
    
        //publshing data to robot
        ROS_INFO("sending throttle=%f, steering=%f", twist_msg.linear.x, twist_msg.angular.z);
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
      linearVel.x = 13*car_msg.throttle;
      linearVel.y = 0;
      linearVel.z = 0;

      //Since the robot can turn around z-axis only,
      //we set the other two to zero
      angularVel.x = 0;
      angularVel.y = 0;
      angularVel.z = 30/*30 degrees max wheel turn*/*car_msg.steering*car_msg.throttle/30/*30 cm wheel base length*/;

      twistReturn.linear = linearVel;
      twistReturn.angular = angularVel;

      return twistReturn;
  }
