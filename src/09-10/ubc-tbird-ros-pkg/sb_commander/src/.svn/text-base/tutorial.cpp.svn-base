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

#include <ros/ros.h>
#include <iostream>
#include <stdlib.h>
#include <string>
#include <math.h>
#include <sb_msgs/RobotState.h>
#include <sb_msgs/CarCommand.h>

using namespace std;

//global constants for ros node name, topics and loop rate
static const string CAR_PUBLISH_TOPIC = "car_command";
static const string IR_SUBSCRIBE_TOPIC = "ir_state";
static const int LOOP_FREQ = 30; // Hz
static const string NODE_NAME = "tutorial_commander";
static const int SECOND = 1000000; //1 million us

//IR values, approximately at -45, -22.5, 0, 22.5 and 45 respectively
class IR
{
    //distance from vehicle in cm
public:
    int W;
    int NW;
    int N;
    int NE;
    int E;
};

//global variable for IR data
IR state;

//call back for new IR data
void ir_state_callback(const sb_msgs::RobotStateConstPtr& msg_ptr)
{
	    
	state.W = msg_ptr->ir[0];
    state.NW = msg_ptr->ir[1];
    state.N = msg_ptr->ir[2];
    state.NE = msg_ptr->ir[3];
    state.E = msg_ptr->ir[4];
	
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
sb_msgs::CarCommand driver(ros::Time begin)
{
    sb_msgs::CarCommand car_msg;
	ros::Time now = ros::Time::now();
    int throttle, steering ;
    // Begin writing here
    if(now - begin < 3)
	{
		throttle = 0.5;
    } else {
		throttle = 0;
	}
	steering = 0;
    // End writing
    
    //uncomment the two lines below and populate car_msg with data 
    car_msg.throttle = throttle; //**some value**
    car_msg.steering = steering; //**some value**    

    return car_msg;
}

/////////////////////////////////////////////////////////////////////////////////////////

int main( int argc, char** argv )
{
    //ros initialization
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle n;
    
    //subscribes to IR topic to receive data from arduino
    ros::Subscriber IR_state = n.subscribe(IR_SUBSCRIBE_TOPIC, 1, ir_state_callback);
    
    //publisher to publish new data back to arduino
    ros::Publisher car_pub = n.advertise<sb_msgs::CarCommand>(CAR_PUBLISH_TOPIC, 1);

    //controls how fast it goes
    ros::Rate loop_rate(LOOP_FREQ);

    ROS_INFO("ready to go");
    usleep(3*SECOND);
       
    ROS_INFO("going");   
    
    //set start time
    ros::Time begin = ros::Time::now();
     
    while(ros::ok())
    {
        //driver is navigation function
        sb_msgs::CarCommand car_msg = driver(begin);
    
        //publshing data to robot
        ROS_INFO("sending steering=%f, throttle=%f", car_msg.steering, car_msg.throttle);
        car_pub.publish(car_msg);
        
        //checking callbacks and sleeping to match loop rate
        ros::spinOnce();
        loop_rate.sleep();	
    }
    ROS_INFO("shutting down node");
  
    return 0;
}
