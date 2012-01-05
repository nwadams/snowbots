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
#include <sb_msgs/LidarNav.h>

// blizzard driver rosrun sb_arduinoDriver bliz(tab)
// rosrun hok(tab) hok(tab)
// Function Signatures
geometry_msgs::Twist twistConvertor(sb_msgs::CarCommand car_msg);

class lidar_class
{
    double throttle;
    double steering;

    public:
    lidar_class();
    void callback(const sb_msgs::LidarNavConstPtr& msg_in);

    sb_msgs::CarCommand get_car_msg();
};

lidar_class::lidar_class()
{
    throttle = 0;
    steering = 0;
}

void lidar_class::callback(const sb_msgs::LidarNavConstPtr& msg_in)
{
    throttle = msg_in->distance;
    steering = msg_in->direction;

    
    //publisher to publish new data back to arduino
}

sb_msgs::CarCommand lidar_class::get_car_msg()
{
    sb_msgs::CarCommand car_msg;
    car_msg.throttle = throttle;
    car_msg.steering = steering;
    return car_msg;
}


using namespace std;

// Global Constants
static const int SECOND = 1000000; //1 million us

// ROS-related Constants
static const string NODE_NAME          = "magical_commander";
static const string CAR_PUBLISH_TOPIC  = "cmd_vel";
static const string IR_SUBSCRIBE_TOPIC = "base_scan";
static const string LIDAR_SUBSCRIBE_TOPIC = "lidar_nav";
static const int LOOP_FREQ             = 30; // Hz

double my_data[550];
int i = 0;

//call back for lidar directions
void ir_state_callback(const sensor_msgs::LaserScanConstPtr& msg_ptr)
{
	


}



int main( int argc, char** argv )
{
 
    //ros initialization
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle n;
    
    //subscribes to IR topic to receive data from arduino
    lidar_class my_lidar_class;

    ros::Subscriber IR_state = n.subscribe(IR_SUBSCRIBE_TOPIC, 20, ir_state_callback);
    ros::Subscriber Lidar_instructions = n.subscribe(LIDAR_SUBSCRIBE_TOPIC, 3, &lidar_class::callback,&my_lidar_class);
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

	sb_msgs::CarCommand car_msg = my_lidar_class.get_car_msg();
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
      cout<<"throttle is "<<car_msg.throttle<<endl;
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
