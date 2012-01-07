#include <ros/ros.h>
#include <sb_msgs/CarCommand.h>
#include <sb_msgs/TurretCommand.h>
#include <sb_msgs/LidarNav.h>
#include <sb_msgs/VisionNav.h>
#include <sb_msgs/IMU.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>

#include <stdlib.h>
#include <string>
#include <math.h>

struct NavigationCommand
{
  double steering;
  double throttle;
};

struct IMU_data 
{
	int c_x;
	int c_y;
	int c_z;
};

using namespace std;

/* ROS-Related Constants */
static const int LOOP_FREQ = 30; // Hz
static const string NODE_NAME            = "blizzard_command";
static const string CAR_PUBLISH_TOPIC    = "cmd_vel";
static const string TURRET_PUBLISH_TOPIC = "turret_command";
static const string LIDAR_NAV_SUB_TOPIC  = "lidar_nav";//"lidar_nav";
static const string VISION_NAV_SUB_TOPIC = "vision_nav";
static const string STOP_SIGN_TOPIC      = " "; //TODO
static const string TRAFFIC_LIGHT_TOPIC  = " "; //TODO
static const int SECOND = 1000000; //1 million us


/* Global Variables */
int vision_direction;
int vision_distance;
int vision_confidence;
NavigationCommand lidarNav;
IMU_data imu;


//vision call back function
void vision_nav_callback(const sb_msgs::VisionNavConstPtr& msg_ptr)
{
	vision_direction = msg_ptr->direction;
	vision_distance = msg_ptr->distance;
	vision_confidence = msg_ptr->confidence;
}

//lidar callback function
void lidar_nav_callback(const geometry_msgs::TwistConstPtr& msg_ptr)
{
  lidarNav.steering = msg_ptr->angular.z;
  lidarNav.throttle = msg_ptr->linear.x;
  //ROS_INFO("I Recieved: throttle - %f, steering - %f", msg_ptr->linear.x,msg_ptr->angular.z);
}

//stop sign call back function

void stop_sign_callback(const std_msgs::Float64ConstPtr& msg_ptr)
{
  if (msg_ptr->data > 0.6){
    lidarNav.throttle = 0;
    lidarNav.steering = 0;
    usleep(3000000);
  }
}

//IMU callback
void IMU_callback(const sb_msgs::IMUConstPtr& msg_ptr)
{
	imu.c_x = msg_ptr->c_x;
	imu.c_y = msg_ptr->c_y;
	imu.c_z = msg_ptr->c_z;
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, NODE_NAME);
  ros::NodeHandle n;

  ros::Publisher car_pub = n.advertise<sb_msgs::CarCommand>(CAR_PUBLISH_TOPIC, 1);
  ros::Publisher turret_pub = n.advertise<sb_msgs::TurretCommand>(TURRET_PUBLISH_TOPIC, 1);

  ros::Subscriber vision_nav = n.subscribe(VISION_NAV_SUB_TOPIC, 1, vision_nav_callback);
  ros::Subscriber lidar_nav = n.subscribe(LIDAR_NAV_SUB_TOPIC, 1, lidar_nav_callback);
  
  ros::Subscriber IMU_nav = n.subscribe("IMU", 1, IMU_callback);

  ros::Rate loop_rate(LOOP_FREQ);

  usleep(3*SECOND);

  ROS_INFO("Blizzard ready");

  float throt = 0, steer = 0;
  sb_msgs::CarCommand car_msg;
  sb_msgs::TurretCommand turret_msg;
		
  while(ros::ok())
  {
    //If see stop sign -> stop
    //TODO
    //If see traffic light -> stop
    //TODO
    
    //Use lidar navigation first
    car_msg.steering = lidarNav.steering;
    car_msg.throttle = lidarNav.throttle;

    //TODO: under what condition(s) are we going to switch to vision nav?
/*
    if(!vision_confidence)
    {
      throt = 0;
      steer = 0;
    }
    else
    {	
      throt = 0.10;
			
      if (abs(vision_direction) < 40)
      {
        steer = -vision_direction/50.0;	
      }
      else
      {
        if (vision_direction > 0)
          steer = -1.0;
        else
          steer = 1.0;
      }
    }
		
    car_msg.throttle = throt;
    car_msg.steering = steer;
*/
    ROS_INFO("sending steering=%f, throttle=%f", car_msg.steering, car_msg.throttle);
    car_pub.publish(car_msg);

    ros::spinOnce();
    loop_rate.sleep();	
  }
	
  ROS_INFO("shutting down Blizzard Commander");
}
