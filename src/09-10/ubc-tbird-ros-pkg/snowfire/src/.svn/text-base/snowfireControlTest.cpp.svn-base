/* 
Snowbots::snowfire
Node: snowfireControl
Author: Jeremy Whyte

Robot: snowfire
Team: UBC Thunderbird Robotics Snowbots Team
Crew: snowfire
Members: 
Jeremy Whyte
Edu Silva

This is the snowfireControl ROS node in the snowfire package, part of the ubc-tbird-ros-pkg stack.
snowfireControl is written in C++ on the ROS framework.
snowfireControl communicates with the furious package, using the servoCommand and furiousState topics.
The snowfireControl node publishes to the servoCommand topic and subscribes to the furiousState topic.

- servoCommand.msg: servoCommand message type
string servo
int32 value

- furiousState.msg: furiousState message type
float32[6] analog
float32[6] ir
uint32 logic_battery
uint32 motor_battery
float64 odometer
string portname
float32[] sonar

To Do:
- (imports/includes) interface with furious board and servoCommand, furiousState messages
- define namespace?
- specify constants
- write data structure/class to hold snowfire data
- process furiousState message data and update snowfire data 
- write furiousState topic callback function (part done)
- set up snowfireControl ROS node (done) (need to test/check)
- initialize publisher to servoCommand topic (done) (need to check)
- initialize subscriber to furiousState topic (done) (need to check)
- initialize sequence of servo commands
- process servo commands and prepare/publish servoCommand message
*/


//import c++ headers
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
/* !
  - import furious interface here ?
*/
#include <furious/ServoCommand.h> // ??? - import servoCommand msg interface ?
#include <furious/FuriousState.h> // ??? - import furiousState msg interface ?

/* !
  - define constants here ?
  - check string type ? 
*/
const string NODE_NAME = "snowfireControl"; // ? 
const float LOOP_RATE = 20.0; // 20 cycles per second
const string PUB_TOPIC = "servoCommand";
const string SUB_TOPIC = "furiousState";
const string STEERING_SERVO = "steering"; // ?
const string THROTTLE_SERVO = "throttle"; // ?
const float TARGET_DISTANCE = 2.0; // 2.0 meters ?

/* !?
  - set up data structure for snowfire control data
  - latest furious board data values
  - latest servo command values
  - planned path parameters
  - history of data
*/

bool snowfireStartUp = 1;
bool snowfireReceivingData = 0;
bool snowfireControlReady = 0;
float odometerReading = 0.0;
float lastOdometer = 0.0;
float intervalDistance = 0.0;
float totalDistance = 0.0;


/* !?
  - function to process furiousState msg ?
*/

/* !
  - callback function for subscribing to furiousState topic ?
  - parameter msg will be furiousState message type ?
*/
void snowfireControlCallback(const furious::FuriousState& msg) // ?
{
 //snowfireControlReady = 0; // furious data incoming, don't let driver send new commands to furious until data updated ?
  /*
    - pull data out of msg here ?
  */   
  ROS_INFO("snowfireControl: Received message from port %s", 
	msg->portname.c_str());
  
  
  odometerReading = msg->odometer; // ???
  snowfireReceivingData = 1; // getting data, tell main/driver that we can drive
  intervalDistance = odometerReading - lastOdometer;
  lastOdometer = odometerReading;
  totalDistance += intervalDistance;
  //snowfireControlReady = 1; 
  //more
}

/* !
  - function to process sequence of commands and prepare servoCommand messages ?
*/


/* !?
  - 
*/
int main(int argc, char** argv)
{
  // initialize publisher to servoCommand
  ros::init(argc, argv, NODE_NAME);
  ros::NodeHandle snowfireNodeHandle;
  ros::Publisher pub = snowfireNodeHandle.advertise<furious::ServoCommand>(PUB_TOPIC, 100); // need to specify message type?
  // initialize subscriber to furiousState
  ros::Subscriber sub = snowfireNodeHandle.subscribe(SUB_TOPIC, 100, snowfireControlCallback);
  ros::Rate loopRate(LOOP_RATE);
  
  furious::ServoCommand msg;
  string setServo = THROTTLE_SERVO;
  int setValue = 0; 
  
  while (ros::ok())
  {
    /*
      -create msg & prepare to publish
	*/
    if (snowfireStartUp) {
	  if (snowfireReceivingData) {
	    // start driving
	    setValue = 30;
	    snowfireStartUp = 0;
	  }
	  else { // !snowfireReceivingData
	    setValue = 0;
	  }
	}
	else { // !snowfireStartUp
	  if (totalDistance >= TARGET_DISTANCE) {
	    setValue = 0;
	  }
	}
	msg->servo = setServo;
	msg->value = setValue;
    pub.publish(msg);
    ROS_INFO("snowfireControl: Published message to %s servo", msg->servo.c_str());
    ros::spinOnce();
    loopRate.sleep();
  } // end while
} // end main



