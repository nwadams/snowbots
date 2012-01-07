/*
 File: arduino_driver.cpp
 
 Author: Nick Adams and Jarek Ignasmenzies
 Last Edited: September 11, 2010
 Version 0.9
 
 driver node to communicate and control arduino
 sorts incoming sensor information and outputs correct ros messages 
*/

#include <iostream>
#include <sstream>
#include <string>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include "sb_msgs/CarCommand.h"
#include <sb_msgs/TurretCommand.h>
#include <sb_msgs/ServoCommand.h>
#include <sb_msgs/RobotState.h>
#include <sb_msgs/IMU.h>
#include "SerialCommunication.h"
#include "arduino_driver.h"

#define VEHICLE_NAME Blizzard
#include "Blizzard.h"

using namespace std;
using namespace ros;

//global constants
static const string ROS_NODE_NAME = "arduino_driver";
static const int ROS_LOOP_RATE = 20; //hz

static const int BAUD_RATE = 115200;
static const string PORT_NAME = "/dev/ttyUSB";
static const string BLUETOOTH_PORT_NAME = "/dev/rfcomm";

static const string CAR_COMMAND_TOPIC = "car_command";
static const string TURRET_COMMAND_TOPIC = "turret_command";
static const string SERVO_COMMAND_TOPIC = "servo_command";
static const string ESTOP_TOPIC = "eStop";

static const int SECOND = 1000000;
static const int SEND_BYTES = 6; 

//global variables
MotorControl motor;
TurretControl turret;
ServoControl servo;

bool eStop = false;

int main(int argc, char** argv)
{
    //initialize ros
    init(argc, argv, ROS_NODE_NAME);
	NodeHandle n;
	Rate loop_rate(ROS_LOOP_RATE);

	//initialize serial communication
	SerialCommunication link;
	for (int i = 0; ; i++)
	{
	    stringstream ss;
	    ss << i;	    
	    //link.connect(BAUD_RATE,(PORT_NAME + ss.str())) || 
	    if (link.connect(BAUD_RATE,(BLUETOOTH_PORT_NAME + ss.str())))
	    {
	        cout << "connected on port " << i << endl;
	        break;
	    }	    
	    if (i > 15)
	    {
	        cout << "unable to find a device" << endl;
	        return 0;
	    }
	}
	
	usleep(1*SECOND);
		
	//subscribers and publishers
	Subscriber car_command = n.subscribe(CAR_COMMAND_TOPIC, 1, car_command_callback);
	Subscriber turret_command = n.subscribe(TURRET_COMMAND_TOPIC, 1, turret_command_callback);
	Subscriber servo_command = n.subscribe(SERVO_COMMAND_TOPIC, 1, servo_command_callback);
	Subscriber eStop = n.subscribe(ESTOP_TOPIC, 1, eStop_callback);
	
	//handshake with arduino and initialize robot
	VEHICLE_NAME snowbot;
	link.writeData(snowbot.getInit(),SEND_BYTES);
	
	sb_msgs::RobotState state;
	sb_msgs::IMU imu;
	
	ROS_INFO("arduino_driver ready");
	
	//while ros is good
	while(ok())
	{
	    //write to arduino	 
	    if (eStop)
	    {
	        link.writeData(snowbot.seteStopData(turret), SEND_BYTES);
	    } else {  
	        if (servo.usingServo)
	        {
	            //direct servo control
				//link.writeData(getServoControl(servo), SEND_BYTES);
	            stringstream ss;
	            char temp[4] = {servo.throttle, servo.steering, servo.pan, servo.tilt};
	            ss << snowbot.getUniqueIdentifier() << temp[0] << temp[1] << temp [2] << temp[3] << snowbot.getUniqueIdentifier();
	            link.writeData(ss.str(), SEND_BYTES);
	        } else {
	            //use carCommand and turretCommand
	            link.writeData(snowbot.setWriteData(motor, turret),SEND_BYTES);
	        }
	    }
	    
	    //delay for sync
	    usleep(20000);
	    
	    //read from arduino
	    snowbot.getReadData(link.readData(), state, imu);
	    
	    //populate GUI data
	    //TODO
	    
	    //publish data
	    //Robot_State.publish(state);
	    
	    //clear buffer (MAY NOT WORK)
	    link.clearBuffer();
	    
	    //log and loop
	    ROS_INFO("%i,%i,%i,%i",snowbot.getThrottle(),snowbot.getSteering(),snowbot.getPan(),snowbot.getTilt());
	    spinOnce();
		loop_rate.sleep();
	}
	
	//killing driver
	ROS_INFO("shutting down arduino_driver");
	link.writeData(snowbot.getInit(),SEND_BYTES);
	
	return 0;
}

//car_command_callback
void car_command_callback(const sb_msgs::CarCommandConstPtr& msg_ptr)
{
    motor.throttle = msg_ptr->throttle;
    motor.steering = msg_ptr->steering;
}

//turret_command_callback
void turret_command_callback(const sb_msgs::TurretCommandConstPtr& msg_ptr)
{
    turret.pan = msg_ptr->pan;
    turret.tilt = msg_ptr->tilt;
}

//servo_command_callback
void servo_command_callback(const sb_msgs::ServoCommandConstPtr& msg_ptr)
{
    if ((servo.usingServo = msg_ptr->usingServo))
    {
        servo.throttle = msg_ptr->throttle;
        servo.steering = msg_ptr->steering;
        servo.pan = msg_ptr->pan;
        servo.tilt = msg_ptr->tilt;
    }
}

//eStop_callback
void eStop_callback(const std_msgs::BoolConstPtr& msg_ptr)
{
    eStop = msg_ptr->data;
}
