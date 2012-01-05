/*
 File: arduino_driver.h
 
 Author: Nick Adams and Jarek Ignasmenzies
 Last Edited: December 21, 2010
 Version 1.0
 
 header file for driver node
*/
#ifndef ARDUINO_DRIVER
#define ARDUINO_DRIVER

#include <iostream>
#include <sstream>
#include <string>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include "sb_msgs/TurretCommand.h"
#include "SerialCommunication.h"

using namespace std;

//servo control struct for servo_command
struct ServoControl
{
    int throttle;
    int steering;
    int pan;
    int tilt;
};

void processData(string data, sb_msgs::RobotState &state);

/*
 car_command_callback function
 takes a Twist message and puts data into struct to send to robot to control motors
*/
void car_command_callback(const geometry_msgs::TwistConstPtr& msg_ptr);

/*
 turret_command_callback function
 takes a TurretCommand message and puts data into struct to send to robot to control turret servos
*/
void turret_command_callback(const sb_msgs::TurretCommandConstPtr& msg_ptr);

/*
 eStop_callback function
 takes a bool which is the on/off state for the wireless eStop
*/
void eStop_callback(const std_msgs::BoolConstPtr& msg_ptr);

#endif
