/*
 File: arduino_driver.h
 
 Author: Nick Adams and Jarek Ignasmenzies
 Last Edited: August 20, 2010
 Version 0.9
 
 header file for driver node
*/
#ifndef ARDUINO_DRIVER
#define ARDUINO_DRIVER

#include <iostream>
#include <sstream>
#include <string>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <sb_msgs/CarCommand.h>
#include <sb_msgs/ServoCommand.h>
#include <sb_msgs/TurretCommand.h>
#include "SerialCommunication.h"

using namespace std;

//motor control struct for car_command
struct MotorControl
{
    float throttle;
    float steering;
};

//turret control struct for turret_command
struct TurretControl
{
    float pan;
    float tilt;
};

//servo control struct for servo_command
struct ServoControl
{
    int throttle;
    int steering;
    int pan;
    int tilt;
    bool usingServo;
    
    ServoControl()
    {
        usingServo = false;
    }
};
/*
 car_command_callback function
 takes a CarCommand message and puts data into struct to send to robot to control motors
*/
void car_command_callback(const sb_msgs::CarCommandConstPtr& msg_ptr);

/*
 turret_command_callback function
 takes a TurretCommand message and puts data into struct to send to robot to control turret servos
*/
void turret_command_callback(const sb_msgs::TurretCommandConstPtr& msg_ptr);

/*
 servo_command_callback function
 takes a ServoCommand message to control the motors directly
*/
void servo_command_callback(const sb_msgs::ServoCommandConstPtr& msg_ptr);

/*
 eStop_callback function
 takes a bool which is the on/off state for the wireless eStop
*/
void eStop_callback(const std_msgs::BoolConstPtr& msg_ptr);

#endif
