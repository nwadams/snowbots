/*
 File: Blizzard.cpp
 
 Author: Nick Adams
 Last Edited: August 20, 2010
 
 Blizard robot class source file for arduino_driver
*/

#include "Blizzard.h"
#include <stdio.h> //standard c library
#include <iostream>//standard cpp library
#include <string>//controlling strings
#include "arduino_driver.h"
#include <sb_msgs/RobotState.h>
#include <sb_msgs/IMU.h>

using namespace std;

Blizzard::Blizzard()
{
    init[0] = init_identifier;
    init[1] = servo_stop;
    init[2] = servo_stop;
    init[3] = servo_stop;
    init[4] = servo_stop;
    init[5] = init_identifier;
}

string Blizzard::getInit()
{
    return init;
}

int Blizzard::getThrottle()
{
    return send_message[1];
}

int Blizzard::getSteering()
{
    return send_message[2];
}

int Blizzard::getPan()
{
    return send_message[3];
}

int Blizzard::getTilt()
{
    return send_message[4];
}

string Blizzard::setWriteData(MotorControl motor, TurretControl turret)
{
    send_message[0] = unique_identifier;
    send_message[1] = motor.throttle * 90 + 90;
    send_message[2] = -motor.steering * 60 + 90;
    send_message[3] = turret.pan * 90 + 90;
    send_message[4] = turret.tilt * 90 + 90;
    send_message[5] = unique_identifier;
    
    string message; 
    for (int i = 0; i < 6; i++)
    {
        message += send_message[i];
    }
    return message;
}

string Blizzard::seteStopData(TurretControl turret)
{
    send_message[0] = unique_identifier;
    send_message[1] = servo_stop;
    send_message[2] = servo_stop;
    send_message[3] = turret.pan * 90 + 90;
    send_message[4] = turret.tilt * 90 + 90;
    send_message[5] = unique_identifier;
    
    string message; 
    for (int i = 0; i < 6; i++)
    {
        message += send_message[i];
    }
    return message;
}

void Blizzard::getReadData(string data, sb_msgs::RobotState &robot, sb_msgs::IMU &imu)
{

}
    
char Blizzard::getUniqueIdentifier()
{
    return unique_identifier;
}
