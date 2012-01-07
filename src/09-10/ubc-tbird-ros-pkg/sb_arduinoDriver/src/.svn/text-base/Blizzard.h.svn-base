/*
 File: Blizzard.h
 
 Author: Nick Adams
 Last Edited: August 20, 2010
 
 Blizzard Robot class header for arduino_driver
*/
#ifndef BLIZZARD
#define BLIZZARD

#include <stdio.h> //standard c library
#include <iostream>//standard cpp library
#include <string>//controlling strings
#include "arduino_driver.h"
#include <sb_msgs/RobotState.h>
#include <sb_msgs/IMU.h>

using namespace std;

class Blizzard
{
public:
    
    //robot handshake message
    /*
    static const char handshake[6] = {unique_identifier*16 + handshake_identifier,
                                    servo_ignore,
                                    servo_ignore,
                                    servo_ignore,
                                    servo_ignore,
                                    unique_identifier*16 + handshake_identifier};
    */
    
    /*
     constructor for Robot class
     initializes robot
    */
    Blizzard();
    
    string getInit();
    
    string setWriteData(MotorControl motor, TurretControl turret);
    
    string seteStopData(TurretControl turret);
    
    void getReadData(string data, sb_msgs::RobotState &robot, sb_msgs::IMU &imu);
    
    int getThrottle();
    
    int getSteering();
    
    int getPan();
    
    int getTilt();
    
    static char getUniqueIdentifier();
    
    
private:

    //13 is unique robot identifier
    //1001 (9) is init
    //0110 (6) is handshake
    //0000/1111 is normal operation
    static const char unique_identifier = 13;
    static const char init_identifier = 9;
    static const char handshake_identifier = 6;
    static const char servo_stop = 90;
    static const char servo_ignore = 255;
    
    //robot initialization message
    char init[6];
    
    char send_message[6];

};

#endif
