/*
 File: Robot.h
 
 Author: Nick Adams
 Last Edited: August 1, 2010
 
 Generic Robot class header for arduino_driver
*/
#ifndef ROBOT
#define ROBOT

#include <stdio.h> //standard c library
#include <iostream>//standard cpp library
#include <string>//controlling strings

using namespace std;

class Robot
{
public:
    //13 is unique robot identifier
    //1001 (9) is init
    //0110 (6) is handshake
    //0000/1111 is normal operation
    static const char unique_identifier = 13;
    static const char init_identifier = 9;
    static const char handshake_identifier = 6;
    static const char normal_start = 0;
    static const char normal_start = 15;
    static const char servo_stop = 90;
    static const char servo_ignore = 255;
    
    //robot initialization message
    static const char init[] = {unique_identifier*16 + init_identifier,
                                servo_stop,
                                servo_stop,
                                servo_stop,
                                servo_stop,
                                unique_identifier*16 + init_identifier};
    
    //robot handshake message
    static const char handshake[] = {unique_identifier*16 + handshake_identifier,
                                    servo_ignore,
                                    servo_ignore,
                                    servo_ignore,
                                    servo_ignore,
                                    unique_identifier*16 + handshake_identifier};
    
    /*
     constructor for Robot class
     initializes robot
    */
    Robot();
    
    
    
private:


};

#endif
