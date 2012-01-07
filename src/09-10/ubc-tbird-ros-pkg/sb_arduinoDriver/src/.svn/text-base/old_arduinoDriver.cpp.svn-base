#include <string>
#include <iostream>
#include "serialCommunication.h"
#include <sstream>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <stdio.h>
#include <cstring>
#include <stdlib.h>
#include <sb_msgs/RobotState.h>
#include <sb_msgs/CarCommand.h>
#include <sb_msgs/TurretCommand.h>
#include <sb_msgs/ServoCommand.h>

using namespace std;


const string NODE_NAME = "arduinoDriver";
#define STARTCHAR 'B'
#define ENDCHAR 'G'

//Global variables
float pan = 0.0;
float tilt = 0.0;
float throttle = 0.0;
float steer = 0.0;
bool eStop = 0;

//parses data into an array, start character and end character tossed
//data all ints, can be taken from array to do useful things
int parseData(string input, int data[])
{
	istringstream linestream(input);
	string temp;
	int i;
	getline(linestream, temp, ',');
	if (*temp.c_str() == STARTCHAR)
	{
		for ( i = 0; getline(linestream,temp,',') && *temp.c_str() != ENDCHAR; i++) 
		{
			//convert string to int
			data[i] = atoi(temp.c_str());
			
		}
		//dont want terminating character
		return (i-1);
	} else {
		//error occured
		return -2;
	}
}

void eStop_callback(const std_msgs::BoolConstPtr& msg_ptr)
{
	//eStop call back
	eStop = msg_ptr->data;
}

void servo_command_callback(const sb_msgs::ServoCommandConstPtr& msg_ptr)
{
	//callback for servo command.
	//to control servos directly (for calibration)
}

void car_command_callback(const sb_msgs::CarCommandConstPtr& msg_ptr)
{
	throttle = msg_ptr->throttle;
	steer = msg_ptr->steering;
        ROS_INFO("I Recieved: Throttle: %f, Steering: %f", throttle, steer);
        std::cout << "I got HERE!" << std::endl;
}


void turret_command_callback(const sb_msgs::TurretCommandConstPtr& msg_ptr)
{
	//callback for turret command
	//to control pan/tilt unit
	pan = msg_ptr->pan;
	tilt = msg_ptr->tilt;
}


int main (int argc, char** argv)
{
	serialCommunication fd(115200, "/dev/ttyUSB0");
	
	ros::init(argc, argv, "arduinoDriver");
	ros::NodeHandle n;
	
	ros::Subscriber Car_Command = n.subscribe("car_command", 1, car_command_callback);
	ros::Subscriber Turret_Command = n.subscribe("TurretCommand", 1, turret_command_callback);
	ros::Subscriber Servo_Command = n.subscribe("ServoCommand", 1, servo_command_callback);
	ros::Subscriber eStop_Command = n.subscribe("eStop", 1, eStop_callback);
	
	ros::Publisher Robot_State = n.advertise<sb_msgs::RobotState>("RobotState",1);
	
	ros::Rate loop_rate(20);
	
	ROS_INFO("arduinoDriver ready");

	usleep(1000000);
	
	//set init stuff
	stringstream init;
	init << "90" << "90";
	string output = init.str();
	fd.writeData(output, output.length());
	
	sb_msgs::RobotState msg;

	while(ros::ok())
	{
		stringstream data;
		string input;
		int send_throttle, send_steer, send_pan, send_tilt;
		int eStop_count = 0;
		
		if(!eStop)
		{
			//normal operation
			eStop_count = 0;
			send_throttle =  90; //throttle*90 + 90;
			send_steer = 90; //-steer*60 + 90;
			send_pan = pan*90 + 90;
			send_tilt = tilt*90+90;
		} else {
			//eStop
			if(eStop_count > 30)
			{
				send_throttle = 90;
			} else {
				send_throttle = 70;
			}
			send_steer = 90;
			send_pan = 90;
			send_tilt = 90;
			
			eStop_count++;	
		}
		
		//data << STARTCHAR << "," << (char)send_throttle; //<< ',' << (char)send_steer << ',' << (char)send_pan << ',' << (char)send_tilt << ENDCHAR;
		data << (int)send_throttle << (int)send_steer;
		output = data.str();
		//output = init.str();
		fd.writeData(output,output.length());
		
		
		input = fd.readData(100);
		fd.clearBuffer();
		
		int inData[30];
		int n = parseData(input,inData);
	
		//aaahhh bad data
		if (n  >= 16)
		{
			
			//populate message data
			//msg.analog[0] = inData[0];
		
			Robot_State.publish(msg);
			ROS_INFO("I published RobotState");
		}
		
		loop_rate.sleep();
	}
	
	ROS_INFO("Shutting down %s", NODE_NAME.c_str());
	output = init.str();
	fd.writeData(output,output.length());
	
	return 0;	
}
