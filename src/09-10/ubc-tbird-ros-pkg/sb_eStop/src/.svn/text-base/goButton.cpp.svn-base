/*
Nick Adams
Last Edited July 15, 2010
A simple ros node for an emergency stop.
the get_eStop function can be replaced with a function that receives instructions from a wireless or bluetooth device to wirelessly stop the vehicle
*/

#include <cstdlib>
#include <iostream>
#include <stdio.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <wiimote/State.h>
#include <wiimote/TimedSwitch.h>
#include <wiimote/LEDControl.h>

using namespace std;

bool eStop = 0;

void wiimote_state_callback(const wiimote::StateConstPtr& msg_ptr)
{
	cout << (int)msg_ptr->buttons[4] << endl;
	eStop = msg_ptr->buttons[4];
}
	
int main (int argc, char** argv)
{
	
	ros::init(argc, argv, "eStop");
	ros::NodeHandle n;
	ros::Publisher wii_led_pub = n.advertise<wiimote::LEDControl>("/wiimote/leds", 1);
	ros::Publisher chatter_pub = n.advertise<std_msgs::Bool>("go_button", 1);
	ros::Subscriber wii_button = n.subscribe("/wiimote/state", 1, wiimote_state_callback);
	ros::Rate loop_rate(10);	
	

	std_msgs::Bool msg;
	
	chatter_pub.publish(msg);
	ROS_INFO("I published [%d]", eStop);
	
	ROS_INFO("e-stop ready");


	wiimote::LEDControl LED_message;
	
	while(ros::ok()) 
	{
		//LED Control stuff that is broken
		//LED_message.timed_switch_array[0] = wiimote::TimedSwitch(int switch_mode = 1);
		//int offSwitch = wiimote::TimedSwitch(switch_mode=TimedSwitch.OFF);
		//a.switch_mode = a.ON;
		//wiimote::LEDControl LEDMessage(timed_switch_array);

		if (eStop)
		{
			msg.data = 1;
			chatter_pub.publish(msg);
			ROS_INFO("I published [%d]", eStop);
			
			break;
		}
		
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
