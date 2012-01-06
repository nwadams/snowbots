/**

ros mapping

using gmapping node

version 0.3

testing the communication between

gmapping via map_server


*/


#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Float64.h>
#include <string>
#include <iostream>
#include <stdlib.h>

using namespace ros;
using namespace std;

const static string NODE_NAME = "map_listener";
const static string ROS_SUBSCRIBE_TOPIC = "map";
const static int LOOP_FREQ = 30;

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	double data_num = msg->data.size();
	ROS_INFO("I have got the map");
	cout << "msg data size is " << data_num << endl;
	/*
	for(int i = 0; i < data_num; i++)
	{
		cout << "The map data " << msg->data[i] << endl;
	}
*/
}

void mapCallback2(const std_msgs::Float64::ConstPtr& msg)
{
	double data = msg->data;
	ROS_INFO("I have recieved entropy");
	cout << "entropy data is " << data << endl;

}

int main (int argc, char** argv)
{
	init(argc,argv,NODE_NAME);

	NodeHandle n;
	Subscriber sub = n.subscribe(ROS_SUBSCRIBE_TOPIC, 1000, mapCallback);
	Subscriber sub1 = n.subscribe("slam_gmapping/entropy", 1000, mapCallback2);

	//ros::Rate loop_rate(LOOP_FREQ);

	while(ros::ok())
	{
		spin();
	}


}
