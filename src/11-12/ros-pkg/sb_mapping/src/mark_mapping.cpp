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
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_listener.h>

using namespace ros;
using namespace std;
using namespace tf;

const static string NODE_NAME = "map_listener";
const static string ROS_SUBSCRIBE_TOPIC = "map";
const static int LOOP_FREQ = 30;

int robot_x = 0;
int robot_y = 0;


void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	double data_num = msg->data.size();
	long count = 0;

	ROS_INFO("I have got the map");
	cout << "msg data size is " << data_num << endl;
	for(long i = 0; i < data_num; i++)
	{
		if(msg->data[i] != -1)
			count++;
		//cout << "The map data " << msg->data[i] << endl;
	}
	cout << "the number of known data is " << count << endl;


}

int main (int argc, char** argv)
{
	init(argc,argv,NODE_NAME);

	NodeHandle n;
	Subscriber sub = n.subscribe(ROS_SUBSCRIBE_TOPIC, 1000, mapCallback);
	//Subscriber sub1 = n.subscribe("slam_gmapping/entropy", 1000, mapCallback2);

	
	Rate loop_rate(LOOP_FREQ);
	TransformListener listener;


	while(ros::ok())
	{
		StampedTransform transform;
		try
		{
			listener.lookupTransform("map", "odom", ros::Time(0), transform);
		}
		catch(TransformException ex)
		{
			ROS_ERROR("%s", ex.what());
		}
		robot_x = transform.getOrigin().x();
		robot_y = transform.getOrigin().y();
		cout << "the robot is " << transform.getOrigin().x() << ", "
				<< transform.getOrigin().y() << "when new map data was fed in" << endl;
		loop_rate.sleep();
	}


}
