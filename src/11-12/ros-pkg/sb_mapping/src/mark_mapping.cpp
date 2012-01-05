/**

ros mapping

using gmapping node

version 0.1

testing the communication between

gmapping via map_server


*/


#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <string>

using namespace ros;
using namespace std;

const static string NODE_NAME = "map_listener";
const static string ROS_SUBSCRIBE_TOPIC = "map";
const static int LOOP_FREQ = 30;

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	ROS_INFO("I have got the map");
}

int main (int argc, char** argv)
{
	init(argc,argv,NODE_NAME);

	NodeHandle n;
	Subscriber sub = n.subscribe(ROS_SUBSCRIBE_TOPIC, 1000, mapCallback);

	ros::Rate loop_rate(LOOP_FREQ);

	while(ros::ok())
	{
		spin();

	}


}
