#include <iostream>
#include <math.h>
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sb_msgs/LidarNav.h>
#include "ConePosition.h"
#include "LongestLineNav.h"
#include "NormalizeWall.h"
#include "VectorPath.h"

using namespace std;

//global strings for ros node and topic names
static const string NODE_NAME = "pathing_node";
static const string LIDAR_SUBSCRIBE_TOPIC = "scan";
static const string LIDAR_PUBLISH_TOPIC = "lidar_nav";
ros::Publisher lidar_nav;

static const bool upside_down = true;

void LidarScan_callback(const sensor_msgs::LaserScanConstPtr& msg_ptr)
{	
	//initialize ConePosition
	ConePosition data;

	//grab constants from Lidar
	data.GetConstantsFromLidar(msg_ptr, upside_down);

	//find the cones
	data.IdentifyCones(msg_ptr->ranges);

	//sort into left and right wall
	data.separateCones();
	
	//debug
	data.PrintLeftCones();
	data.PrintRightCones();

	/*
	 * This is Nick's experimental stuff
	 * It will make you cry
	 * 0.1 is distance in meters between cones (smaller means more cones)
	 */
	NormalizeWall normal(data.LeftWall, data.RightWall, 0.1);

	
	VectorPath vector(normal.normalizedLeftWall,normal.normalizedRightWall);

	//debug
	normal.PrintGraphLeftCones();
	normal.PrintGraphRightCones();

	vector.printGraph();

	//Do pathing
	LongestLineNav testnav(normal.normalizedLeftWall, normal.normalizedRightWall); //add in third "Object" parameter later

	//make map and save data
	sb_msgs::LidarNav msg;

	Cone nav;
	if (!normal.normalizedLeftWall.empty() && !normal.normalizedRightWall.empty()) {
		Cone left = normal.normalizedLeftWall.back();
		Cone right = normal.normalizedRightWall.back();
		nav.x = left.x + right.x;
		nav.y = (left.y+right.y)/2;
	} else if(!normal.normalizedLeftWall.empty()){
		Cone left = normal.normalizedLeftWall.back();
		nav = left;
	} else if(!normal.normalizedRightWall.empty()) {
		Cone right = normal.normalizedRightWall.back();
		nav = right;
	}
/*
	Cone nav;
	if(vector.path.size() > 120)
	{
		nav = vector.path[20];
	} else if (vector.path.size() > 0){
		nav = vector.path[vector.path.size() -1];
	} else {
		nav.x = 0;
		nav.y = 0;
	}/*
	Cone nav;
	for (int i = 0; i < vector.path.size(); i++)
	{
		Cone temp = vector.path[i];
		nav.y = temp.y;
		nav.x += (temp.x * i/10);
	}
	*/
	msg.direction = nav.x;
	msg.distance = nav.y;

	lidar_nav.publish(msg);
	ROS_INFO("direction = %f, distance = %f, confidence = %d", msg.direction, msg.distance, msg.confidence);



	//publish results
	//testnav.printInfo();
	//cout << "looping" << endl;
}
	
int main(int argc, char** argv)
{
	//node init stuff
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle n;
    //publishers and subcribers
    ros::Subscriber LidarScan = n.subscribe(LIDAR_SUBSCRIBE_TOPIC, 1, LidarScan_callback);
    lidar_nav = n.advertise<sb_msgs::LidarNav>(LIDAR_PUBLISH_TOPIC,1);

    //loop rate
    ros::Rate loop_rate(10);

    ROS_INFO("Lidar Nav ready");

    //main loop
    while (ros::ok())
    {
        //sleep for loop rate and check callbacks for subscribed topics
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
