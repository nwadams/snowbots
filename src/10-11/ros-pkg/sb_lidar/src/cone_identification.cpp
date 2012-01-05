#include <iostream>
#include <math.h>
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "cone_identification.h"

using namespace std;

//global strings for ros node and topic names
static const string NODE_NAME = "cone_identification_node";
static const string LIDAR_SUBSCRIBE_TOPIC = "base_scan";
static const string LIDAR_PUBLISH_TOPIC = "lidar_nav";

static const double pi = 3.14159;
float min_angle, max_angle, d_angle, max_distance, min_distance;
int num_vals;

//struct containing r,theta,x,y
LidarData lidar_data;

void LidarScan_callback(const sensor_msgs::LaserScanConstPtr& msg_ptr)
{
	std::cout << "callback" <<std::endl;
	min_angle = msg_ptr->angle_min;
	max_angle = msg_ptr->angle_max;
	d_angle = msg_ptr->angle_increment;
	
	num_vals = ((max_angle - min_angle) / d_angle);
	max_distance = msg_ptr->range_max;
	min_distance = msg_ptr->range_min;

	//distances in m
	//angles in radians
	for (int j = 0; j < num_vals; j++)
	{
		//put into polar format
		lidar_data.polar[j] = msg_ptr->ranges[j];
		
		//if > max set to max
		if (lidar_data.polar[j] > max_distance)
			lidar_data.polar[j] = max_distance;
		
		// if < 5cm set to max (<5cm means no data)
		if (lidar_data.polar[j] <= 0.050)
			lidar_data.polar[j] = max_distance;
		
		//if < min set to min
		if (lidar_data.polar[j] < min_distance)
			lidar_data.polar[j] = min_distance;

		//store angle
		lidar_data.theta[j] = d_angle*j-((max_angle-min_angle)/2);

		//cartesian format
		lidar_data.x[j] = lidar_data.polar[j]*cos(lidar_data.theta[j]);
		lidar_data.y[j] = lidar_data.polar[j]*sin(lidar_data.theta[j]);
	}

	int count = 0;
	double last_data;
	int cones = 0;

	for(int i = 0; i < num_vals; i++)
	{
		if(lidar_data.polar[i] > min_distance && lidar_data.polar[i] < max_distance)
		{
			if(count == 0 || fabs(lidar_data.polar[i] - last_data) < 0.1)
			{
				//cout << lidar_data.polar[i] << endl;
				count++;
				//cout << "count:" << count << endl;
				last_data = lidar_data.polar[i];
			} else {
				if(count >= 2)
				{
					cones++;
					//cout << "cones:" << cones << endl;
					double total = 0;
					int j;
					for (j = 0; j < count;j++)
					{
						total += lidar_data.polar[i-j];
					}
					double av_r = total/j;
					double av_theta = (lidar_data.theta[i]+lidar_data.theta[j]) /2;
				}
				count = 0;
			}
		}
		else if(count >= 2)
		{
			cones++;
			count = 0;
		}
	}
	cout << cones << endl;
}
	
int main(int argc, char** argv)
{
	//node init stuff
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle n;
    //publishers and subcribers
    ros::Subscriber LidarScan = n.subscribe(LIDAR_SUBSCRIBE_TOPIC, 1, LidarScan_callback);
    //ros::Publisher lidar_nav = n.advertise<sb_msgs::LidarNav>(LIDAR_PUBLISH_TOPIC,1);

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
