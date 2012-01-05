#include <iostream>
//#include <vector.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sb_msgs/LidarNav.h>
#include <math.h>

static const double pi = 3.14159;
static const double MINIMUM_DISTANCE = 0.50;
static const double MED_SPEED_DISTANCE = 2.00;
static const double FAST_SPEED_DISTANCE = 3.00;
static const double EFFECTIVE_ZERO_DISTANCE = 0.05;
float min_angle, max_angle, d_angle, max_distance, min_distance;
double distance[1000];
int num_vals;

void LidarScan_callback(const sensor_msgs::LaserScanConstPtr& msg_ptr)
{
	std::cout << "callback" <<std::endl;
	min_angle = msg_ptr->angle_min;
	max_angle = msg_ptr->angle_max;
	d_angle = msg_ptr->angle_increment;
	
	num_vals = ((max_angle - min_angle) / d_angle);
	max_distance = msg_ptr->range_max;
	min_distance = msg_ptr->range_min;
	for (int j = 1; j <= num_vals; j++)
	{
		distance[j] = (msg_ptr->ranges[j]);
		if (distance[j] > max_distance)
		{
			distance[j] = max_distance;
		}
		
		if (distance[j] <= EFFECTIVE_ZERO_DISTANCE)
		{
			distance[j] = max_distance;
		}
		
		if (distance[j] < min_distance)
		{
			distance[j] = min_distance;
		}
	}

}
	
int main(int argc, char** argv)
{
    ros::init(argc, argv, "oscar_lidar");
    ros::NodeHandle n;
	ros::Subscriber LidarScan = n.subscribe("scan", 1, LidarScan_callback); //base_scan for stage, scan for real lidar
	ros::Publisher lidar_nav = n.advertise<sb_msgs::LidarNav>("lidar_nav",1);
	
	ros::Rate loop_rate(10);
	
	ROS_INFO("Lidar Nav ready");
	

	sb_msgs::LidarNav msg;
	
	while (ros::ok())
	{
		double certainty = 0;
		double x = 0;
		double y = distance[num_vals/2];

		for (int i = 1; d_angle*180/pi * i < max_angle; i++)
		{
			x+=pow((distance[num_vals/2 + i]*sin(d_angle*i)),3);
			x-=pow((distance[num_vals/2 - i]*sin(d_angle*i)),3);
			y+=pow((distance[num_vals/2 + i]*cos(d_angle*i)),3);
			y+=pow((distance[num_vals/2 - i]*cos(d_angle*i)),3);
		}
		
		double xscale=10000;
		double yscale=1000;

		x*=xscale/(num_vals - 1);
		y*=yscale/num_vals;


		double speed=atan(y)/pi;//also try y/abs(x)
		double steer=atan(x)/pi;//also try x/y

		msg.direction = steer; 
		msg.distance = speed;
		msg.confidence = 0.8;
		
		lidar_nav.publish(msg);
		ROS_INFO(" \ndirection = %6.4E \ndistance = %6.4E", msg.direction, msg.distance, msg.confidence);
		
	    ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;	
}
