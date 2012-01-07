#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sb_msgs/LidarNav.h>
#include <math.h>

static const double pi = 3.14159;
static const double MINIMUM_DISTANCE = 0.50;
static const double MED_SPEED_DISTANCE = 2.00;
static const double FAST_SPEED_DISTANCE = 3.00;
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
		distance[j] = msg_ptr->ranges[j];
		if (distance[j] > max_distance)
			distance[j] = max_distance;
		
		if (distance[j] <= 0.050)
			distance[j] = max_distance;
		
		if (distance[j] < min_distance)
			distance[j] = min_distance;
	}

}
	
int main(int argc, char** argv)
{
    ros::init(argc, argv, "oscar_lidar");
    ros::NodeHandle n;
	ros::Subscriber LidarScan = n.subscribe("base_scan", 1, LidarScan_callback);
	ros::Publisher lidar_nav = n.advertise<sb_msgs::LidarNav>("lidar_nav",1);
	
	ros::Rate loop_rate(10);
	
	ROS_INFO("Lidar Nav ready");
	

	sb_msgs::LidarNav msg;
	
	while (ros::ok())
	{
		int state = 1;
		
		for (int i = 0; d_angle*180/pi * i < 4.0; i++)
		{
			if (distance[num_vals/2 + i] < MINIMUM_DISTANCE || distance[num_vals/2 - i] < MINIMUM_DISTANCE)
			{
				state = -1; //aah go backwards
				break;
			}
		}
		if (state != -1)
		{	
			for (int i = 0; d_angle*180/pi * i < 10.0; i++)
			{	
			 	if (distance[num_vals/2 + i] < MED_SPEED_DISTANCE || distance[num_vals/2 - i] < MED_SPEED_DISTANCE)
				{
					state = 1; //normal
					break;
				}
					state = 2; //go med
			}
			if (state > 1)
			{
				for (int i = 0; d_angle*180/pi * i < 13.0; i++)
				{	
				 	if (distance[num_vals/2 + i] < FAST_SPEED_DISTANCE || distance[num_vals/2 - i] < FAST_SPEED_DISTANCE)
					{
						state = 2; //go med
						break;
					}
						state = 3; //go fast
				}
			}
		}
			
		float angle = 0, range = 0;

		//int confidence; 
		//do navigation algorithm	
		if (state > 0)
		{

			float distance_r_last = distance[num_vals/2], distance_l_last = distance[num_vals/2];
			double moment_x = 0.0, moment_y = 0.0, mass = 0.0;
		
			for (int i = 0; d_angle*180/pi * i < 80.0; i++)
			{
				if (distance[num_vals/2 + i] > distance_l_last)
					distance[num_vals/2 + i] = distance_l_last;
					
				moment_y +=distance[num_vals/2 + i]*cos(d_angle*i) * 0.5;
				moment_x +=distance[num_vals/2 + i]*sin(d_angle*i);
				mass+=distance[num_vals/2 + i];
				
				if (distance[num_vals/2 - i] > distance_r_last)
					distance[num_vals/2 - i] = distance_r_last;
					
				moment_y +=distance[num_vals/2 - i]*cos(d_angle*i)* 0.5;
				moment_x -=distance[num_vals/2 - i]*sin(d_angle*i);
				mass+=distance[num_vals/2 + i];	
				
				distance_r_last = distance[num_vals/2 - i];
				distance_l_last = distance[num_vals/2 + i];
			}
			
			if (mass)
			{		
				angle = atan((moment_x / mass) / (moment_y / mass) ) * 180/pi;
				range = sqrt(moment_x/mass * moment_x/mass + moment_y/mass * moment_y/mass)* 100;
			} else {
			
				state = 0; //wtf
			}
				
		}

		msg.direction = angle; 
		msg.distance = range;
		msg.confidence = state;
		
		lidar_nav.publish(msg);
		ROS_INFO("direction = %d, distance = %d, confidence = %d", msg.direction, msg.distance, msg.confidence);
		
	    ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;	
}
