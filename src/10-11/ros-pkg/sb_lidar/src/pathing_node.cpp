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
#include "cv.h"
#include "highgui.h"
#include <vector>

using namespace std;
using namespace cv;

//global strings for ros node and topic names
static const string NODE_NAME = "pathing_node";
static const string LIDAR_SUBSCRIBE_TOPIC = "base_scan";
static const string LIDAR_PUBLISH_TOPIC = "lidar_nav";
ros::Publisher lidar_nav;

static const bool upside_down = true;
Mat image;

double getAngle(const sensor_msgs::LaserScanConstPtr& msg_ptr)
{
	float min_angle = msg_ptr->angle_min;
	float max_angle = msg_ptr->angle_max;
	float d_angle = msg_ptr->angle_increment;
	
	int num_vals = ((max_angle - min_angle) / d_angle);
	float max_distance = msg_ptr->range_max;
	float min_distance = msg_ptr->range_min;

	vector<float> distance = msg_ptr->ranges;

	float distance_r_last = distance[num_vals/2], distance_l_last = distance[num_vals/2];
	double moment_x = 0.0, moment_y = 0.0, mass = 0.0;

	for (int i = 0; d_angle*180/M_PI * i < 80.0; i++)
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
		return atan((moment_x / mass) / (moment_y / mass) ) * 180/M_PI * 4;
	} else {
		return 0;
	}

}

void drawCones(vector<Coordinate> points,vector<Cone>left,vector<Cone>right, float angle)
{
	image = Mat(500,500,CV_8UC3,Scalar(0,0,0,0));
	
	Point center(image.size().width/2, image.size().height/2);

	circle(image,center,10,Scalar(255,255,255,0),-1,CV_AA);
	float y2;
	if (angle >= 0) {
		angle = 90 - angle;
		y2 = sin(angle*M_PI/180)/4;
	} else {
		angle = -90 - angle;
		y2 = -sin(angle*M_PI/180)/4;
	}
	cout << angle << endl;
	float slope = tan(angle*M_PI/180);
	cout << slope << endl;
	//cout << sin(angle*M_PI/180) << endl;
	//float y2 = sin(angle*M_PI/180)/4;
	cout << y2 << endl;
	Point p2(y2/slope*500,y2*500);
	p2.x = center.x - p2.x;
	p2.y = center.y - p2.y;

	//line(image,center,p2,Scalar(255,255,255,0),5);

	for(int i = 0; i < points.size(); i++) {
		//cout << points.size()<< endl;
		Point p(points[i].x*50,points[i].y*50);
		p.x = center.x - p.x;
		p.y = center.y - p.y;
		circle(image,p,1,Scalar(255,255,255,0),-1,CV_AA);
	}

	for(int i = 0; i < left.size(); i++) {
			//cout << points.size()<< endl;
			Point p(left[i].x*50,left[i].y*50);
			p.x = center.x - p.x;
			p.y = center.y - p.y;
			circle(image,p,1,Scalar(255,0,0,0),-1,CV_AA);
		}

	for(int i = 0; i < right.size(); i++) {
			//cout << points.size()<< endl;
			Point p(right[i].x*50,right[i].y*50);
			p.x = center.x - p.x;
			p.y = center.y - p.y;
			circle(image,p,1,Scalar(0,0,255,0),-1,CV_AA);
		}
	//CV_MAT_ELEM(image, elem, center.x, center.y).x = 255;
	}

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


	data.PrintConePositions();
	
	//debug
	//data.PrintLeftCones();
	//data.PrintRightCones();

	float angle = getAngle(msg_ptr);

	/*
	 * This is Nick's experimental stuff
	 * It will make you cry
	 * 0.1 is distance in meters between cones (smaller means more cones)
	 */
	NormalizeWall normal(data.LeftWall, data.RightWall, 0.1);

	cout << angle << endl;
	VectorPath vector(normal.normalizedLeftWall,normal.normalizedRightWall);

	//debug
	//normal.PrintGraphLeftCones();
	//normal.PrintGraphRightCones();

	//vector.printGraph();

	//Do pathing
	LongestLineNav testnav(normal.normalizedLeftWall, normal.normalizedRightWall); //add in third "Object" parameter later

	//make map and save data
	sb_msgs::LidarNav msg;

	drawCones(data.points, data.LeftWall, data.RightWall, angle);

/*
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
*/
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
	msg.direction = angle;
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
    namedWindow( "image", CV_WINDOW_AUTOSIZE );

    //main loop
    while (ros::ok())
    {
        //sleep for loop rate and check callbacks for subscribed topics
        ros::spinOnce();
        loop_rate.sleep();
        if(!image.empty()){
        	imshow("image",image);
        	cvWaitKey(33);
        }
    }

    return 0;
}
