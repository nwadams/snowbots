#include <cstdlib>
#include <iostream>
#include "cv.h"
#include "highgui.h"
#include <stdio.h>
#include <std_msgs/Float64.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

using namespace std;
using namespace cv;

const string NODE_NAME = "stop_sign";
ros::Publisher ss_pub;
ros::Subscriber img_subscribe;
std_msgs::Float64 ss_msg;
const string SS_TOPIC = "stopsign";
const int MSG_QUEUE_SIZE = 20;

//99% garantee this won't work with a camera, will try finding contours next
void houghDetector(Mat img)
{
	//imshow("frame", img);
	vector<Mat> planes;
	//split into BGR planes
	split(img, planes);

	//get maximum of BG
	Mat maximum = max(planes[0], planes[1]);
	Mat red, out;

	//red - max(BG)
	out = planes[2]-maximum;
	//imshow("red", out);

	//threshold to make it a binary image
	threshold(out, red, 50, 255,THRESH_BINARY);
	//imshow("binary", red);

	vector<Vec2f> lines;
	Mat color_dst, dst;
	//canny to get lines
	Canny( red, dst,0,100,3 );
	//imshow("canny", dst);

	//convert to color for output
	cvtColor( dst, color_dst, CV_GRAY2BGR );

	vector<Vec4i> lines2;

	//houghlines to find edges of stop sign
    HoughLinesP( dst, lines2, 2.0, CV_PI/360, 60, 50, 10 );

	// a boolean for each side, starting from top going clockwise
	bool sides[8];
	for(int i = 0; i < 8; i++) {
		sides[i] = false;
	}

	// each point in the set of lines
	cout << lines2.size() << endl;
	Point average;
	average.x = 0;
	average.y = 0;
	for( size_t i = 0; i < lines2.size(); i++ )
    {
        line( color_dst, Point(lines2[i][0], lines2[i][1]),
            Point(lines2[i][2], lines2[i][3]), Scalar(0,0,255), 3, 8 );
        float slope;

		Point p1 = Point(lines2[i][0],lines2[i][1]);
		Point p2 = Point(lines2[i][2],lines2[i][3]);
		average.x += p1.x;
		average.y += p2.y;
	}

	if (lines2.size() != 0) {
		average.x /= lines2.size();
		average.y /= lines2.size();
	} else {
		ss_msg.data = 0;
		ss_pub.publish(ss_msg);
	}
    for( size_t i = 0; i < lines2.size(); i++ )
    {
        line( color_dst, Point(lines2[i][0], lines2[i][1]),
            Point(lines2[i][2], lines2[i][3]), Scalar(0,0,255), 3, 8 );
        float slope;

		Point p1 = Point(lines2[i][0],lines2[i][1]);
		Point p2 = Point(lines2[i][2],lines2[i][3]);
        if((lines2[i][2] - lines2[i][0]) != 0) {
        	slope = (float)(lines2[i][3] - lines2[i][1]) / (lines2[i][2] - lines2[i][0]);
			if(p1.x < average.x && p1.y < average.y && p2.x < average.x && p2.y < average.y) {
				sides[0] = true;
				//cout << "top left" << endl;
			} else if(p1.x < average.x && p2.x < average.x && p1.y > average.y && p2.y > average.y) {
				sides[6] = true;
				//cout << "lower left" << endl;
			} else if(p1.x > average.x && p2.x > average.x && p1.y > average.y && p2.y > average.y) {
				sides[4] = true;
				//cout << "lower right" << endl;
			} else if(p1.x > average.x && p2.x > average.x && p1.y < average.y && p2.y < average.y) {
				sides[2] = true;
				//cout << "upper right" << endl;
			} else if( p1.y > average.y && p2.y > average.y) {
				sides[5] = true;
				//cout << "lower center" << endl;
			} else if(p1.y < average.y && p2.y < average.y) {
				sides[1] = true;
				//cout << "upper center" << endl;
			} else if(p1.x < average.x && p2.x < average.x) {
				sides[7] = true;
				//cout << "left side" << endl;
			} else if(p1.x > average.x && p2.x > average.x) {
				sides[3] = true;
				//cout << "right side" << endl;
			}
        } else {
			//cout << "vertical line" << endl;
			if(p1.x < average.x && p2.x < average.x) {
				sides[7] = true;
				//cout << "left side" << endl;
			} else if(p1.x > average.x && p2.x > average.x) {
				sides[3] = true;
				//cout << "right side" << endl;
			}

        }
		
    }
    //imshow( "Detected Lines", color_dst );
	int count = 0;
	for(int i = 0; i < 8; i++) {
		if(sides[i] == true)
			count++;
	}
	cout << count << " sides of stop sign detected" << endl;

  	ss_msg.data = (double)count / (double)8;
  	ss_pub.publish(ss_msg);
	//TODO: calculate slope of lines, and distance between endpoints to determine octagonal shape
	//TODO: figure out how to identify letters
	//waitKey(0);
}

void doImage(Mat img)
{
	houghDetector(img);
}

void doVideo(VideoCapture capture)
{
	Mat frame;
	int i = 0;
	while(true)
	{
		capture >> frame;
		cout << "loop" << endl;
    	if (! frame.data ) {
    		cout << "unable to get frame" << endl;
    		return;
		}

		if(i%1 == 0)
			doImage(frame);

		if(waitKey(30) >= 0)
        	return;

		i++;
	}

}

void cvImgCallback(const sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

	Mat image = cv_ptr->image;
	
	doImage(image);
}
	
int main(int argc, char** argv)
{
	ros::init(argc, argv, NODE_NAME);
  	ros::NodeHandle node;
	ss_pub = node.advertise<std_msgs::Float64>(
      SS_TOPIC, MSG_QUEUE_SIZE
  	);
	img_subscribe = node.subscribe("cv_bridge", 1, cvImgCallback);
	if(argc ==3)
	{
		cout << "file" << endl;
		VideoCapture capture(argv[1]);
  		if ( capture.isOpened() ) 
		{
    		doVideo(capture);
    		return 0;
		}
	} else if(argc == 2)
	{
		cout << argv[1] << endl;
		Mat img = imread(argv[1]);
    	if (! img.data ) 
		{
      		cout << "unable to open image at " << argv[1] << endl;
     		return 1;
		}
		doImage(img);
		//waitKey(0);
	} else {		
		cout << "camera" << endl;
		for ( int cam_id = 5; cam_id >= 0; cam_id-- ) 
		{
      		VideoCapture capture(cam_id);
      		if ( capture.isOpened() ) 
			{
        		doVideo(capture);
        		return 0;
    		}
		}
		cout << "could not open any cameras" << endl;
		ros::spin();
    }
	return 0;
}
