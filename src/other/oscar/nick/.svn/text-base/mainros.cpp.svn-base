#include <cstdlib>
#include <iostream>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <stdio.h>
#include "vision.cpp"
#include "vision.h"

int main( int argc, char** argv ) { 
	CvCapture* capture;

	//capture from webcam if found, and image if a file name is passed	
	capture = cvCreateCameraCapture(CV_CAP_ANY);
	assert( capture != NULL );

	double angle;

	// Create
	IplImage *in;
	in = cvQueryFrame(capture);
	IplImage *out = cvCreateImage(
		cvGetSize(in),
		IPL_DEPTH_8U, 
		3 );	

	// Display
	cvNamedWindow("Output");

	ros::init(argc, argv, "vision");
	ros::NodeHandle n;
	ros::Publisher chatter_pub = n.advertise<std_msgs::Int32("angle", 100);
	ros::Rate loop_rate(5);

//print image to screen

    for(ros::ok()) {
        in = cvQueryFrame( capture );
	    if( !in ) break;

		angle = doPicture(in, out);

		cvShowImage("Output", out);
        char c = cvWaitKey(33);
        if( c == 27 ) break;
	
	std_msgs::Int32 msg;
	msg.data = angle;
	chatter_pub.publish(msg);
	ROS_INFO("I published [%i]", angle);
	loop_rate.sleep();
	}
// Cleanup

	cvReleaseImage( &out );
	cvReleaseCapture( &capture );
	cvDestroyAllWindows();
	

	return 0;
}
