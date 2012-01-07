#include <cstdlib>
#include <iostream>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <stdio.h>
#include <ros/ros.h>
#include "vision.h"
#include <std_msgs/Int32.h>
#include <sb_msgs/VisionNav.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/CvBridge.h>


using namespace std;


int main( int argc, char** argv ) { 
	CvCapture* capture = NULL;

	//capture from webcam if found, and image if a file name is passed	

	capture = cvCaptureFromCAM(CV_CAP_V4L + 1);

	assert( capture != NULL );

	path a;

	// Create
	IplImage *in; 
	
	in = cvQueryFrame(capture);
	//cvSmooth(in, in, CV_GAUSSIAN, 2,2);
	
	IplImage *out_hue = cvCreateImage(cvGetSize(in),IPL_DEPTH_8U, 1);
	IplImage *out_sat = cvCreateImage(cvGetSize(in),IPL_DEPTH_8U, 1); 
	IplImage *out_val = cvCreateImage(cvGetSize(in),IPL_DEPTH_8U, 1);
				  

	
	IplImage *out =  cvCreateImage(
		cvGetSize(in),
		IPL_DEPTH_8U, 
		3 );	

	// Display
	cvNamedWindow("Output Hue");
	cvNamedWindow("Output Sat");

	ros::init(argc, argv, "vision");
	ros::NodeHandle n;
	image_transport::ImageTransport it(n);
	ros::Publisher chatter_pub = n.advertise<sb_msgs::VisionNav>("Vision_Nav", 1);
	image_transport::Publisher image_pub = it.advertise("cv_bridge",1);
	ros::Rate loop_rate(30);

	

    while(ros::ok()) {
    	//cvGrabFrame(capture);
        in = cvQueryFrame( capture );
	    if( !in ) break;
		
		//send through ros
		sensor_msgs::CvBridge image;
		//image.cvToImgMsg(in,"bgr8");
		//image_pub.publish(image);
		image_pub.publish(image.cvToImgMsg(in,"bgr8"));
		ROS_INFO("I published an image");


		//play with smoothing
		cvSmooth( in, in, CV_BLUR, 4,4);
		
		//convert BGR to HSV
		cvCvtColor(in, out, CV_BGR2HSV);



		//split image to 3 single channel images
    	cvSplit(out,out_hue,out_sat,out_val, NULL);
		a = doPicture(out_hue, out_sat);

		//display output images
		cvShowImage("Output Hue", out_hue);
		cvShowImage("Output Sat", out_sat);

		//esc to quit        
		char c = cvWaitKey(33);
        if( c == 27 ) break;
	

		sb_msgs::VisionNav msg;
		msg.confidence = a.confidence;
		msg.direction = a.angle;
		msg.distance = a.distance;
		chatter_pub.publish(msg);
		ROS_INFO("I published %d, %d, %d", msg.confidence, msg.direction, msg.distance);
		loop_rate.sleep();
	}
// Cleanup

	cvReleaseImage( &out_hue );
	cvReleaseImage( &out_sat );
	cvReleaseCapture( &capture );
	cvDestroyAllWindows();
	

	return 0;
}
