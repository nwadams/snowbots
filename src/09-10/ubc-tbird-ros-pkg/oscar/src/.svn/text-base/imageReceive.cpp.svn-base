#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/CvBridge.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <iostream>

using namespace std;

IplImage *cv_image = NULL;
sensor_msgs::CvBridge bridge;

void imageCallback(const sensor_msgs::ImageConstPtr& msg_ptr)
{
	ROS_INFO("I received an image");

	
	try
	{
		cv_image = bridge.imgMsgToCv(msg_ptr, "bgr8");
	}
	catch (sensor_msgs::CvBridgeException error)
	{
		ROS_ERROR("error");
	}
	
	ROS_INFO("I received an image");
	

}	

int main( int argc, char** argv)
{
	ros::init(argc, argv, "imageReceive");
	ros::NodeHandle n;
	image_transport::ImageTransport it(n);
	image_transport::Subscriber image_sub;
	ros::Rate loop_rate(30);
	cvNamedWindow("Image window");
	
	while(ros::ok()) {
		//cout <<"test1" <<endl;
		image_sub = it.subscribe( "cv_bridge", 1, imageCallback);

	
		cvShowImage("Image window", cv_image);
		//esc to quit        
		char c = cvWaitKey(33);
        if( c == 27 ) break;
		
		ros::spinOnce();
		loop_rate.sleep();
		
	}
	//cvReleaseImage(&cv_image);
	cvDestroyAllWindows();
}



