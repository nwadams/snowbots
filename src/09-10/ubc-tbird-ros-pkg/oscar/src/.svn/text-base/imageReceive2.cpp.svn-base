#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/CvBridge.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include "cvblob/cvblob.h"
#include <math.h>
#include <std_msgs/Bool.h>

static const unsigned int SIZE_THRESHOLD = 50;
static const int LIGHT_THRESHOLD = 100;
static const int MIN_SIZE = 20;

using namespace cvb;
bool status;
int pic_count;


bool findLight(IplImage * img, IplImage * out)
{


	cvSmooth( img, img, CV_GAUSSIAN, 9,9 );

	IplImage* red=cvCreateImage(
		cvGetSize(img),
		IPL_DEPTH_8U, 
		1) ;
	IplImage* green=cvCreateImage(
		cvGetSize(img),
		IPL_DEPTH_8U, 
		1);
	IplImage* blue=cvCreateImage(
		cvGetSize(img),
		IPL_DEPTH_8U, 
		1);
	IplImage* temp=cvCreateImage(
		cvGetSize(img),
		IPL_DEPTH_8U, 
		1);

	cvSplit(img, blue, green, red, 0);
	
	cvMax(green,blue, temp);
	cvSub(red,temp, out);
	
	cvThreshold(out, out, LIGHT_THRESHOLD, 255, CV_THRESH_BINARY);
	int number = cvCountNonZero(out);
	
//	std::cout << number << std::endl;
	
	IplImage *labelImg=cvCreateImage(cvGetSize(out), IPL_DEPTH_LABEL, 1);
	cvb::CvBlobs blobs;
	unsigned int result=cvLabel(out, labelImg, blobs);
	
	cvRenderBlobs(labelImg, blobs, img, img);

	CvMemStorage* storage = cvCreateMemStorage(0);

	CvSeq* circles = cvHoughCircles( out,
		storage,
		CV_HOUGH_GRADIENT,
		2,
		out->height/50,
		5, 50);

	int i;
	for( i = 0; i < circles->total; i++ )
	{
	//	std::cout<<circles->total <<std::endl;
		float* p = (float*)cvGetSeqElem( circles, i );
		cvCircle( img,

		cvPoint(cvRound(p[0]),cvRound(p[1])),
		3,
		CV_RGB(0,255,0),
		-1, 8, 0 );
		cvCircle( img,
		cvPoint(cvRound(p[0]),cvRound(p[1])),
		cvRound(p[2]),

		CV_RGB(255,0,0),
		3, 8, 0 );
		//std::cout<<p[0]<<p[1]<<p[2]<<std::endl;
	}


	
	for (CvBlobs::const_iterator it=blobs.begin(); it!=blobs.end(); ++it)
	{
  		std::cout << "Blob #" << it->second->label << ": Area=" << it->second->area << ", Centroid=(" << it->second->centroid.x << ", " << 
  			it->second->centroid.y << ")" <<  std::endl;
  		
  		if (it->second->area > SIZE_THRESHOLD)
  			return 1;
	}

	
	return 0;
}


class ImageConverter {

public:
  
  IplImage *cv_image;

ImageConverter(ros::NodeHandle &n) :
  n_(n), it_(n_)
{
	cv_image = NULL;

  cvNamedWindow("Image window");
  image_sub_ = it_.subscribe(
    "cv_bridge", 1, &ImageConverter::imageCallback, this);
}

~ImageConverter()
{
  cvDestroyWindow("Image window");
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg_ptr)
{


    cv_image = bridge_.imgMsgToCv(msg_ptr, "bgr8");
    IplImage* out = cvCreateImage(cvGetSize(cv_image), IPL_DEPTH_8U, 1);
    //cvSetImageROI( out,  cvRect( 0,0, out->width*2/3,out->height*2/3 ) ); 
    
    
    	if (cv_image != NULL && pic_count % 10 == 0)
    	{
    		std::cout << "test" <<std::endl;
    		status = findLight(cv_image, out);
    		cvShowImage("Image window", out);
    	}
    	pic_count++;

      cvWaitKey(33);


}

protected:

ros::NodeHandle n_;
image_transport::ImageTransport it_;
image_transport::Subscriber image_sub_;
sensor_msgs::CvBridge bridge_;
image_transport::Publisher image_pub_;


};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ros::NodeHandle n;
  ImageConverter ic(n);
  
  ros::Rate loop_rate(3);
  ros::Publisher traffic_pub = n.advertise<std_msgs::Bool>("traffic_light", 1);
  
  std_msgs::Bool traffic_status;
  traffic_status.data = 0;

  while(ros::ok())
  {
	if (status)
		std::cout << "red light detected" << std::endl;
	else
		std::cout << "red light not detected" << std::endl;
		
    traffic_status.data = status;
    traffic_pub.publish(traffic_status);
	
	ros::spinOnce();
	loop_rate.sleep();
  }	

  return 0;
}

