/*
 *	Basic Functions: open file, open webcam, timing, save image,
 *	Reminder: add additional dependency to the project property.
*/


#include <cv.h>
#include <highgui.h>
#include <time.h>
#include <stdio.h>
#include <unistd.h>
#include "flavours.h"

#define IMPORT_IMAGE "traffic_light.jpg"
#define IMAGE_OUTPUT "bin.JPG"
#define OPEN_TYPE 1 //1. to deal with raw image 0. to deal with black and white

#define TRUE 1
#define FALSE 0

int main()
{
	CvCapture* capture=0;
	IplImage* original=0;
	IplImage* result=0;
	char name[20]={0};
	char* ptr2Name=&name[0];
	clock_t lastTime; //not used
	double duration=0; //need to be phrased out
	//CvHistogram* hist=0;//else return error

	//open file/video
	printf("waiting for a file name: ");
	scanf("%s",ptr2Name);
	if(name[0]=='#')
	{
		capture=cvCaptureFromCAM(CV_CAP_DSHOW);

		if(!capture)
		{
			printf("can open cam! quitting...");
			cvReleaseCapture( &capture );
			exit(0);
		}
		original=cvRetrieveFrame(capture);
	}
	else if(name[0]=='$')
		original=cvLoadImage(IMPORT_IMAGE,OPEN_TYPE);
	else 
		original=cvLoadImage(ptr2Name,OPEN_TYPE);
	if(!original)
	{
		printf("fail to open file! quitting...");
		exit(0);
	}

	if(!capture)
	// processing an image
	{
		/*
		// Function: figure out if red light is on
		result=cvCreateImage(cvGetSize(original),8,1);
		getTrafficLight(original,result);

		cvNamedWindow("isolated!",1);
		cvShowImage("isolated!",result);
		cvSaveImage(SAVE_TRAFFIC_LIGHT,result,0);
		cvReleaseImage(&result);*/

		/*
		// Function: use generateHSVSpace function
		cvReleaseImage(&original);
		original=cvCreateImage(cvSize(360,255),8,3);
		generateHSVSpace(original);*/
		
		
		
		// Function: basic h-s histogram
		lastTime=clock();
		//process using histogram
		useHSVHistogram(original,result);

		//usleep(100000);

		// end timer
		//duration=(clock()-lastTime) / CLOCKS_PER_SEC;
		duration=(double)(clock()-lastTime)/CLOCKS_PER_SEC; // assuming 
		printf("processing time: %lf \n", duration);
		printf("clock per sec: %li \n", CLOCKS_PER_SEC);

		//show image, save image
		cvSaveImage(IMAGE_OUTPUT,original);
		cvNamedWindow("show it!",1);
		cvShowImage("show it!",original);
		cvReleaseImage(&original);

		/*
		// function: cone spotter colour isolation
		lastTime=clock();
		result=cvCreateImage(cvGetSize(original),8,1);
		//process using histogram
		useHSVHistogram_cone(original,result);

		Sleep(1000);

		// end timer
		//duration=(clock()-lastTime) / CLOCKS_PER_SEC;
		duration=(double)(clock()-lastTime)/1000; // assuming 
		printf("processing time: %lf \n", duration);
		printf("clock per sec: %i \n", CLOCKS_PER_SEC);

		//show image, save image
		cvSaveImage(IMAGE_OUTPUT,original);
		cvNamedWindow("show it!",1);
		cvShowImage("show it!",original);
		cvReleaseImage(&original);

		cvNamedWindow("show it !",1);
		cvShowImage("show it !",result);
		cvSaveImage("hist-result.JPG",result,0);
		cvReleaseImage(&result);*/
	}
	else
	// stream the webcam
	{
		cvNamedWindow("stream it!",CV_WINDOW_AUTOSIZE);
		original=cvQueryFrame(capture);
		while(original)
		{
			//do stuff here
			
			cvShowImage("stream it!",original);
			cvWaitKey(1);

			original=0;
			original=cvQueryFrame(capture);
		}
		cvReleaseImage(&original);
	}

	cvWaitKey(0);
	return 0;
}
