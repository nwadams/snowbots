/**
 * @file blobtest.cpp
 * @author Matthew Baumann
 * @brief This file is the main program loops for and example blob-based
 * visual robot guidance program.  It capturres and image, detets blobs
 * of a particular colour, and determiens what path avoids as many of the
 * blobs as possible.
 */


#include <stdlib.h>
#include <stdio.h>

#include <vector>

#include "cv.h"
#include "highgui.h"

#include "blobdetector.h"
#include <tbrclient.h>
#include "pid.h"


//#define IMAGE_WIDTH 320
//#define IMAGE_HEIGHT 240

//#define MAP_WIDTH 320
//#define MAP_LENGTH 480

#define IMAGE_WIDTH 240
#define IMAGE_HEIGHT 180

#define MAP_WIDTH 240
#define MAP_LENGTH 320

#define TB1_MAX 2000
#define TB1_SCALE 0.05
#define TB2_MAX 1000

using namespace std;
using namespace tbrprobe07;


CvCapture *cam;        					// Highgui camera handle

mscr_blob::blobdetector BD;			//Blob detection apparatus 
//(images in, lists of blobs out + manipulation & filtering tools)	

CvPoint anchor = cvPoint(MAP_WIDTH / 2,MAP_LENGTH);  //position of robot within map image

mscr_blob::pathFinder PF(11,anchor,30.0,1.0);				//pathfinder apparatus
//blob lists in map space in, steering values  out.

tbrclient TC;			//communications unit sends UDP messages to tbrprobe

//colour filter params
int hue = 300;
int hue_tol = 40;
int sat_tol = 50;
int val_tol = 60;

//vehicle control values
double steering = 0.0;
double ds = 0.05;

double throttle = 0.0; 

//---------------------------------------------------------

/*
 *  Callback functions
 */
void callback_trackbar1(int cval) {
  BD.setMargin((double)cval/TB1_MAX*TB1_SCALE);
  printf("min_margin = %g\n",BD.getMargin());
};

void callback_trackbar2(int cval) {
  BD.setTimestep(cval);
  printf("timesteps = %d\n",BD.getTimestep());
};

void callback_trackbar3(int cval) {
  //hue = cval;
  printf("hue = %d\n",hue);
};

void callback_trackbar4(int cval) {
  //hue_tol = cval;
  printf("hue_tol = %d\n",hue_tol);
};

void callback_trackbar5(int cval) {
  //sat_tol = cval;
  printf("sat_tol = %d\n",sat_tol);
};

void callback_trackbar6(int cval) {
  //val_tol = cval;
  printf("val_tol = %d\n",val_tol);
};

void processKey(char key)

{

	switch(key)

	{

		case 27:

			//esc key

		break;

		case '-':
			throttle -= 4.0;
		break;
		case '=':
			throttle += 4.0;
		break;
		case ' ':
			throttle = 0;
		break;

	}
}

int main(int argc, char* argv[]){
	char key;
	IplImage *im_raw, *im_lowres, *im_display, *im_map;

	
	im_lowres = cvCreateImage(cvSize(IMAGE_WIDTH,IMAGE_HEIGHT),8,3);
	im_display = cvCreateImage(cvSize(IMAGE_WIDTH,IMAGE_HEIGHT),8,3);
	
	im_map = cvCreateImage(cvSize(MAP_WIDTH,MAP_LENGTH),8,3);

	const char* window1 = "camera";
	const char* window2 = "map";
	//const char* window3 = "control";
	const char* trackbar1 = "margin";
	const char* trackbar2 = "timesteps";
	const char* trackbar3 = "hue";
	const char* trackbar4 = "hue tol";
	const char* trackbar5 = "sat tol";
	const char* trackbar6 = "val tol";

	cvInitSystem( argc,argv );
	
	//if a video is provided as an argument, open it, otherwise, search for a camera
	if(argc > 2){
		cam = cvCreateFileCapture(argv[2]); 
	
	}
	else if(argc > 1){
		cam = cvCreateCameraCapture(atoi(argv[1]));
	}
	else
	{
		// Get an OpenCV camera handle
    	cam = cvCreateCameraCapture(0);
    }
    
    cvNamedWindow(window1, CV_WINDOW_AUTOSIZE);
    cvNamedWindow(window2, CV_WINDOW_AUTOSIZE);
    //cvNamedWindow(window3, CV_WINDOW_AUTOSIZE);
	//cvSetMouseCallback(window1,callback_mouse, NULL );

	cvCreateTrackbar(trackbar1,window1,NULL,TB1_MAX,callback_trackbar1);
	cvCreateTrackbar(trackbar2,window1,NULL,TB2_MAX,callback_trackbar2);
	cvCreateTrackbar(trackbar3,window1,&hue,360,callback_trackbar3);
	cvCreateTrackbar(trackbar4,window1,&hue_tol,360,callback_trackbar4);
	cvCreateTrackbar(trackbar5,window1,&sat_tol,100,callback_trackbar5);
	cvCreateTrackbar(trackbar6,window1,&val_tol,100,callback_trackbar6);

	// tbrclient-interface stuff

	// setObserver allows for debug statements

  	TC.setObserver(&std::cout);

	// initialize() makes TC (a tbrclient object) talk to the tbrprobe process

	// listening on UDP_PORT

  	TC.initialize(UDP_PORT,LOCALHOST_IP);

	key=(char)cvWaitKey(200);
	
	//create bounding box on map
	CvPoint tl = cvPoint(0,0);
	CvPoint tr = cvPoint(im_lowres->width-1,0);
	CvPoint bl = cvPoint(0,im_lowres->height-1);
	CvPoint br = cvPoint(im_lowres->width-1,im_lowres->height-1);
	
	CvPoint tl_warped = BD.warp(tl,cvSize(im_lowres->width,im_lowres->height),cvSize(im_map->width,im_map->height));
	CvPoint tr_warped = BD.warp(tr,cvSize(im_lowres->width,im_lowres->height),cvSize(im_map->width,im_map->height));
	CvPoint bl_warped = BD.warp(bl,cvSize(im_lowres->width,im_lowres->height),cvSize(im_map->width,im_map->height));
	CvPoint br_warped = BD.warp(br,cvSize(im_lowres->width,im_lowres->height),cvSize(im_map->width,im_map->height));
	
	
	CvPoint anchor = cvPoint(IMAGE_WIDTH/2,IMAGE_HEIGHT);
	
	while ( key !=27 ) {
    	im_raw = cvQueryFrame(cam);
    	cvResize(im_raw,im_lowres);
    	cvCopy(im_lowres,im_display);
    	
    	cvZero(im_map);

		//create lists of blobs to be filled
		vector<mscr_blob::blob> bloblist;
		vector<mscr_blob::blob> map_bloblist;
	
	  //remove all blobs that are not the desired colour
		bloblist = BD.HSVFilterBlobs(BD.detectblobs(im_lowres),
					cvScalar((double) hue,1.0,1.0),
					cvScalar((double) hue_tol,
					(double) sat_tol/100.0,(double) val_tol/100.0));
	
		//create the visualisation of the blobs in the camera window
		BD.drawMarkers(bloblist,im_display);
	
		//create a distorted version of the blobs - trnasforming them (approximately) to the ground plane
		map_bloblist = BD.UnwarpBlobs(bloblist,cvSize(IMAGE_WIDTH,IMAGE_HEIGHT),cvSize(MAP_WIDTH,MAP_LENGTH));
		
		//draw the visualisation of the blobs in the ground plane map
		BD.drawMarkers(map_bloblist,im_map);
		
		//visualise th guidance sampling 
		PF.drawCurves(im_map);
		PF.drawPath(im_map);
		
		//determine the best steering path
		double s = PF.evaluate(map_bloblist);
		
		//send the messages to tbrpobe
		TC.setSteering((int) (-s*100.0));
		TC.setThrottle(throttle);
		
		//draw bounding box on map
		cvLine(im_map,tl_warped,tr_warped,CV_RGB(0,255,255),1);
		cvLine(im_map,tr_warped,br_warped,CV_RGB(0,255,255),1);
		cvLine(im_map,br_warped,bl_warped,CV_RGB(0,255,255),1);
		cvLine(im_map,bl_warped,tl_warped,CV_RGB(0,255,255),1);
	
		cvShowImage(window1, im_display);
		cvShowImage(window2, im_map);
	
		key = (char) cvWaitKey(5);
		processKey(key); 
	}
	
	cvReleaseImage(&im_raw);
	cvReleaseImage(&im_lowres);
	cvReleaseImage(&im_display);
	
	cvDestroyAllWindows();
	cvReleaseCapture(&cam);
	return 0;
}

