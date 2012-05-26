/*
**  A demo/test program for the MSCR wrapper using OpenCV GUI (highgui)
**
**   C source files for mscr. (c) 2007 Per-Erik Forssen
**   C++ wrapper (blobdetector namspace) (c) 2009 Matthew Baumann
**
**  This program is free software; you can redistribute it and/or modify
**  it under the terms of the GNU General Public License as published by
**  the Free Software Foundation; either version 2 of the License, or
**  (at your option) any later version.
**  
**  See the file COPYING for details.
**
*/


#include <stdlib.h>
#include <stdio.h>

#include <vector>

#include "cv.h"
#include "highgui.h"

#include "blobdetector.h"


#define IMAGE_WIDTH 176
#define IMAGE_HEIGHT 144

#define TB1_MAX 2000
#define TB1_SCALE 0.05
#define TB2_MAX 1000

using namespace std;

CvCapture *cam;        /* Highgui camera handle */
mscr_blob::blobdetector BD;

int hue = 270;
int hue_tol = 60;
int sat_tol = 60;
int val_tol = 60;

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

int main(int argc, char* argv[]){
	int ikey=0;
	char key;
	IplImage *im_raw, *im_lowres, *im_display;

	
	im_lowres = cvCreateImage(cvSize(IMAGE_WIDTH,IMAGE_HEIGHT),8,3);
	im_display = cvCreateImage(cvSize(IMAGE_WIDTH,IMAGE_HEIGHT),8,3);

	const char* window1 = "blobtest";
	const char* trackbar1 = "margin";
	const char* trackbar2 = "timesteps";
	const char* trackbar3 = "hue";
	const char* trackbar4 = "hue tolerance";
	const char* trackbar5 = "saturation tolerance";
	const char* trackbar6 = "value tolerance";

	cvInitSystem( argc,argv );
	
	//if a video is provided as an argument, open it, otherwise, search for a camera
	if(argc > 1){
		cam = cvCreateFileCapture(argv[1]); 
	
	}
	else
	{
		// Get an OpenCV camera handle
    	cam = cvCreateCameraCapture(0);
    }
    
    cvNamedWindow(window1, CV_WINDOW_AUTOSIZE);
	//cvSetMouseCallback(window1,callback_mouse, NULL );

	cvCreateTrackbar(trackbar1,window1,NULL,TB1_MAX,callback_trackbar1);
	cvCreateTrackbar(trackbar2,window1,NULL,TB2_MAX,callback_trackbar2);
	cvCreateTrackbar(trackbar3,window1,&hue,360,callback_trackbar3);
	cvCreateTrackbar(trackbar4,window1,&hue_tol,360,callback_trackbar4);
	cvCreateTrackbar(trackbar5,window1,&sat_tol,100,callback_trackbar5);
	cvCreateTrackbar(trackbar6,window1,&val_tol,100,callback_trackbar6);

	key=(char)cvWaitKey(200);

	while ( key !=27 ) {
    	im_raw = cvQueryFrame(cam);
    	cvResize(im_raw,im_lowres);
    	cvCopy(im_lowres,im_display);

		vector<mscr_blob::blob> bloblist;
	
		bloblist = BD.HSVFilterBlobs(BD.detectblobs(im_lowres),cvScalar((double) hue,1.0,1.0),cvScalar((double) hue_tol,(double) sat_tol/100.0,(double) val_tol/100.0));
	
		BD.drawMarkers(bloblist,im_display);
	
		cvShowImage(window1, im_display);
	
		key = (char) cvWaitKey(5); 
	}
	
	cvReleaseImage(&im_raw);
	cvReleaseImage(&im_lowres);
	cvReleaseImage(&im_display);
	
	cvDestroyAllWindows();
	cvReleaseCapture(&cam);
	return 0;
}

