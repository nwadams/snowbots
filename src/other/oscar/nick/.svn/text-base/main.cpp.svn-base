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

	// Create
	IplImage *in;
	in = cvQueryFrame(capture);
	IplImage *out = cvCreateImage(
		cvGetSize(in),
		IPL_DEPTH_8U, 
		3 );	

	// Display
	cvNamedWindow("Output");

//print image to screen
	double angle;

    for(;;) {
        in = cvQueryFrame( capture );
	    if( !in ) break;

		angle = doPicture(in, out);

		cvShowImage("Output", out);
        char c = cvWaitKey(33);
        if( c == 27 ) break;

	}
// Cleanup

	cvReleaseImage( &out );
	cvReleaseCapture( &capture );
	cvDestroyAllWindows();
	

	return 0;
}
