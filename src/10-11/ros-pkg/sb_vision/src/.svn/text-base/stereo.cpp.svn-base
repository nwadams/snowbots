#include <cstdlib>
#include <iostream>
#include "cv.h"
#include "highgui.h"
#include <stdio.h>

int main( int argc, char** argv ) {
    cvNamedWindow( "cam 1", CV_WINDOW_AUTOSIZE ); 
	cvNamedWindow( "cam 2", CV_WINDOW_AUTOSIZE ); 
	CvCapture* capture;
	CvCapture* capture2;

	//capture from webcam if found, and image if a file name is passed	
	if( argc==1 ) {
    	capture = cvCreateCameraCapture(201);
		capture2 = cvCreateCameraCapture(200);
	} else {
    	capture = cvCreateFileCapture( argv[1] );
	} assert( capture != NULL );

	//print image to screen
    IplImage* frame;
	IplImage* frame2;
    while(1) {
        frame = cvQueryFrame( capture );
		frame2 = cvQueryFrame(capture2);
        if( !frame || !frame2) break;
        cvShowImage( "cam 1", frame );
		cvShowImage("cam 2", frame2);
        char c = cvWaitKey(33);
        if( c == 27 ) break;
    }

	cvReleaseCapture( &capture );
    cvDestroyAllWindows();
	return 0;
}
