#include <cstdlib>
#include <iostream>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <stdio.h>

int main( int argc, char** argv ) {
    cvNamedWindow( "Example2", CV_WINDOW_AUTOSIZE ); 
	CvCapture* capture;

	//capture from webcam if found, and image if a file name is passed	
	if( argc==1 ) {
    	capture = cvCreateCameraCapture(CV_CAP_ANY);
	} else {
    	capture = cvCreateFileCapture( argv[1] );
	} assert( capture != NULL );

	//print image to screen
    IplImage* frame;
    while(1) {
        frame = cvQueryFrame( capture );
        if( !frame ) break;
        cvShowImage( "Example2", frame );
        char c = cvWaitKey(33);
        if( c == 27 ) break;
    }

	cvReleaseCapture( &capture );
    cvDestroyAllWindows();
	return 0;
}
