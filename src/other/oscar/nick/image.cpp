#include <cstdlib>
#include <iostream>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <stdio.h>

int main( int argc, char** argv ) {
    IplImage* img = cvLoadImage(argv[1]);
	// Display
	cvNamedWindow("Input" );
	cvShowImage("Input", img);
	cvWaitKey(0);

	// Cleanup
	cvReleaseImage( &img );
	cvDestroyWindow("Input");
}
