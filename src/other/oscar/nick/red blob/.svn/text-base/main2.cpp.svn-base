#include <opencv/highgui.h>
#include <opencv/cv.h>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>

int main( int argc, char** argv ) {
    IplImage* img = cvLoadImage( argv[1] );
    cvNamedWindow( "Example1", CV_WINDOW_AUTOSIZE );
    cvShowImage( "Example1", img );
	
	IplImage* dst0=cvCreateImage(
		cvGetSize(img),
		IPL_DEPTH_8U, 
		1) ;
	IplImage* dst1 =cvCreateImage(
		cvGetSize(img),
		IPL_DEPTH_8U, 
		1);
	IplImage* dst2=cvCreateImage(
		cvGetSize(img),
		IPL_DEPTH_8U, 
		1);	
	IplImage* dst3=cvCreateImage(
		cvGetSize(img),
		IPL_DEPTH_8U, 
		3);	

	cvSplit(img, dst0, dst1, dst2, 0);
cvSaveImage("greyred.jpg", dst2);	
cvSaveImage("greygreen.jpg", dst1);	
cvSaveImage("greyblue.jpg", dst0);	
	cvSetZero (dst0); 
    cvSetZero (dst1);
	
	cvMerge(dst0, dst1, dst2, 0, dst3);
    cvNamedWindow( "Example2", CV_WINDOW_AUTOSIZE );
    cvShowImage( "Example2", dst0 );
    cvNamedWindow( "Example3", CV_WINDOW_AUTOSIZE );
    cvShowImage( "Example3", dst3 );
    cvNamedWindow( "Example4", CV_WINDOW_AUTOSIZE );
    cvShowImage( "Example4", dst2 );



	cvWaitKey(0);
    cvReleaseImage( &img );
cvReleaseImage( &dst0 );
cvReleaseImage( &dst1);
cvReleaseImage( &dst2);

 cvDestroyWindow( "Example2" );
 cvDestroyWindow( "Example3" );
 cvDestroyWindow( "Example4" );
    cvDestroyWindow( "Example1" );
}

