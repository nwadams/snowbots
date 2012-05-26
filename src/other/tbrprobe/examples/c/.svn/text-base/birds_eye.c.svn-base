#ifdef _CH_
#pragma package <opencv>
#endif

#ifndef _EiC
#include "cv.h"
#include "highgui.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <math.h>
#include <float.h>
#include <limits.h>
#include <time.h>
#include <ctype.h>
#endif

//Call:
// birds-eye board_w board_h instrinics distortion image_file
// ADJUST VIEW HEIGHT using keys 'u' up, 'd' down. ESC to quit.
//
int main(int argc, char* argv[]) {

// INPUT PARAMETERS:
int board_w = atoi(argv[1]);
int board_h = atoi(argv[2]);
int board_n = board_w * board_h;
CvSize board_sz = cvSize( board_w, board_h );
CvMat* intrinsic;
CvMat* distortion;
CvMat intrinsicz[4];
CvMat distortionz[4];	
IplImage* image = 0;
IplImage* gray_image = 0;
IplImage* mapx = 0;
IplImage* mapy = 0;
IplImage *t = 0;
CvPoint2D32f* corners =0;
CvPoint2D32f cornersz[4];
int corner_count = 0;
int found =0;
CvPoint2D32f objPts[4], imgPts[4];
float Z = 25;
int key = 0;
IplImage *birds_image =0;
CvMat *H;
CvMat Hz[4];
CvCapture* capture = 0;

corners = &cornersz[0];
intrinsic  = &intrinsicz[0];
distortion  = &distortionz[0];
H  = &Hz[0];

if(argc < 5) {return -1;};

intrinsic = (CvMat*)cvLoad(argv[3],0,0,0);
distortion = (CvMat*)cvLoad(argv[4],0,0,0);

/*
if( (image = cvLoadImage(argv[5],0)) == 0 ) {
	printf("Trying video as this Couldn't load %s\n", argv[5]);
	return -1;
}
*/

capture = cvCreateCameraCapture( 0 );
assert( capture );
cvNamedWindow("Chessboard", 1);
cvNamedWindow("Chessboard gray", 1);

do
{
image = cvQueryFrame( capture );

gray_image = cvCreateImage( cvGetSize(image), 8, 1 );

cvCvtColor(image, gray_image, CV_BGR2GRAY );

// UNDISTORT OUR IMAGE
//
mapx = cvCreateImage( cvGetSize(image), IPL_DEPTH_32F, 1 );
mapy = cvCreateImage( cvGetSize(image), IPL_DEPTH_32F, 1 );

//This initializes rectification matrices
//
cvInitUndistortMap(
intrinsic,
distortion,
mapx,
mapy
);
t = cvCloneImage(image);


// Rectify our image
//
cvRemap( t, image, mapx, mapy,0, cvScalarAll(0));


// GET THE CHESSBOARD ON THE PLANE
//
*corners = cvPoint2D32f((board_w*1.0), (board_h*1.0));//new CvPoint2D32f[ board_n ];
corner_count = 0;

found = cvFindChessboardCorners(
	image,
	board_sz,
	corners,
	&corner_count,
	CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS
);


cvShowImage( "Chessboard", image );
cvShowImage( "Chessboard gray", gray_image);

if(!found){
		printf("Couldn't aquire chessboard on %s, only found %d of %d corners\n",
			argv[5],corner_count,board_n);
}

key = cvWaitKey(500);		//throttle display
}while(!found);



//Get Subpixel accuracy on those corners:
cvFindCornerSubPix(
	gray_image,
	corners,
	corner_count,
	cvSize(11,11),
	cvSize(-1,-1),
	cvTermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1 )
);


printf("GET THE IMAGE\n");
cvNamedWindow("Birds_Eye", 1);
//GET THE IMAGE AND OBJECT POINTS:
// We will choose chessboard object points as (r,c):
// (0,0), (board_w-1,0), (0,board_h-1), (board_w-1,board_h-1).
//
objPts[0].x = 0; 
objPts[0].y = 0;
objPts[1].x = board_w-1; 
objPts[1].y = 0;
objPts[2].x = 0; 
objPts[2].y = board_h-1;
objPts[3].x = board_w-1; 
objPts[3].y = board_h-1;
imgPts[0] = corners[0];
imgPts[1] = corners[board_w-1];
imgPts[2] = corners[(board_h-1)*board_w];
imgPts[3] = corners[(board_h-1)*board_w + board_w-1];

// DRAW THE POINTS in order: B,G,R,YELLOW
//
cvCircle( image, cvPointFrom32f(imgPts[0]), 9, CV_RGB(0,0,255), 3, 1,0);
cvCircle( image, cvPointFrom32f(imgPts[1]), 9, CV_RGB(0,255,0), 3, 1,0);
cvCircle( image, cvPointFrom32f(imgPts[2]), 9, CV_RGB(255,0,0), 3, 1,0);
cvCircle( image, cvPointFrom32f(imgPts[3]), 9, CV_RGB(255,255,0), 3, 1,0);

// DRAW THE FOUND CHESSBOARD
//
cvDrawChessboardCorners(
	image,
	board_sz,
	corners,
	corner_count,
	found
);
cvShowImage( "Chessboard", image );

// FIND THE HOMOGRAPHY
H = cvCreateMat( 3, 3, CV_32F);
cvGetPerspectiveTransform( objPts, imgPts, H);

// LET THE USER ADJUST THE Z HEIGHT OF THE VIEW
//
Z = 15;
key = 0;
birds_image = cvCloneImage(image);

// LOOP TO ALLOW USER TO PLAY WITH HEIGHT:
//
// escape key stops
//
while(key != 27) {

image = cvQueryFrame( capture );

// Set the height
//
CV_MAT_ELEM(*H,float,2,2) = Z;
// COMPUTE THE FRONTAL PARALLEL OR BIRD'S-EYE VIEW:
// USING HOMOGRAPHY TO REMAP THE VIEW
//
cvWarpPerspective(
	image,
	birds_image,
	H,
	(CV_INTER_LINEAR | CV_WARP_INVERSE_MAP | CV_WARP_FILL_OUTLIERS),
	cvScalarAll(0) //color to fill in unknown data 
);

cvShowImage( "Birds_Eye", birds_image );
	
key = cvWaitKey(500);
if(key == 'u') Z += 0.5;
if(key == 'd') Z -= 0.5;
}

cvSave("H.xml",H,0, "birds eye example", cvAttrList(0,0) ); //We can reuse H for the same camera mounting
return 0;
}

#ifdef _EiC
main(1,"birds_eye.c");
#endif
