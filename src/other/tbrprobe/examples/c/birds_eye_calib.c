// calib.cpp
// Calling convention:
// calib board_w board_h number_of_views
//
// Hit 'p' to pause/unpause, ESC to quit
//
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

int n_boards = 0; //Will be set by input list
const int board_dt = 10; //Wait 20 frames per chessboard view
int board_w;
int board_h;


int main(int argc, char* argv[]) {
	
int board_n = 0;
CvMat* image_points;
CvMat* object_points;
CvMat* point_counts;
CvMat* intrinsic_matrix;
CvMat* distortion_coeffs;
CvMat image_pointsz[4];
CvMat object_pointsz[4];
CvMat point_countsz[4];
CvMat intrinsic_matrixz[4];
CvMat distortion_coeffsz[4];
CvSize board_sz;
CvCapture* capture = 0;
CvPoint2D32f* corners =0;
CvPoint2D32f cornersz[4];
int corner_count;
int successes = 0;
int step, frame = 0;
IplImage *image;
IplImage *gray_image;
int found;
int i,j,c;
CvMat* object_points2;
CvMat* image_points2;
CvMat* point_counts2;
CvMat *intrinsic;
CvMat *distortion;
CvMat object_points2z[4];
CvMat image_points2z[4];
CvMat point_counts2z[4];
CvMat intrinsicz[4];
CvMat distortionz[4];
IplImage* mapx;
IplImage* mapy;
IplImage* t;


if(argc != 4){
printf("ERROR: Wrong number of input parameters\n");
return -1;
}

board_w = atoi(argv[1]);
board_h = atoi(argv[2]);
n_boards = atoi(argv[3]);


capture = cvCreateCameraCapture( 0 );
assert( capture );
board_sz = cvSize( board_w, board_h );
board_n = board_w * board_h;


cvNamedWindow( "Calibration", 1 );

printf("\n Calibration   \n");

//ALLOCATE STORAGE
corners = &cornersz[0];
image_points  = &image_pointsz[0];
object_points  = &object_pointsz[0];
point_counts  = &point_countsz[0];
intrinsic_matrix  = &intrinsic_matrixz[0];
distortion_coeffs  = &distortion_coeffsz[0];

object_points2  = &object_points2z[0];
image_points2  = &image_points2z[0];
point_counts2  = &point_counts2z[0];
intrinsic  = &intrinsicz[0];
distortion  = &distortionz[0];
	
*corners = cvPoint2D32f((board_w*1.0), (board_h*1.0));

image_points = cvCreateMat(n_boards*board_n,2,CV_32FC1);
object_points = cvCreateMat(n_boards*board_n,3,CV_32FC1);
point_counts = cvCreateMat(n_boards,1,CV_32SC1);
intrinsic_matrix = cvCreateMat(3,3,CV_32FC1);
distortion_coeffs = cvCreateMat(4,1,CV_32FC1);   //5,1 crashes the code


image = cvQueryFrame( capture );
gray_image = cvCreateImage(cvGetSize(image),8,1);//subpixel

// CAPTURE CORNER VIEWS LOOP UNTIL WE'VE GOT n_boards
// SUCCESSFUL CAPTURES (ALL CORNERS ON THE BOARD ARE FOUND)
//
while(successes < n_boards) {
//Skip every board_dt frames to allow user to move chessboard

	
if(frame++ % board_dt == 0) {

//Find chessboard corners:
found = cvFindChessboardCorners(
	image, 
	board_sz, 
	corners, 
	&corner_count,
	CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS
);
	
printf(".");
	
//Get Subpixel accuracy on those corners
cvCvtColor(image, gray_image, CV_BGR2GRAY);
	
cvFindCornerSubPix(
	gray_image, 
	corners, 
	corner_count,
	cvSize(11,11),
	cvSize(-1,-1), 
	cvTermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
	
//Draw it
cvDrawChessboardCorners(
	image, 
	board_sz, 
	corners,
	corner_count, 
	found);
	
cvShowImage( "Calibration", image );

	// If we got a good board, add it to our data
	if( corner_count == board_n ) {

		printf("\n Success: good board was seen \n");
			
			step = successes*board_n;
		
			for(i=step, j=0; j<board_n; ++i,++j ) {
				CV_MAT_ELEM(*image_points, float,i,0) = corners[j].x;
				CV_MAT_ELEM(*image_points, float,i,1) = corners[j].y;
				CV_MAT_ELEM(*object_points,float,i,0) = j/board_w;
				CV_MAT_ELEM(*object_points,float,i,1) = j%board_w;
				CV_MAT_ELEM(*object_points,float,i,2) = 0.0f;
			}
		
			CV_MAT_ELEM(*point_counts, int,successes,0) = board_n;
		successes++;
	}
	
} //end skip board_dt between chessboard capture
//Handle pause/unpause and ESC

c = cvWaitKey(15);
if(c == 'p'){
	c = 0;
	while(c != 'p' && c != 27){
	c = cvWaitKey(250);
	}
}
if(c == 27){return 0; }

image = cvQueryFrame( capture ); //Get next image

} //END COLLECTION WHILE LOOP.



//ALLOCATE MATRICES ACCORDING TO HOW MANY CHESSBOARDS FOUND
object_points2 = cvCreateMat(successes*board_n,3,CV_32FC1);
image_points2 = cvCreateMat(successes*board_n,2,CV_32FC1);
point_counts2 = cvCreateMat(successes,1,CV_32SC1);

//TRANSFER THE POINTS INTO THE CORRECT SIZE MATRICES
//Below, we write out the details in the next two loops. We could
//instead have written:
//image_points->rows = object_points->rows = \
//successes*board_n; point_counts->rows = successes;
//

for(i = 0; i<successes*board_n; ++i) {
	CV_MAT_ELEM( *image_points2, float, i, 0) = CV_MAT_ELEM( *image_points, float, i, 0);
	CV_MAT_ELEM( *image_points2, float,i,1) = CV_MAT_ELEM( *image_points, float, i, 1);
	CV_MAT_ELEM(*object_points2, float, i, 0) = CV_MAT_ELEM( *object_points, float, i, 0) ;
	CV_MAT_ELEM( *object_points2, float, i, 1) = CV_MAT_ELEM( *object_points, float, i, 1) ;
	CV_MAT_ELEM( *object_points2, float, i, 2) = CV_MAT_ELEM( *object_points, float, i, 2) ;
}

for(i=0; i<successes; ++i){ //These are all the same number
	CV_MAT_ELEM( *point_counts2, int, i, 0) = CV_MAT_ELEM( *point_counts, int, i, 0);
}


cvReleaseMat(&object_points);
cvReleaseMat(&image_points);
cvReleaseMat(&point_counts);

// At this point we have all of the chessboard corners we need.
// Initialize the intrinsic matrix such that the two focal
// lengths have a ratio of 1.0
//
CV_MAT_ELEM( *intrinsic_matrix, float, 0, 0 ) = 1.0f;
CV_MAT_ELEM( *intrinsic_matrix, float, 1, 1 ) = 1.0f;

//CRASH MSG:  Bad argument (Distortion coefficients must be 4x1 or 1x4 floating-point matrix)

printf("\n -------------CALIBRATE (crash on bad distortion_coeffs) ---------------  \n");

//CALIBRATE THE CAMERA!
cvCalibrateCamera2(
	object_points2, image_points2,
	point_counts2, cvGetSize( image ),
	intrinsic_matrix, distortion_coeffs,
	NULL, NULL,0 //CV_CALIB_FIX_ASPECT_RATIO
);


printf("\n -------------SAVE---------------  \n");


// SAVE THE INTRINSICS AND DISTORTIONS
cvSave("Intrinsics.xml",intrinsic_matrix, 0, "birds eye calib", cvAttrList(0,0) );
cvSave("Distortion.xml",distortion_coeffs, 0, "birds eye distorted calib", cvAttrList(0,0) ); 

// EXAMPLE OF LOADING THESE MATRICES BACK IN:
intrinsic = (CvMat*)cvLoad("Intrinsics.xml",0,0,0);
distortion = (CvMat*)cvLoad("Distortion.xml",0,0,0);
// Build the undistort map that we will use for all
// subsequent frames.
//
mapx = cvCreateImage( cvGetSize(image), IPL_DEPTH_32F, 1 );
mapy = cvCreateImage( cvGetSize(image), IPL_DEPTH_32F, 1 );

cvInitUndistortMap(
	intrinsic,
	distortion,
	mapx,
	mapy
);

// Just run the camera to the screen, now showing the raw and
// the undistorted image.
//
cvNamedWindow( "Undistort",1 );

while(image) {
	t = cvCloneImage(image);
	cvShowImage( "Calibration", image ); // Show raw image
	cvRemap( t, image, mapx, mapy,0, cvScalarAll(0) ); // Undistort image
	cvReleaseImage(&t);
	cvShowImage("Undistort", image); // Show corrected image
	//Handle pause/unpause and ESC
	c = cvWaitKey(15);
	if(c == 'p') {
	c = 0;
	while(c != 'p' && c != 27) {
	c = cvWaitKey(250);
	}
	}
	if(c == 27)
	break;
	image = cvQueryFrame( capture );
}
return 0;
}

#ifdef _EiC
main(1,"birds_eye_calib.c");
#endif
