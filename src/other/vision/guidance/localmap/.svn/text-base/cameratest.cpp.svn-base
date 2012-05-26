
#include <stdio.h> 
#include <stdlib.h>
#include <iostream>
#include <string>

#include <cv.h>
#include <highgui.h>
#include <imageops.h>

#include "camera.h"
#include "localmap.h"

int img_width = 320;
int img_height = 240;
int horizon = 120;

int map_img_width = 320;
int map_img_height = 320;
double map_width = 2.0;
double map_height = 2.0;

int board_squares_w = 6;
int board_squares_h = 3;

int ox = 160;
int oy = 280;

double rect_width = 0.164;
double rect_height = 0.066;
double rect_range = 0.4;

int hue = 0;
int hue_tol = 30;
int sat_min = 100;

int imageindex = 0;
char namebuffer[256];


double centering = 10;
double turningradius = 0.55;

using namespace std;

int main(int argc, char* argv[]){
	
	cout << "Camera Test program init" << endl; 
	if(argc < 4){
		printf("Error: not enough arguments\n");
		printf("       Usage: cameratest <cam index> <intrinsics> <distortions> [homography]\n");
	}
	
	cout << "Instantiating camera object" << endl;
	vision::camera cam(cvSize(img_width,img_height),atoi(argv[1]),horizon,argv[2],argv[3],argc > 4 ? argv[4] : NULL);
	
	vision::localmap map(cvSize(map_img_width,map_img_height), map_width, map_height, cvPoint(ox,oy));
	
	vision::pathFinder PF(map.getOrigin(), turningradius * map.getScale(), 1.0, 7, 3, 12, centering);
	
	CvPoint2D32f* corners = map.getCalibrationRectangle(rect_width,rect_height,rect_range);
	cout << "Calibration rect location:" << endl;
	cout << "[" << corners[0].x << "," << corners[0].y << "]" << endl;
	cout << "[" << corners[1].x << "," << corners[1].y << "]" << endl;
	cout << "[" << corners[2].x << "," << corners[2].y << "]" << endl;
	cout << "[" << corners[3].x << "," << corners[3].y << "]" << endl;
	
	IplImage* image = cvCreateImage(cvSize(img_width,img_height),8,3);
	IplImage* image_corrected = cvCreateImage(cvSize(img_width,img_height),8,3);
	IplImage* image_upper = cvCreateImage(cvSize(img_width,horizon-1),8,3);
	IplImage* image_lower = cvCreateImage(cvSize(img_width,img_height-horizon),8,3);
	IplImage* image_undistorted_upper = cvCreateImage(cvSize(img_width,horizon-1),8,3);
	IplImage* image_undistorted_lower = cvCreateImage(cvSize(img_width,img_height-horizon),8,3);
	//IplImage* image_birdseye = cvCreateImage(cvSize(img_width,img_width),8,3);
	
	int key = 0; 
  	cvNamedWindow("Raw Image");
  	cvNamedWindow("Undistorted");
  	//cvNamedWindow("Upper Image");
  	//cvNamedWindow("Lower Image");
  	cvNamedWindow("Upper Undistorted");
  	cvNamedWindow("Lower Undistorted");
  	cvNamedWindow("Map");
  	while(key != 27) { 

		//cout << "Capturing frame" << endl;
		if(!cam.captureFrame()){cout << "captureFrame() failed" << endl;}
		//cout << "retrieving frame" << endl;
		if(!cam.getFrame(image)){cout << "getFrame() failed" << endl;}
		//cout << "undistorting frame" << endl;
		if(!cam.getUndistortedFrame(image_corrected)){cout << "getUndistortedFrame() failed" << endl;}
		//cout << "retrieving upper frame" << endl;
		if(!cam.getUpperFrame(image_upper)){cout << "getUpperFrame() failed" << endl;}
		//cout << "retrieving lower frame" << endl;
		if(!cam.getLowerFrame(image_lower)){cout << "getLowerFrame() failed" << endl;}
		//cout << "retrieving upper undistorted frame" << endl;
		if(!cam.getUndistortedUpperFrame(image_undistorted_upper)){cout << "getUpperFrame() failed" << endl;}
		//cout << "retrieving lower undistorted frame" << endl;
		if(!cam.getUndistortedLowerFrame(image_undistorted_lower)){cout << "getLowerFrame() failed" << endl;}
		//cout << "transforming frame to birdseye" << endl;
		if(!cam.getBirdsEyeFrame(map.getVisual())){cout << "getBirdsEyeFrame() failed" << endl;}
		
		//cout << "Extracting color " << endl;
		map.selectColour(hue,hue_tol,sat_min);

		//cout << "Evaluating Steering" << endl;
		double steering = PF.evaluate(map.getOccupancy());
		cout << "Steering = " << steering << endl;
		
		//cout << "Drawing pathfinder on Map" << endl;
		PF.draw(map.getVisual(),CV_RGB(255,255,0));

		cvShowImage( "Raw Image", image );
		cvShowImage( "Undistorted", image_corrected );
		//cvShowImage( "Upper Image", image_upper );
		//cvShowImage( "Lower Image", image_lower );
		cvShowImage( "Upper Undistorted", image_undistorted_upper );
		cvShowImage( "Lower Undistorted", image_undistorted_lower );
		cvShowImage( "Map", map.getDisplayImg() );
		key = cvWaitKey(5); 
		
		switch(key){
			case ' ':
				sprintf(namebuffer,"image_raw_%d.jpg",imageindex);
				cvSaveImage(namebuffer,image);
				sprintf(namebuffer,"image_cor_%d.jpg",imageindex);
				cvSaveImage(namebuffer,image_corrected);
				imageindex++;
			break;
			case 'h':
				if(cam.computeHomography(board_squares_w,board_squares_h,image_undistorted_lower,corners)){
					cout << "Successfully computed homography matrix" << endl;
					if(!cam.saveHomography("Homography.xml")){
						cout << "Error saving homography matrix" << endl;
					}
				}
				else{
					cout << "Failed to compute homography matrix" << endl;
				}
			break;
		}
		
		
  	}
  	
  	cvReleaseImage(&image);
  	cvReleaseImage(&image_corrected);
  	cvReleaseImage(&image_upper);
  	cvReleaseImage(&image_lower);
  	cvReleaseImage(&image_undistorted_upper);
  	cvReleaseImage(&image_undistorted_lower);
  	//cvReleaseImage(&image_birdseye);
  	
  	cvDestroyWindow("Raw Image");
  	cvDestroyWindow("Undistorted");
  	//cvDestroyWindow("Upper Image");
  	//cvDestroyWindow("Lower Image");
  	cvDestroyWindow("Upper Undistorted");
  	cvDestroyWindow("Lower Undistorted");
  	cvDestroyWindow("Map");
	
	return 0;
}
