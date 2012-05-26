
#include <stdio.h> 
#include <stdlib.h>
#include <iostream>
#include <string>

#include <cv.h>
#include <highgui.h>
#include <imageops.h>
#include <ConfigFile.h>

#include "camera.h"
#include "localmap.h"

/*******************************************************************************/

using namespace std;

//openCV capture
CvCapture* capture = NULL;

//camera index
int camIndex = CV_CAP_ANY;

//captured image size
int img_width = 320;
int img_height = 240;
//dividing line between upper and lower image (measured from top down)
int horizon = 120;

//size of map in pixels
int map_img_width = 320;
int map_img_height = 320;

//size of map in scale units
double map_scale_width = 2.0;
double map_scale_height = 2.0;

//size of calibration chessboard
int board_squares_w = 6;
int board_squares_h = 3;

//position of vehicle origin on map
int ox = 160;
int oy = 280;

//thickness of the sampled curves
int thickness = 3;

//size of calibration rectangle in scale units
double rect_width = 0.164;
double rect_height = 0.066;

//horizontal distance of calibration grid center from camera in scale units
double rect_range = 0.4;

//colour to avoid 
int hue = 0;

//colour tolerance
int hue_tol = 30;

//colour saturation minimum threshold
int sat_min = 100;

int val_min = 0;

//save file index
int imageindex = 0;
char namebuffer[256];

//names of calibration, distortion and transform matrices for loading
string intrinsicsFile;
string distortionFile;
string homographyFile;

bool iFile = false;
bool dFile = false;
bool hFile = false;

//steering smoohting parameter [0,1]
double mix = 0.5;

//strength of centering penalty
double centering = 10;

double steering_gain = 1.0;
double turret_gain = 0.2;
double image_rotation_gain = 0.2;

int turret_tilt = 0;

//vehicle's turning radius in scale units
double turningradius = 0.55;
double wheelbase = 0.275;
double trackwidth = 0.32;

int displayMode = 0;

bool readConfigFile(char* filename){

	string fname(filename);
	ConfigFile config( fname );
	
	bool success = true;
	
	if(!config.readInto(camIndex,"cam_index")){cout << "   CONFIGFILE ERROR: cannot read cam_index" << endl;};
	cout << "camera index is " << camIndex << endl;
	if(!config.readInto(img_width,"img_width")){cout << "   CONFIGFILE ERROR: cannot read img_width" << endl;};
	if(!config.readInto(img_height,"img_height")){cout << "   CONFIGFILE ERROR: cannot read img_height" << endl;};
	cout << "image size is " << img_width << "x" << img_height << endl;
	if(!config.readInto(horizon,"horizon")){cout << "   CONFIGFILE ERROR: cannot read horizon" << endl;};
	
	if(! (iFile = config.readInto(intrinsicsFile,"intrinsicsFile"))){cout << "   CONFIGFILE ERROR: cannot read intrinsicsFile" << endl;};
	if(! (dFile = config.readInto(distortionFile,"distortionFile"))){cout << "   CONFIGFILE ERROR: cannot read distortionFile" << endl;};
	if(! (hFile = config.readInto(homographyFile,"homographyFile"))){cout << "   CONFIGFILE ERROR: cannot read homographyFile" << endl;};
	
	if(!config.readInto(map_img_width,"map_img_width")){cout << "   CONFIGFILE ERROR: cannot read map_img_width" << endl;};
	if(!config.readInto(map_img_height,"map_img_height")){cout << "   CONFIGFILE ERROR: cannot read map_img_height" << endl;};
	
	if(!config.readInto(map_scale_width,"map_scale_width")){cout << "   CONFIGFILE ERROR: cannot read map_scale_width" << endl;};
	if(!config.readInto(map_scale_height,"map_scale_height")){cout << "   CONFIGFILE ERROR: cannot read map_scale_height" << endl;};
	
	if(!config.readInto(board_squares_w,"board_squares_w")){cout << "   CONFIGFILE ERROR: cannot read board_squares_w" << endl;};
	if(!config.readInto(board_squares_h,"board_squares_h")){cout << "   CONFIGFILE ERROR: cannot read board_squares_h" << endl;};
	
	if(!config.readInto(ox,"anchorx")){cout << "   CONFIGFILE ERROR: cannot read anchorx" << endl;};
	if(!config.readInto(oy,"anchory")){cout << "   CONFIGFILE ERROR: cannot read anchory" << endl;};
	
	if(!config.readInto(thickness,"thickness")){cout << "   CONFIGFILE ERROR: cannot read thickness" << endl;};
	
	if(!config.readInto(rect_width,"rect_width")){cout << "   CONFIGFILE ERROR: cannot read rect_width" << endl;};
	if(!config.readInto(rect_height,"rect_height")){cout << "   CONFIGFILE ERROR: cannot read rect_height" << endl;};
	if(!config.readInto(rect_range,"rect_range")){cout << "   CONFIGFILE ERROR: cannot read rect_range" << endl;};
	
	if(!config.readInto(hue,"hue")){cout << "   CONFIGFILE ERROR: cannot read hue" << endl;};
	if(!config.readInto(hue_tol,"hue_tolerance")){cout << "   CONFIGFILE ERROR: cannot read hue_tolerance" << endl;};
	if(!config.readInto(sat_min,"min_saturation")){cout << "   CONFIGFILE ERROR: cannot read min_saturation" << endl;};
	if(!config.readInto(val_min,"min_lightness")){cout << "   CONFIGFILE ERROR: cannot read min_lightness" << endl;};
	
	if(!config.readInto(mix,"steering_mix")){cout << "   CONFIGFILE ERROR: cannot read steering_mix" << endl;};
	if(!config.readInto(centering,"steering_centering")){cout << "   CONFIGFILE ERROR: cannot read steering_centering" << endl;};
	
	if(!config.readInto(steering_gain,"steering_gain")){cout << "   CONFIGFILE ERROR: cannot read steering_gain" << endl;};
	cout << "steering gain =" << steering_gain << endl;
	if(!config.readInto(turret_gain,"turret_gain")){cout << "   CONFIGFILE ERROR: cannot read turret_gain" << endl;};
	cout << "turret gain =" << turret_gain << endl;
	if(!config.readInto(turret_tilt,"turret_tilt")){cout << "   CONFIGFILE ERROR: cannot read turret_tilt" << endl;};
	
	if(!config.readInto(image_rotation_gain,"image_rotation_gain")){cout << "   CONFIGFILE ERROR: cannot read image_rotation_gain" << endl;};
	cout << "image rotation gain =" << image_rotation_gain << endl;
	
	if(!config.readInto(displayMode,"default_displaymode")){cout << "   CONFIGFILE ERROR: cannot read default_displaymode" << endl;};
	
	if(!config.readInto(turningradius,"turningradius")){cout << "   CONFIGFILE ERROR: cannot read turningradius" << endl;};
	if(!config.readInto(wheelbase,"wheelbase")){cout << "   CONFIGFILE ERROR: cannot read wheelbase" << endl;};
	if(!config.readInto(trackwidth,"trackwidth")){cout << "   CONFIGFILE ERROR: cannot read trackwidth" << endl;};
	
	return success;	
}


int main(int argc, char* argv[]){
	
	cout << "Camera Test program init" << endl; 
	if(argc > 1){
		cout << "reading config file: " << argv[1] << endl;
		if(!readConfigFile(argv[1])){
			cout << "Error in config file, using default values for unread parameters." << endl;
		}
	}
	if(argc > 2){
		cout << "Capturing video from file " << argv[2] << endl; 
		capture = cvCreateFileCapture(argv[2]);
	}
	else
	{
		cout << "Capturing from camera on bus " << camIndex << endl;
		capture = cvCaptureFromCAM(camIndex);
	}
	
	cout << "Instantiating camera object" << endl;
	vision::camera cam(capture,cvSize(img_width,img_height),camIndex,horizon,iFile?intrinsicsFile.c_str():NULL,dFile?distortionFile.c_str():NULL,hFile?homographyFile.c_str():NULL);
	
	vision::localmap map(cvSize(map_img_width,map_img_height), map_scale_width, map_scale_height, cvPoint(ox,oy));
	map.setDisplayMode(displayMode);
	
	
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

		map.clear();

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
			case 'd':
				if(displayMode < 2){
					displayMode++;
					
				}
				else
				{
					displayMode = 0;
				}
				
				map.setDisplayMode(displayMode);
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
