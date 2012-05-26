//-----------------------------------------------------------------------------
#include <iostream>
#include <string>

#include <cv.h>
#include <highgui.h>

#include <imageops.h>
#include <tbrclient.h>
#include <ConfigFile.h>
#include <pidthrottle.h>

#include "pathfinder.h"
#include "map.h"

//-----------------------------------------------------------------------------

//#define CLAMP(val,min,max)	(MAX(MIN(val,max),min))

//-----------------------------------------------------------------------------

using namespace std;

//Camera Capture opject pointer
CvCapture *capture = 0;

//camera capture index
int cameraIndex = CV_CAP_ANY;

//image size: controls size of processed image
int img_width = 320;
int img_height = 240;

int horizon = 120;

int map_width = 320;
int map_height = 320;

int gap = 100;

//position of image and curves on map
int foot = 0;
int trap_width = map_width / 3;
int anchorx = map_width/2;
int anchory = map_height;

//colour extraction parameters
int hue = 15;
int hue_tolerance = 10;
int min_saturation = 190;
int min_lightness = 100;
double max_val = 100;

double radius = 50;

double mix = 0.8;

double kp = 0.1;
double ki = 0.01;
double kd = 0.01;
double maxreverse = 100.0;
double maxforward = 100.0;

//program state variables
bool running = true;
int displayMode = 0;
double factor = 0.5;

int lastSteering = 0;



double steering_gain = 1.0;
double centering = 1.0;

int throttle = 0;
double targetspeed = 0;

//mode 0 = manual throttle
//mode 1 = pid throttle
int throttleMode = 0;

int turretpan = 0;
int turrettilt = 0;
double turret_gain = 1.0;
double image_rotation_gain = 0.1;

//tbrprobe UDP communication
tbrprobe07::tbrclient TC;
//vision::PIDthrottle PT;

vision::map M(map_width,map_height,radius,9,1.0);



//=============================================================================

int interpolate(int x, int x0, int x1, int y0, int y1)
{
	return y0 + (x - x0) * ((float) (y1-y0) / (float) (x1-x0));
}

//-----------------------------------------------------------------------------


///called at each cycle to check for key presses.
void processKey(char key)
{
	switch(key)
	{
		case 27:
			running = false;
		break;
		case 'd':
			displayMode++;
			if(displayMode > 3) {displayMode = 0;}
		break;
		case '-':
			if(throttleMode == 0)
			{
			
				throttle -= 4;
			}
			else{
				targetspeed -= 0.1;
				//PT.setTargetSpeed(targetspeed);
			}
		break;
		
		case '=':
			
			if(throttleMode == 0){
				throttle += 4;
			}
			else
			{
				targetspeed += 0.1;
				//PT.setTargetSpeed(targetspeed);
			}
		break;
		
		case ' ':
			if(throttleMode == 0)
			{
			
				throttle = 0;
			}
			else
			{

				targetspeed = 0;
				//PT.setTargetSpeed(targetspeed);
			}
		break;

		
	}
}

bool readConfigFile(char* filename){

	string fname(filename);
	ConfigFile config( fname );
	
	bool success = true;
	
	success = success && config.readInto(cameraIndex,"cam_index");
	cout << "camera index is " << cameraIndex << endl;
	success = success && config.readInto(img_width,"img_width");
	success = success && config.readInto(img_height,"img_height");
	cout << "image size is " << img_width << "x" << img_height << endl;
	success = success && config.readInto(map_width,"map_width");
	success = success && config.readInto(map_height,"map_height");
	
	success = success && config.readInto(gap,"gap");
	M.setGap(gap);
	success = success && config.readInto(horizon,"horizon");
	
	success = success && config.readInto(foot,"foot");
	success = success && config.readInto(trap_width,"trap_width");
	success = success && config.readInto(anchorx,"anchorx");
	success = success && config.readInto(anchory,"anchory");
	
	success = success && config.readInto(radius,"radius");
	
	success = success && config.readInto(hue,"hue");
	success = success && config.readInto(hue_tolerance,"hue_tolerance");
	success = success && config.readInto(min_saturation,"min_saturation");
	success = success && config.readInto(min_lightness,"min_lightness");
	success = success && config.readInto(max_val,"max_val");
	success = success && config.readInto(displayMode,"default_displaymode");
	success = success && config.readInto(mix,"steering_mix");
	success = success && config.readInto(centering,"steering_centering");
	
	success = success && config.readInto(steering_gain,"steering_gain");
	cout << "steering gain =" << steering_gain << endl;
	success = success && config.readInto(turret_gain,"turret_gain");
	cout << "turret gain =" << turret_gain << endl;
	success = success && config.readInto(turrettilt,"turrettilt");
	
	success = success && config.readInto(image_rotation_gain,"image_rotation_gain");
	cout << "image rotation gain =" << image_rotation_gain << endl;
	
	success = success && config.readInto(kp,"kp");
	success = success && config.readInto(ki,"ki");
	success = success && config.readInto(kd,"kd");
	success = success && config.readInto(maxreverse,"maxreverse");
	success = success && config.readInto(maxforward,"maxforward");
	
	//PT.setPIDParams(kp,ki,kd);
	//PT.setReverseLimit(maxreverse);
	//PT.setForwardLimit(maxforward);
	
	success = success && config.readInto(throttleMode,"throttleMode");
	
	return success;
}

int main(int argc, char **argv) {
	
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
		cout << "Capturing from camera on bus " << cameraIndex << endl;
		capture = cvCaptureFromCAM(cameraIndex);
	}
	
	
	
	cvNamedWindow("Camera",1);
	cvNamedWindow("Map",1);
	//cvNamedWindow("util",1);
	cvCreateTrackbar( "Hue", "Camera", &hue, 255, 0 );
	cvCreateTrackbar( "Tol", "Camera", &hue_tolerance, 255, 0 );
	cvCreateTrackbar( "Sat", "Camera", &min_saturation, 255, 0 );
	cvCreateTrackbar( "Hor", "Camera", &horizon, img_height-1, 0 );
	
	cvCreateTrackbar( "Foot", "Map", &foot, map_height/2, 0 );
	cvCreateTrackbar( "Trap", "Map", &trap_width, map_width, 0 );
  	
	// tbrclient-interface stuff
	// setObserver allows for debug statements
  	//TC.setObserver(&std::cout);
	// initialize() makes TC (a tbrclient object) talk to the tbrprobe process
	// listening on UDP_PORT
  	TC.initialize(UDP_PORT,LOCALHOST_IP);

	IplImage* frame = NULL;		//the pointer to the frame buffer
	IplImage* image = cvCreateImage(cvSize(img_width,img_height),8,3);
	
	while(running)
	{
		//cout << "capturing from camera" << endl;
	
		frame = cvQueryFrame( capture );
		if(frame == NULL){break;}
		
		//change the image to the designated size
		cvResize(frame,image);
		
		//cout << "clearing map" << endl;
		M.clear();
		//cout << "applying transformed visual" << endl;
		M.setTransformedVisual(image,horizon,foot,trap_width,TC.getSteering()*image_rotation_gain);
		//cout << "filtering colour" << endl;
		M.selectColour(hue,hue_tolerance,min_saturation);
		//cout << "computing steering" << endl;
		double s_conf = 0;
		double s = M.getSteering(mix,centering,s_conf);
		//cout << "sending messages" << endl;
		
		//send the messages to tbrpobe
		TC.setSteering((int) (-s*100.0*steering_gain));
		
		turretpan = (int) (-s*100*steering_gain*turret_gain);
		TC.setTurret(turrettilt,turretpan);
		
		//if(throttleMode == 1){
		//	double velocity = TC.getOdometerVelocity();
		//	throttle = PT.getThrottle(velocity, 1.0);
		//}
		
		TC.setThrottle(throttle);
		
		cvLine(image,cvPoint(0,horizon),cvPoint(img_width-1,horizon),CV_RGB(0,255,0),1);
		
		switch(displayMode)
		{
			default:
			case 0:
				cvShowImage("Camera",image);
				cvShowImage("Map",M.getVisual());
			break;
			case 1:
				cvShowImage("Camera",image);
				cvShowImage("Map",M.getOccupancy());
			break;
			case 2:
				cvShowImage("Camera",image);
				cvShowImage("Map",M.getDriveability());
			break;
			case 3:
				cvShowImage("Camera",image);
				cvShowImage("Map",M.getDisplayImg());
			break;
		}
		
		//cout << "done loop" << endl;
		
		char c = cvWaitKey(10);
		processKey(c);
	}

	TC.finalize();


	cvDestroyWindow("Camera");
	cvDestroyWindow("Map");
	
	return 0;
}



