


//-----------------------------------------------------------------------------


#include <stdio.h>
#include <cv.h>
#include <highgui.h>

#include "imageOps.h"
#include "configReader.h"
#include <tbrclient.h>

//-----------------------------------------------------------------------------

#define CLAMP(val,min,max)	(MAX(MIN(val,max),min))

//-----------------------------------------------------------------------------

//Camera Capture opject pointer
CvCapture *capture = 0;

//camera capture index
int cameraIndex = CV_CAP_ANY;

//image size: controls size of processed image
int width = 320;
int height = 240;

//colour extraction parameters
int hue = 15;
int hue_tolerance = 10;
int min_saturation = 190;
int min_lightness = 100;
double max_val = 100;

//hough transform parameters
double rho_res = 4;
double theta_res = CV_PI/180;
int threshold = 100;

//program state variables
bool running = true;
int displayMode = 0;

double factor = 0.5;
int lastSteering = 0;

int throttle = 0;

//tbrprobe UDP communication
tbrprobe07::tbrclient TC;

struct vehicleState{
	int steering;
	int drive;
};

using namespace std;

//=============================================================================

int interpolate(int x, int x0, int x1, int y0, int y1)
{
	return y0 + (x - x0) * ((float) (y1-y0) / (float) (x1-x0));
}

//-----------------------------------------------------------------------------


///This function extracts the default parameters from a configuration file using
///a configreader object.
bool checkArguments(int argc, char** argv)
{
	if(argc < 2)
	{
		return false;
	}
	
	if(argc >= 3)
	{
		cameraIndex = atoi(argv[2]);
		
	}
	
	configReader CR;
	CR.open(argv[1]);
	
	vector<sVec> width_entries = CR.getEntry("width");
	if(width_entries.size() > 0 && width_entries[0].size() > 1)
	{
		width = atoi(width_entries[0][1].c_str());
		cout << "width = " << width << endl;
	}
	
	
	vector<sVec> height_entries = CR.getEntry("height");
	if(height_entries.size() > 0 && height_entries[0].size() > 1)
	{
		height = atoi(height_entries[0][1].c_str());
		cout << "height = " << height << endl;
	}
	
	vector<sVec> hue_entries = CR.getEntry("hue");
	if(hue_entries.size() > 0 && hue_entries[0].size() > 1)
	{
		hue = atoi(hue_entries[0][1].c_str());
		cout << "hue = " << hue << endl;
	}
	
	
	vector<sVec> huetol_entries = CR.getEntry("hue_tolerance");
	if(huetol_entries.size() > 0 && huetol_entries[0].size() > 1)
	{
		hue_tolerance = atoi(huetol_entries[0][1].c_str());
		cout << "hue_tolerance = " << hue_tolerance << endl;
	}
	
	vector<sVec> minsat_entries = CR.getEntry("min_saturation");
	if(minsat_entries.size() > 0 && minsat_entries[0].size() > 1)
	{
		min_saturation = atoi(minsat_entries[0][1].c_str());
		cout << "min_saturation = " << min_saturation << endl;
	}
	
	vector<sVec> maxval_entries = CR.getEntry("max_val");
	if(maxval_entries.size() > 0 && maxval_entries[0].size() > 1)
	{
		max_val = atof(maxval_entries[0][1].c_str());
		cout << "max_val = " << max_val << endl;
	}
	
	
	
	vector<sVec> rhores_entries = CR.getEntry("rho_res");
	if(rhores_entries.size() > 0 && rhores_entries[0].size() > 1)
	{
		rho_res = atof(rhores_entries[0][1].c_str());
		cout << "rho_res = " << rho_res << endl;
	}
	
	vector<sVec> thetares_entries = CR.getEntry("theta_res");
	if(thetares_entries.size() > 0 && thetares_entries[0].size() > 1)
	{
		theta_res = atof(thetares_entries[0][1].c_str());
		cout << "theta_res = " << theta_res << endl;
	}
	
	vector<sVec> threshold_entries = CR.getEntry("threshold");
	if(threshold_entries.size() > 0 && threshold_entries[0].size() > 1)
	{
		threshold = atoi(threshold_entries[0][1].c_str());
		cout << "threshold = " << threshold << endl;
	}

	return true;
}

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
			if(displayMode > 2) {displayMode = 0;}
		break;
		case '-':
			throttle -= 2;
		break;
		case '=':
			throttle += 2;
		break;
		case ' ':
			throttle = 0;
		break;

		
	}
}

///Process the set of lines extracted from the image using imageops, and determine
///a steering value from the lines detected
vehicleState lineGuidance(vector<imageops::line> lineList, IplImage* image)
{	
	imageops::line left;
	imageops::line right;
	int lCount = 0;
	int rCount = 0;
	
	left.rho = 0;
	left.theta = 0;
	
	right.rho = 0;
	right.theta = 0;
	
	bool leftPresence = false;
	bool rightPresence = false;
	
	
	vehicleState state;
	state.drive = 0;
	state.steering = 0;
	
	for(int l = 0; l < lineList.size(); l++)
	{
		if( imageops::angleInRange(lineList[l].theta, CV_PI/16, 7*CV_PI/16, CV_PI) )
		{
			rCount++;
			right.rho += lineList[l].rho;
			right.theta += lineList[l].theta;
		}
		else if( imageops::angleInRange(lineList[l].theta, 9*CV_PI/16, 15*CV_PI/16, CV_PI) )
		{
			lCount++;
			left.rho += lineList[l].rho;
			left.theta += lineList[l].theta;
		}
		else
		{
			//std::cout << "theta is out of range" << std::endl;
		}
	
		DrawLineOnImg(lineList[l], image, CV_RGB(0,0,255));
	}
	
	int countThreshold = 4;
	
	if(lCount > 0)
	{
		left.rho = left.rho / (double) lCount;
		left.theta = left.theta / (double) lCount;
		
		if(lCount > countThreshold)
		{
			leftPresence = true;
			DrawLineOnImg(left, image, CV_RGB(0,255,0));
		}
	}
	
	if(rCount > 0)
	{
		right.rho = right.rho / (double) rCount;
		right.theta = right.theta / (double) rCount;
		
		if(rCount > countThreshold)
		{
			rightPresence = true;
			DrawLineOnImg(right, image, CV_RGB(255,0,0));
		}
	}
	
	
	
	
	//four cases:
	if(leftPresence && rightPresence)
	{
		//we see two lines: compute intersection and head for apex
		CvPoint isect = imageops::lineIntersect(left,right);
		int difference = isect.x - (image->width / 2);

		state.steering = (int) ((double) (difference/2) * factor + (double) (lastSteering) * (1-factor));
		lastSteering = state.steering;
		//std::cout << isect.x << " - " << (image->width /2) << " = " << difference << std::endl;	
	}
	else if(leftPresence)
	{
		//we only see lines on the left.  Turn right
		state.steering = (int) ((double) (90) * factor + (double) (lastSteering) * (1-factor));
		lastSteering = state.steering;
	}
	else if(rightPresence)
	{
		//we only see lines on the right.  Turn left 
		state.steering = (int) ((double) (-90) * factor + (double) (lastSteering) * (1-factor));
		lastSteering = state.steering;
	}
	else
	{
		//we see no lines.  Go straight
		state.steering = (int) ((double) (0) * factor + (double) (lastSteering) * (1-factor));
		lastSteering = state.steering;
	}
	
	return state;
}

int main(int argc, char **argv) {

	checkArguments(argc,argv);
	
	
	cout << "Capturing from camera on bus " << cameraIndex << endl;
	capture = cvCaptureFromCAM(cameraIndex);
	
	cvNamedWindow("CourseFollow",1);
	cvCreateTrackbar( "Hue", "CourseFollow", &hue, 255, 0 );
	cvCreateTrackbar( "Tol", "CourseFollow", &hue_tolerance, 255, 0 );
	cvCreateTrackbar( "Sat", "CourseFollow", &min_saturation, 255, 0 );
  	
	// tbrclient-interface stuff
	// setObserver allows for debug statements
  	TC.setObserver(&std::cout);
	// initialize() makes TC (a tbrclient object) talk to the tbrprobe process
	// listening on UDP_PORT
  	TC.initialize(UDP_PORT,LOCALHOST_IP);

	IplImage* image = cvCreateImage(cvSize(width,height),8,3);
	IplImage* display = cvCreateImage(cvSize(width,height),8,3);
	IplImage* colourness = cvCreateImage( cvGetSize(image), 8, 1 );
	IplImage* frame = NULL;
	
	while(running)
	{
		frame = cvQueryFrame( capture );
		if(frame == NULL){break;}
		
		cvResize(frame,image);
		cvCopy(image,display);
		
		imageops::isolateColor(image,colourness,hue,hue_tolerance,min_saturation,max_val);
		
		vector<imageops::line> lineList = imageops::findLines(colourness,rho_res,theta_res,threshold);
		
		vehicleState state = lineGuidance(lineList,display);
				
		cout << "[" << state.steering << "," << state.drive << "]" << endl;
		
		TC.setSteering(-state.steering);
		TC.setThrottle(throttle);
		
		
		
		switch(displayMode)
		{
			case 0:
				cvShowImage("CourseFollow",image);
			break;
			case 1:
				cvSetZero(display);
				cvCopy(image,display,colourness);
				cvShowImage("CourseFollow",display);
			break;
			case 2:
				cvShowImage("CourseFollow",display);
			break;
		}
		
		char c = cvWaitKey(10);
		processKey(c);
	}

	TC.finalize();
	
	cvReleaseImage(&image);
	cvReleaseImage(&display);
	cvReleaseImage(&colourness);

	cvDestroyWindow("CourseFollow");
	
	return 0;
}



