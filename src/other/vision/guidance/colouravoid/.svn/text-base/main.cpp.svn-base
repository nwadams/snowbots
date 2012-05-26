//-----------------------------------------------------------------------------

#include <stdio.h>
#include <string>

#include <cv.h>
#include <highgui.h>

#include <imageops.h>
#include <tbrclient.h>
#include <ConfigFile.h>
#include <pidthrottle.h>

#include "pathfinder.h"

//-----------------------------------------------------------------------------

//#define CLAMP(val,min,max)	(MAX(MIN(val,max),min))

//-----------------------------------------------------------------------------

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

int throttle = 0;
double targetspeed = 0;

//mode 0 = manual throttle
//mode 1 = pid throttle
int throttleMode = 0;

int turretpan = 0;
int turrettilt = 0;
double turret_gain = 1.0;
double image_rotation_gain = 0.1;

int pollSensors = 0;

//tbrprobe UDP communication
tbrprobe07::tbrclient TC;
vision::PIDthrottle PT;

using namespace std;

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
			if(displayMode > 4) {displayMode = 0;}
		break;
		case '-':
			if(throttleMode == 0)
			{
			
				throttle -= 4;
			}
			else{
				targetspeed -= 0.1;
				PT.setTargetSpeed(targetspeed);
			}
		break;
		
		case '=':
			
			if(throttleMode == 0){
				throttle += 4;
			}
			else
			{
				targetspeed += 0.1;
				PT.setTargetSpeed(targetspeed);
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
				PT.setTargetSpeed(targetspeed);
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
	success = success && config.readInto(horizon,"horizon");
	success = success && config.readInto(map_width,"map_width");
	success = success && config.readInto(map_height,"map_height");
	
	
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
	
	success = success && config.readInto(steering_gain,"steering_gain");
	success = success && config.readInto(turret_gain,"turret_gain");
	success = success && config.readInto(turrettilt,"turrettilt");
	success = success && config.readInto(image_rotation_gain,"image_rotation_gain");
	
	success = success && config.readInto(displayMode,"default_displaymode");
	
	success = success && config.readInto(kp,"kp");
	success = success && config.readInto(ki,"ki");
	success = success && config.readInto(kd,"kd");
	success = success && config.readInto(maxreverse,"maxreverse");
	success = success && config.readInto(maxforward,"maxforward");
	
	PT.setPIDParams(kp,ki,kd);
	PT.setReverseLimit(maxreverse);
	PT.setForwardLimit(maxforward);
	
	success = success && config.readInto(throttleMode,"throttleMode");
	
	success = success && config.readInto(pollSensors,"pollSensors");
	
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
  	
	// tbrclient-interface stuff
	// setObserver allows for debug statements
  	TC.setObserver(&std::cout);
	// initialize() makes TC (a tbrclient object) talk to the tbrprobe process
	// listening on UDP_PORT
  	TC.initialize(UDP_PORT,LOCALHOST_IP);

	CvPoint anchor = cvPoint(anchorx,anchory);  //position of robot within map image
	vision::pathFinder PF(7,anchor,map_height/15,1.0);


	//compute the source image quadrangle
	
	//NOTE!! The source image is image_ground, the image AFTER you've
	//cut off everything above the horizon.  The horizon is measured from the BOTTOM
	//of the image UP!!!!
	CvPoint2D32f src_quad[4];
	
	src_quad[0].x = 0;	//top left
	src_quad[0].y = horizon;
	
	src_quad[1].x = img_width-1;	//top right
	src_quad[1].y = horizon;
	
	src_quad[2].x = img_width-1;	//bottom right
	src_quad[2].y = (img_height-1);
	
	src_quad[3].x = 0;	//bottom left
	src_quad[3].y = (img_height-1);
	
	//compute the map quadrangle
	CvPoint2D32f map_quad[4];
	
	map_quad[0].x = 0; //top left
	map_quad[0].y = 0;
	
	map_quad[1].x = map_width-1;		//top right
	map_quad[1].y = 0;
	
	map_quad[2].x = (map_width / 2) + (trap_width/2);	//bottom right
	map_quad[2].y = map_height-1 - foot;
	
	map_quad[3].x = (map_width / 2) - (trap_width/2);	//bottom left
	map_quad[3].y = map_height-1 - foot;

	//compute the image transform
	CvMat* transform = cvCreateMat( 3, 3, CV_32FC1 );		//matrix for the transform to reside in
	cvGetPerspectiveTransform( src_quad, map_quad, transform );	//compute transform matrix
	
	IplImage* image = cvCreateImage(cvSize(img_width,img_height),8,3);					//the captured image
	IplImage* map = cvCreateImage(cvSize(map_width,map_height),8,3);					//the transformed ground map
	IplImage* map_rotated = cvCreateImage(cvSize(map_width,map_height),8,3);			//the rotated transformed ground map
	IplImage* map_display = cvCreateImage(cvSize(map_width,map_height),8,3);
	IplImage* frame = NULL;																//the pointer to the frame buffer
	
	CvMat* rotation = cvCreateMat( 2, 3, CV_32FC1 );
		CvPoint2D32f center;
		center.x = anchor.x;
		center.y = anchor.y;
	
	while(running)
	{
		frame = cvQueryFrame( capture );
		if(frame == NULL){break;}
		
		//change the image to the designated size
		cvResize(frame,image);
		//cvCopy(image,display);
		
		//CvMat subRect;
		//cvGetSubRect( image, &subRect, cvRect( 0, img_height-horizon-1, img_width, horizon-1) );
		//cvGetImage( &subRect, image_ground );

		IplImage* image_ground = NULL;	//the captured image below the horizon
		IplImage* image_colourness = NULL; 	//the binary colour map
		IplImage* map_colourness = NULL; 	//the binary colour map
		IplImage* map_colourness_inv = NULL;
		
		IplImage* map_disttransform = NULL;

		cvSetImageROI(image, cvRect(0, horizon, img_width, img_height-horizon));
		
		image_ground = cvCreateImage(cvGetSize(image),image->depth, image->nChannels);
		image_ground->origin = image->origin;
		cvCopy(image, image_ground, NULL);
		
		cvResetImageROI(image);
		
		cvWarpPerspective( image, map, transform, CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS);
		//printf("set center\n");
		cv2DRotationMatrix(center, (double) turretpan / 100.0 * 90.0 * image_rotation_gain,1.0, rotation );
		//printf("computed matrix\n");
		cvZero(map_rotated);
		//printf("reset buffer\n");
		cvWarpAffine( map, map_rotated, rotation ); 
		//printf("performed affine transform\n");
			
		image_colourness = cvCreateImage( cvGetSize(image),8,1);
		map_colourness = cvCreateImage( cvGetSize(map_rotated), 8,1 );
		map_colourness_inv = cvCreateImage( cvGetSize(map_rotated), 8,1 );
		
		imageops::isolateColor(image,image_colourness,hue,hue_tolerance,min_saturation,max_val);
		imageops::isolateColor(map_rotated,map_colourness,hue,hue_tolerance,min_saturation,max_val);
		
		imageops::invertImage(map_colourness,map_colourness_inv);
		imageops::threshold(map_colourness_inv, 200);
		
		map_disttransform = cvCreateImage( cvGetSize(map), IPL_DEPTH_32F, 1);
		cvDistTransform( map_colourness_inv, map_disttransform);
		
		cvLine( map, cvPoint(map_quad[0].x,map_quad[0].y), cvPoint(map_quad[1].x,map_quad[1].y), CV_RGB(0,255,0));
		cvLine( map, cvPoint(map_quad[1].x,map_quad[1].y), cvPoint(map_quad[2].x,map_quad[2].y), CV_RGB(0,255,0));
		cvLine( map, cvPoint(map_quad[2].x,map_quad[2].y), cvPoint(map_quad[3].x,map_quad[3].y), CV_RGB(0,255,0));
		cvLine( map, cvPoint(map_quad[3].x,map_quad[3].y), cvPoint(map_quad[0].x,map_quad[0].y), CV_RGB(0,255,0));

		cvLine(image, cvPoint(src_quad[0].x,src_quad[0].y), cvPoint(src_quad[1].x,src_quad[1].y), CV_RGB(0,255,0));
		cvLine(image, cvPoint(src_quad[1].x,src_quad[1].y), cvPoint(src_quad[2].x,src_quad[2].y), CV_RGB(0,255,0));
		cvLine(image, cvPoint(src_quad[2].x,src_quad[2].y), cvPoint(src_quad[3].x,src_quad[3].y), CV_RGB(0,255,0));
		cvLine(image, cvPoint(src_quad[3].x,src_quad[3].y), cvPoint(src_quad[0].x,src_quad[0].y), CV_RGB(0,255,0));

		

		//visualise th guidance sampling 
		PF.drawCurves(map);
		PF.drawPath(map);
		
		//determine the best steering path
		//double s = PF.evaluate(map_colourness,radius, mix);
		double s = PF.evaluate(map_disttransform,radius, mix);
		
		bool paused = false;
		
		if(pollSensors != 0){
			double KS = TC.getKillSwitchVal();
			printf("killswitch = %f\n",KS);
			if(KS < 1.0){paused = true;}
		
			double IRNW = TC.getInfraredNW();
			double IRN = TC.getInfraredN();
			double IRNE = TC.getInfraredNE();
		
			printf("Infrared NW N NE [ %f  %f  %f ]\n",IRNW,IRN,IRNE);
		
		}
		
		//send the messages to tbrpobe
		TC.setSteering((int) (-s*100.0*steering_gain));
		
		turretpan = (int) (-s*100*steering_gain*turret_gain);
		TC.setTurret(turrettilt,turretpan);
		
		if(!paused){
			if(throttleMode == 1){
				double velocity = TC.getOdometerVelocity();
				throttle = PT.getThrottle(velocity, 1.0);
			}
		
			TC.setThrottle(throttle);
		}
		else{
			TC.setThrottle(0);
		}
		
		
		
		switch(displayMode)
		{
			default:
			case 0:
				cvShowImage("Camera",image);
				cvShowImage("Map",map);
			break;
			case 1:
				cvShowImage("Camera",image);
				cvZero(map_display);
				cvCopy(map_rotated,map_display,map_colourness);
				PF.drawCurves(map_display);
				PF.drawPath(map_display);
				cvShowImage("Map",map_display);
			break;
			case 2:
				cvShowImage("Camera",image);
				cvShowImage("Map",map_colourness);
			break;
			case 3:
				cvShowImage("Camera",image);
				cvShowImage("Map",map_colourness_inv);
			break;
			case 4:
				cvShowImage("Camera",image);
				cvShowImage("Map",map_disttransform);
			break;
		}
		
		cvReleaseImage(&image_ground);
		cvReleaseImage(&image_colourness);
		cvReleaseImage(&map_colourness);
		
		cvReleaseImage(&map_colourness_inv);
		cvReleaseImage(&map_disttransform);
		
		char c = cvWaitKey(10);
		processKey(c);
	}

	TC.finalize();
	
	cvReleaseImage(&image);
	
	cvReleaseImage(&map);

	cvDestroyWindow("Camera");
	cvDestroyWindow("Map");
	//cvDestroyWindow("util");
	
	return 0;
}



