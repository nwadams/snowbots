///main.cpp, the main file for raceguidance
///@author Matthew Baumann
///@date July 4 2009

//-----------------------------------------------------------------------------

#include <stdio.h>
#include <string>
#include <math.h>
#include <time.h>

#include <cv.h>
#include <highgui.h>

#include <imageops.h>
#include <tbrclient.h>
#include <ConfigFile.h>
#include <pidthrottle.h>
#include <localmap/camera.h>

#include "pathfinder.h"
#include "ranger.h"
//-----------------------------------------------------------------------------

//Camera Capture object pointer
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

int trap_top_width = map_width;
int trap_width = map_width / 3;
int anchorx = map_width/2;
int anchory = map_height;

//colour extraction parameters
int hue = 15;
int hue_tolerance = 10;
int min_saturation = 190;
int min_lightness = 100;

int hue2 = 15;
int hue_tolerance2 = 10;
int min_saturation2 = 190;
int min_lightness2 = 100;


double max_val = 100;

double colourAhead_thresh = 0.2;

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

double throttle_stop = 0.0;
double throttle_slow = 0.3;
double throttle_fast = 0.6;
double throttle_rev = -0.3;

int turretpan = 0;
int turrettilt = 0;
double turret_gain = 1.0;
double image_rotation_gain = 0.1;

int pollSensors = 0;

double ir_minrange = 0.1;
double ir_maxrange = 1.0;
double ir_mix = 0.5;  //IR data smoothing
double ir_scale = 160;

//IR sensor positions RELATIVE TO ANCHOR
int irnw_x = -10;
int irnw_y = -10;
double irnw_angle = -0.785;
int irnw_radius = 10;

int irn_x = 0;
int irn_y = -20;
double irn_angle = 0.0;
int irn_radius = 10;

int irne_x = 10;
int irne_y = -10;
double irne_angle = 0.785;
int irne_radius = 10;


//AI parameters:
double vision_conf_thresh = 0.3;
double range_conf_thresh = 0.4;
double stop_conf_thresh = 0.7;



//------------------------------------------------------------------------------

//sensor values
double KS = 0; //kill switch
double IRNW = 2.0; //Infrared rangefinder
double IRN = 2.0;
double IRNE = 2.0;

//odometry
double odoPos = 0;
double odoVel = 0;
double odoAcc = 0;

//tbrprobe UDP communication
tbrprobe07::tbrclient TC;
vision::PIDthrottle PT;

using namespace std;

//============================================================================

//int interpolate(int x, int x0, int x1, int y0, int y1)
//{
//	return y0 + (x - x0) * ((float) (y1-y0) / (float) (x1-x0));
//}

//-----------------------------------------------------------------------------

bool radialBlur(IplImage* src, IplImage* dst, const CvPoint2D32f& center, int steps, double increment){
	IplImage* acc = cvCreateImage(cvGetSize(src),8,1);
	cvCopy(src,acc);
	
	CvMat* rotation = cvCreateMat( 2, 3, CV_32FC1 );
	
	IplImage* current = cvCreateImage(cvGetSize(src),8,1);
	
	for(int i = -steps; i <= steps; i++){
		if(i != 0){
			cvZero(current);
		
			double angle = (double) i * increment;
			cv2DRotationMatrix(center, angle,1.0, rotation );
			cvWarpAffine( src, current, rotation );
		
			IplImage* temp = cvCloneImage(acc);
			cvMax(temp,current,acc);
			cvReleaseImage(&temp);
		}
	}
	cvResize(acc,dst);
	cvReleaseMat(&rotation);
	cvReleaseImage(&current);
	cvReleaseImage(&acc);
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

	if(!config.readInto(cameraIndex,"cam_index") ){cout << "ConfigFile Error: cannot load cam_index" << endl;}
	cout << "camera index is " << cameraIndex << endl;
	if(!config.readInto(img_width,"img_width") ){cout << "ConfigFile Error: cannot load img_width" << endl;}
	if(!config.readInto(img_height,"img_height") ){cout << "ConfigFile Error: cannot load img_height" << endl;}
	cout << "image size is " << img_width << "x" << img_height << endl;
	if(!config.readInto(horizon,"horizon") ){cout << "ConfigFile Error: cannot load horizon" << endl;}
	if(!config.readInto(map_width,"map_width") ){cout << "ConfigFile Error: cannot load map_width" << endl;}
	if(!config.readInto(map_height,"map_height") ){cout << "ConfigFile Error: cannot load map_height" << endl;}

	if(!config.readInto(foot,"foot") ){cout << "ConfigFile Error: cannot load foot" << endl;}

	if(!config.readInto(trap_top_width,"trap_top_width") ){cout << "ConfigFile Error: cannot load trap_top_width" << endl;}
	if(!config.readInto(trap_width,"trap_width") ){cout << "ConfigFile Error: cannot load trap_width" << endl;}
	if(!config.readInto(anchorx,"anchorx") ){cout << "ConfigFile Error: cannot load anchorx" << endl;}
	if(!config.readInto(anchory,"anchory") ){cout << "ConfigFile Error: cannot load anchory" << endl;}

	if(!config.readInto(colourAhead_thresh,"colourAhead_thresh") ){cout << "ConfigFile Error: cannot load colourAhead_thresh" << endl;}
	if(!config.readInto(radius,"radius") ){cout << "ConfigFile Error: cannot load radius" << endl;}

	if(!config.readInto(hue,"hue") ){cout << "ConfigFile Error: cannot load hue" << endl;}
	if(!config.readInto(hue_tolerance,"hue_tolerance") ){cout << "ConfigFile Error: cannot load hue_tolerance" << endl;}
	if(!config.readInto(min_saturation,"min_saturation") ){cout << "ConfigFile Error: cannot load min_saturation" << endl;}
	if(!config.readInto(min_lightness,"min_lightness") ){cout << "ConfigFile Error: cannot load min_lightness" << endl;}
	
	if(!config.readInto(hue2,"hue2") ){cout << "ConfigFile Error: cannot load hue2" << endl;}
	if(!config.readInto(hue_tolerance2,"hue_tolerance2") ){cout << "ConfigFile Error: cannot load hue_tolerance2" << endl;}
	if(!config.readInto(min_saturation2,"min_saturation2") ){cout << "ConfigFile Error: cannot load min_saturation2" << endl;}
	if(!config.readInto(min_lightness2,"min_lightness2") ){cout << "ConfigFile Error: cannot load min_lightness2" << endl;}
	
	
	if(!config.readInto(max_val,"max_val") ){cout << "ConfigFile Error: cannot load max_val" << endl;}
	if(!config.readInto(displayMode,"default_displaymode") ){cout << "ConfigFile Error: cannot load default_displaymode" << endl;}
	if(!config.readInto(mix,"steering_mix") ){cout << "ConfigFile Error: cannot load steering_mix" << endl;}

	if(!config.readInto(steering_gain,"steering_gain") ){cout << "ConfigFile Error: cannot load steering_gain" << endl;}
	if(!config.readInto(turret_gain,"turret_gain") ){cout << "ConfigFile Error: cannot load turret_gain" << endl;}
	if(!config.readInto(turrettilt,"turrettilt") ){cout << "ConfigFile Error: cannot load turrettilt" << endl;}
	if(!config.readInto(image_rotation_gain,"image_rotation_gain") ){cout << "ConfigFile Error: cannot load image_rotation_gain" << endl;}

	if(!config.readInto(displayMode,"default_displaymode") ){cout << "ConfigFile Error: cannot load default_display_mode" << endl;}

	if(!config.readInto(kp,"kp") ){cout << "ConfigFile Error: cannot load kp" << endl;}
	if(!config.readInto(ki,"ki") ){cout << "ConfigFile Error: cannot load ki" << endl;}
	if(!config.readInto(kd,"kd") ){cout << "ConfigFile Error: cannot load kd" << endl;}
	if(!config.readInto(maxreverse,"maxreverse") ){cout << "ConfigFile Error: cannot load maxreverse" << endl;}
	if(!config.readInto(maxforward,"maxforward") ){cout << "ConfigFile Error: cannot load maxforward" << endl;}

	PT.setPIDParams(kp,ki,kd);
	PT.setReverseLimit(maxreverse);
	PT.setForwardLimit(maxforward);

	if(!config.readInto(throttleMode,"throttleMode") ){cout << "ConfigFile Error: cannot load throttleMode" << endl;}
	
	if(!config.readInto(throttle_stop,"throttle_stop") ){cout << "ConfigFile Error: cannot load throttle_stop" << endl;}
	if(!config.readInto(throttle_fast,"throttle_fast") ){cout << "ConfigFile Error: cannot load throttle_fast" << endl;}
	if(!config.readInto(throttle_slow,"throttle_slow") ){cout << "ConfigFile Error: cannot load throttle_slow" << endl;}
	if(!config.readInto(throttle_rev,"throttle_rev") ){cout << "ConfigFile Error: cannot load throttle_rev" << endl;}
	
	
	if(!config.readInto(pollSensors,"pollSensors") ){cout << "ConfigFile Error: cannot load pollSensors" << endl;}
	cout << "pollSensors = " << pollSensors << endl;

	if(!config.readInto(ir_minrange,"ir_minrange") ){cout << "ConfigFile Error: cannot load ir_minrange" << endl;}
	if(!config.readInto(ir_maxrange,"ir_maxrange") ){cout << "ConfigFile Error: cannot load ir_maxrange" << endl;}
	if(!config.readInto(ir_mix,"ir_mix") ){cout << "ConfigFile Error: cannot load ir_mix" << endl;}
	if(!config.readInto(ir_scale,"ir_scale") ){cout << "ConfigFile Error: cannot load ir_scale" << endl;}
	if(!config.readInto(irnw_x,"irnw_x") ){cout << "ConfigFile Error: cannot load irnw_x" << endl;}
	if(!config.readInto(irnw_y,"irnw_y") ){cout << "ConfigFile Error: cannot load irnw_y" << endl;}
	if(!config.readInto(irnw_angle,"irnw_angle") ){cout << "ConfigFile Error: cannot load irnw_angle" << endl;}
	if(!config.readInto(irnw_radius,"irnw_radius") ){cout << "ConfigFile Error: cannot load irnw_radius" << endl;}
	if(!config.readInto(irn_x,"irn_x") ){cout << "ConfigFile Error: cannot load irn_x" << endl;}
	if(!config.readInto(irn_y,"irn_y") ){cout << "ConfigFile Error: cannot load irn_y" << endl;}
	if(!config.readInto(irn_angle,"irn_angle") ){cout << "ConfigFile Error: cannot load irn_angle" << endl;}
	if(!config.readInto(irn_radius,"irn_radius") ){cout << "ConfigFile Error: cannot load irn_radius" << endl;}
	if(!config.readInto(irne_x,"irne_x") ){cout << "ConfigFile Error: cannot load irne_x" << endl;}
	if(!config.readInto(irne_y,"irne_y") ){cout << "ConfigFile Error: cannot load irne_y" << endl;}
	if(!config.readInto(irne_angle,"irne_angle") ){cout << "ConfigFile Error: cannot load irne_angle" << endl;}
	if(!config.readInto(irne_radius,"irne_radius") ){cout << "ConfigFile Error: cannot load irne_radius" << endl;}
	
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
	
	vision::camera cam(capture,cvSize(img_width,img_height),cameraIndex,horizon,"Intrinsics.xml","Distortion.xml","Homography.xml");
	
	cvNamedWindow("Camera",1);
	cvNamedWindow("Map",1);
	cvCreateTrackbar( "Hue", "Camera", &hue, 255, 0 );
	cvCreateTrackbar( "Tol", "Camera", &hue_tolerance, 255, 0 );
	cvCreateTrackbar( "Sat", "Camera", &min_saturation, 255, 0 );
  	
	// tbrclient-interface stuff
	// setObserver allows for debug statements
  	//TC.setObserver(&std::cout);
	// initialize() makes TC (a tbrclient object) talk to the tbrprobe process
	// listening on UDP_PORT
  	TC.initialize(UDP_PORT,LOCALHOST_IP);

	CvPoint anchor = cvPoint(anchorx,anchory);  //position of robot within map image

	vision::pathFinder PF(7,anchor,map_height/15,1.0);
	
	vision::ranger IR(ir_minrange,ir_maxrange,1.0);


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
	
	map_quad[0].x = (map_width / 2) - (trap_top_width/2); //top left
	map_quad[0].y = 0;
	
	map_quad[1].x = (map_width / 2) + (trap_top_width/2);		//top right
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
	
	IplImage* roadMask = cvCreateImage(cvSize(map_width,map_height),8,1);
	//fill the roadmask with a triangle
	for(int row = 0; row < roadMask->height; row++){
		for(int col = 0; col < roadMask->width; col++){
			int mincol = roadMask->width/2 - row/2;
			int maxcol = roadMask->height/2 + row/2;
			if(col > mincol && col < maxcol){
				cvSet2D(roadMask,row,col,cvScalarAll(255));
			}
		}
	}
	
	CvMat* rotation = cvCreateMat( 2, 3, CV_32FC1 );
		CvPoint2D32f center;
		center.x = anchor.x;
		center.y = anchor.y;
	
	
	
	//values that will se actually sent to chassis
	double masterSteering = 0;
	double masterThrottle = 0;
	
	int AI_case = 0;
	
	double lastOdoPos = 0;
	double lastTime = 0;
	
	//these variables allow the AI to lock itself into an action (e.g. stop, back up)
	//for a certain distance or time, whichever comes first;
	bool AI_lock = false;
	double AI_lockpos = 0;
	double AI_lockdist = 0;
	time_t AI_locktime = 0;
	double AI_lockduration = 0;
	
	while(running)
	{
		//frame = cvQueryFrame( capture );
		//if(frame == NULL){break;}
		
		//change the image to the designated size
		//cvResize(frame,image);
		//cvCopy(image,display);
	
	if(!cam.captureFrame()){cout << "captureFrame() failed" << endl;}
		
		cam.getUndistortedFrame(image);
	
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
		
		IplImage* map_orange = cvCreateImage( cvGetSize(map_rotated), 8,1 );
		IplImage* map_green = cvCreateImage( cvGetSize(map_rotated), 8,1 );
		
		imageops::isolateColor(map_rotated,map_orange,hue,hue_tolerance,min_saturation,max_val);
		imageops::isolateColor(map_rotated,map_green,hue2,hue_tolerance2,min_saturation2,max_val);
		
		map_colourness = cvCreateImage( cvGetSize(map_rotated), 8,1 );
		cvMax(map_orange,map_green,map_colourness);
		
		cvReleaseImage(&map_orange);
		cvReleaseImage(&map_green);
	
		map_colourness_inv = cvCreateImage( cvGetSize(map_rotated), 8,1 );
		
		imageops::isolateColor(image,image_colourness,hue,hue_tolerance,min_saturation,max_val);
		//imageops::isolateColor(map_rotated,map_colourness,hue,hue_tolerance,min_saturation,max_val);
		
		//IplImage* buffer = cvCloneImage(map_colourness);
		//cvDilate(buffer,map_colourness,NULL,2);
		//cvReleaseImage(&buffer);
		
		IplImage* buffer = cvCloneImage(map_colourness);
		CvPoint2D32f cen;
		cen.x = anchorx;
		cen.y = anchory;
		radialBlur(buffer, map_colourness, cen, 2, 5.0);
		cvReleaseImage(&buffer);
		
		//if the IR sensors are active, poll them and draw a circle onto the map-colourness image for
		//each sensor if it's reading an object within [ir_minrange,ir_maxrange].  This object will
		//be treated by the system as if it were a coloured object, when in fact it's just a simulacrum
		bool paused = false;
		
		if(pollSensors != 0){
			KS = TC.getKillSwitchVal();

			//printf("killswitch = %f\n",KS);
			if(KS < 2.0){paused = true;}

			IRNW = TC.getInfraredNW();
			IRN = TC.getInfraredN();
			IRNE = TC.getInfraredNE();

			//printf("Infrared NW N NE [ %#1.3f  %#1.3f  %#1.3f ]\n",IRNW,IRN,IRNE);
		
			odoPos = TC.getOdometerDistance();
			odoVel = TC.getOdometerVelocity();
			odoAcc = TC.getOdometerAcceleration();
		
			IR.setRanges(IRNW,IRN,IRNE, ir_mix);
			
			IRNW = IR.getSmoothRange(0);
			IRN = IR.getSmoothRange(1);
			IRNE = IR.getSmoothRange(2);
			
			//printf("IR SYSTEM SAYS steering=%#1.3f throttle=%#1.3f conf=%#1.3f\n",IR.getSteering(),IR.getThrottle(),IR.getConfidence());
			
			//draw the IR data into the map_colourness image
			if(IRNW > ir_minrange && IRNW < ir_maxrange){
				CvPoint NW_pos = cvPoint(anchor.x + irnw_x + ir_scale*IRNW*sin(irnw_angle),anchor.y + irnw_y - ir_scale*IRNW*cos(irnw_angle));
				//cvCircle(map_colourness, NW_pos, irnw_radius, cvScalarAll(255), -1);
				cvCircle(map, NW_pos, irnw_radius, CV_RGB(255,255,0), 1);
			}
			
			if(IRN > ir_minrange && IRN < ir_maxrange){
				CvPoint N_pos = cvPoint(anchor.x + irn_x + ir_scale*IRN*sin(irn_angle),anchor.y + irn_y - ir_scale*IRN*cos(irn_angle));
				//cvCircle(map_colourness, N_pos, irn_radius, cvScalarAll(255), -1);
				cvCircle(map, N_pos, irn_radius, CV_RGB(255,255,0), 1);
			}
			if(IRNE > ir_minrange && IRNE < ir_maxrange){
				CvPoint NE_pos = cvPoint(anchor.x + irne_x + ir_scale*IRNE*sin(irne_angle),anchor.y + irne_y - ir_scale*IRNE*cos(irne_angle));
				//cvCircle(map_colourness, NE_pos, irne_radius, cvScalarAll(255), -1);
				cvCircle(map, NE_pos, irne_radius, CV_RGB(255,255,0), 1);
			}	
			
		}
	
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


		IplImage* roadAhead = cvCreateImage(cvGetSize(map_colourness),8,1);
		cvZero(roadAhead);
		cvCopy(map_colourness,roadAhead,roadMask);
		double inRoad = cvCountNonZero(roadAhead);
		double roadCount = cvCountNonZero(roadMask);
		double colourAhead = inRoad/roadCount;
		cvReleaseImage(&roadAhead);

		//visualise th guidance sampling 
		PF.drawCurves(map);
		PF.drawPath(map);
		
		//determine the best steering path
		//double s = PF.evaluate(map_colourness,radius, mix);
		double v_steering = 0;
		if(colourAhead > colourAhead_thresh ) {
			v_steering = 1.5*  PF.evaluate(map_disttransform,radius, mix);
			printf("PATHFINDER %#1.2f vs. %#1.2f\n",colourAhead,colourAhead_thresh);
		}	
		else{
			printf("STRAIGHT %#1.2f vs. %#1.2f\n",colourAhead,colourAhead_thresh);
		}
		
		double v_throttle = vision::interpolate(fabs(v_steering),0,1.0,throttle_fast,throttle_slow);
		printf("fabs(vsteering) = %#1.2f v_throttle = %#1.2f\n",fabs(v_steering),v_throttle);
		double v_confidence = 10.0 * (double) cvCountNonZero( map_colourness ) / (double) (map_width*map_height);




		double r_steering = 0;
		double r_throttle = 0;
		double r_confidence = 0;
		
		if(pollSensors != 0){
			
			printf("IR getSteering = %#1.2f getThrottle = %#1.2f\n",IR.getSteering(),IR.getThrottle());
			r_steering = IR.getSteering();
			r_throttle = IR.getThrottle();
			
			
			if(r_throttle > 0){
				r_throttle = vision::interpolate(r_throttle,0,1.0,0,throttle_fast);
			}
			else{
				r_throttle = vision::interpolate(r_throttle,-1.0,0,throttle_rev,0);
			}
			r_confidence = IR.getConfidence();
		}
	
		double s_confidence = 0;
	
		/************************************************************/
	
		//MASTER AI
		
		//compute distance travelled;
		
		double dp = 0;
		
		//compute time elapsed
		time_t currTime = time(NULL);
		
		/*if(AI_lock){
			
			printf("-- AI LOCKED!    d = %#1.2f / %#1.2f t = %#1.2f / %#1.2f",fabs(odoPos - AI_lockpos),fabs(AI_lockdist),difftime(AI_locktime,currTime),AI_lockduration);
			
			//if the lock time has expired
			if( difftime(AI_locktime,currTime) > AI_lockduration ){
				AI_lock = false;
			}
			//if we've travelled the lock distance
			if( fabs(odoPos - AI_lockpos) > fabs(AI_lockdist) ){
				AI_lock = false;
			}
		
		}
		
		
		if(!AI_lock)
		{
			printf("-- AI : vision [%#1.2f,%#1.2f,%#1.2f] ir = [%#1.2f,%#1.2f,%#1.2f] stopsign = [%#1.2f]\n",v_steering,v_throttle,v_confidence,r_steering,r_throttle,r_confidence,s_confidence);
		//IF WE ARE MORE THAN 4M PAST THE LAST STOPSIGN

			//check for a stopsign
			
			//if the stopsign confidence > threshold
			
				//declare a stopsign
				
				//note the current dist
				
				//declare a pause of 3s
		
			//CASE: neither vision nor range confidence is high
			if(v_confidence < vision_conf_thresh && r_confidence < range_conf_thresh)
			{
				AI_case = 0;
				//go straight, 50% speed
				masterSteering = v_steering*0.25;
				masterThrottle = v_throttle;
			}
			//CASE: vision confidence is high, range low
			else if(v_confidence >= vision_conf_thresh && r_confidence < range_conf_thresh){
				AI_case = 1;
				//go with vision, 100% speed
				masterSteering = v_steering;
				masterThrottle = v_throttle;
			}
			//CASE: vision confidence is low, range high
			else if(v_confidence < vision_conf_thresh && r_confidence >= range_conf_thresh){
				AI_case = 2;
				//go with range data, ranger determined speed
				masterSteering = r_steering;
				masterThrottle = r_throttle;
			}
			//CASE: vision and range confidence high
			else if(v_confidence >= vision_conf_thresh && r_confidence >= range_conf_thresh){
				AI_case = 3;
				//SUBCASE: steering agrees and ranger throttle is forward
				if(v_steering * r_steering > 0 && r_throttle > 0){
					//go with it, 75% throttle
					masterSteering = v_steering > r_steering ? v_steering : r_steering;
					masterThrottle = v_throttle;
				}
				//SUBCASE: steering is opposed and ranger throttle is backwards
				else if(v_steering * r_steering <= 0 && r_throttle < 0){
					//back up, use ranger steering, which agrees with vision steering (reversed) for 0.4m
					
					//back up
					masterSteering = r_steering;
					masterThrottle = throttle_rev;
					
					//lock the AI for 0.4m or 4s, whichever comes first
					AI_lock = true;
					AI_lockpos = odoPos;
					AI_lockdist = -0.4;
					AI_locktime = time(NULL);
					AI_lockduration = 4.0;
					
				}
				//SUBCASE: steering is opposed and ranger throttle is forward
				else if(v_steering * r_steering <= 0 && r_throttle > 0){
					//back up, straight for 0.3m
					masterSteering = 0;
					masterThrottle = throttle_rev;
					
					//lock the AI for 0.4m or 4s, whichever comes first
					AI_lock = true;
					AI_lockpos = odoPos;
					AI_lockdist = -0.4;
					AI_locktime = time(NULL);
					AI_lockduration = 4.0;
				}
				//SUBCASE: otherwise, forward at 25% throttle
				else{
					masterSteering = 0;
					masterThrottle = throttle_slow;
				}
			}
		}*/
		
		printf("-- AI : vision [%#1.2f,%#1.2f,%#1.2f] ir = [%#1.2f,%#1.2f,%#1.2f] stopsign = [%#1.2f]\n",v_steering,v_throttle,v_confidence,r_steering,r_throttle,r_confidence,s_confidence);


		if(v_confidence != 0 && r_confidence !=0){
			masterSteering = (v_steering * v_confidence + r_steering * r_confidence ) / (v_confidence + r_confidence);
			masterThrottle = (v_throttle * v_confidence + r_throttle * r_confidence ) / (v_confidence + r_confidence);		
		}
		else
		{
			masterThrottle = 0;
			masterSteering = 0;
		}
		printf("-- Master AI case %d : steering = %#1.3f, throttle = %#1.3f\n",AI_case,masterSteering,masterThrottle);
		
		TC.setSteering((int) (-masterSteering*100.0*steering_gain));
		
		turretpan = (int) (-masterSteering*100*turret_gain);
		TC.setTurret(turrettilt,turretpan);

		if(!paused){
			if(throttleMode == 1){
				double velocity = TC.getOdometerVelocity();
				throttle = PT.getThrottle(velocity, 1.0);
			}
		
			//TC.setThrottle(throttle);
			TC.setThrottle(masterThrottle + throttle);
		}
		else{
			TC.setThrottle(0);
		}
		
		
		/************************************************************/
	
		
	
		
		//send the messages to tbrprobe
		/*TC.setSteering((int) (-s*100.0*steering_gain));
		
		turretpan = (int) (-s*100*turret_gain);
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
		}*/
		
		
		
		
		
		
		
		
		
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
	cvReleaseImage(&roadMask);	
	
	cvDestroyWindow("Camera");
	cvDestroyWindow("Map");
	//cvDestroyWindow("util");
	
	return 0;
}
