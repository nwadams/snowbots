


//-----------------------------------------------------------------------------


#include <stdio.h>
#include <iostream>
#include <cv.h>
#include <highgui.h>
#include <imageops.h>


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
int hue = 0;
int hue_tolerance = 10;
int min_saturation = 190;
int min_lightness = 100;
double max_val = 100;

//match threshold
int thresh = 200;

//hough transform parameters
double rho_res = 4;
double theta_res = CV_PI/180;
int hough_threshold = 100;

//program state variables
bool running = true;
int displayMode = 0;
int pyramidLevel = 0;

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
			if(displayMode > 3) {displayMode = 0;}
		break;
		case 'p':
			pyramidLevel++;
			if(pyramidLevel >= 7) {pyramidLevel = 0;}
		break;
	}
}


int main(int argc, char **argv) {

	if(argc < 2)
	{
		std::cout << "Error: arguments.  Usage: " << argv[0] << " <template image> [camera index]." << std::endl;
		return 0;
	}
	
	if(argc == 3)
	{
		cameraIndex = atoi(argv[2]);
	}
	
	char* template_filename = argv[1];
	
	cout << "Capturing from camera on bus " << cameraIndex << endl;
	capture = cvCaptureFromCAM(cameraIndex);
	
	cvNamedWindow("TemplateMatch",1);
	cvCreateTrackbar( "Hue", "TemplateMatch", &hue, 255, 0 );
	cvCreateTrackbar( "Tol", "TemplateMatch", &hue_tolerance, 255, 0 );
	cvCreateTrackbar( "Sat", "TemplateMatch", &min_saturation, 255, 0 );
	cvCreateTrackbar( "Thresh", "TemplateMatch", &thresh, 255, 0 );
	cvCreateTrackbar( "Hough_thresh", "TemplateMatch", &hough_threshold, 255, 0 );
	
	cvNamedWindow("Template",1);
	
	cvNamedWindow("Pyramid",1);

	IplImage* image = cvCreateImage(cvSize(width,height),8,3);
	IplImage* color = cvCreateImage(cvSize(width,height),8,1);
	IplImage* templ = cvLoadImage(template_filename,CV_LOAD_IMAGE_GRAYSCALE);
	
	IplImage* frame = NULL;
	IplImage* result = NULL;
	IplImage* laplace = NULL;
	
	imageops::IplImagePyramid* pyramid = new imageops::IplImagePyramid(color,7,15);
	imageops::IplImagePyramid* result_pyr = new imageops::IplImagePyramid();
	
	cvShowImage("Template",templ);
	
	while(running)
	{
		frame = cvQueryFrame( capture );
		if(frame == NULL){break;}
		
		cvResize(frame,image);
		imageops::isolateColor(image,color,hue,hue_tolerance,min_saturation,max_val);
		
		pyramid->regenerate(color,7,15);
		
		result_pyr->release();
		for(int i = 0; i < pyramid->levels(); i++)
		{
			IplImage* match = imageops::templateMatch(pyramid->at(i),templ);
			IplImage* match_thresh = cvCreateImage(cvSize(match->width,match->height),32,1);
			cvThreshold(match,match_thresh,(double) thresh / 255.0,1.0,CV_THRESH_TOZERO);
			cvReleaseImage(&match);
			result_pyr->push_back(match_thresh);
		}
		
		result = imageops::templateMatch(color,templ);
		laplace = cvCreateImage(cvSize(result->width,result->height),32,1);
		cvLaplace(result,laplace,1);

		
		//double min;
		//double max;
		//CvPoint min_loc;
		//CvPoint max_loc;
		
		//cvMinMaxLoc(result,&min,&max,&min_loc,&max_loc,NULL);
		//std::cout << "min/max = "<< min << "/" << max << " minloc=["<<min_loc.x<<","<<min_loc.y<<"]" << endl;
		
		int nonzero = cvCountNonZero(result);
		
		if(nonzero > 0)
		{
			//std::vector<imageops::interestPoint> minima = imageops::findLocalMinMax(result,true);
			std::vector<imageops::feature> features = imageops::locateScaledFeatures(result_pyr,templ);
			
			std::cout << "There are " << features.size() << " features." << std::endl;
			
			for(int i = 0; i < features.size(); i++){
				//std::cout << "feature at: [" << features[i].pos.x << "," << features[i].pos.y << "], scale = " << features[i].scale << std::endl;
				cvCircle(image, features[i].pos, (int) ((double) templ->width * features[i].scale),CV_RGB(0.0,1.0,0.0));
			}
			
			
			
			IplImage* result_8 = cvCreateImage(cvSize(result->width,result->height),8,1);
			cvConvertScaleAbs(result,result_8);
			
			std::vector<imageops::line> lines =  imageops::findLines(result_8, rho_res, theta_res,hough_threshold);
			
			for(int l = 0; l < lines.size(); l++ ){
				//imageops::DrawLineOnImg(lines[l], image, CV_RGB(0,0,255));
			}
			
			cvReleaseImage(&result_8);
			
		}

		switch(displayMode){
			default:
			case 0:
				cvShowImage("TemplateMatch",image);
				break;
			case 1:
				cvShowImage("TemplateMatch",color);
				break;
			case 2:
				cvShowImage("TemplateMatch",result);
				break;
			case 3:
				cvShowImage("TemplateMatch",laplace);
			break;
		}
		
		cvReleaseImage(&laplace);
		
		cvShowImage("Pyramid",result_pyr->at(pyramidLevel));
		
		char c = cvWaitKey(10);
		processKey(c);
	}
	cvReleaseImage(&image);
	cvReleaseImage(&templ);
	

	cvDestroyWindow("TemplateMatch");
	cvDestroyWindow("Template");
	cvDestroyWindow("Pyramid");
	
	return 0;
}
