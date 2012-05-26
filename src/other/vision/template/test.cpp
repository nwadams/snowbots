


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
			if(displayMode > 2) {displayMode = 0;}
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
	
	//cout << "Capturing from camera on bus " << cameraIndex << endl;
	//capture = cvCaptureFromCAM(cameraIndex);
	
	cvNamedWindow("TemplateMatch",1);
	cvCreateTrackbar( "Hue", "TemplateMatch", &hue, 255, 0 );
	cvCreateTrackbar( "Tol", "TemplateMatch", &hue_tolerance, 255, 0 );
	cvCreateTrackbar( "Sat", "TemplateMatch", &min_saturation, 255, 0 );
	
	cvNamedWindow("Template",1);
	
	cvNamedWindow("Pyramid",1);

	IplImage* image = cvCreateImage(cvSize(width,height),8,3);
	IplImage* color = cvCreateImage(cvSize(width,height),8,1);
	IplImage* templ = cvLoadImage(template_filename,CV_LOAD_IMAGE_GRAYSCALE);
	
	IplImage* frame = cvLoadImage("Course.ppm",CV_LOAD_IMAGE_COLOR);
	IplImage* result = NULL;
	
	imageops::IplImagePyramid* pyramid = new imageops::IplImagePyramid(color,7,15);
	imageops::IplImagePyramid* result_pyr = new imageops::IplImagePyramid();
	
	cvShowImage("Template",templ);
	
	while(running)
	{
		//frame = cvQueryFrame( capture );
		//if(frame == NULL){break;}
		
		cvResize(frame,image);
		imageops::isolateColor(image,color,hue,hue_tolerance,min_saturation,max_val);
		
		pyramid->regenerate(color,7,15);
		
		result_pyr->release();
		for(int i = 0; i < pyramid->levels(); i++)
		{
			result_pyr->push_back(imageops::templateMatch(pyramid->at(i),templ));
		}
		
		result = imageops::templateMatch(color,templ);
		
		double min;
		double max;
		CvPoint min_loc;
		CvPoint max_loc;
		
		cvMinMaxLoc(result,&min,&max,&min_loc,&max_loc,NULL);
		std::cout << "min/max = "<< min << "/" << max << " minloc=["<<min_loc.x<<","<<min_loc.y<<"]" << endl;
		
		
		std::vector<imageops::interestPoint> minima = imageops::findLocalMinMax(result,true);
		
		
		
		for(int i = 0; i < minima.size(); i++){
			std:;cout << "Minimum at: [" << minima[i].pos.x << "," << minima[i].pos.y << "]" << std::endl;
			//cvCircle(image, minima[i].pos, 2,CV_RGB(0.0,1.0,0.0));
		}

		switch(displayMode){
			case 0:
				cvShowImage("TemplateMatch",image);
				break;
			case 1:
				cvShowImage("TemplateMatch",color);
				break;
			case 2:
				cvShowImage("TemplateMatch",result);
				break;
		}
		
		cvShowImage("Pyramid",result_pyr->at(pyramidLevel));
		
		char c = cvWaitKey(0);
		processKey(c);
	}
	cvReleaseImage(&image);
	cvReleaseImage(&templ);
	

	cvDestroyWindow("TemplateMatch");
	cvDestroyWindow("Template");
	cvDestroyWindow("Pyramid");
	
	return 0;
}
