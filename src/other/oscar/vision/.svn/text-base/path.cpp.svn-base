#include <cstdlib>
#include <iostream>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <stdio.h>

#define BLUE  cvScalar(255, 0, 0)
#define GREEN cvScalar(0, 255, 0
#define RED   cvScalar(0, 0, 255)
//tolerance calibration
#define TOLERANCE 20

CvPoint getSeed(IplImage *img)
{
	return cvPoint(img->width / 2, img->height - (img->height / 20));
}

void circlePoint(IplImage *img, CvPoint point, CvScalar color=BLUE)
{
	const int RADIUS = 15;
	const int THICKNESS = 3;
	cvCircle(img, point, RADIUS, color, THICKNESS);
}

void findPath(IplImage *out, CvPoint seed)
{
	int diff = 5;
	cvFloodFill(
		out, 
		seed, 
		cvScalarAll(255),
		cvScalarAll(diff), 
		cvScalarAll(diff), 
		0, 4, 0);
}

void compressLightness(IplImage *hsv_img)
{
	for (int y=0; y < hsv_img->height; y++)
	{
		// point at the beginning of the row
		uchar *row = (uchar*) (
			hsv_img->imageData + (y * hsv_img->widthStep) );
		// iterate over the columns
		for (int x=0; x < hsv_img->width; x++ )
		{
			row[(3 * x) + 2] = (row[(3*x) + 2] / 4) + 64;
		}
	}
}

CvPoint getCenterOfGravity(IplImage *hsv_img, CvPoint seed)
{
	
    // get a pointer to the row that the seed is in
	uchar *seed_row = (uchar *) (hsv_img->imageData + 
		(seed.y * hsv_img->widthStep) );
	int seed_hue = seed_row[(3 * seed.x)];
	std::cout << "Hue is " << seed_hue << std::endl;

	int mass = 0, mid_mass_row;
	int cur_x_l, cur_x_r;
	int moment_x = 0, moment_y = 0;

	int row_seed_x = seed.x;
	// Upward
	for ( int cur_y = seed.y - 1; cur_y > 0; cur_y-- )
	{
		//total mass before scanning this row
		int mass_before = mass;
		uchar *row = (uchar *)(hsv_img->imageData + 
				(cur_y * hsv_img->widthStep));
		// Leftward
		for ( int cur_x = row_seed_x; cur_x > 0; cur_x-- )
		{
			int hue = row[(3 * cur_x)];
			if ( abs(hue - seed_hue) > TOLERANCE ) break;
			// max out color value
			row[(3 * cur_x) + 2] = 255;
			moment_x += ( cur_x - seed.x );
			moment_y += 2*( cur_y - seed.y );
			cur_x_l = cur_x;
			mass++;
		}
		// Rightward
		for ( int cur_x = row_seed_x + 1; cur_x < hsv_img->width; cur_x++ )
		{
			int hue = row[(3 * cur_x)];
			if ( abs(hue - seed_hue) > TOLERANCE ) break;
			// max out color value
			row[(3 * cur_x) + 2] = 255;
			moment_x += ( cur_x - seed.x );
			moment_y += 2*( cur_y - seed.y );
			cur_x_r = cur_x;
			mass++;
		}
		// If no matching points are found in this row, stop going up.
		if ( mass_before == mass ) break;

		//get new seed point for next row
		mid_mass_row = ( cur_x_l + cur_x_r )/2 ;
		row_seed_x = mid_mass_row;
	}
	return cvPoint( 
		seed.x + (moment_x / mass),
		seed.y + (moment_y / mass) );
}

void doVideo(int argc, char **argv)
{
	// Declare variables
	CvCapture *capture = NULL;
	CvSize framesize;
	int framedelay = 20;
	IplImage *frame, *out;
	char key;

	// Load video
	cvNamedWindow( "Input", CV_WINDOW_AUTOSIZE );
	cvNamedWindow( "Output", CV_WINDOW_AUTOSIZE );
	capture = cvCreateCameraCapture( CV_CAP_ANY );
	if ( ! capture )
	{
		std::cout << "Unable to open camera device, dude." << std::endl;
	}
	framesize = cvSize(
		(int) cvGetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH),
		(int) cvGetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT) );
	//out = cvCreateImage( framesize, IPL_DEPTH_8U, 3 );
	while (1)
	{
		// Grab frame
		frame = cvQueryFrame( capture );
		if (! frame ) break;
		cvShowImage( "Input", frame );

		// Change it
		//cvCopy(frame, out);
		//cvCvtColor(frame, out, CV_BGR2HSV);
		//compressLightness(out);
		//findPath(out);
		//cvShowImage( "Output", out);
		
		// Quit when key is pressed
		key = cvWaitKey( framedelay );
		if ( key == 'q' ) break;
	}

	// Shutdown
	cvReleaseCapture( &capture );
	cvReleaseImage( &out );
	cvDestroyWindow( "Input" );
	cvDestroyWindow( "Output" );
}

void doPicture(int argc, char **argv)
{
	// Init
	if (argc < 2)
	{
		std::cout << "Please enter a filename" << std::endl;
		exit(1);
	}
	// Create
	IplImage *in = cvLoadImage(argv[1]);
	IplImage *out = cvCreateImage(
		cvGetSize(in),
		IPL_DEPTH_8U, 
		3 );

	// Modify
	//cvCopy(in, out);
	cvCvtColor(in, out, CV_BGR2HSV);
	//compressLightness(out);
	CvPoint seed = getSeed(out);
	//findPath(out, seed);
	CvPoint center = getCenterOfGravity(out, seed);
	cvCvtColor(out, out, CV_HSV2BGR);
	circlePoint(out, seed, BLUE);
	circlePoint(out, center, RED);

	// Display
	cvNamedWindow("Input");
	cvShowImage("Input", in);
	cvNamedWindow("Output");
	cvShowImage("Output", out);
	cvWaitKey(0);

	// Cleanup
	cvReleaseImage( &in );
	cvReleaseImage( &out );
	cvDestroyWindow("Input");
	cvDestroyWindow("Output");
}

int main( int argc, char **argv )
{
	//doVideo(argc, argv);
	doPicture(argc, argv);
}


