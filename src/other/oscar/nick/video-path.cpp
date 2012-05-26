#include <cstdlib>
#include <iostream>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <stdio.h>

#define BLUE  cvScalar(255, 0, 0)
#define GREEN cvScalar(0, 255, 0
#define RED   cvScalar(0, 0, 255)

#define PI 3.14159
//tolerance calibration
#define TOLERANCE 10

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

/**
 This function does this.
*/
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

//clamp function
double clamp (double value, int min, int max)
{
	if(value < min)
		return min;

	if(value > max)
		return max;
	else
	return value;
}

CvPoint getCenterOfGravity(IplImage *hsv_img, CvPoint seed)
{
	
    // get a pointer to the row that the seed is in
	uchar *seed_row = (uchar *) (hsv_img->imageData + 
		(seed.y * hsv_img->widthStep) );
	int seed_hue = seed_row[(3 * seed.x)];

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

	if(mass==0)
	return cvPoint(seed.x, seed.y);

	return cvPoint( 
		seed.x + (moment_x / mass),
		seed.y + (moment_y / mass) );
}


void doPicture(IplImage *in, IplImage *out)
{
	double angle;

	
	cvCvtColor(in, out, CV_BGR2HSV);
	//compressLightness(out);
	CvPoint seed = getSeed(out);
	CvPoint center = getCenterOfGravity(out, seed);
	cvCvtColor(out, out, CV_HSV2BGR);
	circlePoint(out, seed, BLUE);
	circlePoint(out, center, RED);

	if(center.y - seed.y == 0 )
		angle = 0; // TODO 
	else	
		angle = -atan((center.x - seed.x) / (center.y - seed.y))*180/PI;
	
	printf("angle = %f\n", angle);
}


int main( int argc, char** argv ) { 
	CvCapture* capture;

	//capture from webcam if found, and image if a file name is passed	
	capture = cvCreateCameraCapture(CV_CAP_ANY);
	assert( capture != NULL );

	// Create
	IplImage *in;
	in = cvQueryFrame(capture);
	IplImage *out = cvCreateImage(
		cvGetSize(in),
		IPL_DEPTH_8U, 
		3 );	

	// Display
	cvNamedWindow("Output");

//print image to screen

    for(;;) {
        in = cvQueryFrame( capture );
	    if( !in ) break;

		doPicture(in, out);

		cvShowImage("Output", out);
        char c = cvWaitKey(33);
        if( c == 27 ) break;

	}
// Cleanup

	cvReleaseImage( &out );
	cvReleaseCapture( &capture );
	cvDestroyAllWindows();
	

	return 0;
}
