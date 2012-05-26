/*Vision Code*/
#include <cstdlib>
#include <iostream>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <stdio.h>
#include "vision.h"

#define BLUE  cvScalar(255, 0, 0)
#define GREEN cvScalar(0, 255, 0
#define RED   cvScalar(0, 0, 255)

#define PI 3.14159
//tolerance calibration
#define TOLERANCE 10

using namespace std;

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
			double p_conf = 1 - clamp(((abs(hue - seed_hue)) / (double) seed_hue ),0,1);
			if ( p_conf < 0.7 ) break;
			// max out color value
			//row[(3 * cur_x) + 2] = 255 * p_conf;
			moment_x += ( cur_x - seed.x ) * p_conf;
			moment_y += 2*( cur_y - seed.y ) * p_conf;
			cur_x_l = cur_x;
			mass++;
		}
		// Rightward
		for ( int cur_x = row_seed_x + 1; cur_x < hsv_img->width; cur_x++ )
		{
			int hue = row[(3 * cur_x)];
			double p_conf = 1 - clamp( ((abs(hue - seed_hue)) / (double) seed_hue),0,1);
			if ( p_conf < 0.7 ) break;
			// max out color value
			//row[(3 * cur_x) + 2] = 255 * p_conf;
			moment_x += ( cur_x - seed.x ) * p_conf;
			moment_y += 2*( cur_y - seed.y ) * p_conf;
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


double doPicture(IplImage *in, IplImage *out)
{
	double angle;
	cvCvtColor(in, out, CV_BGR2HSV);
	//compressLightness(out);
	CvPoint seed = getSeed(out);
	//findPath(out, seed);
	CvPoint center = getCenterOfGravity(out, seed);
	cvCvtColor(out, out, CV_HSV2BGR);
	circlePoint(out, seed, BLUE);
	circlePoint(out, center, RED);

	if(center.y - seed.y == 0 )
		angle = 0; // TODO 
	else	
		angle = -atan((center.x - seed.x) / (center.y - seed.y))*180/PI;


	
	return angle;
}

