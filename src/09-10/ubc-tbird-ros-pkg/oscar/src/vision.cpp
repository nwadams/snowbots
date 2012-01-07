/*Vision Code*/
#include <cstdlib>
#include <iostream>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <stdio.h>
#include "vision.h"
#include <math.h>

#define BLUE  cvScalar(255, 0, 0)
#define GREEN cvScalar(0, 255, 0
#define RED   cvScalar(0, 0, 255)

#define PI 3.14159
//tolerance calibration
#define VERTFACT 1
#define HORIZFACT 2.0

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

CvPoint getCenterOfGravity(IplImage *img, CvPoint seed, double tolerance)
{
	
    // get a pointer to the row that the seed is in
	uchar *seed_row = (uchar *) (img->imageData + 
		(seed.y * img->widthStep) );
	int seed_val = seed_row[(seed.x)];

	int mass = 0, mid_mass_row;
	int cur_x_l=seed.x, cur_x_r=seed.x;
	int moment_x = 0, moment_y = 0;

	int row_seed_x = seed.x;
	// Upward
	for ( int cur_y = seed.y - 1; cur_y > 0; cur_y-- )
	{
		//total mass before scanning this row
		int mass_before = mass;
		uchar *row = (uchar *)(img->imageData + 
				(cur_y * img->widthStep));
		// Leftward
		for ( int cur_x = row_seed_x; cur_x > 0; cur_x-- )
		{
			
			int val = row[(cur_x)];
			double p_conf = 1.0 - clamp( (abs(val - seed_val)),0,100)/100;
			if ( p_conf < tolerance ) break;
			// max out color value
			//row[(3 * cur_x) + 2] = 255 * p_conf;
			moment_x += HORIZFACT *( cur_x - seed.x ) * p_conf;
			moment_y += VERTFACT*( cur_y - seed.y ) * p_conf;
			cur_x_l = cur_x;
			mass++;
		}
		// Rightward
		for ( int cur_x = row_seed_x + 1; cur_x < img->width; cur_x++ )
		{
			int val = row[(cur_x)];
			double p_conf = 1.0 - clamp( (abs(val - seed_val)),0,100)/100;
			if ( p_conf < tolerance ) break;
			// max out color value
			//row[(3 * cur_x) + 2] = 255 * p_conf;
			moment_x += HORIZFACT *( cur_x - seed.x ) * p_conf;
			moment_y += VERTFACT*( cur_y - seed.y ) * p_conf;
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


path doPicture(IplImage *out_hue, IplImage *out_sat)
{
	path a;
	//compressLightness(out);
	CvPoint seed_hue = getSeed(out_hue);
	CvPoint center_hue = getCenterOfGravity(out_hue, seed_hue, 0.7);
	CvPoint seed_sat = getSeed(out_sat);
	CvPoint center_sat = getCenterOfGravity(out_sat, seed_sat, 1.0);

	//findPath(out, seed);

	circlePoint(out_hue, seed_hue, BLUE);
	circlePoint(out_hue, center_hue, BLUE);
	circlePoint(out_sat, seed_sat, BLUE);
	circlePoint(out_sat, center_sat, BLUE);

	double sat_angle, hue_angle, sat_distance, hue_distance;
	
/***********************************************************/	
// FIX CONDITIONAL LOGIC

	if(center_sat.y - seed_sat.y == 0 )
		sat_angle = -90;
	else
		sat_angle = atan((double)(center_sat.x - seed_sat.x) / (center_sat.y - seed_sat.y))*180/PI;

	
	sat_distance =  sqrt(pow(center_sat.x - seed_sat.x,2) + pow(center_sat.y - seed_sat.y,2));
	
	
	
	if(center_hue.y - seed_hue.y == 0 )
		hue_angle = 90;
		
	else	
		hue_angle = atan((double)(center_hue.x - seed_hue.x) / (center_hue.y - seed_hue.y))*180/PI;
	
	hue_distance =  sqrt(pow(center_hue.x - seed_hue.x,2) + pow(center_hue.y - seed_hue.y,2));

/*****************************************************************************************/
	
	if(hue_distance > sat_distance)
	{
		a.distance = hue_distance;
		a.angle = hue_angle;
	} else {
		a.distance = sat_distance;
		a.angle = sat_angle;
	}
	
	if( center_hue.x - seed_hue.x == 0 && center_sat.x - seed_sat.x == 0 )
		a.confidence = 0;
	else
		a.confidence = 100 - clamp(abs(hue_distance - sat_distance)*abs(hue_angle - sat_angle),0,100)/50;
	
	
	
	return a;
}

