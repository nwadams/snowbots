//Copyright (C) 2007 Matthew Baumann

#ifndef UTIL_HPP
#define UTIL_HPP

//opencv
#include <cv.h>
#include <highgui.h>

#include "defs.h"

//---------------------------------------------------------

#define ABS(x)    (((x) > 0) ? (x) : (-(x)))
#define CLAMP(val,min,max)	(MAX(MIN(val,max),min))

//---------------------------------------------------------

namespace util{

	///extract the horizontal size of an OpenCV Sequence
	int hSize(CvSeq* seq);
	
	///extract the vertical size of an OpenCV Sequence
	int vSize(CvSeq* seq);
	
	///Knock out any coutour segment that is below the minimum length
	CvSeq* trimShortContours(CvSeq* contours, int min);
	
	///simple integer clamp function
	int clampIndex(int val, int min, int max);
	
	///computer an angular difference in degrees
	double anglediff_d(double angle1, double angle2);
	
	///compute and angular difference in radians
	double anglediff_r(double angle1, double angle2);
	
}

#endif



