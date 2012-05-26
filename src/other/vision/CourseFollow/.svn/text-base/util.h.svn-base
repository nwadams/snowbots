//Copyright (C) 2007 Matthew Baumann

#ifndef UTIL_HPP
#define UTIL_HPP

//opencv
#include <cv.h>
#include <highgui.h>

#include "defs.h"

//---------------------------------------------------------

#define ABS(x)    (((x) > 0) ? (x) : (-(x)))
//#define MAX(x,y)  (((x) > (y)) ? (x) : (y))
//#define MIN(x,y)  (((x) < (y)) ? (x) : (y))

#define CLAMP(val,min,max)	(MAX(MIN(val,max),min))

//---------------------------------------------------------

namespace util{

	int hSize(CvSeq* seq);
	int vSize(CvSeq* seq);
	
	CvSeq* trimShortContours(CvSeq* contours, int min);
	
	double anglediff_d(double angle1, double angle2);
	double anglediff_r(double angle1, double angle2);
	
}

#endif



