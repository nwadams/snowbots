//Copyright (C) 2007 Matthew Baumann

#ifndef IMAGEOPS_H
#define IMAGEOPS_H

//opencv
#include <cv.h>
#include <highgui.h>

#include <iostream>
#include <math.h>
#include <stdio.h>

#include "defs.h"
#include "util.h"

namespace imageops{
	
	struct point{
		double x;
		double y;
	};
	
	struct line{
		double rho;
		double theta;
	};
	
	struct linepair{
		line l1;	//what was the mean of lines with positive slope?
		line l2;	//what was the mean of lines with negative slope?
		
		int l1_count;	//how many lines of positive slope were there?
		int l2_count;	//how many lines of negative slope were there?
		
		line l1_var;	//what was the variance of the line 1 parameters?
		line l2_var;	//what was the variance of the line 2 parameters?
	};

	CvPoint lineIntersect(const line& l1, const line& l2);
	void DrawLineOnImg(const line& l, IplImage* img, CvScalar color);
	void DrawLinePairOnImg(const linepair* pr, IplImage* img);
	void PrintLinePairIntoString(char* str, const linepair* pr);

	bool isolateColor(CvArr* src, CvArr* dst, double hue, double hue_tolerance, double min_saturation, double max_val);
	linepair* findTriangle(IplImage* img,double rho_res, double theta_res, int threshold);
	std::vector<line> findLines(IplImage* img,double rho_res, double theta_res, int threshold);

	bool angleInRange(double angle, double min, double max, double interval = CV_PI*2);
}

#endif
