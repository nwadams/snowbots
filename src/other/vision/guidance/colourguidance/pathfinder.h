/** 
 * @author Matthew Baumann
 * @date Feb 1 2009
 * @brief The blobdetector class provides a simple C++ wrapper around the mscr
 * blob detector written by Per-Erik Forssen
 * 
 * This class is intended to provide an abstract interface for the MSCR blob 
 * detector, and to output its results as a self-contained data structure for
 * easy use in a larger vision system.
 */

#ifndef PATHFINDER_H
#define PATHFINDER_H

#include <math.h>
#include <vector>
#include <stdio.h>

#include "cv.h"
#include "highgui.h"

#define PATH_NODES 8	//number of nodes in the pathfinding curve
#define PATH_COUNT 7	//number of paths the pathfinder uses

using namespace std;

namespace vision{

	class curve{
		public:
			curve(const CvPoint& anchor, double step, double steeringLimit);
		
			void setSteering(double s);
			double getSteering();
			
			double minDist(const CvPoint& p);
			double nearestDist(const CvPoint& p);
			
			double penalty(IplImage* img, int distlimit);
			double imgpenalty(IplImage* img, double radius);
		
			void drawPath(IplImage* img, CvScalar color, int thickness = 1);
		
		protected:
		
			//double radius(double steering);
			//double incrementAngle(double steering, int increments);
			//CvPoint nextPerimeterPoint(CvPoint& prevPoint, double steering, double step, int increments, int index);
		
			//determine if a point in a binary image is nonzero
			//location in the image determined in the bottom-center centric map coordinate space
			bool nonzero(int x, int y, IplImage* img);
		
			void refreshPath(double steering);
			double dist(const CvPoint& p1, const CvPoint& p2);
			double dot(const CvPoint& p1, const CvPoint& p2);
			CvPoint normalise(const CvPoint& p);
			
		
			CvPoint anchor;
			double steering;
			double steeringLimit;
			double step;
			CvPoint nodes[PATH_NODES];
	
	};
	
	class pathFinder{
		public:
			pathFinder(int count, CvPoint anchor, double step, double steeringLimit);
			~pathFinder();
			
			double evaluate(IplImage* img, double radius, double mix = 0.8, double centering = 1.0);
			
			void drawCurves(IplImage* img);
			void drawPath(IplImage* img);
		
		protected:
			curve* mainPath;
		
			std::vector<curve> curves;
			std::vector<double> penalties;
			
			double min_penalty;
			double max_penalty;
	};
	
	double interpolate(double x, double x1, double x2, double y1, double y2);
	double double_interpolate(double x, double x1, double x2, double x3, double y1, double y2, double y3);
	double saturation(double x, double x0, double x1, double y0, double y1);
	
	void saturation(IplImage* src, IplImage* dst,double x0, double x1, double y0, double y1);
	
}

#endif
