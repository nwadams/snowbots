
#ifndef VISION_TEMPLATEMATCHER_H
#define VISION_TEMPLATEMATCHER_H

#include <stdio.h>
#include <string>
#include <math.h>
#include <vector>

#include <cv.h>
#include <highgui.h>

#include <imageops.h>

using namespace std;
using namespace imageops;

#define VISION_MINIMA -1
#define VISION_MAXIMA 1

namespace vision{

	class imagePyramid{
		public:
		
			imagePyramid(IplImage* src, int maxwidth, int minwidth, int levels);
			imagePyramid(imagePyramid& P, CvSize templateSize);
			
			
			~imagePyramid();
			
			int levels() const;
			IplImage* getImage(int level, IplImage* dst = NULL);
			IplImage* operator[](int level);
		
			double scale(int level);
			double getAspect();
			
			double findMinimum(int& level, CvPoint& pos, double& val);
			double findMinimum();
			
			vector<CvPoint> findLocalExtrema(IplImage* src, int mode, std::vector<double>& values);
		
		protected:
			vector<IplImage*> images;
			
			int maxwidth;
			int minwidth;
			double aspect;
		
	};


}

#endif
