#ifndef VISION_MAP_H
#define VISION_MAP_H

#include <stdio.h> 
#include <stdlib.h>
#include <iostream>
#include <string>

#include <cv.h>
#include <highgui.h>
#include <imageops.h>

using namespace std;

namespace vision{
	
	class camera{
		/**************************************************************************/
		public:
		
			camera(const CvSize& s);
			camera(const CvSize& s, int index);
			camera(const CvSize& s, int index, int horizon, const char* intrinsics_file = NULL, const char* distortions_file = NULL, const char* homography_file = NULL);
			camera(CvCapture* capture, const CvSize& s, int index, int horizon, const char* intrinsics_file = NULL, const char* distortions_file = NULL, const char* homography_file = NULL);
			
			~camera();
			
			bool captureFrame();
			
			bool getFrame(IplImage* dst);
			bool getLowerFrame(IplImage* dst);
			bool getUpperFrame(IplImage* dst);
			bool getUndistortedFrame(IplImage* dst);
			bool getUndistortedUpperFrame(IplImage* dst);
			bool getUndistortedLowerFrame(IplImage* dst);
			bool getBirdsEyeFrame(IplImage* dst);

			bool calibrated();
			bool hasHomography();
			
			bool computeDistortionMaps();
			bool computeHomography(int board_w, int board_h, IplImage* image, CvPoint2D32f* map_rect);
			
			bool loadIntrinsics(const char* filename);
			bool loadDistortions(const char* filename);
			bool loadHomography(const char* filename);
			
			bool saveIntrinsics(const char* filename);
			bool saveDistortions(const char* filename);
			bool saveHomography(const char* filename);
			
			void setHorizon(int horizon);
			int getHorizon();
		
		/**************************************************************************/
		protected:
		
			bool initialize(CvCapture* cap, const CvSize& s, int index, int horizon, const char* intrinsics_file, const char* distortions_file, const char* homography_file);
			bool finalize();	
			
		/**************************************************************************/
		
			CvCapture* capture;
			IplImage* frame;
			IplImage* image_buffer;
			IplImage* undistorted_image_buffer;
			
			bool undistort_up_to_date;
		
			int horizon;
		
			CvMat* intrinsics;
			CvMat* distortions;
			CvMat* homography;
			
			IplImage* mapx;
			IplImage* mapy;

	};

}

#endif