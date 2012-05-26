///localmap.h
/// @file localmap.h
/// @author Matthew Baumann
/// @date July 1 2009
/// @brief This file defines the localmap class, which provides a scaled map of the immediate
/// surroundings of the vehicle.  The vehicle lies at the origin of the map, which is not
/// necessarily at the center.  
///

#ifndef VISION_LOCALMAP_H
#define VISION_LOCALMAP_H

#include <stdio.h> 
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <string>
#include <vector>

#include <cv.h>
#include <highgui.h>
#include <imageops.h>

using namespace std;

namespace vision{


	struct vehicle;
	class localmap;
	class pathFinder;
	class curve;


	struct vehicle{
		double wheelbase;
		double trackwidth;
		double turnradius;
		CvPoint2D32f camerapos;
	};

	class localmap{
	
		public:
		
			///create a new localmap
			localmap(CvSize img_size, double width, double height, CvPoint origin);
			
			///release all localmap data
			~localmap();
		
			///clear the localmap of all data
			void clear();
			
			///set the visual (colour) localmap image
			void setVisual(IplImage* src);
			
			///set the Visual to a perspective transformed version of the src image, using the
			///quadrangles defined by src_quad and localmap_quad
			void setTransformedVisual(IplImage* src, CvPoint2D32f* src_quad, CvPoint2D32f* localmap_quad, double rotation = 0.0);
			
			///set the Visual to a perspective transformed version of the src image, using the
			///a trapezoid defined by foot and trap_width, and a region of the src image whose rows
			///are below the horizon (measured form the top of the image down)
			void setTransformedVisual(IplImage* src, int horizon, int foot, int trap_width, double rotation = 0.0);
			
			///override the existing occupancy localmap with a single-channel image
			void setOccupancy(IplImage* src);
			
			///AND a single-channel localmap with the existing occupancy image
			void addOccupancy(IplImage* src);
			
			///rotate the visual about the specified point by the specified angle in degrees
			void rotateVisual(double degrees, CvPoint2D32f center);
			
			///get a pointer to the visual localmap image.
			///@warning The returned pointer will go out of scope when the localmap is
			///deinstantiated
			IplImage* getVisual();
			
			///get a pointer to the occupancy localmap image.
			///@warning The returned pointer will go out of scope when the localmap is
			///deinstantiated
			IplImage* getOccupancy();
			
			IplImage* getDriveability();
			
			///get a pointer to the distance transform image.
			///@warning The returned pointer will go out of scope when the localmap is
			///deinstantiated
			IplImage* getDistTransform();
			
			///get a pointer to the display image.
			///@warning The returned pointer will go out of scope when the localmap is
			///deinstantiated
			IplImage* getDisplayImg();
		
			///accessor gets the image width
			double getWidth() const;
			
			///accessor gets the image height
			double getHeight() const;
			
			CvSize getImgSize() const;
			
			CvPoint getOrigin() const;
			
			double getScale() const;
		
			///isolate a particular colour range from the visual image and ADD it to the
			///occupancy image
			void selectColour(double hue, double hue_tol, double sat_min);
		
			
			void setDisplayMode(int mode);
			
			CvPoint2D32f* getCalibrationRectangle(double width, double height, double range);
		
			
		
		protected:
		
			void drawGrid(IplImage* img, CvPoint origin, CvScalar color, double interval);
			void drawVehicle(CvPoint origin, const vehicle& v);
			void drawRect(IplImage* img, CvPoint2D32f* corners, CvScalar color);
			
			CvPoint2D32f pixel2scale(CvPoint p, CvPoint origin, double scale);
			CvPoint scale2Pixel(CvPoint2D32f p, CvPoint origin, double scale);
		
			
		
			double width;
			double height;
			double scale;  //pixels per unit
			
			CvSize img_size;
			
			CvPoint origin;
			
			int displayMode;
		
			///image buffers
			IplImage* visual;
			IplImage* occupancy;
			IplImage* driveability;
			IplImage* display;	
	};
	
	
	class pathFinder{
		public:
		
			pathFinder(CvPoint anchor, double radius, double maxcurvature, int curvecount = 7, int thickness = 3, int segments = 12, double centering = 0);
	
			double evaluate(IplImage* img);
	
			void draw(IplImage* img, CvScalar color);
	
	
		protected:
		
			double steering;
			double centering;
			int thickness;
			vector<curve> curves;
	};
	
	class curve{
		public:
			curve(CvPoint anchor, double radius, double curvature, int segments = 12);
			
			void drawCurve(IplImage* img, CvScalar color, int thickness);
			
			double overlap(IplImage* src, int thickness, IplImage* dst = NULL);
			
			void setCurvature(double c);
			double getCurvature();
			
		private:
		
			void refreshCurve(double s, double theta, int segments, double curvature);
		
			//regular n-gon segment length
			double s(double radius, int n);
		
			//circle radius at maximal curvature
			double radius;
			
			//angle between sides of inscribed regular n-gon
			double theta;
			
			//the angle at which each segment deviates from the previous one.
			double phi;
			
			//number of sides of 1/4 of inscribed n-gon = n/4
			int segments;
			
			//magnitude of curvature.
			//1.0 means the curve represents a quarter-circle to the right
			//-1.0 means the curve represents a quarter-circle to the left
			//0.0 means the curve is a straight line
			double curvature;
			
			//the actual points of the curve
			vector<CvPoint> nodes;
			
			//the bottom of the curve
			CvPoint origin;
			
	};




}

#endif
