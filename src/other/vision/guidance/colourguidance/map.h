
#ifndef VISION_MAP_H
#define VISION_MAP_H


#include <iostream>
#include <string>

#include <cv.h>
#include <highgui.h>

#include <imageops.h>

#include "pathfinder.h"

namespace vision{
	
	class map{
	
		public:
		
			///create a new map
			map(int width, int height, double sampleradius, int pathCount = 7, double steeringLimit = 1.0);
			
			///release all map data
			~map();
		
			///clear the map of all data
			void clear();
			
			///set the visual (colour) map image
			void setVisual(IplImage* src);
			
			///set the Visual to a perspective transformed version of the src image, using the
			///quadrangles defined by src_quad and map_quad
			void setTransformedVisual(IplImage* src, CvPoint2D32f* src_quad, CvPoint2D32f* map_quad, double rotation = 0.0);
			
			///set the Visual to a perspective transformed version of the src image, using the
			///a trapezoid defined by foot and trap_width, and a region of the src image whose rows
			///are below the horizon (measured form the top of the image down)
			void setTransformedVisual(IplImage* src, int horizon, int foot, int trap_width, double rotation = 0.0);
			
			///override the existing occupancy map with a single-channel image
			void setOccupancy(IplImage* src);
			
			///AND a single-channel map with the existing occupancy image
			void addOccupancy(IplImage* src);
			
			///get a pointer to the visual map image.
			///@warning The returned pointer will go out of scope when the map is
			///deinstantiated
			IplImage* getVisual();
			
			///get a pointer to the occupancy map image.
			///@warning The returned pointer will go out of scope when the map is
			///deinstantiated
			IplImage* getOccupancy();
			
			IplImage* getDriveability();
			
			///get a pointer to the distance transform image.
			///@warning The returned pointer will go out of scope when the map is
			///deinstantiated
			IplImage* getDistTransform();
			
			///get a pointer to the display image.
			///@warning The returned pointer will go out of scope when the map is
			///deinstantiated
			IplImage* getDisplayImg();
		
			///accessor gets the image width
			int getWidth() const;
			
			///accessor gets the image height
			int getHeight() const;
			
			int getGap();
			void setGap(int gap);
			
			CvPoint getAnchor(int x_offset, int y_offset) const;
		
			///isolate a particular colour range from the visual image and ADD it to the
			///occupancy image
			void selectColour(double hue, double hue_tol, double sat_min);
			
			///use the pathfinder to compute a path through the occupancy map
			double getSteering(double mix, double centering);
			
			///use the pathfinder to compute a path through the occupancy map,
			///also filling the reference parameter confidence with a value representing
			///the ratio of occupied pixels, giving a general sense of the intensity of
			///the pathfinder's level of available data.
			double getSteering(double mix, double centering, double& confidence);
		
		protected:
		
			void floodOut(IplImage* img, IplImage* dst, int mingap);
			void floodIn(IplImage* img, IplImage* dst, int mingap);
		
			///computes a distance transform from the current occupancy image
			void computeDistTransform();
		
			///pathFinder AI module will analyse the map to determine the best course
			pathFinder* PF;
		
			int width;
			int height;
			
			int gap;
			
			double sampleradius;
		
			///image buffers
			IplImage* visual;
			IplImage* occupancy;
			IplImage* driveability;
			IplImage* disttransform;
			IplImage* display;
			
			///variable denotes if there is an up-to-date distance transform
			bool dt_up_to_date;
	
	
	};

}

#endif
