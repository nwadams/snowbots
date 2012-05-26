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

#ifndef BLOBDETECTOR_H
#define BLOBDETECTOR_H

#include <math.h>
#include <vector>
#include "R.h"

#include "cv.h"
#include "highgui.h"

#include "measure_time.h"
#include "image_buffer.h"
#include "msr_util.h"
#include "visualise.h"

// Default settings
#define DEF_MARGIN 0.0015
#define DEF_TIMESTEPS 200
#define DEF_N8FLAG 0	// 1 for 8 neighbours instead of 4
#define DEF_NORMFL 1	// 1 for Chi2 edges instead of Euclidean
#define DEF_BLURFL 0	// 1 for image blur instead of edge blur
#define DEF_LOWRESFL 0	// Toggles 320x240 or 640x480
#define DEF_TIMESIZE 10 // default timestep size
#define DEF_CSPACE 0	// Toggles Y,Cr,Cb and RGB

#define MARGIN_MAX 100	// maximum blob margin
#define TIMESTEP_MAX 1000  //maximum timestep value

#define PATH_NODES 8	//number of nodes in the pathfinding curve
#define PATH_COUNT 7	//number of paths the pathfinder uses

//#define PI 3.1415926535

/**
 * @brief The mscr_blob namespace contains the classes, structs and functions necessary for use
 * of the MSCR blob detector.
 */
namespace mscr_blob{
	
	/**
 	 * @brief The blob struct contains the data necessary to describe a single blob.  It does
 	 * not contain information describing the Maximally Stable invariant Colour Region upon
 	 * which the blob is based, only an approximation written as an ellipse.
	 */
	struct blob{
		///blob centroid position
		CvPoint pos;
		
		///blob covariance ellipse eigenvalues
		double D[2];
		
		///blob covariance ellipse eigenvectors
		double E[4];
		
		///blob colour (represented as BGR, according to OpenCV convention)
		CvScalar color;	
	};
	
	
	/**
	 * @brief The blobdetector class provides an object that processes images to produce lists
	 * of MSCR blobs.  
	 *
	 * The key method is detectBlobs.  This method takes an OpenCV IplImage as input and returns 
	 * a list of blob structs.  detectBlobs is robust to changes in the size of the image. It 
	 * assumes it is receiving an 8-bit 3-channel colour image.  It can be configured with two 
	 * parameters: margin and timestep, using the setMargin and setTimestep methods.  
	 */
	class blobdetector{
		public:
			/**
			 * @brief Constructor: instantiates a new instance of the blobdetector class.
			 *
			 * Sets all the flags and parameters for the MSCR detector.
			 */
			blobdetector();
			
			/**
			 * @brief Destructor: deinstantiates the object.  (Currently unused)
			 */
			~blobdetector();
			
			/**
			 * @brief detectBlobs applies the MSCR feature detection algorithm to the provided
			 * IplImage and returns a list of the detected features as blob structs (bounding 
			 * ellipses).
			 *
			 * This method is a wrapper for the key functions of the MSCR detector, which was 
			 * originally written in C.  The critical function detect_and_render_blobs formed
			 * the basis for this code.
			 * @param img The image to be processed.  It should be an 8-bit, 3-channel IplImage,
			 * in RGB colour according to the OpenCV conventions.
			 * @return A vector of blob structs, each corresponding to a single MSCR in the image.
			 */
			std::vector<blob> detectblobs(IplImage* img);
			
			/**
			 * @brief This method can post-process the list of blobs returned by detectblobs to
			 * remove any blobs that are not sufficiently similar to the specified HSV colour.
			 *
			 * The intent of this method is to provide simple colour discrimination capabilities, 
			 * using the intuitive HSV colour space.  The user can specify a range of colours
			 * as a sector of the HSV colour wheel.  Only blobs whose mean colour falls in that
			 * sector will be returned.
			 * @param bloblist A vector of blob structs, presumably from detectblobs.
			 * @param color A CvScalar representing the desired filter colour in the HSV colour
			 * space.  Ranges of value are: H[0 359.9999999] S[0 1.0] V[0 1.0]
			 * @param margin A CvScalar representing the allowable deviation of colours in the
			 * H, S, and V colour dimensions.  Note that these values are bidirectional, so a
			 * desired value of 0.3 with a margin of 0.2 defines the range 0.1 - 0.5 as acceptable.
			 * Also note that the Hue dimension is an angle value in degrees and thus wraps around, 
			 * so a desired hue of 10 and a hue margin of 30 means that the ranges 0 - 40 and 
			 * 340 - 359.99999999 are all allowable.
			 * @return Returns the list of acceptable blobs, which may be empty.
			 */
			std::vector<blob> HSVFilterBlobs(const std::vector<blob>& bloblist, const CvScalar& color, const CvScalar& margin);
			
			CvPoint warp(CvPoint p, CvSize src_size, CvSize dst_size);
			
			std::vector<blob> UnwarpBlobs(const std::vector<blob>& bloblist, CvSize src_size, CvSize dst_size);
			
			/**
			 * @brief Draws a representation of the blobs on the provided image
			 *
			 * This function is provided to give a simple visualisation of the detected blobs.
			 * The blobs are rendered as an overlay of green ellipses on the provided image.  Each
			 * ellipse has an inner ellipse of the mean colour of the blob, which should be barely
			 * visible in most cases.
			 * @param bloblist The list of blobs to draw, presumably from detectblobs
			 * @param img The IplImage onto which the function should draw.
			 */
			void drawMarkers(std::vector<blob>& bloblist,IplImage* img);
			
			
			void drawPath(IplImage* img, double r, int ox, int oy, CvScalar color);
			
			
			/**
			 * @brief Converts a BGR CvScalar colour to an HSV CvScalar colour
			 *
			 & This method is used by the HSVFilterBlobs method and is prvided fro the user's 
			 * convenience in generating HSV values.
			 * @param colorBGR the colour value to be converted
			 * @return A CvScalar with the H,S and V values in the 0,1 and 2 indices, respectively.
			 */
			CvScalar BGR2HSV(const CvScalar& colorBGR);
			
			/**
			 * @brief Sets the value of the margin parameter of the MSCR algorithm.  A larger value
			 * of the margin parameter will produce fewer small blobs, favouring larger blobs.
			 * Recommended range [0 0.05] default 0.0015
			 * @param m the new margin value.
			 */
			void setMargin(double m);
			
			/**
			 * @brief Sets the value of the timestep parameter of the MSCR algorithm.
			 * Recommended range [0 1000], default 200
			 *
			 * The timestep parameter controls the level of persistence required for blobs to 
			 * be considered maximally stable colour regions.  The higher the timestep parameter,
			 * the longer a region must remain stable before it is returned by detectblobs.
			 * @param t the new margin value.
			 */
			void setTimestep(int t);
			
			/**
			 * @brief Accessor for the margin parameter.
			 * @return Returns the margin parameter
			 */
			double getMargin();
			
			/**
			 * @brief Accessor for the timestep parameter
			 * @return Returns the timestep parameter
			 */
			int getTimestep();
		
		protected:
		
			blob extract_ellipse(fpnum *mvec,unsigned char *pvec);
			std::vector<blob> extract_ellipses(bbuffer *bf_img,buffer *bf_mvec,bbuffer *bf_pvec);
		
			//trapezoidal warp
			CvPoint tWarp(CvPoint& p_rect, CvSize& rect_size, CvSize& trap_size, double delta);
			CvPoint tUnWarp(CvPoint& p_trap, CvSize& rect_size, CvSize& trap_size, double delta);
		
			//coordinate transforms
			CvPoint scrn2map(const CvPoint& ps,const CvSize& s);
			CvPoint map2scrn(const CvPoint& pm,const CvSize& s);
		
			void flip_image(IplImage *image1,IplImage *image2);
			void buffer_iplcopy(buffer *bf_img,IplImage *ipl_img,int flipfl);
			void upsample_image(IplImage *image1,IplImage *image2);
			void paste_image(IplImage *ipl_img,bbuffer *bf_img,int xoffset);	
	
			double tb_margin;
			int tb_timesteps;
			
			int kb_n8flag;		// 1 for 8 neighbours instead of 4
			int kb_normfl;		// 1 for Chi2 edges instead of Euclidean
			int kb_blurfl;		// 1 for image blur instead of edge blur
			int kb_lowresfl; 	// Toggles 320x240 or 640x480
			int kb_cspace;		// Toggles Y,Cr,Cb and RGB
	};

	class curve{
		public:
			curve(const CvPoint& anchor, double step, double steeringLimit);
		
			void setSteering(double s);
			double getSteering();
			
			double minDist(const CvPoint& p);
			double nearestDist(const CvPoint& p);
			
			double autoSteering(std::vector<blob>& blobList, double gain, int maxIterations = 10);
		
			double curveFit(std::vector<blob>& blobList, double gain, int maxIterations);
		
			double curveError(std::vector<blob>& blobList);
		
			void drawPath(IplImage* img, CvScalar color, int thickness = 1);
		
		protected:
		
			//double radius(double steering);
			//double incrementAngle(double steering, int increments);
			//CvPoint nextPerimeterPoint(CvPoint& prevPoint, double steering, double step, int increments, int index);
		
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
			
			double evaluate(std::vector<blob>& bloblist);
			
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
}

#endif
