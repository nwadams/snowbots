//Copyright (C) 2007 Matthew Baumann

#ifndef IMAGEOPS_H
#define IMAGEOPS_H

//opencv
#include <cv.h>
#include <highgui.h>

#include <iostream>
#include <vector>
#include <math.h>
#include <stdio.h>

#include "defs.h"
#include "util.h"

namespace imageops{
	
	///This simple structure represents a point in the image with subpixel precision
	struct point{
		double x;
		double y;
	};
	
	///This struct represents a line in the image using the normal-offset method
	struct line{
		double rho;		//the orthogonal distance from the line to the origin
		double theta;	//the angle between the line and the +x axis
	};
	
	///This struct represents a pair of lines, intended as the basic operating 
	///structure of the guidance program: nameley two lines, each representing
	///one side of the road and the related statistics.
	struct linepair{
		line l1;	//what was the mean of lines with positive slope?
		line l2;	//what was the mean of lines with negative slope?
		
		int l1_count;	//how many lines of positive slope were there?
		int l2_count;	//how many lines of negative slope were there?
		
		line l1_var;	//what was the variance of the line 1 parameters?
		line l2_var;	//what was the variance of the line 2 parameters?
	};
	
	
	///This struct encapsulates the position, normalized position and value
	///of a pixel that is at feature point in the image
	struct interestPoint{
		CvPoint pos;	//pixel coordinates
		point rel_pos;	//normalized coordinates
		double value;	//pixel value;
	};
	
	///This struct represents a feature by its location and size in the image
	struct feature{
		CvPoint pos;
		double scale;
	};
	
	class IplImagePyramid;

	///This function computes the intersection point of two lines
	CvPoint lineIntersect(const line& l1, const line& l2);
	
	///This function draws a line on the image in the specified colour using the 
	///OpenCV bresenham line drawing function
	void DrawLineOnImg(const line& l, IplImage* img, CvScalar color);
	
	///This function is a macro which draws a linepair (including a visualisation of the
	///line set variance) on an image.
	void DrawLinePairOnImg(const linepair* pr, IplImage* img);
	
	///This function writes the line pair information to a string for text output and logging
	void PrintLinePairIntoString(char* str, const linepair* pr);

	/**
	 * @brief isolateColor takes an IplImage as input and extracts all the pixels whose
	 * colour falls within the specified sector the the HSV colour space.
	 *
	 * Specifically, the dst array is filled with a copy of the src image, but any pixel
	 * whose hue is not within the tolerance of the specified hue, or whose saturation
	 * or value is below the thresholds is set to 0 (black), thus leaving only the
	 * specified colour range.
	 * @param src The source IplImage: should be an 3-channel, 8-bit BGR image.
	 * @param dst The destination IplImage: should be a 1-channel, 8-bit black & white image.
	 * @param hue The desired hue angle in degrees
	 * @param hue_tolerance THe maximum allowed hue deviation in degrees
	 * @param min_saturation The minimum allowable colour saturation
	 * @param min_val The minimum allowable value (brightness)
	 * @return Returns true on success, false if either array is NULL
	 */
	bool isolateColor(CvArr* src, CvArr* dst, double hue, double hue_tolerance, double min_saturation, double max_val, CvScalar fillval = cvScalarAll(0));
	
	/**
	 * @brief findTriangle uses the OpenCV hough transform to locate lines in a
	 * black and white image.  It summarises all found lines by partitioning them
	 * into those with positive or negative slope, and then returning the average
	 * of the positive lines and an average of the negative lines as a linepair struct.
	 *
	 * The purpose of this algorithm is to detect the roadside markers which appear in the 
	 * image as a series of features (white pixels) in roughly straight lines converging
	 * at a vanishing point in the distance.  Thus, the drivable road ahead of the robot
	 * can be represnted as a triangle whose two sides are the two lines of roadside features,
	 * and the third side is the bottom of the image.
	 * @param img The source image - should be a 1-channel 8-bit IplImage (e.g. from isolateColor)
	 * @param rho_res The resolution of the hough space in the rho dimension (the number of bins in the histogram - try 20-50)
	 * @param theta_res The resolution of the hough space in the theta dimension (the number of bins in the histogram try 20-50)
	 * @param threshold The minimum number of lines of a roadside features set to be considered to exist.
	 *  This will vary by the resolution and density of features.  Experiment with values less than the width of the image in pixels
	 */
	linepair* findTriangle(IplImage* img,double rho_res, double theta_res, int threshold);
	
	///A function used by findTriangle to invoke the OpenCV hough transform.
	std::vector<line> findLines(IplImage* img,double rho_res, double theta_res, int threshold);
	
	/**
	 * @brief Invokes the OpenCV template match function, but with a simplified interface.
	 * This function generates a correlation map of the template image over the source img.
	 * Depending on which method is used, the method may indicate a match with a locally
	 * maximal or locally minimal value.
	 * @param img The image to be searched for a match.
	 * @param templ THe template image : should be smalled than img and have the same number 
	 *  channels, the same bit depth, and colour space.
	 * @param method The corrlection metric to be used.  See the documentation on the OpenCV
	 *  template match functino for  list of options.
	 * @return returns the correlation map as a 1-channel IplImage.  Note that it will have
	 *  the dimensions img.width-templ.width+1, img.height-templ.height+1
	 */
	IplImage* templateMatch(IplImage* img, IplImage* templ, int method = CV_TM_SQDIFF_NORMED);

	/**
	 * @brief scans the result image form templateMatch and returns a vector of locations on the 
	 *  image that are local maxima or minim (greater or lesser value than all 8 neighbouring pixels)
	 * @param img The imge to be scanned.  SHould be a 1-channel 8-bit IplImage, such as one produced
	 *  by templateMatch.
	 * @param mode Determines whether the function looks for minima or maxima.  True = maximum, false = minimum
	 */
	std::vector<interestPoint> findLocalMinMax(IplImage* img, bool mode);
	
	
	void invertImage(IplImage* src, IplImage* dst);
	void invertImage(IplImage* src);
	void threshold(IplImage* src, int val);
	
	///ancillary predicate for findLocalMinMax, determines if a given pixel is greater in value
	/// than its 8 (or fewer) neighbours
	bool localMin(IplImage* img,int row, int col);
	
	///ancillary predicate for findLocalMinMax, determines if a given pixel is less in value
	/// than its 8 (or fewer) neighbours
	bool localMax(IplImage* img,int row, int col);
	
	/**
	 * @brief applies temaplte matching at all levels of an Image Pyramid, determining
	 * not only all features that match, but at which level the correlation is strongest,
	 * thus giving an estimate on the relative scale of the correlated feature in the
	 * image.
	 * @param pyr An IplPyramid generated from a source image.
	 * @param tmplt The template image : should have the smae channel count and bit depth
	 *  as pyr
	 * @return Returns a vector of features complete with their scale information.
	 */
	std::vector<feature> locateScaledFeatures(IplImagePyramid* pyr, IplImage* tmplt);

	///Checks to see if an angle is in the range [min-max].  The interval determines
	/// the wraparound offset of the number system.  Use 2*PI for radians or 360.0 for
	///degrees.
	bool angleInRange(double angle, double min, double max, double interval = CV_PI*2);

	/**
	 * @brief The IplImage pyramid is a aggregate of different-scaled images used
	 * for multi-scale template matching.
	 * 
	 * Unlike a gaussian or lapalacian pyramid, this type of pyramid's layers differ by
	 * a factor between 1 and 2.  Generally, each layer should be only about 10% bigger
	 * than the previous one, allowing relatively fine-grained discrimination between
	 * the correlation at different scales.  This allows the multi-scale matching
	 * to determine which scale correlates the most and return the approximate scale of the
	 * matched feature.
	 * 
	 * The IplImagePyramid itself i no more than a vector of pointers to IplImages, but the
	 * associated methods will generate the pyramid from a source image, provide information
	 * used to reconstruct the scale of correlates at a given layer, and release the layered
	 * images on deinstatiation.
	 */
	class IplImagePyramid : public std::vector<IplImage*>{
		public:
		
			///creates a new, empty image pyramid
			IplImagePyramid();
			
			///creates a new image pyramid based on the source image, with k
			///levels and a minimum level size of min_width
			IplImagePyramid(IplImage* src,int k,int min_width);
			
			///deinstantiates the pyramid, releasing all the levels' IplImages
			~IplImagePyramid();
			
			///dumps all data and re-creates the pyramid.  This effectively allows
			///the smae pyramid to be used for many successive images, such as
			///the frames of a video.
			void regenerate(IplImage* src,int k,int min_width);

			///returns the number of levels in the pyramid
			int levels() const;
			
			///releases all stored images
			void release();
	};
}

#endif
