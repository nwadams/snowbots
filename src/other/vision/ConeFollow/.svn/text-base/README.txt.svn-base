ConeFollow:

author: Matthew Baumann - mabauman@cs.ubc.ca

Purpose: to detect two lines of (orange) cones in an image using openCV.  The idea is that a 
a racecourse lined with cones appears (approximately) as two rows of cones receding from the 
bottom corners of the image to a point in the upper center.  By detecting these two rows and 
computing their convergence point, the robot can choose a heading to travel between the rows
of cones.

main.cpp is a driver which tries to detect a camera, and if one is found, displays the image
from the camera in a glut-powered window, overlaying the detected lines in blue if any are
present.

the most important functions are in imageops.h, imageops.cpp, particularly:

- bool isolateColor(CvArr* src, CvArr* dst, double hue, double hue_tolerance, double min_saturation, double max_val);
	This function takes a source image "src" and wirtes a binary image into "dst".  The
	binary image has a value of 1 if the source image has a corresponding pixel of a 
	colour whose saturation exceeds "min_saturation" and the pixel's hue is within 
	"hue_tolerance" of "hue".  So it finds highly saturated instances of a particular 
	colour. 
	
- linepair* findTriangle(IplImage* img,double rho_res, double theta_res, int threshold);
	This function take an IplImage (binary, grayscale), and performs a line-detecting 
	Hough transform on it usgin openCV's HoughLines function.  The function returns a 
	linepair if there exists a line with positive slope in the image, AND a line with 
	negative slope.  Multiple positive or negative sloped lines are averaged for output.
	