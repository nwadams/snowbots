
#include "localmap.h"

namespace vision{

localmap::localmap(CvSize img_size, double width, double height, CvPoint origin){	
	
	cout << "Instantiating localmap" << endl;
	
	this->img_size.width = img_size.width;
	this->img_size.height = img_size.height;
	
	this->width = width;
	this->height = height;
	
	this->origin.x = origin.x;
	this->origin.y = origin.y;
	
	scale = (double) img_size.width / width;
	cout << "  scale = " << scale << endl;
	
	displayMode = 0;
	
	//instantiate image buffers
	visual = cvCreateImage(img_size,8,3);
	occupancy = cvCreateImage(img_size,8,1);
	driveability = cvCreateImage(img_size,8,1);
	display = cvCreateImage(img_size,8,3);

}

localmap::~localmap(){
	
	//release the buffered images
	cvReleaseImage(&visual);
	cvReleaseImage(&occupancy);
	cvReleaseImage(&driveability);
	cvReleaseImage(&display);

}

void localmap::clear(){

	cvZero(visual);
	cvZero(occupancy);
	cvZero(driveability);
	cvZero(display);

}

void localmap::setVisual(IplImage* src){
	cvResize(src,visual);
}

void localmap::setTransformedVisual(IplImage* src, CvPoint2D32f* src_quad, CvPoint2D32f* map_quad, double rotation){
	
	//create a 3x3 matrix to store the perspective homography
	CvMat* transform = cvCreateMat( 3, 3, CV_32FC1 );
	//cout << "declared transform" << endl;
	
	//create rotation matrix
	CvMat* rot_matrix = cvCreateMat( 2, 3, CV_32FC1 );
	CvPoint2D32f center;
	center.x = img_size.width/2;
	center.y = img_size.height;
	//cout << "declared rotation matrix" << endl;
	
	IplImage* buffer = cvCreateImage(img_size,8,3);
	
	//compute the perspective homography
	cvGetPerspectiveTransform( src_quad, map_quad, transform );
	//cout << "computed transform" << endl;
	
	if(rotation == 0.0)
	{
		//if there's no rotation, just perform the warp
		cvWarpPerspective( src, visual, transform, CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS);
		//cout << "performed warp, rotation = 0" << endl;
	}
	else
	{
		//otherwise, rotate the image after it's transformed
		
		cvWarpPerspective( src, buffer, transform, CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS);
		//cout << "performed warp" << endl;
		
		//compute the matrix
		cv2DRotationMatrix(center,rotation,1.0,rot_matrix);
		//cout << "computed rotation matrix" << endl;
		
		//perform the rotation
		cvWarpAffine(buffer,visual,rot_matrix);
		//cout << "performed affine warp" << endl;
		
		//release the temporary storage
		//cout << "performed warp, rotation != 0" << endl;
	}
	
	cvReleaseMat( &rot_matrix);
	cvReleaseImage(&buffer);
	cvReleaseMat(&transform);
}

void localmap::setTransformedVisual(IplImage* src, int horizon, int foot, int trap_width, double rotation){

	CvPoint2D32f src_quad[4];
	
	src_quad[0].x = 0;	//top left
	src_quad[0].y = horizon;
	
	src_quad[1].x = src->width-1;	//top right
	src_quad[1].y = horizon;
	
	src_quad[2].x = src->width-1;	//bottom right
	src_quad[2].y = (src->height-1);
	
	src_quad[3].x = 0;	//bottom left
	src_quad[3].y = (src->height-1);
	
	//compute the map quadrangle
	CvPoint2D32f map_quad[4];
	
	map_quad[0].x = 0; //top left
	map_quad[0].y = 0;
	
	map_quad[1].x = width-1;		//top right
	map_quad[1].y = 0;
	
	map_quad[2].x = (width / 2) + (trap_width/2);	//bottom right
	map_quad[2].y = height-1 - foot;
	
	map_quad[3].x = (width / 2) - (trap_width/2);	//bottom left
	map_quad[3].y = height-1 - foot;
	
	setTransformedVisual(src,src_quad,map_quad,rotation);
	
}

void localmap::setOccupancy(IplImage* src){
	cvCopy(src,occupancy);
}

void localmap::addOccupancy(IplImage* src){
	IplImage* buffer = cvCreateImage(img_size,8,1);
	cvMax(src,occupancy,buffer);
	cvCopy(buffer,occupancy);
	cvReleaseImage(&buffer);
}


void localmap::rotateVisual(double degrees, CvPoint2D32f center){
	IplImage* buffer = cvCloneImage(visual);
	CvMat* rot_matrix = cvCreateMat( 2, 3, CV_32FC1 );
	
	//compute the matrix
	cv2DRotationMatrix(center,degrees,1.0,rot_matrix);
	//cout << "computed rotation matrix" << endl;
	
	//perform the rotation
	cvWarpAffine(buffer,visual,rot_matrix);
	
	//release the matrix and the buffer
	cvReleaseMat(&rot_matrix);
	cvReleaseImage(&buffer);
	
}

IplImage* localmap::getVisual(){
	return visual;
}

IplImage* localmap::getOccupancy(){
	return occupancy;
}

IplImage* localmap::getDriveability(){
	return driveability;
}

IplImage* localmap::getDisplayImg(){

	switch(displayMode){
		default:
		case 0:
			cvCopy(visual,display);
		break;
		case 1:
			cvCvtColor(occupancy,display,CV_GRAY2BGR);
		break;
		case 2:
			cvCvtColor(driveability,display,CV_GRAY2BGR);
		break;
	}
	
	drawGrid(display,origin,CV_RGB(0,63,0),0.25);
	drawGrid(display,origin,CV_RGB(0,127,0),1.0);
	drawRect(display,getCalibrationRectangle(0.164,0.066,0.4),CV_RGB(255,255,0));
	
	return display;
}


CvPoint localmap::getOrigin() const{
	return origin;
}

double localmap::getScale() const{
	return scale;
}

double localmap::getWidth() const{
	return width;
}
double localmap::getHeight() const{
	return height;
}

void localmap::selectColour(double hue, double hue_tol, double sat_min){
	IplImage* buffer = cvCreateImage(cvGetSize(visual),8,1);
	//create an image of pixels that are of the specified colour
	imageops::isolateColor(visual,buffer,hue,hue_tol,sat_min,255,cvScalarAll(255));
	//AND the filtered pixels to the occupancy map
	addOccupancy(buffer);
	//release the buffer
	cvReleaseImage(&buffer);
}


void localmap::setDisplayMode(int mode){
	displayMode = mode;
}


CvPoint2D32f* localmap::getCalibrationRectangle(double width, double height, double range){
	CvPoint2D32f* rect = new CvPoint2D32f[4];
	
	rect[0].x = (double) origin.x + scale*width/2.0;
	rect[0].y = (double) origin.y + scale*(-range + height/2.0);
	
	rect[1].x = (double) origin.x - scale*width/2.0;
	rect[1].y = (double) origin.y + scale*(-range + height/2.0);
	
	rect[2].x = (double) origin.x + scale*width/2.0;
	rect[2].y = (double) origin.y + scale*(-range - height/2.0);
	
	rect[3].x = (double) origin.x - scale*width/2.0;
	rect[3].y = (double) origin.y + scale*(-range - height/2.0);
	
	
	
	
	
	return rect;
}



void localmap::drawGrid(IplImage* img, CvPoint origin, CvScalar color, double interval){
	//cout << "drawing lines" << endl;
	
	int step = (int) (interval * scale);
	//cout << "interval " << interval << " * scale " << scale <<" = step " << step << endl;
	
	//draw origin
	cvCircle( img, origin, 6, color,-1);
	//drawRect( img, getCalibrationRectangle(0.30,0.25,0.60), CV_RGB(255,255,0));
	
	//draw horizontal lines
	for(int row = origin.y; row < img->height-1; row += step){
		cvLine(img,cvPoint(0,row),cvPoint(img->width-1,row),color);
		//cout << "Line: y = " << row << endl;
	}
	for(int row = origin.y-step; row >= 0; row -= step){
		cvLine(img,cvPoint(0,row),cvPoint(img->width-1,row),color);
		//cout << "Line: y = " << row << endl;
	}
	
	//draw vertical lines
	for(int col = origin.x; col < img->width-1; col += step){
		cvLine(img,cvPoint(col,0),cvPoint(col,img->height-1),color);
		//cout << "Line: x = " << col << endl;
	}
	for(int col = origin.x-step; col >= 0; col -= step){
		cvLine(img,cvPoint(col,0),cvPoint(col,img->height-1),color);
		//cout << "Line: x = " << col << endl;
	}
	
}

void localmap::drawRect(IplImage* img, CvPoint2D32f* corners, CvScalar color){
	cvRectangle(img, cvPoint(corners[0].x,corners[0].y), cvPoint(corners[2].x,corners[2].y), color,1);
}

/****************************************************************************/


pathFinder::pathFinder(CvPoint anchor, double radius, double maxcurvature, int curvecount, int thickness, int segments, double centering){
	//cout << "Instantiating pathfinder" << endl;
	double step = 2.0 * maxcurvature / double (curvecount - 1 );
	
	for(int c = 0; c < curvecount; c++){
		//cout << "Instantiating curve " << c << " (" << radius << "," << -maxcurvature + (c * step) << "," << segments << ")" << endl;
		curves.push_back(curve(anchor,radius,-maxcurvature + (c * step),segments));
	}
	this->centering = centering;
	this->thickness = thickness;
	steering = 0;
}
	
double pathFinder::evaluate(IplImage* img){
	
	double min = DBL_MAX;
	int min_index = 0;
	
	cout << "Penalties [ ";
	
	//for each curve
	for(int c = 0; c < (int) curves.size(); c++){
	
		//compute the centering penalty
		double centeringpenalty = centering * curves[c].getCurvature() * curves[c].getCurvature();
		
		//compute the image penalty
		double imgpenalty = curves[c].overlap(img,thickness,NULL) / 255;
		
		//check if this is the minimum
		double penalty = centeringpenalty + imgpenalty;
		cout << penalty << " " ;
		
		if(penalty < min){
			min = penalty;
			min_index = c;
		}
	}
	cout << endl;
	
	//return the minimum
	return curves[min_index].getCurvature();
}

void pathFinder::draw(IplImage* img, CvScalar color){
	//for each curve
	for(int c = 0; c < (int) curves.size(); c++){
		curves[c].drawCurve(img,color,1);
	
	}
}

/****************************************************************************/

curve::curve(CvPoint anchor, double radius, double curvature, int segments){
	this->radius = radius;
			
	//angle between sides of inscribed regular n-gon
	double n = 4*segments;
	//theta = (PI * (n-2))/n;
	theta = (1-2/n)*PI;
	phi = PI/2.0 - theta/2.0;

	this->segments = segments;
	
	//magnitude of curvature.
	//1.0 means the curve represents a quarter-circle to the right
	//-1.0 means the curve represents a quarter-circle to the left
	//0.0 means the curve is a straight line
	this->curvature = curvature;
	
	//the actual points of the curve
	for(int i = 0; i < segments+1; i++){
		nodes.push_back(cvPoint(0,0));
	}
	
	//the bottom of the curve
	origin.x = anchor.x;
	origin.y = anchor.y;
	
	refreshCurve(s(radius,n),theta,segments,curvature);
}

void curve::drawCurve(IplImage* img, CvScalar color, int thickness){
	for(int i = 0; i < segments; i++){
		cvLine( img, nodes[i], nodes[i+1], color, thickness);
	}
}

double curve::overlap(IplImage* src, int thickness, IplImage* dst){
	//create an image of the curve to serve as a mask
	IplImage* curve = cvCreateImage(cvGetSize(src),8,1);
	drawCurve(curve,cvScalarAll(255),thickness);
	
	//create a buffer to store the result
	IplImage* buffer = cvCreateImage(cvGetSize(src),8,1);
	
	//copy the src pixels into the buffer IF they correspond to 
	//pixels in the curve.  (Curve is the mask);
	cvCopy(src,buffer,curve);
	
	//if a destination array provided, fill it with the buffer contents
	if(dst != NULL){
		cvCopy(buffer,dst);
	}
	
	double sum = cvSum(buffer).val[0];
	
	cvReleaseImage(&curve);
	cvReleaseImage(&buffer);
	
	//return the sum of all pixels in the buffer
	return sum;
}

void curve::setCurvature(double c){
	curvature = c;
	refreshCurve(s(radius,segments*4),theta,segments,curvature);
}

double curve::getCurvature(){
	return curvature;
}


void curve::refreshCurve(double s, double theta, int segments, double curvature){
	nodes[0].x = origin.x;
	nodes[0].y = origin.y;
	
	//cout << "node " << 0 << " ["<<nodes[0].x<<","<<nodes[0].y<<"]" << endl;
	
	for(int i = 0; i < segments; i++){
		//compute segment angle (note curvature varies between -1 and 1 to 
		//control actual resultant angle)
		
		
		//double theta_i = ((double) i + 0.5) * theta * curvature;
		//cout << "theta_i = " << theta_i << " = (" << i << "+0.5) * " << theta << " * " << curvature << endl; 
		double phi_i = (1 + 2*i) * phi * curvature;
		
		
		//compute displacement from current node to the next one
		//int dx = (int) (s * cos(theta_i));
		//int dy = (int) -(s * sin(theta_i));
		int dx = (int) (s * sin(phi_i));
		int dy = (int) -(s * cos(phi_i));
		
		//compute the next nodes position
		nodes[i+1].x = nodes[i].x + dx;
		nodes[i+1].y = nodes[i].y + dy;
		
		//cout << "node " << i+1 << " ["<<nodes[i+1].x<<","<<nodes[i+1].y<<"], phi = " << phi_i << endl;
	}
}

double curve::s(double radius, int n){
	return 2.0*radius*sin(PI/(double) n);
}



}
