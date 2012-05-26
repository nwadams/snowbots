
#include "map.h"

namespace vision{

map::map(int width, int height, double sampleradius, int pathCount, double steeringLimit){
	this->width = width;
	this->height = height;
	this->sampleradius = sampleradius;
	
	gap = this->width/4;
	
	//instantiate image buffers
	visual = cvCreateImage(cvSize(width,height),8,3);
	occupancy = cvCreateImage(cvSize(width,height),8,1);
	driveability = cvCreateImage(cvSize(width,height),8,1);
	disttransform = cvCreateImage(cvSize(width,height),IPL_DEPTH_32F,1);
	display = cvCreateImage(cvSize(width,height),8,3);
	
	//create a pathFinder
	PF = new pathFinder(pathCount, getAnchor(0,0), height/15, steeringLimit);
	
	dt_up_to_date = false;
}

map::~map(){
	
	//release the buffered images
	cvReleaseImage(&visual);
	cvReleaseImage(&occupancy);
	cvReleaseImage(&driveability);
	cvReleaseImage(&disttransform);
	cvReleaseImage(&display);
	
	//release the pathFinder
	delete PF;
}

void map::clear(){

	cvZero(visual);
	cvZero(occupancy);
	cvZero(driveability);
	cvZero(disttransform);
	cvZero(display);
	
	dt_up_to_date = false;
}

void map::setVisual(IplImage* src){
	cvCopy(src,visual);
}

void map::setTransformedVisual(IplImage* src, CvPoint2D32f* src_quad, CvPoint2D32f* map_quad, double rotation){
	
	//create a 3x3 matrix to store the perspective homography
	CvMat* transform = cvCreateMat( 3, 3, CV_32FC1 );
	//cout << "declared transform" << endl;
	
	//create rotation matrix
	CvMat* rot_matrix = cvCreateMat( 2, 3, CV_32FC1 );
	CvPoint2D32f center;
	center.x = width/2;
	center.y = height;
	//cout << "declared rotation matrix" << endl;
	
	IplImage* buffer = cvCreateImage(cvSize(width,height),8,3);
	
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

void map::setTransformedVisual(IplImage* src, int horizon, int foot, int trap_width, double rotation){

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

void map::setOccupancy(IplImage* src){
	cvCopy(src,occupancy);
	dt_up_to_date = false;
}

void map::addOccupancy(IplImage* src){
	IplImage* buffer = cvCreateImage(cvSize(width,height),8,1);
	cvMax(src,occupancy,buffer);
	cvCopy(buffer,occupancy);
	cvReleaseImage(&buffer);
	dt_up_to_date = false;
}

IplImage* map::getVisual(){
	return visual;
}

IplImage* map::getOccupancy(){
	return occupancy;
}

IplImage* map::getDriveability(){
	return driveability;
}

IplImage* map::getDistTransform(){
	computeDistTransform();
	return disttransform;
}

IplImage* map::getDisplayImg(){
	return display;
}

int map::getWidth() const{
	return width;
}

int map::getHeight() const{
	return height;
}

int map::getGap(){return gap;}
void map::setGap(int gap){this->gap=gap;}

CvPoint map::getAnchor(int x_offset, int y_offset) const{
	return cvPoint(width/2 + x_offset, height + y_offset);
}

void map::selectColour(double hue, double hue_tol, double sat_min){
	IplImage* buffer = cvCreateImage(cvSize(width,height),8,1);
	//create an image of pixels that are of the specified colour
	imageops::isolateColor(visual,buffer,hue,hue_tol,sat_min,255,cvScalarAll(255));
	//AND the filtered pixels to the occupancy map
	addOccupancy(buffer);
	//release the buffer
	cvReleaseImage(&buffer);
}

double map::getSteering(double mix, double centering){
	//printf("\tcomputing distance transform\n");
	computeDistTransform();
	//printf("\tcopying to display\n");
	cvCopy(visual,display,driveability);
	//printf("\tevaluating paths\n");
	double steering = PF->evaluate(disttransform,sampleradius,mix,centering);
	//printf("\twrite them onto the display image\n");
	PF->drawCurves(display);
	PF->drawPath(display);
	//printf("\tdone\n");
	return steering;
}

double map::getSteering(double mix, double centering, double& confidence){
	double steering = getSteering(mix,centering);
	//printf("\tcounting nonzero occupancy pixels\n");
	int nonzero = cvCountNonZero(occupancy);
	confidence = (double) nonzero / (double) (width*height);
	//printf("\t %d nonzero pixels / %d = %#1.3f\n",nonzero,width*height,confidence);
	return steering;
}

void map::floodOut(IplImage* src, IplImage* dst, int mingap){
	//assumes that the images are both 8-bit, single-channel images of the same size
		
	int height    = src->height;
  	int width     = src->width;
  	int step      = src->widthStep;
  	int channels  = src->nChannels;
	
	 uchar* src_data = (uchar *)src->imageData;
	 uchar* dst_data = (uchar *)dst->imageData;
	
	int right = src->width-1;
	int left = 0;
	
	//from the bottom of the image to the top
	for(int r = src->height-1; r >= 0; r--)
	{
		//from the centerline right, find the first occupied pixel
		
		for(int i = src->width/2; i < src->width; i++){
			uchar val_r = src_data[r*step+i];
			if(val_r > 0){
				if(i < right) {right = i; }
				break;
			}
		}
		
		//from the centerline left
		
		for(int i = src->width/2; i >=0 ; i--){
			uchar val_r = src_data[r*step+i];
			if(val_r > 0){
				if(i > left){left = i;}
				break;
			}
		}
		
		if(right-left > mingap){
			//if it's wide enough, fill out from the edges
			for(int i = left; i >=0 ; i--){
				dst_data[r*step+i] = 255;
			}
			for(int i = right; i < dst->width ; i++){
				dst_data[r*step+i] = 255;
			}
			
		}
		else if(right - left > 1){
			//if it's not wide enough, fill out between those points
			for(int i = left; i <= right; i++){
				dst_data[r*step+i] = 255;
			}
		}
		else{
			for(int i = 0; i < width; i++){
				dst_data[r*step+i] = 255;
			}
		}
	
	}
}

void map::floodIn(IplImage* src, IplImage* dst, int mingap){
	
	cvCopy(src,dst);
	
	int height    = src->height;
  	int width     = src->width;
  	int step      = src->widthStep;
  	int channels  = src->nChannels;
	
	 uchar* src_data = (uchar *)src->imageData;
	 uchar* dst_data = (uchar *)dst->imageData;
	
	int right = src->width-1;
	int left = 0;
	
	//for each row from the bottom
	for(int row = src->height-1; row >= 0; row--)
	{
		//for each pixel from the right
		int rcount = 0;
		int lastright = src->width-1;
		for(int i = src->width-1; i >= 0; i--){
			//get the value
			uchar val_r = src_data[row*step+i];
			if(val_r > 0){
				rcount = 0;
				lastright = i;
				}
			else{
				rcount++;
			}	
			if(rcount > mingap){
				right = lastright;
				break;
			}
			
		}
		
		//for each pixel from the left
		int lcount = 0;
		int lastleft = 0;
		for(int i = 0; i < src->width; i++){
			//get the value
			uchar val_r = src_data[row*step+i];
			if(val_r > 0){
				lcount = 0;
				lastleft = i;
			}
			else{
				lcount++;
			}
			if(lcount > mingap){
				left = lastleft;
				break;
			}
			
		}
		
		//fill left of left and right of right
		for(int i = left; i >=0 ; i--){
				dst_data[row*step+i] = 255;
			}
		for(int i = right; i < dst->width ; i++){
				dst_data[row*step+i] = 255;
			}
		
	}
}

void map::computeDistTransform(){
	if(!dt_up_to_date){
		floodIn(occupancy,driveability,gap);
		IplImage* occ_inv = cvCreateImage(cvSize(width,height),8,1);
		imageops::invertImage(driveability,occ_inv);
		imageops::threshold(occ_inv, 200);
		
		cvZero(disttransform);
		cvDistTransform(occ_inv, disttransform);
		
		cvReleaseImage(&occ_inv);
	}
}


}
