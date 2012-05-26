#include "pathfinder.h"

using namespace std;

namespace vision{


pathFinder::pathFinder(int count, CvPoint anchor, double step, double steeringLimit){
	//create the curves
	
	double interval = steeringLimit * 2 / (double) (count-1);
	
	for(int i = 0; i < count; i++){
		curves.push_back(curve(anchor,step,steeringLimit));
		curves[i].setSteering(-steeringLimit + ( (double) i * interval) );
		penalties.push_back(0);
	}
	
	mainPath = new curve(anchor,step,steeringLimit);
	
	min_penalty = 0.0;
	max_penalty = 1.0;
}
		
pathFinder::~pathFinder(){
	delete mainPath;
}
		
/*double pathFinder::evaluate(std::vector<blob>& bloblist){
	int min = 0;
	double minval = DBL_MAX;
	double maxval = 0.0;
	
	
	for(int i = 0; i < (int) curves.size(); i++){
		penalties[i] = curves[i].curveError(bloblist);
		if(penalties[i] < minval){
			min = i;
			minval = penalties[i];
		}
		if(penalties[i] > minval){
			maxval = penalties[i];
		}
	}
	
	printf("penalties=[");
	for(int i = 0; i < (int) curves.size(); i++){
		printf(" %#1.3f,",penalties[i]);
	}
	printf(" ]\n");

	//if(minval < min_penalty){min_penalty = minval;}
	//if(maxval > max_penalty){max_penalty = maxval;}
	min_penalty = minval;
	max_penalty = maxval;
	
	
	double mix = 0.2;
	mainPath->setSteering(curves[min].getSteering() * mix + mainPath->getSteering() * (1.0-mix));
	
	return mainPath->getSteering();
	
}*/

/*double pathFinder::evaluate(IplImage* img, double radius, double mix){
	int min = 0;
	double minval = DBL_MAX;
	double maxval = 0.0;
	
	for(int i = 0; i < (int) curves.size(); i++){
		penalties[i] = curves[i].penalty(img,radius);
		if(penalties[i] < minval){
			min = i;
			minval = penalties[i];
		}
		if(penalties[i] > minval){
			maxval = penalties[i];
		}
	}
	
	printf("penalties=[");
	for(int i = 0; i < (int) curves.size(); i++){
		printf(" %#1.3f,",penalties[i]);
	}
	printf(" ]\n");
	
	mainPath->setSteering(curves[min].getSteering() * mix + mainPath->getSteering() * (1.0-mix));
	
	return mainPath->getSteering();
}*/

double pathFinder::evaluate(IplImage* img, double radius, double mix, double centering){
	int min = 0;
	double minval = DBL_MAX;
	double maxval = 0.0;
	
	CvScalar one_scalar = cvScalar(1.0);
	CvScalar thresh = cvScalar(radius*2);
	
	IplImage* trunc = cvCreateImage( cvGetSize(img),IPL_DEPTH_32F,1);
	IplImage* penalty = cvCreateImage( cvGetSize(img),IPL_DEPTH_32F,1);
	IplImage* curveimg = cvCreateImage( cvGetSize(img),IPL_DEPTH_32F,1);
	IplImage* prod = cvCreateImage( cvGetSize(img),IPL_DEPTH_32F,1);


	
	
	for(int i = 0; i < (int) curves.size(); i++){
		cvZero(trunc);
		cvZero(penalty);
		cvZero(curveimg);
		cvZero(prod);
		
		//create an image if the curve
		
		//perform a inverted saturation step on the image, as low distances
		//should get high penalties.
		
		cvThreshold(img, trunc, radius*2,radius*2, CV_THRESH_TRUNC);
		
		cvSubS( trunc, thresh, penalty);
		
		
		cvZero(curveimg);
		curves[i].drawPath(curveimg,CV_RGB(255,255,255),ceil(radius));
		
		//printf("\t\tdrew path\n");
		
		
		cvMul(penalty,curveimg,prod);
		
		//cvShowImage("util",img);
		//printf("\t\tmultiplied\n");
		
		
		
		CvScalar sum = cvSum(prod);
		//printf("\t\tsummed.  sum = %f\n",sum.val[0]);
		
		//compute an additiona penalty to tend the system towards auto-centering
		double centeringpenalty = centering * curves[i].getSteering() * curves[i].getSteering();
		
		penalties[i] = -sum.val[0] + centeringpenalty;
		if(penalties[i] < minval){
			min = i;
			minval = penalties[i];
		}
		if(penalties[i] > minval){
			maxval = penalties[i];
		}
		
		
	}
	
	cvReleaseImage(&curveimg);
	cvReleaseImage(&prod);
	cvReleaseImage(&penalty);
	cvReleaseImage(&trunc);
	
	printf("\tpenalties=[");
	for(int i = 0; i < (int) curves.size(); i++){
		printf(" %#1.3f,",penalties[i]);
	}
	printf(" ]\n");
	
	double min_significant_penalty = 100.0;
	if(maxval < min_significant_penalty){
		mainPath->setSteering(curves[min].getSteering() * mix);
	}
	else
	{
		mainPath->setSteering(curves[min].getSteering() * mix + mainPath->getSteering() * (1.0-mix));
	}
	
	return mainPath->getSteering();
}


void pathFinder::drawCurves(IplImage* img){
	for(int i = 0; i < (int) curves.size(); i++){
		//compute the colour
		double r = double_interpolate(penalties[i],
				min_penalty,(min_penalty+max_penalty)/2.0,max_penalty,
				0,255,255);
		double g = double_interpolate(penalties[i],
				min_penalty,(min_penalty+max_penalty)/2.0,max_penalty,
				255,255,0);
		
		curves[i].drawPath(img,CV_RGB((int) r,(int) g,0),1);
	}
}

void pathFinder::drawPath(IplImage* img){
	mainPath->drawPath(img,CV_RGB(0,255,255),3);
}



/************************************************************************/

void curve::drawPath(IplImage* img, CvScalar color, int thickness){
	for(int i = 0; i < PATH_NODES - 1; i++)
	{
		cvLine( img, nodes[i], nodes[i+1], color,thickness);
	}
}

curve::curve(const CvPoint& anchor, double step, double steeringLimit){
	this->anchor.x = anchor.x;
	this->anchor.y = anchor.y;
	this->step = step;
	this->steeringLimit = steeringLimit;
	steering = 0;
	refreshPath(steering);
}

/*double curve::curveError(std::vector<blob>& blobList){
	
	double d = 0;
	//for each blob
	for(int b = 0; b < (int) blobList.size(); b++){
		//get the distance, add to sum
		double md = fabs(minDist(blobList[b].pos));
		double distPenalty = saturation(md,0,50,100,0);
		double area = PI * sqrtf(blobList[b].D[0]) * sqrtf(blobList[b].D[1]);
		
		printf("blob %d, dist=%#1.3f, pen=%#1.3f, area=%#1.3f, prod=%#1.3f\n",b,md,distPenalty,area,area*distPenalty);
		
		d += (area*distPenalty);
		
	}
	return d;
}*/

double curve::penalty(IplImage* img,int distlimit){
	double d = 0;
	//for each pixel
	for(int row = 0; row < img->height; row++)
	{
		for(int col = 0; col < img->width; col++){
			
			//check if it is nonzero
			if( cvGet2D( img, col, row ).val[0] > 0)
			{
				//get the map coordinates of the pixel
				//CvPoint map_pos = cvPoint(col - img->width/2,img->height-row);
				
				CvPoint map_pos = cvPoint(col,row);
				double mindist = fabs(minDist(map_pos));
				double distPenalty = saturation(mindist,0,distlimit,100,0);
				d += distPenalty;
				//printf("pixel [%d,%d], map[%d,%d], dist=%#1.3f, pen=%#1.3f\n",col,row,map_pos.x,map_pos.y,mindist,distPenalty);
			}
			

		}
	}
	return d;
}

double curve::minDist(const CvPoint& p){
	double min = 100000.0;
	double dir = 1;
	//printf("curve dists: ");
	for(int i = 1; i < PATH_NODES; i++){
		double d = dist(p,nodes[i]);
		//printf(" p=[%d,%d] %#1.3f",nodes[i].x,nodes[i].y,d);
		if(d < min){
			min = d;
			p.x < nodes[i].x ? dir = 1 : dir = -1;
			//printf("p.x = %d, nodes[%d].x = %d, dir = %#1.1f\n",p.x,i,nodes[i].x,dir);
		}
		
	}
	//printf("\n");
	return min * dir;
}

double curve::nearestDist(const CvPoint& p){
	double min = DBL_MAX;
	for(int i = 1; i < PATH_NODES; i++){
		double d = dist(p,nodes[i]);
		if(d < min){
			min = d;
		}
	}
	return min;
}

void curve::setSteering(double s){
	steering = s;
	if(steering > steeringLimit){steering = steeringLimit;}
	if(steering < -steeringLimit){steering = -steeringLimit;}
	refreshPath(steering);
}

double curve::getSteering(){
	return steering;
}

bool nonzero(int x, int y, IplImage* img){
	//convert map coords to image coords:
	
	int cv_x = x + img->width/2;
	int cv_y = -y + img->height;
	
	CvScalar pixel = cvGet2D( img, cv_x, cv_y );
	
	return pixel.val[0] > 0;
}

void curve::refreshPath(double steering){
	
	double step_angle = steering / PATH_NODES;
	nodes[0].x = anchor.x;
	nodes[0].y = anchor.y; 
	
	double step_length = step * (2.0-fabs(steering)/steeringLimit);
	
	for(int i = 1; i < PATH_NODES; i++){
		nodes[i].x = nodes[i-1].x + (step_length * sin(i*step_angle));
		nodes[i].y = nodes[i-1].y - (step_length * cos(i*step_angle));
	}
}

double curve::dist(const CvPoint& p1, const CvPoint& p2){
	double dx = (double) p1.x - (double) p2.x;
	double dy = (double) p1.y - (double) p2.y;
	return sqrtf( dx*dx + dy*dy );
}

double curve::dot(const CvPoint& p1, const CvPoint& p2){
	return p1.x*p2.x + p1.y*p2.y;
}

CvPoint curve::normalise(const CvPoint& p){
	double norm = sqrt(p.x * p.x + p.y * p.y);
	return cvPoint(p.x / norm , p.y / norm);
}

/************************************************************************************/

double interpolate(double x, double x0, double x1, double y0, double y1){
	return y0 + (x-x0)*(y1-y0)/(x1-x0);
}

double double_interpolate(double x, double x0, double x1, double x2, double y0, double y1, double y2){
	if(x < x1){
		return interpolate(x,x0,x1,y0,y1);
	}
	else{
		return interpolate(x,x1,x2,y1,y2);
	}
}

double saturation(double x, double x0, double x1, double y0, double y1){
	if(x < x0){return y0;}
	else if(x > x1){return y1;}
	else{return interpolate(x,x0,x1,y0,y1);}
}

void saturation(IplImage* src, IplImage* dst,double x0, double x1, double y0, double y1){
  int height,width,step,channels;
  uchar *src_data, *dst_data;
  int i,j,k;
  
  // get the image data
  height    = src->height;
  width     = src->width;
  step      = src->widthStep;
  channels  = src->nChannels;
  src_data      = (uchar *)src->imageData;
  dst_data      = (uchar *)dst->imageData;

  // invert the image
  for(i=0;i<height;i++){
  	for(j=0;j<width;j++){ 
		for(k=0;k<channels;k++){
    		dst_data[i*step+j*channels+k] = saturation(src_data[i*step+j*channels+k],x0,x1,y0,y1);	
    		//printf("dist: %#1.3f, penalty = %#1.3f\n",src_data[i*step+j*channels+k],dst_data[i*step+j*channels+k]);
    	}
   	}
  }
}

}
