
#include "templateMatch.h"

using namespace imageops;

namespace vision{

//---------------------------------------------------------------------

imagePyramid::imagePyramid(IplImage* src, int maxwidth, int minwidth, int levels){
	this->maxwidth = maxwidth;
	this->minwidth = minwidth;
	
	aspect = (double) src->width / (double) src->height;

	for(int i = levels-1; i >= 0; i--){
	
		int width = minwidth + (int) i* ( (double) (maxwidth-minwidth)/ (double) (levels-1) );
		int height = (int) ( (double) width / aspect);
	
		
	
		IplImage* newimg = cvCreateImage(cvSize(width,height),8,3);
		cvResize(src,newimg);
		images.push_back(newimg);
		
		//printf("image level %d : %dx%d, scale = %#1.4f\n",(int) images.size()-1,width,height,scale(images.size()-1));
	}
}

imagePyramid::imagePyramid(imagePyramid& P, CvSize templateSize){
	
	int l = P.levels();
	maxwidth = P[0]->width - templateSize.width +1;
	minwidth = P[l-1]->width - templateSize.width +1;
	
	aspect - P.getAspect();
	
	for(int i = 0; i < l; i++){
		IplImage* newimg = cvCreateImage(cvSize(P[i]->width - templateSize.width + 1,P[i]->height - templateSize.height + 1),IPL_DEPTH_32F,1);
		images.push_back(newimg);
		
		//printf("image level %d : %dx%d, scale = %#1.4f\n",i,newimg->width,newimg->height,scale(i));
	}
	
}

imagePyramid::~imagePyramid(){
	for(int i = 0; i < (int) images.size(); i++){
		IplImage* temp = images[i];
		cvReleaseImage(&temp);
	}
}
			
int imagePyramid::levels() const{
	return (int) images.size();
}

double imagePyramid::getAspect(){
	return aspect;
}

IplImage* imagePyramid::getImage(int level, IplImage* dst){
	
	if(level < 0 || level >= (int) images.size()){
		return NULL;
	}
	
	if(dst != NULL){
		cvResize(images[level],dst);
	}
	
	return images[level];
}

IplImage* imagePyramid::operator[](int level){
	return getImage(level,NULL);
}

double imagePyramid::scale(int level){
	if(level < 0 || level >= (int) images.size()){return -1;}
	
	return (double) images[level]->width/ (double) maxwidth;
}

double imagePyramid::findMinimum(int& level, CvPoint& pos, double& val){
	double min_val = DBL_MAX;
	int min_level = 0;
	CvPoint min_pos;
	
	for(int l = 0; l < (int) images.size(); l++){
		CvScalar s;
		for( int i = 0 ; i < images[l]->height ; i++) {
			for( int j = 0 ; j < images[l]->width ; j++) {
				/* get an element */
				s = cvGet2D( images[l], i, j );
				
				//printf("level %d pixel [%d,%d] value %#1.4f\n",l,j,i,s.val[0]);
				
				if(s.val[0] < min_val){
					min_val = s.val[0];
					min_level = l;
					min_pos.x = j;
					min_pos.y = i;
				}
			}
		}
	}
	
	level = min_level;
	pos.x = min_pos.x;
	pos.y = min_pos.y;
	val = min_val;
	
	return min_val;
}

double imagePyramid::findMinimum(){
	int l;
	CvPoint p;
	double v;
	return findMinimum(l,p,v);
}

std::vector<CvPoint> imagePyramid::findLocalExtrema(IplImage* src, int mode, std::vector<double>& values){
//mode = true = max
//mode = false = min
	
	std::vector<CvPoint> result;
	
	
	
	if(src == NULL)
	{
		return result;
	}
	
	//for each pixel
	for(int col = 0; col< src->width; col++)
	{
		for(int row = 0; row < src->height; row++)
		{
			bool minimum = true;
			
			CvScalar centerval = cvGet2D(src,row,col);	//get current pixel using row, column coord order
			
			//if it's greater than its neighbours...
			bool peak = mode > 0 ? localMax(src,row,col) : localMin(src,row,col);
			
			if(peak){
				//create a new interestpoint
				CvPoint p = cvPoint(col,row);
			
				//add the interestpoint to the list
				result.push_back(p);
				values.push_back(centerval.val[0]);
			}
		}
	}
	return result;
}

//---------------------------------------------------------------------

}

