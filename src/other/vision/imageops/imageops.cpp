//Copyright (C) 2007 Matthew Baumann
#include "imageops.h"

using namespace util;

namespace imageops{

CvPoint lineIntersect(const line& l1, const line& l2)
{
	//intersection algorithm from:
	//http://local.wasp.uwa.edu.au/~pbourke/geometry/lineline2d/

	if(l1.theta == l2.theta)
	{
		//lines are parallel
		CvPoint p;
		p.x = 0;
		p.y = 0;
		return p;
	}

	double x1 = cos(l1.theta)*l1.rho;
	double y1 = sin(l1.theta)*l1.rho;
	
	double x2 = x1 + -sin(l1.theta);
	double y2 = y1 + cos(l1.theta);
	
	double x3 = cos(l2.theta)*l2.rho;
	double y3 = sin(l2.theta)*l2.rho;
	
	double x4 = x3 + -sin(l2.theta);
	double y4 = y3 + cos(l2.theta);
	
	double ua = ( (x4-x3)*(y1-y3) - (y4-y3)*(x1-x3) )/( (y4-y3)*(x2-x1) - (x4-x3)*(y2-y1) );
	
	CvPoint isect;
	isect.x = x1+ua*(x2-x1);
	isect.y = y1+ua*(y2-y1);
	
	return isect;
}

void DrawLineOnImg(const line& l, IplImage* img, CvScalar color)
{
	CvPoint pt1, pt2;
	double a = cos(l.theta), b = sin(l.theta);
	double x0 = a*l.rho, y0 = b*l.rho;
	pt1.x = cvRound(x0 + 1000*(-b));
	pt1.y = cvRound(y0 + 1000*(a));
	pt2.x = cvRound(x0 - 1000*(-b));
	pt2.y = cvRound(y0 - 1000*(a));
	//cvLine( img, pt1, pt2, CV_RGB(0,0,255), 3, 8 );
	cvLine( img, pt1, pt2, color, 3, 8 );
}

void DrawLinePairOnImg(const linepair* pr, IplImage* img)
{
	//if the data is valid
	if(pr != NULL && img != NULL)
	{
		//if there are left lines..
		if(pr->l1_count > 0)
		{	
			//draw the mean
			DrawLineOnImg(pr->l1, img, CV_RGB(0,0,255));
		}
		
		if(pr->l2_count > 0)
		{
			//draw the mean
			DrawLineOnImg(pr->l2, img, CV_RGB(0,0,255));
		}
		
		if(pr->l1_count > 0 && pr->l2_count > 0)
		{
			//Draw the InterSection point
			CvPoint p = lineIntersect(pr->l1,pr->l2);
			CvPoint bdc;
			bdc.x = img->width / 2;
			bdc.y = img->height;
			cvLine( img, p, bdc, CV_RGB(0,255,0), 3, 8 );
			
		}
	}
}


void PrintLinePairIntoString(char* str, const linepair* pr)
{
	sprintf(str,"1:[%f,%f,%d,%f,%f] 2:[%f,%f,%d,%f,%f]",
		pr->l1.rho,pr->l1.theta,pr->l1_count,pr->l1_var.rho,pr->l1_var.theta,
		pr->l2.rho,pr->l2.theta,pr->l2_count,pr->l2_var.rho,pr->l2_var.theta);
}


bool isolateColor(CvArr* src, CvArr* dst, double hue, double hue_tolerance, double min_saturation, double max_val, CvScalar fillval)
//extract a binary image representing all pixels that are within a tolerance of the specified hue
//and has greater than the specified saturation
{
	if(NULL == src)
	{
		std::cout << "Error: isolateColor : src is NULL" << std::endl;
		return false;
	}
	
	if(NULL == dst)
	{
		std::cout << "Error: isolateColor : dst is NULL" << std::endl;
		return false;
	}
	
	//find the image sizes
	CvSize src_size = cvGetSize(src);
	CvSize dst_size = cvGetSize(dst);
	
	if(src_size.width != dst_size.width || src_size.height != dst_size.height)
	{
		std::cout << "Error: isolateColor : src and dst image sizes do not match.\n" << std::endl;
		return false;
	}
	
	//create an HSV thresholded version of the image
	IplImage* hsvimg =  cvCreateImage( cvSize(src_size.width,src_size.height), 8, 3 );
	cvCvtColor(src,hsvimg,CV_BGR2HSV);
	
	for(int i = 0; i < src_size.height; i++)
	{
		for(int j = 0; j < src_size.width; j++)
		{
			CvScalar o;
			o=cvGet2D(src,i,j);
			if(o.val[0] == 0 && o.val[1] == 0 && o.val[2] == 0){
				cvSet2D(dst,i,j,fillval);
			}
			else
			{
			
				CvScalar s;
				s=cvGet2D(hsvimg,i,j); // get the (i,j) pixel value
				double p_hue = s.val[0];
				double p_saturation = s.val[1];
				double p_intensity = s.val[2];
				
				double diff = util::anglediff_d(hue,p_hue);
				//if(diff > hue_tolerance){
					//printf("diff = %f, hue = %f, p_hue %f\n",diff,hue,p_hue);
				//}
				
				CvScalar colorness;
				colorness.val[0]=0;
				
				//if(diff < hue_tolerance && p_saturation > min_saturation)
				if( p_saturation > min_saturation && diff < hue_tolerance)
				{
					//printf("hue = %f diff = %f thresh = %f\n",p_hue,diff,hue_tolerance);
					
					colorness.val[0]=max_val;
				}
	
				cvSet2D(dst,i,j,colorness);
			}
		}
	}
	
	cvReleaseImage( &hsvimg );
	
	return true;
}


linepair* findTriangle(IplImage* img,double rho_res, double theta_res, int threshold)
{
	//detect lines in the image using the Hough Transform

	CvSeq* lines1 = 0;
	CvMemStorage* lines1_storage = cvCreateMemStorage(0);  
	lines1 = cvHoughLines2( img, lines1_storage, CV_HOUGH_STANDARD, rho_res, theta_res, threshold, 0, 0 );
	
	float L_rho = 0;		//average rho (y-intercept) value for positive-slope lines
	float L_theta = 0;		//average theta (angle) value for positive-slope lines
	int L_count = 0;		//number of positive-slope lines
	float L_rho_var = 0;	//variance of rho parameter
	float L_theta_var = 0;	//variance of theta parameter
	
	float R_rho = 0;		//same as above but for negative-sloped lines.
	float R_theta = 0;
	int R_count = 0;
	float R_rho_var = 0;
	float R_theta_var = 0;
  
  	//consider the lines in the image
  	//partition them into positive and negative sloped lines and compute the means
  	//of the those two clusters
  	for(int i = 0; i < MIN(lines1->total,100); i++ )
	{
		float* line = (float*)cvGetSeqElem(lines1,i);
		float rho = line[0];
		float theta = line[1];
		
		if(theta > 0 && theta <= CV_PI/2)
		{
			//left line (+ve slope)
			L_rho += rho;
			L_theta += theta;
			L_count++;
		
		}
		else if(theta > CV_PI/2 && theta < CV_PI)
		{
			//right line (-ve slope)
			R_rho += rho;
			R_theta += theta;
			R_count++;
		}
	}
	
	//compute the averages for the left and right lines and draw them in:
	if(L_count > 0)
	{
		L_rho = L_rho / (float) L_count;
		L_theta = L_theta / (float) L_count;
	}
	else
	{
		L_rho = 0;
		L_theta = 0; 
	}
	
	if(R_count > 0)
	{
		R_rho = R_rho / (float) R_count;
		R_theta = R_theta / (float) R_count;
	}
	else
	{
		R_rho = 0;
		R_theta = 0;
	}
	
	
	if(L_count > 0 || R_count > 0)
	{
		//compute the variances:
		for(int j = 0; j < MIN(lines1->total,100); j++ )
		{
			float* line = (float*)cvGetSeqElem(lines1,j);
			float rho = line[0];
			float theta = line[1];
			
			if(L_count > 0 && theta > 0 && theta <= CV_PI/2)
			{
				//left line variance (+ve slope)
				L_rho_var += (L_rho - rho) * (L_rho - rho);
				L_theta_var += (L_theta - theta) * (L_theta - theta);
			
			}
			else if(R_count > 0&& theta > CV_PI/2 && theta < CV_PI)
			{
				//right line variance (-ve slope)
				R_rho_var += (R_rho - rho) * (R_rho - rho);
				R_theta_var += (R_theta - theta) * (R_theta - theta);
			}
		}
		
		
		//normalize the variances:
		if(L_count > 0)
		{
			L_rho_var /= (float) L_count;
			L_theta_var /= (float) L_count;
		}
		
		if(R_count > 0)
		{
			R_rho_var /= (float) R_count;
			R_theta_var /= (float) R_count;
		}
		
	}

	//fill in the linepair struct;	
	linepair* pr = new linepair;
	
	//left line stats
	pr->l1.rho = L_rho;
	pr->l1.theta = L_theta;
	pr->l1_count = L_count;
	pr->l1_var.rho = L_rho_var;
	pr->l1_var.theta = L_theta_var;
	
	//right line stats
	pr->l2.rho = R_rho;
	pr->l2.theta = R_theta;
	pr->l2_count = R_count;
	pr->l2_var.rho = R_rho_var;
	pr->l2_var.theta = R_theta_var;
	
	cvReleaseMemStorage(&lines1_storage);
	
	return pr;
}

std::vector<line> findLines(IplImage* img,double rho_res, double theta_res, int threshold)
{
	std::vector<line> lineList;
	
	CvSeq* lines1 = 0;
	CvMemStorage* lines1_storage = cvCreateMemStorage(0);  
	lines1 = cvHoughLines2( img, lines1_storage, CV_HOUGH_STANDARD, rho_res, theta_res, threshold, 0, 0 );

	//consider the lines in the image
  	//partition them into positive and negative sloped lines and compute the means
  	//of the those two clusters
  	for(int i = 0; i < MIN(lines1->total,100); i++ )
	{
		float* lineVal = (float*)cvGetSeqElem(lines1,i);
		line L;
		
		L.rho = lineVal[0];
		L.theta = lineVal[1];
		
		lineList.push_back(L);
	}

	cvReleaseMemStorage(&lines1_storage);

	return lineList;
}

IplImage* templateMatch(IplImage* img, IplImage* templ, int method){
	
	int iwidth = img->width - templ->width + 1; 
	int iheight = img->height - templ->height + 1;
	IplImage* result = cvCreateImage(cvSize(iwidth,iheight),32,1);
	cvMatchTemplate( img, templ, result, method); 
	cvNormalize(result,result,1,0,CV_MINMAX);
	IplImage* one = cvCreateImage(cvSize(iwidth,iheight),32,1);
	cvSet(one, cvScalar(1.0,1.0,1.0,1.0));
	cvAbsDiff(one,result,result);
	return result;
}

std::vector<interestPoint> findLocalMinMax(IplImage* img, bool mode){
//mode = true = max
//mode = false = min
	
	std::vector<interestPoint> result;
	
	
	
	if(img == NULL)
	{
		return result;
	}
	
	//for each pixel
	for(int col = 0; col< img->width; col++)
	{
		for(int row = 0; row < img->height; row++)
		{
			bool minimum = true;
			
			CvScalar centerval = cvGet2D(img,row,col);	//get current pixel using row, column coord order
			//std::cout << "centerval = " << centerval << std::endl;
			
			//if it's greater than its neighbours...
			bool peak = mode ? localMax(img,row,col) : localMin(img,row,col);
			
			if(peak){
				//create a new interestpoint
				interestPoint p;
				p.pos.x = col;
				p.pos.y = row;
				p.rel_pos.x = (double) col / (double) img->width;
				p.rel_pos.y = (double) row / (double) img->height;
				p.value = centerval.val[0];
			
				//add the interestpoint to the list
				result.push_back(p);
			}
		}
	}
	return result;
}

bool localMin(IplImage* img,int row, int col){
	CvScalar centerval = cvGet2D(img,row,col);

	//std::cout << "[" << col <<","<< row <<"] = " << centerval.val[0] << " | ";

	for(int r = clampIndex(row-1,0,img->height-1); r <= clampIndex(row+1,0,img->height-1); r++){
		for(int c = clampIndex(col-1,0,img->width-1); c <= clampIndex(col+1,0,img->width-1); c++){
			CvScalar neighbourval = cvGet2D(img,r,c);

			if(!(row == r && col == c))
			{
				//std::cout << "["<<c<<","<<r<<"]" <<neighbourval.val[0] << " ";
				if(neighbourval.val[0] <= centerval.val[0]){
					//std::cout << std::endl;
					return false;
				}
			}
		}
	}
	//std::cout << " MINIMUM" << std::endl;
	return true;
}


bool localMax(IplImage* img,int row, int col){
	CvScalar centerval = cvGet2D(img,row,col);

	//std::cout << "[" << col <<","<< row <<"] = " << centerval.val[0] << " | ";

	for(int r = clampIndex(row-1,0,img->height-1); r <= clampIndex(row+1,0,img->height-1); r++){
		for(int c = clampIndex(col-1,0,img->width-1); c <= clampIndex(col+1,0,img->width-1); c++){
			CvScalar neighbourval = cvGet2D(img,r,c);

			if(!(row == r && col == c))
			{
				//std::cout << "["<<c<<","<<r<<"]" <<neighbourval.val[0] << " ";
				if(neighbourval.val[0] >= centerval.val[0]){
					//std::cout << std::endl;
					return false;
				}
			}
		}
	}
	//std::cout << " MINIMUM" << std::endl;
	return true;
}

void invertImage(IplImage* src, IplImage* dst){

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
    		dst_data[i*step+j*channels+k] = 255-src_data[i*step+j*channels+k];
    		//printf("255 - %d = %d\n",src_data[i*step+j*channels+k],dst_data[i*step+j*channels+k]);
    	}
   	}
  }
}

void invertImage(IplImage* src){
	int height,width,step,channels;
  uchar *src_data;
  int i,j,k;
  
  // get the image data
  height    = src->height;
  width     = src->width;
  step      = src->widthStep;
  channels  = src->nChannels;
  src_data      = (uchar *)src->imageData;

  // invert the image
  for(i=0;i<height;i++){
  	for(j=0;j<width;j++){ 
		for(k=0;k<channels;k++){
    		src_data[i*step+j*channels+k] = 255-src_data[i*step+j*channels+k];
    		//printf("255 - %d = %d\n",src_data[i*step+j*channels+k],dst_data[i*step+j*channels+k]);
    	}
   	}
  }
}

void threshold(IplImage* src, int val){
	int height,width,step,channels;
  uchar *src_data;
  int i,j,k;
  
  // get the image data
  height    = src->height;
  width     = src->width;
  step      = src->widthStep;
  channels  = src->nChannels;
  src_data      = (uchar *)src->imageData;

  // invert the image
  for(i=0;i<height;i++){
  	for(j=0;j<width;j++){ 
		for(k=0;k<channels;k++){
    		if(src_data[i*step+j*channels+k] < val)
    			{src_data[i*step+j*channels+k] = 0;}
    		else
    			{src_data[i*step+j*channels+k] = 255;}
    		
    	}
   	}
  }
}

std::vector<feature> locateScaledFeatures(IplImagePyramid* pyr, IplImage* tmplt){
	
	std::vector<feature> featureList;
	
	if(pyr->size() <=0){return featureList;}
	
	//get the local minima of the top-level image
	std::vector<interestPoint> ip = findLocalMinMax(pyr->at(0), true);
	
	for(int i = 0; i < ip.size(); i++)
	{
		int level = 0;
		//check the pixel in the lower-res images corresponding to each response s,
		//recording the lowest level in which a response is detected.
		while(level < pyr->levels()){
			int x = (int) (ip[i].rel_pos.x * (double) pyr->at(level)->width);
			int y = (int) (ip[i].rel_pos.y * (double) pyr->at(level)->height);
			
			if(!localMax(pyr->at(level),y,x))
			{
				break;
			}
		
			level++;
		}
		//create a list of the response with the corresponding scale coeff, if an
		//identical one does not already exist
		
		feature f;
		f.pos.x = ip[i].rel_pos.x * pyr->at(0)->width + floor(tmplt->width / 2);
		f.pos.y = ip[i].rel_pos.y * pyr->at(0)->height + floor(tmplt->height / 2);
		f.scale = (double) pyr->at(0)->width / (double) pyr->at(level)->width;
		
		featureList.push_back(f);

	}

	//return the list of responses.
	return featureList;
}

bool angleInRange(double angle, double min, double max, double interval)
{
	//first get the angle into the range
	double a = angle;
	if(a < 0)
	{
		while(a < 0) {a += interval;}
	}
	else if(a > 0)
	{
		while(a > 0) {a -= interval;}
		a += interval;
	}
	
	return (a >= min && a <= max);
}

//-----------------------------------------------------------------------------

IplImagePyramid::IplImagePyramid(){
	
}

IplImagePyramid::IplImagePyramid(IplImage* src,int k, int min_width){
	regenerate(src,k,min_width);
}

IplImagePyramid::~IplImagePyramid(){
	release();
}

void IplImagePyramid::regenerate(IplImage* src,int k,int min_width){
	release();
	int width_range = src->width - min_width;
	double aspect = (double) src->width / (double) src->height;
	for(int i = k-1; i >=0 ; i--)
	{
		int w = floor( (double) (i+1) / (double) k * (double) width_range  );
		int h = (int) ( (double) w / aspect );
		//std::cout << width_range << " ["<<w<<","<<h<<"]" << std::endl;
		IplImage* img = cvCreateImage(cvSize(w,h),src->depth,src->nChannels);
		cvResize(src,img);
		push_back(img);
	}
}

int IplImagePyramid::levels() const{
	return (int) size();
}

void IplImagePyramid::release(){
	for(int i = 0; i < size(); i++)
	{
		cvReleaseImage(&at(i));
	}
	clear();
}

//-----------------------------------------------------------------------------


}


