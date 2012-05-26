#include "imageOps.h"

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


bool isolateColor(CvArr* src, CvArr* dst, double hue, double hue_tolerance, double min_saturation, double max_val)
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

	return lineList;
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

}


