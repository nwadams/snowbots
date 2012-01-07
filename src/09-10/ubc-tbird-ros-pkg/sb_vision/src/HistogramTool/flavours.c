#include "flavours.h"
#include <time.h>

// some cool functions: cvGetSize(IplImage*)
// just realize: when R and B channel are treated conversely, the program see as much blue as there is red.


void generateHSVSpace(IplImage* someImg)
// memory has to be passed into the function
// width has to be a multiple of 180, height has to be a multiple of
{
	int step=someImg->widthStep;
	int channel=someImg->nChannels;
	uchar* someImgData=(uchar*) someImg->imageData;
	double scale_x=someImg->width/180.0;
	double scale_y=someImg->height/255.0;
	int i,j;
	int ii,jj;

	for(j=0;j<255;j++)
	{
		for(i=0;i<180;i++)
		{
			for(ii=i*scale_x;ii<(i+1)*scale_x;ii++)
				for(jj=j*scale_y;jj<(j+1)*scale_y;jj++)
				{
					someImgData[jj*step+ii*channel]=i;
					someImgData[jj*step+ii*channel+1]=j;
					someImgData[jj*step+ii*channel+2]=255;
				}
		}
	}
	cvCvtColor(someImg,someImg,CV_HSV2BGR);

	cvNamedWindow("interesting!",1);
	cvShowImage("interesting!",someImg);
	cvSaveImage(HSV_IMAGE,someImg,0);
	cvReleaseImage(&someImg);
}


void getPureRed(IplImage* justImg, IplImage* returnImg)
// Method: HSV colourspace
// justImg: 3 channel, memory allocated
// returnImg: 1 channel, memory allocated
{
	cvCvtColor(justImg,justImg,CV_BGR2HSV);
	cvZero(returnImg);
	int step=returnImg->widthStep;
	int channel=returnImg->nChannels;
	int stepJ=justImg->widthStep;
	int channelJ=justImg->nChannels;
	int width=returnImg->width;
	int height=returnImg->height;
	uchar* justImgData=(uchar*) justImg->imageData;
	uchar* returnImgData=(uchar*) returnImg->imageData;
	int i,j;

	for(j=0;j<height;j++)
	{
		for(i=0;i<width;i++)
		{
			if(justImgData[j*stepJ+i*channelJ]<30 || justImgData[j*stepJ+i*channelJ]>150)
				returnImgData[j*step+i*channel]=255;
		}
	}

	cvCvtColor(justImg,justImg,CV_HSV2BGR);
}

void getTrafficLight(IplImage* justImg, IplImage* returnImg)
// Method: HSV colourspace, find white spots
// justImg: 3 channel, memory allocated
// returnImg: 1 channel, memory allocated
// comment: need to summarize methodology of isolating color features (we are isolating color features not the object!!)
{
	cvCvtColor(justImg,justImg,CV_BGR2HSV);
	cvZero(returnImg);
	int step=returnImg->widthStep;
	int channel=returnImg->nChannels;
	int stepJ=justImg->widthStep;
	int channelJ=justImg->nChannels;
	int width=returnImg->width;
	int height=returnImg->height;
	uchar* justImgData=(uchar*) justImg->imageData;
	uchar* returnImgData=(uchar*) returnImg->imageData;
	int i,j;

	for(j=0;j<height;j++)
	{
		for(i=0;i<width;i++)
		{
			//if(justImgData[j*stepJ+i*channelJ+1]>200 && justImgData[j*stepJ+i*channelJ+2]>230)
			//if(justImgData[j*stepJ+i*channelJ+1]<50)
			if(justImgData[j*stepJ+i*channelJ+1]>200 && justImgData[j*stepJ+i*channelJ+2]>200)
				returnImgData[j*step+i*channel]=255;
		}
	}

	cvCvtColor(justImg,justImg,CV_HSV2BGR);
}


void useHSVHistogram_cone(IplImage* justImg, IplImage* returnImg)
// only one parameter, returnImg is a single channeled image with the same dimension as the original image
// get most process done within the current scope
{
	IplImage* h_plane = cvCreateImage( cvGetSize(justImg), 8, 1 );
	IplImage* s_plane = cvCreateImage( cvGetSize(justImg), 8, 1 );
	IplImage* v_plane = cvCreateImage( cvGetSize(justImg), 8, 1 );
	IplImage* planes[] = { h_plane, s_plane };
	IplImage* hsv = cvCreateImage( cvGetSize(justImg), 8, 3 );
	int h_bins = 30, s_bins = 32;
	int hist_size[] = {h_bins, s_bins};
	// hue varies from 0 (˜0 deg red) to 180 (˜360 deg red again) 
	float h_ranges[] = { 0, 180 };
	// saturation varies from 0 (black-gray-white) to
	//255 (pure spectrum color)
	float s_ranges[] = { 0, 255 };
	float* ranges[] = { h_ranges, s_ranges };
	int scale = 10;
	IplImage* hist_img = cvCreateImage( cvSize(h_bins*scale,s_bins*scale), 8, 3 );
	CvHistogram* hist;
	float max_value = 0;
	int h, s;
	cvCvtColor( justImg, hsv, CV_BGR2HSV );
	cvCvtPixToPlane( hsv, h_plane, s_plane, v_plane, 0 );
	hist = cvCreateHist( 2, hist_size, CV_HIST_ARRAY, ranges, 1 );
	cvCalcHist( planes, hist, 0, 0 );



	// clear the non-orange color
	for(h=0;h<30;h++)
	{
		for(s=0;s<30;s++)
		{
			if(h>2 || s<29)
			{
				*cvGetHistValue_2D(hist,h,s)=0;
			}
		}
	}

	//cvNormalizeHist(hist,1.0);
	cvGetMinMaxHistValue( hist, 0, &max_value, 0, 0 );
	cvZero( hist_img );
	for( h = 0; h < h_bins; h++ )
	{
		for( s = 0; s < s_bins; s++ )
		{
			float bin_val = cvQueryHistValue_2D( hist, h, s );
			int intensity = cvRound(bin_val*255/max_value);
			cvRectangle( hist_img, cvPoint( h*scale, s*scale ),
			cvPoint( (h+1)*scale - 1, (s+1)*scale - 1),
			CV_RGB(intensity,intensity,intensity),
			CV_FILLED );
		}
	}

	cvCalcBackProject(&planes,returnImg,hist);

	cvNamedWindow("graph it !",1);
	cvShowImage("graph it !",hist_img);

	cvReleaseImage(&hist_img);
	cvReleaseImage(&h_plane);
	cvReleaseImage(&s_plane);
	cvReleaseImage(&v_plane);
	cvReleaseImage(&hsv);
	cvWaitKey(1);

	return;
}

void useHSVHistogram(IplImage* justImg, IplImage* returnImg)
// only one parameter, returnImg is a single channeled image with the same dimension as the original image
// get most process done within the current scope
{
	IplImage* h_plane = cvCreateImage( cvGetSize(justImg), 8, 1 );
	IplImage* s_plane = cvCreateImage( cvGetSize(justImg), 8, 1 );
	IplImage* v_plane = cvCreateImage( cvGetSize(justImg), 8, 1 );
	IplImage* planes[] = { h_plane, s_plane };
	IplImage* hsv = cvCreateImage( cvGetSize(justImg), 8, 3 );
	int h_bins = 30, s_bins = 32;
	int hist_size[] = {h_bins, s_bins};
	// hue varies from 0 (˜0 deg red) to 180 (˜360 deg red again) 
	float h_ranges[] = { 0, 180 };
	// saturation varies from 0 (black-gray-white) to
	//255 (pure spectrum color) 
	float s_ranges[] = { 0, 255 };
	float* ranges[] = { h_ranges, s_ranges };
	int scale = 10;
	IplImage* hist_img = cvCreateImage( cvSize(h_bins*scale,s_bins*scale), 8, 3 );
	CvHistogram* hist;
	float max_value = 0;
	int h, s;
	cvCvtColor( justImg, hsv, CV_BGR2HSV );
	cvCvtPixToPlane( hsv, h_plane, s_plane, v_plane, 0 );
	hist = cvCreateHist( 2, hist_size, CV_HIST_ARRAY, ranges, 1 );
	cvCalcHist( planes, hist, 0, 0 );


	//cvNormalizeHist(hist,1.0);
	cvGetMinMaxHistValue( hist, 0, &max_value, 0, 0 );
	cvZero( hist_img );
	for( h = 0; h < h_bins; h++ )
	{
		for( s = 0; s < s_bins; s++ )
		{
			float bin_val = cvQueryHistValue_2D( hist, h, s );
			int intensity = cvRound(bin_val*255/max_value);
			cvRectangle( hist_img, cvPoint( h*scale, s*scale ),
			cvPoint( (h+1)*scale - 1, (s+1)*scale - 1),
			CV_RGB(intensity,intensity,intensity),
			CV_FILLED );
		}
	}


	cvNamedWindow("graph it !",1);
	cvShowImage("graph it !",hist_img);

	cvReleaseImage(&hist_img);
	cvReleaseImage(&h_plane);
	cvReleaseImage(&s_plane);
	cvReleaseImage(&v_plane);
	cvReleaseImage(&hsv);
	cvWaitKey(1);

	return;
}

/* the wrong version... donno why it doesn't work..
#include "flavours.h"

// some cool functions: cvGetSize(IplImage*)

CvHistogram* findHSVHistogram(IplImage* justImg)
// only on parameter
{
	IplImage* hsv = cvCreateImage(cvGetSize(justImg),8,3);
	IplImage* h_plane = cvCreateImage(cvGetSize(justImg),8,1);
	IplImage* s_plane = cvCreateImage(cvGetSize(justImg),8,1);
	IplImage* v_plane = cvCreateImage(cvGetSize(justImg),8,1);
	IplImage* planes[] = {h_plane,s_plane};

	// assume that we have RGB images.. what if BGR??
	cvCvtColor(justImg,hsv,CV_BGR2HSV);
	cvCvtPixToPlane(justImg,h_plane,s_plane,v_plane,0);

	cvNamedWindow("hsv",1);
	cvShowImage("hsv",hsv);


	int h_bins=30, s_bins=32;
	CvHistogram* hist;
	{
		int hist_size[] = { h_bins, s_bins };
		float h_ranges[] = { 0, 180 }; // hue is [0,180]
		float s_ranges[] = { 0, 255 };
		float* ranges[] = { h_ranges, s_ranges };
		hist = cvCreateHist(2,hist_size,CV_HIST_ARRAY,ranges,1);
	}
	cvCalcHist( planes, hist, 0, 0 ); //Compute histogram
	//cvNormalizeHist( hist, 1.0 ); //Normalize it

	int scale=10;
	IplImage* graph=cvCreateImage(cvSize(h_bins*scale,s_bins*scale),8,1);
	cvZero(graph);

	float max_value;
	cvGetMinMaxHistValue(hist,0,&max_value,0,0);

	int h=0,s=0;
	int intensity=0;
	for(h=0;h<h_bins;h++)
	{
		for(s=0;s<s_bins;s++)
		{
			intensity=cvQueryHistValue_2D(hist,h,s);
			intensity=intensity*255/max_value;
			cvRectangle(graph,cvPoint(h,s),cvPoint(h+scale,s+scale),CV_RGB(intensity,intensity,intensity),CV_FILLED,8,0);
		}
	}

	cvNamedWindow("graph it !",1);
	cvShowImage("graph it !",graph);
	cvReleaseImage(&graph);
	cvReleaseImage(&h_plane);
	cvReleaseImage(&s_plane);
	cvReleaseImage(&v_plane);
	cvReleaseImage(&hsv);
	cvWaitKey(1);

	return hist;
}
*/