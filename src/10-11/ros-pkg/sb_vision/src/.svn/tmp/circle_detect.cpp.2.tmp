// circle_detect.cpp : Defines the entry point for the console application.


#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <iostream>

using namespace cv;
using namespace std;

//#define HSV
//#define CANNY //TODO: uncomment this line to see the canny of the input image

//declarations

int frame_per_sec = 30;
int thresh = 50;
IplImage* img = 0;
IplImage* img0 = 0;
CvMemStorage* storage = cvCreateMemStorage(0);
IplImage* canny = 0;
CvCapture* capture = 0;

char* names[] = { "img/1.jpg","img/2.jpg","img/3.jpg","img/4.jpg","img/5.jpg",
"img/6.jpg","img/7.jpg", "img/8.jpg", "img/97.png", "img/98.jpg", "img/99.jpg",
"img/95.jpg", "img/96.jpg" , 0 };

void find_circle(IplImage* img0, CvMemStorage* storage)
{
  int px[100], py[100];
  IplImage* src = 0;
  IplImage* gray = 0;
  IplImage* edge = 0;
  
  //storage = cvCreateMemStorage(0);
  //create a memory storage that will contain all the dynamic data

  src = cvCreateImage(cvSize(img0->width, img0->height), img0->depth, img0->nChannels);
  gray = cvCreateImage(cvSize(img0->width, img0->height), img0->depth, 1);
  edge = cvCreateImage(cvSize(img0->width, img0->height), img0->depth, 1);
  cvCopyImage(img0, src);
  
  //create window and a trackbar with parent "image" and set callback
	if(src->nChannels > 1)
	{
	  cvCvtColor(src, gray, CV_BGR2GRAY);
	  gray->origin = 1;
	} else
	{
		cvCopyImage(src, gray);
	}
  //color threshold
  cvThreshold(gray, gray, 100, 255, CV_THRESH_BINARY);
  
  //smooth the image to reduce unneccesary result
  cvSmooth(gray, gray, CV_GAUSSIAN, 11, 11);
  
  //get edges
  cvCanny(gray, edge, 0, thresh, 3);
  
  //get circle
  CvSeq* circles = cvHoughCircles( gray, storage, CV_HOUGH_GRADIENT, 1, 50, 5, 50); 
  //TODO: tweak this function
  
  //draw circles
  for(int i = 0; i < circles->total && i < 100; i++)
  {
    float* p = (float*)cvGetSeqElem(circles, i);
    cvCircle(src, cvPoint(cvRound(p[0]), cvRound(p[1])), 3, CV_RGB(0, 0, 255), -1, 8, 0);
    cvCircle(src, cvPoint(cvRound(p[0]), cvRound(p[1])), cvRound(p[2]), CV_RGB(0, 0, 255), 1, 8, 0);
    px[i] = cvRound(p[0]);
    py[i] = cvRound(p[1]);
  }
  
#ifdef CANNY
  canny = cvCreateImage(cvSize(img0->width, img0->height), img0->depth, 1);
  cvCvtColor(img0, canny, CV_RGB2GRAY);
  cvCanny(canny, canny, 0, thresh, 3);
  cvShowImage("canny", canny);
  cvReleaseImage(&canny);
#endif
  cvShowImage("circle", src);
  
  cvReleaseImage(&gray);
  cvReleaseImage(&edge);
  cvReleaseImage(&src);
  cvReleaseImage(&src);
}

IplImage* filter_HSV(Mat img)
{
	Mat image = img.clone();
	vector<Mat> planes;

	split(image, planes);

	Mat maximum = max(planes[0], planes[1]); 
	
	Mat out;

	out = maximum - planes[2];

	IplImage * temp = new IplImage(out);
	return temp;
}



int main(int argc, char** argv)
{
  char k = 0;
	int count = 0;
	IplImage* temp;  

  //process image first
  for(int i = 0; names[i] != 0; i++)
  {
    //load i-th image
    img0 = cvLoadImage(names[i], 1);
//#ifdef HSV
		Mat imgMat(img0);
		temp = filter_HSV(imgMat);

    if(!img0)
    {
      printf("couldn't load %s\n", names[i]);
      continue;
    }

		if(!temp)
		{
			printf("filtering failed .. miserably..");
			continue;
		}
    
    find_circle(temp, storage);

    //wait for key
    k = cvWaitKey(0);
    //release images
    cvReleaseImage(&img0);
    //clear memory storage
    cvClearMemStorage(storage);
    if(k == 27) break;
  }


  //start of camera capture
  capture = cvCaptureFromCAM(0);

  while(1)
  {
    //CvMemStorage* storage1 = cvCreateMemStorage(0);
    img0 = cvQueryFrame(capture);
    if(!img0)
    {
      printf("couldn't capture from camera");
      break;
    }

    if(count++ % frame_per_sec == 0)
    {
      find_circle(img0, storage);
    }
    cvClearMemStorage(storage);
    #ifdef CANNY
    canny = cvCreateImage(cvSize(img0->width, img0->height), img0->depth, 1);
    cvCvtColor(img0, canny, CV_RGB2GRAY);
    cvCanny(canny, canny, 0, thresh, 3);
    cvShowImage("canny", canny);
    #endif
    k = cvWaitKey(33);
    if(k == 27) break;
  }
  cvReleaseImage(&canny);
  cvReleaseImage(&img0);
  cvReleaseCapture(&capture); 
  cvDestroyWindow("circle");

}
