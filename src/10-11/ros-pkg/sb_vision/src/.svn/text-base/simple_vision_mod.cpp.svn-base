#include <cstdlib>
#include <iostream>
#include "cv.h"
#include "highgui.h"
#include <stdio.h>

using namespace std;
using namespace cv;

void imManip( Mat img )
{	
	Mat imgNeg;

	//creates and shows negative of img
	//bitwise operator ~

	//imgNeg= ~img;

	cvtColor(img,imgNeg,CV_BGR2GRAY);
	namedWindow("Negative");
	imshow("Negative", imgNeg );
	waitKey(0);
}

void doImage(Mat img)
{
	imshow("frame", img);
	waitKey(0);
	imManip( img );
}

void doVideo(VideoCapture capture)
{
	Mat frame;
	while(true)
	{
		capture >> frame;
    	if (! frame.data ) {
    		cout << "unable to get frame" << endl;
    		return;
		}
		imshow("frame", frame);

		if(waitKey(30) >= 0)
        	return;
	}

}
	
int main(int argc, char** argv)
{
	namedWindow("frame");
	if(argc > 1)
	{
		Mat img = imread(argv[1]);
    	if (! img.data ) 
		{
      		cout << "unable to open image at" << argv[1] << endl;
     		return 1;
		}
		doImage(img);
	} else {
		VideoCapture capture(1);
		if ( capture.isOpened() ) {
        	doVideo(capture);
        	return 0;
		}
    }
	return 0;
}
