#include <cstdlib>
#include <iostream>
#include "cv.h"
#include "highgui.h"
#include <stdio.h>

using namespace std;
using namespace cv;

void doImage(Mat img)
{
	imshow("frame", img);
	waitKey(0);
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
	if(argc > 2)
	{
		VideoCapture capture(atoi(argv[2]));
  		if ( capture.isOpened() ) 
		{
    		doVideo(capture);
    		return 0;
		}
	}
	else if(argc > 1)
	{
		Mat img = imread(argv[1]);
    	if (! img.data ) 
		{
      		cout << "unable to open image at " << argv[1] << endl;
     		return 1;
		}
		doImage(img);
	} else {		
		for ( int cam_id = 5; cam_id >= 0; cam_id-- ) 
		{
      		VideoCapture capture(cam_id);
      		if ( capture.isOpened() ) 
			{
        		doVideo(capture);
        		return 0;
    		}
		}
		cout << "could not open any cameras" << endl;
    }
	return 0;
}
