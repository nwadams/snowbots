#include "cv.h"
#include "highgui.h"
#include <iostream>

using namespace cv;

int main(int, char**)
{
    VideoCapture capture1(0); // open the default camera
    if( !capture1.isOpened() )  {
        printf("Camera 1 failed to open!\n");
        return -1;
    }

	VideoCapture capture2(1); // open the default camera
    if( !capture2.isOpened() )  {
        printf("Camera 2 failed to open!\n");
        return -1;
    }

    Mat frame1, frame2;
   
    namedWindow("cam1",CV_WINDOW_AUTOSIZE);
    namedWindow("cam2",CV_WINDOW_AUTOSIZE);
    
    for(;;)
    {
        // get a new frame from camera
        capture1 >> frame1;
		capture2 >> frame2; 

        // show frame on screen
        imshow("cam1", frame1); 
		imshow("cam2", frame2);

        if(waitKey(30) >= 0) break;
    }
    return 0;
}
