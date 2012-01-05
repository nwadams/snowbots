#include "cv.h"
#include "highgui.h"
#include <iostream>

using namespace cv;

int main(int, char**)
{
        VideoCapture capture(1); // open the default camera
        if( !capture.isOpened() )  {
                printf("Camera failed to open!\n");
                return -1;
        }
		capture.set(CV_CAP_PROP_FRAME_WIDTH,800);
		capture.set(CV_CAP_PROP_FRAME_HEIGHT,600);
        Mat frame;
        capture >> frame; // get first frame for size
        
        // record video
        VideoWriter record("RobotVideo.avi", CV_FOURCC('D','I','V','X'), 30, frame.size(), true);
        if( !record.isOpened() ) {
                printf("VideoWriter failed to open!\n");
                return -1;
        }
        
        namedWindow("video",1);
        
        for(;;)
        {
                // get a new frame from camera
                capture >> frame; 

                // show frame on screen
                imshow("video", frame); 
                
                // add frame to recorded video
                record << frame; 

                if(waitKey(30) >= 0) break;
        }

        // the camera will be deinitialized automatically in VideoCapture destructor
        // the recorded video will be closed automatically in the VideoWriter destructor
        return 0;
}
