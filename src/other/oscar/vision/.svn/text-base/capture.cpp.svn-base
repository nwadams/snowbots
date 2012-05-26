#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <iostream>

#define WEBCAM_CAPTURE_WIDTH 320
#define WEBCAM_CAPTURE_HEIGHT 240
#define WEBCAM_CAPTURE_FRAMERATE 10.0

int main( int argc, char* argv[] ) {
	CvCapture* capture = cvCreateCameraCapture(CV_CAP_ANY);
//CV_CAP_ANY
	  

	if ( argc < 2 ) 
	{
		std::cerr << "No output file specified.  Exiting." << std::endl;
		exit(-1);
	}

	if (!capture) {
		std::cerr << "Unable to initialize camera capture, exiting." 
			<< std::endl;
		exit(-1);
	}

	std::cout << "Capturing images, press q to stop." << std::endl;
	cvNamedWindow("image", CV_WINDOW_AUTOSIZE );


	cvSetCaptureProperty(capture, 
		CV_CAP_PROP_FRAME_WIDTH, WEBCAM_CAPTURE_WIDTH);
	cvSetCaptureProperty(capture,
		CV_CAP_PROP_FRAME_HEIGHT, WEBCAM_CAPTURE_HEIGHT);
	cvSetCaptureProperty(capture,
		CV_CAP_PROP_FPS, WEBCAM_CAPTURE_FRAMERATE);


	CvVideoWriter *writer = cvCreateVideoWriter(
		argv[1],
		CV_FOURCC('M', 'J', 'P', 'G'),
		cvGetCaptureProperty(capture, CV_CAP_PROP_FPS),
		cvSize(cvGetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH),
			cvGetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT) ) );

while( (bgr_frame=cvQueryFrame(capture)) != NULL ) {
    cvLogPolar( bgr_frame, logpolar_frame,
                cvPoint2D32f(bgr_frame->width/2,
                bgr_frame->height/2),
                40,
                CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS );
    cvWriteFrame( writer, logpolar_frame );


}

//	cvReleaseVideoWriter( &writer );
	cvReleaseCapture( &capture ); 
	cvDestroyAllWindows();
	return 0;
}
