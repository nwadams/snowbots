//capture is a simple program that allows you to capture images from
//a connected camera.  

//Usage: capture [camera index]

#include <cv.h> 
#include <highgui.h> 
#include <stdio.h> 
#include <stdlib.h>

int camindex = CV_CAP_ANY;
CvCapture* capture = NULL;
IplImage* frame = NULL;
int key;
int imageno=0;
char namebuffer[256];

int main(int argc, char* argv[]){

	if(argc > 1){
		camindex = atoi(argv[1]);
	}
	
	printf("*********************************************\n");
	printf("*           Camera Capture Utility          *\n");
	printf("*      Press Space to capture an image      *\n");
	printf("*             Press ESC to quit             *\n");
	printf("*********************************************\n\n");
	
	printf("Initiating camera capture on index %d\n",camindex);
	capture = cvCreateCameraCapture( camindex );
	
	printf("Creating window\n");
	cvNamedWindow( "Camera" ); 
	
	while(key != 27){
		frame = cvQueryFrame( capture );
		IplImage *image = cvCreateImage(cvGetSize(frame),frame->depth,frame->nChannels);
		cvCopy(frame,image);
		
		key = cvWaitKey(5);
		
		switch(key){
			case ' ':
				sprintf(namebuffer,"image%d.jpg",imageno);
				cvSaveImage( namebuffer, image );
				printf("saved image \"%s\"\n",namebuffer);
				imageno++;
			break;
		
		}
		
		cvShowImage("Camera",image);
		
		cvReleaseImage(&image);
	}
	
	
	cvDestroyWindow("Camera");
	return 0;
	
}