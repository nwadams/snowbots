#include <cv.h>
#include <highgui.h>

#define HSV_IMAGE "HSSpace.JPG"
#define SAVE_TRAFFIC_LIGHT "isolatedTrafficLight.JPG"

// use HSV histogram original flavour
void useHSVHistogram(IplImage* justImg, IplImage* returnImg);
// expect the memory of returning image to be allocated ahead of time

// use HSV histogram cone flavour
void useHSVHistogram_cone(IplImage* justImg, IplImage* returnImg);

// isolate red in stop sign image
void getPureRed(IplImage* justImg, IplImage* returnImg);

// isolate bright spots in a traffic light image
void getTrafficLight(IplImage* justImg, IplImage* returnImg);

// generate a image with h(0~180), s(0~255) and v=255
void generateHSVSpace(IplImage* someImg);