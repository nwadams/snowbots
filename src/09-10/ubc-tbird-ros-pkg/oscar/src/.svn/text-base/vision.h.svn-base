/*header file for vision*/
struct path
{
	int confidence; //a percentage
	int angle; //in degrees from center
	int distance;  //in cm
};

CvPoint getSeed(IplImage *img);

void circlePoint(IplImage *img, CvPoint point, CvScalar);

void findPath(IplImage *out, CvPoint seed);

double clamp (double value, int min, int max);

void compressLightness(IplImage *hsv_img);

CvPoint getCenterOfGravity(IplImage *hsv_img, CvPoint seed);

path doPicture(IplImage *in, IplImage *out);


