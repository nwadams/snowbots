#include <cv.h>
#include <highgui.h>

int
main (int argc, char *argv[])
{
  IplImage *src_img, *dst_img;
  CvMat *intrinsic, *distortion;
  CvFileStorage *fs;
  CvFileNode *param;
  int key = 0;
  IplImage *mapx = cvCreateImage (cvSize (320, 240), IPL_DEPTH_32F, 1);
  IplImage *mapy = cvCreateImage (cvSize (320, 240), IPL_DEPTH_32F, 1);
  CvCapture* capture = 0;
	
  capture = cvCreateCameraCapture( 0 );
  assert( capture );
	
  fs = cvOpenFileStorage ("H.xml", 0, CV_STORAGE_READ);
  param = cvGetFileNodeByName (fs, NULL, "intrinsics");
  intrinsic = (CvMat *) cvRead (fs, param,0);
  param = cvGetFileNodeByName (fs, NULL, "distortion");
  distortion = (CvMat *) cvRead (fs, param,0);
  cvReleaseFileStorage (&fs);
  cvInitUndistortMap (intrinsic, distortion, mapx, mapy); 
	
  cvNamedWindow ("Distortion", CV_WINDOW_AUTOSIZE);
  cvNamedWindow ("UnDistortion", CV_WINDOW_AUTOSIZE);
  cvWaitKey(0);
do{	
  src_img = cvQueryFrame( capture );
  dst_img = cvCloneImage (src_img);
	
  cvRemap (src_img, dst_img, mapx, mapy,0, cvScalarAll(0));

  cvShowImage ("Distortion", src_img);
  cvShowImage ("UnDistortion", dst_img);

key = cvWaitKey(500);
}while(key != 27);
  
  cvDestroyWindow ("Distortion");
  cvDestroyWindow ("UnDistortion");
  cvReleaseImage (&src_img);
  cvReleaseImage (&dst_img);
  cvReleaseImage (&mapx);
  cvReleaseImage (&mapy);
  cvReleaseMat (&intrinsic);
  cvReleaseMat (&distortion);

  return 0;
}
