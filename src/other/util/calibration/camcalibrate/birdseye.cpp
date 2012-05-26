//Call: 
//  birds-eye board_w board_h instrinics distortion image_file 
// ADJUST VIEW HEIGHT using keys 'u' up, 'd' down. ESC to quit. 
//

#include <cv.h> 
#include <highgui.h> 
#include <stdio.h> 
#include <stdlib.h>

int main(int argc, char* argv[]) { 
  if(argc != 6) return -1; 
  // INPUT PARAMETERS: 
  // 
  int       board_w    = atoi(argv[1]); 
  int       board_h    = atoi(argv[2]); 
  int       board_n    = board_w * board_h; 
  CvSize    board_sz   = cvSize( board_w, board_h ); 
  CvMat*    intrinsic  = (CvMat*)cvLoad(argv[3]); 
  CvMat*    distortion = (CvMat*)cvLoad(argv[4]); 
  IplImage* image      = 0; 
  IplImage* gray_image = 0; 
  if( (image = cvLoadImage(argv[5])) == 0 ) { 
    printf("Error: Couldn't load %s\n",argv[5]); 
    return -1; 
  } 
  gray_image = cvCreateImage( cvGetSize(image), 8, 1 ); 
  cvCvtColor(image, gray_image, CV_BGR2GRAY ); 
  // UNDISTORT OUR IMAGE 
  // 
  IplImage* mapx = cvCreateImage( cvGetSize(image), IPL_DEPTH_32F, 1 ); 
  IplImage* mapy = cvCreateImage( cvGetSize(image), IPL_DEPTH_32F, 1 ); 
//This initializes rectification matrices 
  // 
  cvInitUndistortMap( 
    intrinsic, 
    distortion, 
    mapx, 
    mapy 
  ); 
  IplImage *t = cvCloneImage(image); 
  // Rectify our image 
  // 
  cvRemap( t, image, mapx, mapy ); 
  // GET THE CHESSBOARD ON THE PLANE 
  // 
  cvNamedWindow("Chessboard"); 
  CvPoint2D32f* corners = new CvPoint2D32f[ board_n ]; 
  int corner_count = 0; 
  int found = cvFindChessboardCorners( 
    image, 
    board_sz, 
    corners, 
    &corner_count, 
    CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS 
  ); 
  if(!found){ 
    printf("Couldn't aquire chessboard on %s, " 
      "only found %d of %d corners\n", 
      argv[5],corner_count,board_n 
    ); 
    return -1; 
  } 
  //Get Subpixel accuracy on those corners: 
  cvFindCornerSubPix( 
    gray_image, 
    corners, 
    corner_count, 
    cvSize(11,11), 
    cvSize(-1,-1), 
    cvTermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1 ) 
  ); 
  //GET THE IMAGE AND OBJECT POINTS: 
  // We will choose chessboard object points as (r,c): 
  // (0,0), (board_w-1,0), (0,board_h-1), (board_w-1,board_h-1). 
  // 
  int x_offset = image->width/3;
  int y_offset = 0;
  
  CvPoint2D32f objPts[4], imgPts[4]; 
  objPts[0].x = 0 + x_offset;         		objPts[0].y = 0 + y_offset; 
  objPts[1].x = board_w-1 + x_offset; 		objPts[1].y = 0 + y_offset; 
  objPts[2].x = 0 + x_offset;         		objPts[2].y = board_h-1 + y_offset; 
  objPts[3].x = board_w-1 + x_offset; 		objPts[3].y = board_h-1 + y_offset; 
  
  imgPts[0]   = corners[0];
  imgPts[1]   = corners[board_w-1]; 
  imgPts[2]   = corners[(board_h-1)*board_w]; 
  imgPts[3]   = corners[(board_h-1)*board_w + board_w-1]; 
  
  // DRAW THE POINTS in order: B,G,R,YELLOW 
  // 
  cvCircle( image, cvPointFrom32f(imgPts[0]), 9, CV_RGB(0,0,255),   3); 
  cvCircle( image, cvPointFrom32f(imgPts[1]), 9, CV_RGB(0,255,0),   3); 
  cvCircle( image, cvPointFrom32f(imgPts[2]), 9, CV_RGB(255,0,0),   3); 
  cvCircle( image, cvPointFrom32f(imgPts[3]), 9, CV_RGB(255,255,0), 3); 
  // DRAW THE FOUND CHESSBOARD 
  // 
  cvDrawChessboardCorners( 
    image, 
    board_sz, 
    corners, 
    corner_count, 
    found 
  ); 
  cvShowImage( "Chessboard", image ); 
  // FIND THE HOMOGRAPHY 
  // 
  CvMat *H = cvCreateMat( 3, 3, CV_32F); 
  cvGetPerspectiveTransform( objPts, imgPts, H); 
  // LET THE USER ADJUST THE Z HEIGHT OF THE VIEW 
  // 
  float Z = 25; 
  int key = 0; 
  IplImage *birds_image = cvCloneImage(image); 
  cvNamedWindow("Birds_Eye"); 
  // LOOP TO ALLOW USER TO PLAY WITH HEIGHT: 
  // 
  // escape key stops 
  // 
  while(key != 27) { 
    // Set the height 
    // 
    CV_MAT_ELEM(*H,float,2,2) = Z; 
    // COMPUTE THE FRONTAL PARALLEL OR BIRD'S-EYE VIEW: 
    // USING HOMOGRAPHY TO REMAP THE VIEW 
    // 
    cvWarpPerspective( 
      image, 
      birds_image, 
      H, 
      CV_INTER_LINEAR | CV_WARP_INVERSE_MAP | CV_WARP_FILL_OUTLIERS 
    ); 
    cvShowImage( "Birds_Eye", birds_image );
    key = cvWaitKey(); 
    if(key == 'u') Z += 0.5; 
    if(key == 'd') Z -= 0.5; 
  } 
  cvSave("H.xml",H); //We can reuse H for the same camera mounting 
  return 0; 
} 
