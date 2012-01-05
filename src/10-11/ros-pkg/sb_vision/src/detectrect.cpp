
//
// The full "Square Detector" program.
// It loads several images subsequentally and tries to find squares in
// each image
//

#pragma package <opencv>



#include <cv.h>
#include <highgui.h>
#include <stdio.h>
#include <math.h>
#include <string.h>

//#define CANNY // TODO: uncomment this line to see the canny of the input image

//double angle_thresh = 80.00; //angle threshold to identify as a rectangle
int min_rect_size = 1000; //minimum size of rectangle
int max_rect_size = 2304001; // maximum size of rectangle, 1920 * 1200 + 1
//TODO: change this 2304001 to filter too large rectangles in the image
int thresh = 50; //upper threshold for canny
IplImage* img0 = 0;
CvMemStorage* storage = 0;
CvCapture* capture = 0;
IplImage* canny = 0;
int every_process = 30;
int frame_count = 0;
CvPoint pt[4];
const char* wndname = "Square Detection Demo";

char* names[] = { "img/1.jpg", "img/2.jpg", "img/3.jpg", "img/4.jpg", "img/5.jpg",
"img/6.jpg", "img/7.jpg", "img/8.jpg", "img/95.jpg", "img/96.jpg", "img/97.png",
 "img/98.jpg", "img/99.jpg", 0 };




// helper function:
// finds a cosine of angle between vectors
// from pt0->pt1 and from pt0->pt2
double angle( CvPoint* pt1, CvPoint* pt2, CvPoint* pt0)
{
  double dx1 = pt1->x - pt0->x;
  double dy1 = pt1->y - pt0->y;
  double dx2 = pt2->x - pt0->x;
  double dy2 = pt2->y - pt0->y;
  return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

// returns sequence of squares detected on the image.
// the sequence is stored in the specified memory storage
CvSeq* findSquares4( IplImage* img, CvMemStorage*storage)
{
  CvSeq* contours;
  int i, c, l, N = 11;
  CvSize sz = cvSize( img->width & -2, img->height & -2 );
  IplImage* timg = cvCloneImage( img ); // make a copy of input image
  IplImage* gray = cvCreateImage( sz, 8, 1 );
  IplImage* pyr = cvCreateImage( cvSize(sz.width/2, sz.height/2), 8, 3 );
  IplImage* tgray;
  CvSeq* result;
  double s, t;
  // create empty sequence that will contain points
  // 4 points per square (the square's vertices)
  CvSeq* squares = cvCreateSeq( 0, sizeof(CvSeq),
  sizeof(CvPoint), storage );

  // select the maximum ROI in the image
  // with the width and height divisible by 2
  cvSetImageROI( timg, cvRect( 0, 0, sz.width, sz.height ));

  // down-scale and upscale the image to filter out the noise
  cvPyrDown( timg, pyr, 7 );
  cvPyrUp( pyr, timg, 7 );
  tgray = cvCreateImage( sz, 8, 1 );

  // find squares in every color plane of the image
  for( c = 0; c < 3; c++ )
    {
    // extract the c-th color plane
    cvSetImageCOI( timg, c+1 );
    cvCopy( timg, tgray, 0 );

    // try several threshold levels
    for( l = 0; l < N; l++ )
      {
      // hack: use Canny instead of zero threshold level.
      // Canny helps to catch squares with gradient shading
      if( l == 0 ) 
        {
          // apply Canny. Take the upper threshold from slider
          // and set the lower to 0 (which forces edges merging)
          cvCanny( tgray, gray, 0, thresh, 3 );
          // dilate canny output to remove potential
          // holes between edge segments
          cvDilate( gray, gray, 0, 1 );
        }
      else
        {
          // apply threshold if l!=0:
          // tgray(x,y) = gray(x,y) < (l+1)*255/N ? 255 : 0
          cvThreshold( tgray, gray, (l+1)*255/N,255, CV_THRESH_BINARY );
        }

      // find contours and store them all as a list
      cvFindContours( gray, storage, &contours, sizeof(CvContour),
      CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0) );

      // test each contour
      while( contours )
      {
        // approximate contour with accuracy proportional
        // to the contour perimeter
        result = cvApproxPoly( contours, sizeof(CvContour), storage,
        CV_POLY_APPROX_DP, cvContourPerimeter(contours)*0.02, 0 );
        // square contours should have 4 vertices after approximation
        // relatively large area (to filter out noisy contours)
        // and be convex.
        // Note: absolute value of an area is used because
        // area may be positive or negative - in accordance with the
        // contour orientation
        if( result->total == 4 && fabs(cvContourArea(result,CV_WHOLE_SEQ)) > min_rect_size &&
        fabs(cvContourArea(result,CV_WHOLE_SEQ)) < max_rect_size && cvCheckContourConvexity(result) )
        {
          s = 0;
          for( i = 0; i < 5; i++ )
          {
            // find minimum angle between joint
            // edges (maximum of cosine)
            if( i >= 2 )
            {
              t = fabs(angle((CvPoint*)cvGetSeqElem(result, i ),
              (CvPoint*)cvGetSeqElem(result, i-2 ),(CvPoint*)cvGetSeqElem(result, i-1 )));
              s = s > t ? s : t;
            }
          }

          // if cosines of all angles are small
          // (all angles are ~90 degree) then write quandrange
          // vertices to resultant sequence
          if( s < 0.1 )
          for( i = 0; i < 4; i++ )
            cvSeqPush( squares,(CvPoint*)cvGetSeqElem( result, i ));
        }

        // take the next contour
        contours = contours->h_next;
      }
    }
  }

  // release all the temporary images
  cvReleaseImage( &gray );
  cvReleaseImage( &pyr );
  cvReleaseImage( &tgray );
  cvReleaseImage( &timg );

  return squares;
}


// the function draws all the squares in the image
void drawSquares( IplImage* img, CvSeq* squares )
{
  CvSeqReader reader;
  IplImage* cpy = cvCloneImage( img );

  // initialize reader of the sequence
  cvStartReadSeq( squares, &reader, 0 );

  // read 4 sequence elements at a time (all vertices of a square)
  for( int i = 0; i < squares->total; i += 4 )
  {
    CvPoint* rect = pt;
    int count = 4;

    // read 4 vertices
    memcpy( pt, reader.ptr, squares->elem_size );
    CV_NEXT_SEQ_ELEM( squares->elem_size, reader);
    memcpy( pt + 1, reader.ptr, squares->elem_size);
    CV_NEXT_SEQ_ELEM( squares->elem_size, reader);
    memcpy( pt + 2, reader.ptr, squares->elem_size);
    CV_NEXT_SEQ_ELEM( squares->elem_size, reader);
    memcpy( pt + 3, reader.ptr, squares->elem_size);
    CV_NEXT_SEQ_ELEM( squares->elem_size, reader);

    // draw the square as a closed polyline
    cvPolyLine( cpy, &rect, &count, 1, 1, CV_RGB(0,255,0), 3, CV_AA, 0 );
  }

  // show the resultant image
  cvShowImage( wndname, cpy );
  cvReleaseImage( &cpy );
}


void on_trackbar( int a )
{
  if( img0 )
    drawSquares( img0, findSquares4( img0, storage ));
}

int main(int argc, char** argv)
{
  capture = cvCaptureFromCAM(1);

  char c;
  // create memory storage that will contain all the dynamic data
  storage = cvCreateMemStorage(0);

  //process images first before processing capture
  for(int i = 0; names[i] != 0; i++ )
  {
    // load i-th image
    img0 = cvLoadImage( names[i], 1 );
    if( !img0 )
    {
      printf("Couldn't load %s\n", names[i] );
      continue;
    }
    #ifdef CANNY
    canny = cvCreateImage(cvSize(img0->width, img0->height), img0->depth, 1);
    cvCvtColor(img0, canny, CV_RGB2GRAY);
    cvCanny(canny, canny, 0, thresh, 3);
    cvShowImage("canny", canny);
    #endif

    // create window and a trackbar (slider) with parent "image" and set callback
    // (the slider regulates upper threshold, passed to Canny edge detector)
    cvNamedWindow( wndname, 1 );
    //cvCreateTrackbar( "canny thresh", wndname, &thresh, 1000, on_trackbar );

    // force the image processing
    on_trackbar(0);
    // wait for key.
    // Also the function cvWaitKey takes care of event processing
    c = cvWaitKey(0);
    // release both images
//    cvReleaseImage( &img );
    cvReleaseImage( &img0 );
    cvReleaseImage( &canny);
    // clear memory storage - reset free space position
    cvClearMemStorage( storage );
    if( c == 27 ) // hit ESC key to terminate program
    break;
  }

//start of video capture
  while(1)
  {
    img0 = cvQueryFrame(capture);
    if(!img0)
    {
      printf("couldn't capture from camera");
      break;
    }
    if(frame_count%every_process == 0)
    {
#ifdef CANNY
      canny = cvCreateImage(cvSize(img0->width, img0->height), frame->depth, 1);
      cvCvtColor(img0, canny, CV_RGB2GRAY);
      cvCanny(canny,canny,0,thresh,3);
      cvShowImage("canny", canny);
#endif
      drawSquares(img0, findSquares4( img0, storage));
    }
    frame_count++;
    cvClearMemStorage(storage);
    cvReleaseImage(&canny);
    c = cvWaitKey(33);
    if( c == 27) break;
  }

  cvDestroyWindow( wndname );
  cvReleaseCapture(&capture);
  return 0;
}
