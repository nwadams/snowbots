// circle_detect.cpp : Defines the entry point for the console application.
//


#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/CvBridge.h>
#include <std_msgs/Float64.h>

using namespace std;
using namespace ros;

//ros related constants and variables
const string NODE_NAME = "traffic_light";
const string CMD_TOPIC = "traffic_light";
const int FREQ = 3;
ros::Publisher cmd_pub;
double confidence = 0;

image_transport::Publisher image_pub;
sensor_msgs::CvBridge image;
//define DEBUG;

//declarations
int thresh = 150;
CvMemStorage* storage = cvCreateMemStorage(0);
CvCapture* capture = 0;
IplImage* target =0;
int hue_low_thresh = 100; // tweak this number to filter red better
int sat_low_thresh = 130; // tweak this number to filter brightness
char k = 0;
	int i =0;
	IplImage* img0 = 0;
	


char* names[] = { "img/1.jpg","img/2.jpg","img/3.jpg","img/4.jpg","img/5.jpg","img/6.jpg","img/7.jpg", "img/8.jpg", "img/97.png", "img/98.jpg", "img/99.jpg","img/95.jpg", "img/96.jpg" , 0 };


IplImage* filter_red(IplImage* img)
{
	IplImage* img0 = cvCreateImage(cvSize(img->width,img->height),img->depth, img->nChannels);
	IplImage* hue = cvCreateImage(cvSize(img->width,img->height), img->depth, 1);
	IplImage* sat = cvCreateImage(cvSize(img->width,img->height), img->depth, 1);
	IplImage* val = cvCreateImage(cvSize(img->width,img->height), img->depth, 1);

	cvCvtColor(img, img0, CV_RGB2HSV);
	cvSplit(img0, hue, sat, val, 0);

	cvThreshold(hue, hue, hue_low_thresh, 255, CV_THRESH_BINARY);// if hue > 100, make it black, if hue < 100, make it white

#ifdef DEBUG
	cvShowImage("thresholded hue", hue);
#endif
	cvReleaseImage(&img0);
	cvReleaseImage(&sat);
	cvReleaseImage(&val);

	return hue;
}

IplImage* filter_brightness(IplImage* img)
{
	IplImage* img0 = cvCreateImage(cvSize(img->width,img->height),img->depth, img->nChannels);
	IplImage* hue = cvCreateImage(cvSize(img->width,img->height), img->depth, 1);
	IplImage* sat = cvCreateImage(cvSize(img->width,img->height), img->depth, 1);
	IplImage* val = cvCreateImage(cvSize(img->width,img->height), img->depth, 1);

	cvCvtColor(img, img0, CV_RGB2HSV);
	cvSplit(img0, hue, sat, val, 0);

	cvThreshold(sat, sat, sat_low_thresh, 255, CV_THRESH_BINARY);

#ifdef DEBUG
	cvShowImage("thresholded sat", sat);
#endif
	cvReleaseImage(&img0);
	cvReleaseImage(&hue);
	cvReleaseImage(&val);

	return sat;
}

double distance( CvPoint* pt0, CvPoint* pt1)
{
    double dx1 = pt1->x - pt0->x;
    double dy1 = pt1->y - pt0->y;
    return sqrt(dx1*dx1 + dy1*dy1);
}


int find_min(double a, double b, double c, double d)
{
	return (int) MIN( MIN(a,b) , MIN(c,d) );
}

int find_max(double a, double b, double c, double d)
{
	return (int) MAX( MAX(a,b), MAX(c, d));
}


void find_circle(IplImage* img, CvMemStorage* storage )
{
	int px[100], py[100];
	int edge_thresh = 1;
  IplImage *gray = 0;
	IplImage *edge = 0;
    // create memory storage that will contain all the dynamic data

	gray = cvCreateImage(cvSize(img->width,img->height), img->depth, 1);
	edge = cvCreateImage(cvSize(img->width,img->height), img->depth, 1);
    // create window and a trackbar (slider) with parent "image" and set callback
    // (the slider regulates upper threshold, passed to Canny edge detector)

	if(img->nChannels ==3)
	{
		cvCvtColor(img,gray,CV_BGR2GRAY);
		gray->origin=1;
	} else
	{
		gray = cvCloneImage(img);
	}

    //color threshold
    cvThreshold(gray,gray,100,255,CV_THRESH_BINARY);

    //smooth the image to reduce unneccesary results
    cvSmooth( gray, gray, CV_GAUSSIAN, 11, 11 );

    //get edges
    cvCanny(gray, edge, 0, thresh, 3);

    //get circles
    CvSeq* circles =  cvHoughCircles( gray, storage, CV_HOUGH_GRADIENT, 1, 50, 1, 35 );

	for(int i = 0; (i < circles->total && i < 3) ; i++ )
	{

		float* p = (float*)cvGetSeqElem( circles, i );
				
		cvCircle( target, cvPoint(cvRound(p[0]),cvRound(p[1])), 3, CV_RGB(0,0,255), -1, 8, 0 );

		cvCircle( target, cvPoint(cvRound(p[0]),cvRound(p[1])), cvRound(p[2]), CV_RGB(0,0,255), 2, 8, 0 );

		px[i]=cvRound(p[0]); py[i]=cvRound(p[1]);

	}
		cvReleaseImage(&gray);
		cvReleaseImage(&edge);

}


// helper function:
// finds a cosine of angle between vectors
// from pt0->pt1 and from pt0->pt2
double angle( CvPoint* pt1, CvPoint* pt2, CvPoint* pt0 )
{
    double dx1 = pt1->x - pt0->x;
    double dy1 = pt1->y - pt0->y;
    double dx2 = pt2->x - pt0->x;
    double dy2 = pt2->y - pt0->y;
    return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

// returns sequence of squares detected on the image.
// the sequence is stored in the specified memory storage
CvSeq* findSquares4( IplImage* img, CvMemStorage* storage )
{
    CvSeq* contours;
    int i, c, l, N = 11;
    CvSize sz = cvSize( img->width & -2, img->height & -2 );
    IplImage* timg = cvCloneImage( img ); // make a copy of input image
    IplImage* gray = cvCreateImage( sz, 8, 1 );
  	IplImage* pyr = cvCreateImage( cvSize(sz.width/2, sz.height/2), 8, img->nChannels );
    IplImage* tgray;
    CvSeq* result;
    double s, t, d, d1;
    // create empty sequence that will contain points -
    // 4 points per square (the square's vertices)
    CvSeq* squares = cvCreateSeq( 0, sizeof(CvSeq), sizeof(CvPoint), storage );

    // select the maximum ROI in the image
    // with the width and height divisible by 2
    cvSetImageROI( timg, cvRect( 0, 0, sz.width, sz.height ));

    // down-scale and upscale the image to filter out the noise
    cvPyrDown( timg, pyr, 7 );
    cvPyrUp( pyr, timg, 7 );
    tgray = cvCreateImage( sz, 8, 1 );

    // find squares in every color plane of the image
	for( c = 0; c < timg->nChannels; c++ )
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
                //     tgray(x,y) = gray(x,y) < (l+1)*255/N ? 255 : 0
                cvThreshold( tgray, gray, (l+1)*255/N, 255, CV_THRESH_BINARY );
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
                if( result->total == 4 &&
                    cvContourArea(result,CV_WHOLE_SEQ,0) > 1000 &&
          					cvContourArea(result,CV_WHOLE_SEQ,0) < timg->imageSize &&
                    cvCheckContourConvexity(result) )
                {
                    s = 0;
					          d= 0;
                    for( i = 0; i < 5; i++ )
                    {
                        // find minimum angle between joint
                        // edges (maximum of cosine)
                        if( i >= 2 )
                        {
                            t = fabs(angle(
                            (CvPoint*)cvGetSeqElem( result, i ),
                            (CvPoint*)cvGetSeqElem( result, i-2 ),
                            (CvPoint*)cvGetSeqElem( result, i-1 )));
							              d1 = distance((CvPoint*)cvGetSeqElem(result,i), 
							                    (CvPoint*)cvGetSeqElem(result, i -1));
                            s = s > t ? s : t;
							              d = d < d1 ? d1 : d;
                        }
                    }

                    // if cosines of all angles are small
                    // (all angles are ~90 degree) then write quandrange
                    // vertices to resultant sequence
                    if( s < 0.1 && d > 100 )
                        for( i = 0; i < 4; i++ )
                            cvSeqPush( squares,
                                (CvPoint*)cvGetSeqElem( result, i ));
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
	IplImage* cpy = cvCloneImage(img);
  CvSeqReader reader;
	IplImage* img2 = 0;
	IplImage* temp = 0;
	IplImage* temp1 = 0;
	double prob =0;
  
    int i;
	int minx;
	int miny;
	int maxx;
	int maxy;
	//IplImage* gray = cvCreateImage(cvSize(img0->width,img0->height), img0->depth, 1);
	//cvCvtColor(img,gray,CV_BGR2GRAY);

    // initialize reader of the sequence
    cvStartReadSeq( squares, &reader, 0 );

    // read 4 sequence elements at a time (all vertices of a square)
    for( i = 0; i < squares->total; i += 4 )
    {
        CvPoint pt[4], *rect = pt;
        int count = 4;
    		int prob_count =0;
    		int prob_total = 0;
    		double prob1 =0;

        // read 4 vertices
        CV_READ_SEQ_ELEM( pt[0], reader );
        CV_READ_SEQ_ELEM( pt[1], reader );
        CV_READ_SEQ_ELEM( pt[2], reader );
        CV_READ_SEQ_ELEM( pt[3], reader );

		minx = find_min(pt[0].x,pt[1].x,pt[2].x,pt[3].x);
		miny = find_min(pt[0].y,pt[1].y,pt[2].y,pt[3].y);
		maxx = find_max(pt[0].x,pt[1].x,pt[2].x,pt[3].x);
		maxy = find_max(pt[0].y,pt[1].y,pt[2].y,pt[3].y);

		cvSetImageROI(cpy, cvRect(minx, miny , maxx - minx ,maxy-miny));
		img2 = cvCreateImage(cvGetSize(cpy),cpy->depth,cpy->nChannels);
		cvCopy( cpy, img2, 0 );
		temp = filter_red(img2);
		temp1 = filter_brightness(img2);
		for(int z = 0; z < temp->width; z++)
		{
			for(int y = 0; y < temp->height; y++)
			{
				CvScalar s, k;
				s = cvGet2D(temp,y,z);
				k = cvGet2D(temp1,y,z);
				if(s.val[0] != 0)
					prob_total++;
				if(s.val[0] != 0 && s.val[0] == k.val[0])
					prob_count++;
			}
		}
		prob1 = (double)prob_count / prob_total;
		if(prob1 > prob)
		  prob = prob1;
		find_circle(temp1, storage);
		cvResetImageROI(cpy);
    confidence = prob; 
        // draw the square as a closed polyline
        cvPolyLine( target, &rect, &count, 1, 1, CV_RGB(0,255,0), 3, CV_AA, 0 );
    }
  
    
 // cout<<"the probability that red light is on is: " << prob<< endl;
	cvReleaseImage(&cpy);
	cvReleaseImage(&img2);
	cvReleaseImage(&temp);
	cvReleaseImage(&temp1);

}

void process_video()
{
	for(int i = 0; i < 5; i++)
  {
    capture=cvCaptureFromCAM(i);
    if(capture != 0)
      break;	
  }
	while(1) {
		  img0 = cvQueryFrame( capture );
      image_pub.publish(image.cvToImgMsg(img0,"bgr8"));

		  if( !img0 )
		  {
			  printf("couldn't capture from camera");
			  break;
		  }
		  target = cvCloneImage(img0);
    if(i++%10 == 0)
    {
		  drawSquares(target, findSquares4(target, storage));
		}
		   cmd_pub.publish(confidence);
 
		  cvShowImage("demo", target);
      cvClearMemStorage( storage );
		  k = cvWaitKey(33);
		  if( k == 27 ) break;
	}

	cvReleaseCapture( &capture );
	cvDestroyWindow( "demo" );
}


int main (int argc, char** argv)
{
  //initialize node
  ROS_INFO("Starting %s", NODE_NAME.c_str());
  ros::init(argc, argv, NODE_NAME);


  ros::NodeHandle node;
  image_transport::ImageTransport it(node);
  image_pub = it.advertise("cv_bridge",1);
        
  
  cmd_pub = node.advertise<std_msgs::Float64>(
      CMD_TOPIC, 1);
  ros::Rate loop_rate(FREQ);

  ROS_INFO("ready to go");
  while(ros::ok())
  {
    process_video();
  }  
      ROS_INFO("shutting down node");
  
    return 0;

}
