//simple stop sign tracker with turret



#define WEBCAM_CAPTURE_WIDTH 352.0
#define WEBCAM_CAPTURE_HEIGHT 288.0
#define WEBCAM_CAPTURE_FRAMERATE 10.0

#include "cv.h"
#include "highgui.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <math.h>
#include <float.h>
#include <limits.h>
#include <time.h>
#include <ctype.h>

#ifdef _EiC
#define WIN32
#endif


#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>

//filters a circular tile marker for color disc
int ProcessColorROI(IplImage* img, int moder);
int lo_diff = 6, up_diff = 106, radius_diff=1;
#define SCAN_FILTER_RED 0
#define SCAN_FILTER_GREEN 1
#define SCAN_FILTER_BLUE 2 

static CvMemStorage* storage = 0;
static CvHaarClassifierCascade* cascade = 0;
CvSeq* circles;
//turret vars
static int turret_Pitch;
static int turret_Yaw;

void detect_and_draw( IplImage* image );

void bindHALClient();	//open network socket
void sendHALCommand(char Msg[]);	//Xfer msgs
	
const char* cascade_name =
    "haarcascade_frontalface_alt.xml";
/*    "haarcascade_profileface.xml";*/

int main( int argc, char** argv )
{
    CvCapture* capture = 0;
    IplImage *frame, *frame_copy = 0;
	
    int optlen = strlen("--cascade=");
    const char* input_name;
	
    if( argc > 1 && strncmp( argv[1], "--cascade=", optlen ) == 0 )
    {
        cascade_name = argv[1] + optlen;
        input_name = argc > 2 ? argv[2] : 0;
    }
    else
    {
        cascade_name = "stop.xml";
        input_name = argc > 1 ? argv[1] : 0;
    }

    cascade = (CvHaarClassifierCascade*)cvLoad( cascade_name, 0, 0, 0 );
    
    if( !cascade )
    {
        fprintf( stderr, "ERROR: Could not load classifier cascade\n" );
        fprintf( stderr,
        "Usage: astopdetect --cascade=\"<cascade_path>\" [filename|camera_index]\n" );
        return -1;
    }
    storage = cvCreateMemStorage(0);
    
    if( !input_name || (isdigit(input_name[0]) && input_name[1] == '\0') )
        capture = cvCaptureFromCAM( !input_name ? 0 : input_name[0] - '0' );
    else
        capture = cvCaptureFromAVI( input_name ); 

    
//set capture resolution only works for linux
    cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH, WEBCAM_CAPTURE_WIDTH);
    cvSetCaptureProperty(capture,CV_CAP_PROP_FRAME_HEIGHT, WEBCAM_CAPTURE_HEIGHT);
    cvSetCaptureProperty(capture,CV_CAP_PROP_FPS, WEBCAM_CAPTURE_FRAMERATE);
    
    cvNamedWindow( "result", 1 );
    
    cvCreateTrackbar( "ThresL", "result", &lo_diff, 255, NULL );
    cvCreateTrackbar( "ThresH", "result", &up_diff, 255, NULL );
    cvCreateTrackbar( "Radius", "result", &radius_diff, 255, NULL );

    bindHALClient();	//open network socket
    turret_Pitch=0;	//safe zone
    turret_Yaw=0;
    
    if( capture )
    {
        for(;;)
        {
            if( !cvGrabFrame( capture ))
                break;
 
//	frame = cvRetrieveFrame( capture,0 );	//Linux GCC
	frame = cvRetrieveFrame( capture );	//Cygwin GCC
 
    
            if( !frame )
                break;
            if( !frame_copy )
                frame_copy = cvCreateImage( cvSize(frame->width,frame->height),
                                            IPL_DEPTH_8U, frame->nChannels );
            if( frame->origin == IPL_ORIGIN_TL )
                cvCopy( frame, frame_copy, 0 );	    
            else
                cvFlip( frame, frame_copy, 0 );
            
            detect_and_draw( frame_copy );

            if( cvWaitKey( 10 ) >= 0 )
                break;
        }

        cvReleaseImage( &frame_copy );
        cvReleaseCapture( &capture );
    }
    else
    {
        const char* filename = input_name ? input_name : (char*)"lena.jpg";
        IplImage* image = cvLoadImage( filename, 1 );

        if( image )
        {
            detect_and_draw( image );
            cvWaitKey(0);
            cvReleaseImage( &image );
        }
        else
        {
            /* assume it is a text file containing the
               list of the image filenames to be processed - one per line */
            FILE* f = fopen( filename, "rt" );
            if( f )
            {
                char buf[1000+1];
                while( fgets( buf, 1000, f ) )
                {
                    int len = (int)strlen(buf);
                    while( len > 0 && isspace(buf[len-1]) )
                        len--;
                    buf[len] = '\0';
                    image = cvLoadImage( buf, 1 );
                    if( image )
                    {
                        detect_and_draw( image );
                        cvWaitKey(0);
                        cvReleaseImage( &image );
                    }
                }
                fclose(f);
            }
        }

    }
    
    cvDestroyWindow("result");

    return 0;
}

/////////////////////////////////////////////////////////////////////////
//must bind in main init code
static int sockfd = -1;                    // UDP socket for sending reply
static struct sockaddr_in their_addr; // client's address information
static int numbytes =0 ;

void bindHALClient(void){
	// Open a UDP socket for sending the requested sensor data
	if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) == -1)
	{
		perror("socket");
		exit(1);
	}

			// Set up the address/port info that we will send the reply to.
	their_addr.sin_family = AF_INET;     // host byte order
	their_addr.sin_port = htons(atoi("1225")); // short, network byte order
	their_addr.sin_addr.s_addr = inet_addr("127.0.0.1");
	memset(their_addr.sin_zero, '\0', sizeof their_addr.sin_zero);
}

/////////////////////////////////////////////////////////////////////////
//sent client command to HAL
void sendHALCommand(char Msg[]){	

		// Send the reply, checking for errors.
	if ((numbytes = sendto(sockfd,
	                       Msg, 
	                       strlen(Msg),
	                       0,
	                       (struct sockaddr *) &their_addr,
	                       sizeof their_addr)) == -1)
	{
		perror("sendto");
		exit(1);
	}

	
}

/////////////////////////////////////////////////////////////////////////
//match feature cluster, then check if > 51% red
void detect_and_draw( IplImage* img )
{
	IplImage *ROIimg=0;
	//Light object marker
	CvConnectedComp comp;
	int bounds_index=0,temper=0;
	CvSize src_img_size;
	
	int tmpY=0;
	int tmpX=0;
	
	//Message assing vars
	char reply[256] = "";  // string buffer for reply
	char replyTmpArray[64] ="";

	
    static CvScalar colors[] = 
    {
        {{0,0,255}},
        {{0,128,255}},
        {{0,255,255}},
        {{0,255,0}},
        {{255,128,0}},
        {{255,255,0}},
        {{255,0,0}},
        {{255,0,255}}
    };

    double scale = 1.3;
    IplImage* gray = cvCreateImage( cvSize(img->width,img->height), 8, 1 );
    IplImage* small_img = cvCreateImage( cvSize( cvRound (img->width/scale),
                         cvRound (img->height/scale)),
                     8, 1 );
    int i;

    cvCvtColor( img, gray, CV_BGR2GRAY );
    cvResize( gray, small_img, CV_INTER_LINEAR );
    cvEqualizeHist( small_img, small_img );
    cvClearMemStorage( storage );

    

//////////////////////////////////////////////// Find Stopsigns
    
    if( cascade )
    {
        double t = (double)cvGetTickCount();
        CvSeq* faces = cvHaarDetectObjects( small_img, cascade, storage,
                                            1.1, 2, 0/*CV_HAAR_DO_CANNY_PRUNING*/,
                                            cvSize(30, 30) );
        t = (double)cvGetTickCount() - t;
        printf( "detection time = %gms\n", t/((double)cvGetTickFrequency()*1000.) );
        for( i = 0; i < (faces ? faces->total : 0); i++ )
        {
            CvRect* r = (CvRect*)cvGetSeqElem( faces, i );
            CvPoint center;
            int radius, checkRedResult=0;
            center.x = cvRound((r->x + r->width*0.5)*scale);
            center.y = cvRound((r->y + r->height*0.5)*scale);
            radius = cvRound((r->width + r->height)*0.25*scale);
		
		if(radius >= radius_diff){
			cvSetImageROI(img, cvRect(cvRound(center.x - radius),cvRound(center.y - radius), cvRound(2*radius), cvRound(2*radius)));
			//check image tile for color blob   
			checkRedResult=ProcessColorROI(img, SCAN_FILTER_RED);
			//mark off scanned area
			//cvSetZero(img);
			
			if(checkRedResult > 0){
				//cvNot(img,img);
				cvSetZero(img);
			}
			
			cvResetImageROI(img);	//next ROI tile
			
			if(checkRedResult > 0){
				cvCircle( img, center, radius, colors[3], 8, 8, 0 );
				src_img_size = cvGetSize(img);
				
				tmpX= -((center.x) - (src_img_size.width/2));		
				//turret tracker Yaw

				if(tmpX > (5))
				{
					turret_Yaw--;
					turret_Yaw--;
					if(turret_Yaw <= -100){
						turret_Yaw=-99;
					}
				}else{					
					if(tmpX  < (-5))
					{
						turret_Yaw++;
						turret_Yaw++;
						if(turret_Yaw >= 100){
							turret_Yaw=99;
						}
					}
				}

				 
				tmpY= ((center.y) - (src_img_size.height/2));	
				//NOTE: inverted turret tracket Pitch

				if(tmpY > (3))
				{
						turret_Pitch--; 
						turret_Pitch--; 
						if(turret_Pitch <= -100){
							turret_Pitch=-99;
						}		
				}else{					
					if(tmpY < (-3))
					{
						turret_Pitch++; 
						turret_Pitch++; 
						if(turret_Pitch >= 100){
							turret_Pitch=99;
						}
					}
					
				}
				
				printf("*******Red stop sign! (%i:1) Y:%i:%i P:%i:%i\n",checkRedResult,turret_Yaw,tmpX, turret_Pitch,tmpY);	//RED
				
							
				// Assemble the reply.
				strcpy(reply, "auto_turret,");
				sprintf(replyTmpArray,"%i", (turret_Pitch));
				strcat(reply, replyTmpArray);	
				strcat(reply, ",");	
				sprintf(replyTmpArray,"%i", (turret_Yaw));
				strcat(reply, replyTmpArray);	
					
				// Send the reply, checking for errors.
				sendHALCommand(reply);
 				
            cvWaitKey(100);	//allow settings to take
			}else{

					printf("No Red seen (%i:1)\n", checkRedResult);		//False hit, no RED
					cvCircle( img, center, radius, colors[6], 2, 8, 0 );
			}
				  
		}
		
		
        }
    }

//////////////////////////////////////////////// Find lights
    
  if(ROIimg)
 {
    cvReleaseImage(&ROIimg);
 }
 
ROIimg = cvCreateImage( cvGetSize(img), IPL_DEPTH_8U, 1 );
  
//cvCvtColor( img, ROIimg, CV_BGR2GRAY );	//grayscale image...
cvSmooth( gray, ROIimg, CV_GAUSSIAN, 9, 9,0 ,0); // smooth it, otherwise a lot of false circles may be detected
  
/* Detect stop-lights using Hough Circle:
CvSeq* cvHoughCircles( CvArr* image, void* circle_storage,
                       int method, double dp, double min_dist,
                       double param1=100, double param2=100,
                       int min_radius=0, int max_radius=0 );
  */
circles = cvHoughCircles( ROIimg, storage, CV_HOUGH_GRADIENT, 4, ROIimg->height/12, 200, 100, 10,180 );

        for( bounds_index = 0; (bounds_index < (circles->total))&&( bounds_index < 10); bounds_index++ )
        {
		float* p = (float*)cvGetSeqElem( circles, bounds_index);
		cvCircle( img, cvPoint(cvRound(p[0]),cvRound(p[1])), 3, CV_RGB(255,0,0), -1, 8, 0 );
		cvCircle( img, cvPoint(cvRound(p[0]),cvRound(p[1])), cvRound(p[2]), CV_RGB(255,0,0), 3, 8, 0 );
		cvSetImageROI(img, cvRect(cvRound(p[0]-p[2]),cvRound(p[1]-p[2]), cvRound(2*p[2]), cvRound(2*p[2])));
		
		//if(p[2] >= radius_diff){
			
		temper=ProcessColorROI(img, SCAN_FILTER_RED);
		if(temper> 0){
			printf("Red Disc Was seen (%i:1)\n",temper);
			/* Save Image */ 
			cvNot(img,img);	//mark off scanned items
		}		
		
		temper=ProcessColorROI(img, SCAN_FILTER_GREEN);
		if(temper> 0){
			printf("Green Disc Was seen (%i:1)\n", temper);
			/* Save Image */ 
			cvNot(img,img);	//mark off scanned items
		}
		
		//}
		
		cvResetImageROI(img);	//next ROI tile
        }

    
    cvShowImage( "result", img );
    cvReleaseImage( &gray );
    cvReleaseImage( &small_img );
    cvReleaseImage(&ROIimg);
}



//filters a circular tile marker for color disc
	IplImage *timg=NULL,*mimg=NULL, 
			*SCANMODEimg=NULL,*FILTER1MODEimg=NULL,*FILTER2MODEimg=NULL,
			*Rimg=NULL,*Gimg=NULL,*Bimg=NULL,*mask=NULL;
	IplConvKernel *element=NULL;
int ProcessColorROI(IplImage* img, int moder)
{ 
	CvSize tile_scan_size ;
	int mm, pp, total; 
	uchar* ptr;
	double pixel;
	
/* Split Blue/Green/Red Channel of image to Gray Scale images */
  if((Bimg)||(Gimg)||(Rimg))
 {
    cvReleaseImage(&Bimg);
    cvReleaseImage(&Gimg);
    cvReleaseImage(&Rimg);
 }
  Bimg = cvCreateImage( cvGetSize(img), IPL_DEPTH_8U, 1 );
  Gimg = cvCreateImage( cvGetSize(img), IPL_DEPTH_8U, 1 );
  Rimg = cvCreateImage( cvGetSize(img), IPL_DEPTH_8U, 1 );
  cvSplit( img,Bimg,Gimg,Rimg,0);

//Setup how the filter scans...
switch(moder)
{
 case(SCAN_FILTER_RED):
   {
     SCANMODEimg = Rimg;
     FILTER1MODEimg = Gimg;
     FILTER2MODEimg = Bimg;
     break;	
   }		 
 case(SCAN_FILTER_GREEN):
   {
     SCANMODEimg = Gimg;
     FILTER1MODEimg = Bimg;
     FILTER2MODEimg = Rimg;
     break;	
   }	
 case(SCAN_FILTER_BLUE):
   {
     SCANMODEimg = Bimg;
     FILTER1MODEimg = Gimg;
     FILTER2MODEimg = Rimg;
     break;	
   }		 	 
 default:
   {
     printf("Wrong mode %d for provessROI chosen.\n", moder);
     exit(0);
   }
}


  /* Copy Target Image to Memory */
  if((mimg) || (timg))
 {
    cvReleaseImage(&mimg);
    cvReleaseImage(&timg);
  }

/* Apply Color Subtraction Operations */
  if(SCANMODEimg->nChannels==1)
  {
	mimg=cvCloneImage(SCANMODEimg);
	timg = cvCloneImage (mimg);
	mask=mimg;
	cvSub(SCANMODEimg, FILTER1MODEimg, timg, mask);
	cvSub(timg, FILTER2MODEimg, mimg, mask);	  
  }else{
	exit(0);		//invalid color for ops...
  }	  


/* Apply Dilate Morphological Operation to fill in pin holes  if(element)
  {
  cvReleaseStructuringElement(&element );
  }
  element = cvCreateStructuringElementEx (33,33,16,16,CV_SHAPE_ELLIPSE, NULL);
  cvDilate (mimg,timg, element, 1);
  cvReleaseStructuringElement(&element );
 */

/* Normal Threshold of image to remove partials */
//cvThreshold (timg, mimg, 99, 255, CV_THRESH_TOZERO );
cvThreshold (timg, mimg, lo_diff, up_diff, CV_THRESH_TOZERO );

/* Equalizes histogram of grayscale image for visual bounds check */
//cvEqualizeHist(mimg,timg);
  
/* Polarize image to B&W */
cvMul(mimg,mimg, timg, 10);
  
tile_scan_size = cvGetSize(SCANMODEimg);
total=0;

  for(mm=0;mm < tile_scan_size.height;mm++)
 { 
	for(pp=0;pp < tile_scan_size.width;pp++)
	{ 
		//pixel = cvmGet(timg,mm,pp);
		ptr = &CV_IMAGE_ELEM(timg,uchar,mm,pp);
		
		// pixel per address
		if (ptr[0] != 0) {
			total++; 
		}else{
			total--; 
		}
	}

 }
  
 //TODO: fix this part
if( total ==0){total=1;}
 total = ((pp*mm)/total);	//scale ratio to tile, +pos ratio if <less than 50% black
 
if(total>12){total=-1111;}	//cut off low red content

 return total;
}
