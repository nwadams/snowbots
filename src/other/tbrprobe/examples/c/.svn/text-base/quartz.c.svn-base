//first flood fill nav

#define WEBCAM_CAPTURE_WIDTH 320
#define WEBCAM_CAPTURE_HEIGHT 240
#define WEBCAM_CAPTURE_FRAMERATE 15

#ifdef _CH_
#pragma package <opencv>
#endif

#ifndef _EiC
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
#endif

#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>

IplImage* color_img0 = 0;
IplImage* mask = 0; 

IplImage* color_img_warp;
CvMat *H;
CvMat Hz[4];
int lo_Z_distort = 15;
int is_warp_perspective =0;

///////////////////////////////////////////////
//set the viedo mean weight vote scans
#define vote_avg_limit 2
int voterAvg[vote_avg_limit+1];

///amount to bias 
int robot_steer_control_offset = 0;
#define ROBOT_STOPPED 0
#define ROBOT_DRIVE 1
#define ROBOT_LOOK_LEFT 2
#define ROBOT_LOOK_RIGHT 3
#define ROBOT_LOOK_TURN 4
static int ROBOT_FSM_MODE=ROBOT_STOPPED;


//number of flood points to try
#define sample_point_scan_count 20

//track current throttle settings to send updates
int robot_throttle_control = 0;
int robot_throttle_control_backup = 0;
int robot_stop_distance = WEBCAM_CAPTURE_HEIGHT;

IplImage* color_img = NULL;
IplImage* gray_img0 = NULL;
IplImage* gray_img = NULL;
int ffill_case = 1;
//int lo_diff = 21, up_diff = 32;
int lo_diff = 69, up_diff = 72;    //highbay
int lo_diff_gap =0;	//min pixel ratio gap to halt scan of fill
int connectivity = 8;
int is_color = 1;
int is_mask = 0;
int is_mask_dialate = 0;
int new_mask_val = 255;


static int sample_point_track =1;
static int sample_point_x =100;
static int sample_point_y =100;
static int show_flood_walls = 1;

//turret vars
static int turret_Pitch=0;
static int turret_Yaw=0;

static CvMemStorage* storage = 0;


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
//perform some graphic processing
void flood_path(int x, int y)
{
            CvPoint seed = cvPoint(x,y);
		int i,j;
	
            int lo = ffill_case == 0 ? 0 : lo_diff;
            int up = ffill_case == 0 ? 0 : up_diff;
            int flags = connectivity + (new_mask_val << 8) +
                        (ffill_case == 1 ? CV_FLOODFILL_FIXED_RANGE : 0);
    //        int b = rand() & 255, g = rand() & 255, r = rand() & 255;
            int b = 255, g = 0, r = 0;
            CvConnectedComp comp;
	     IplConvKernel *elementDialateKern;	//for dialate
	
	if( !color_img ){
	return;
	}
	
	if( is_mask )
	{
               cvThreshold( mask, mask, 1, 16, CV_ADAPTIVE_THRESH_MEAN_C );
  //             cvThreshold( mask, mask, 1, 16, CV_THRESH_BINARY );
	}

	    //------------------------------------------------------------------------------------------------------
            if( is_color )
            {
                CvScalar color = CV_RGB( r, g, b );
		int sz = 1;
		CvPoint pt[21];
		//find the image sizes
		CvSize src_size = cvGetSize(color_img);
		CvSize mask_size = cvGetSize(mask);
		    
#define set_to_horizon seed.y
#define sample_point seed

		pt[0].x = src_size.width;	//mid screen point
		pt[0].y = src_size.height;	//will vary based on plane detection
		pt[1].x = src_size.width;
		pt[1].y = src_size.height;		    
		pt[2].x = src_size.width;	//mid screen point
		pt[2].y = src_size.height;	//will vary based on plane detection
		pt[3].x = 1;
		pt[3].y = src_size.height;
		pt[0].x /=2;			//mid set
		pt[0].y =set_to_horizon;		   
		pt[2].x /=2;
		pt[2].y =set_to_horizon;		    
    
		if(show_flood_walls !=0){    
		cvLine( color_img, pt[0], pt[1], CV_RGB(0, 255,0), 2, 0, 0);	//flood path stop edges
		cvLine( color_img, pt[2], pt[3], CV_RGB(0, 255,0), 2, 0, 0);	//flood path stop edges
		}
		pt[10].x = 1;
		pt[10].y = set_to_horizon;
		pt[11].x = src_size.width;
		pt[11].y =set_to_horizon;
		cvLine( color_img, pt[10], pt[11], CV_RGB(0, 255,0), 2, 0, 0);	//horizon


		//sample_point.x = (src_size.width/2)+(rand() % 43)-(rand() % 34);
		sample_point.x = (src_size.width/2);
		//sample_point.y +=40;
		sample_point.y =(src_size.height) - (sample_point_track+1);
		sample_point_track = ((sample_point_track + 4)% sample_point_scan_count);
		
                cvFloodFill( color_img, sample_point, color, CV_RGB( lo, lo, lo ),
                             CV_RGB( up, up, up ), &comp, flags, is_mask ? mask : NULL );
			     
		cvCircle(color_img, sample_point, 4, CV_RGB(128,128,128), 1,1,0);	//sample point
		
		
		pt[12].x = 0;
		pt[13].x =src_size.width;
		pt[12].y = robot_stop_distance;
		pt[13].y =robot_stop_distance;
		cvLine( color_img, pt[12], pt[13], CV_RGB(255,0,0), 2, 0, 0);	//stop line limit
		
			
		//------------------------------------------------------------------------------------------------------
		if(is_warp_perspective ){
			// COMPUTE THE FRONTAL PARALLEL OR BIRD'S-EYE VIEW:
			// USING HOMOGRAPHY TO REMAP THE VIEW
			//
			CV_MAT_ELEM(*H,float,2,2) = lo_Z_distort;
			//
			cvWarpPerspective(
				color_img,
				color_img_warp,
				H,
				(CV_INTER_LINEAR | CV_WARP_FILL_OUTLIERS), //CV_WARP_INVERSE_MAP | 
				cvScalarAll(0) //color to fill in unknown data 
				);
			
			//(CV_INTER_LINEAR | CV_WARP_INVERSE_MAP | CV_WARP_FILL_OUTLIERS),
			//cvScalarAll(0) //color to fill in unknown data 
			cvShowImage( "image", color_img_warp );
		}else{
			cvShowImage( "image", color_img);
		}


		//------------------------------------------------------------------------------------------------------
		if( is_mask )
		{
			uchar* ptr;
			int minScan[480];
			int maxScan[480];
			int tmpVal = 0;

			if(is_mask_dialate)
			{
				//size of patchwork to apply
//				elementDialateKern = cvCreateStructuringElementEx (33,33,16,16,CV_SHAPE_RECT, NULL);
				elementDialateKern = cvCreateStructuringElementEx (17,17,9,9,CV_SHAPE_RECT, NULL);
				cvDilate (mask,mask, elementDialateKern, 1);
				cvReleaseStructuringElement(&elementDialateKern );
			}
			
			minScan[sample_point.y+1]=0;
			maxScan[sample_point.y+1]=mask_size.width;
			
			for (i= sample_point.y ; ((i > 0) && (minScan[i+1] +5 <= maxScan[i+1])) &&
					((maxScan[i+1]-minScan[i+1]) > (lo_diff_gap+(sample_point.y-i)))
					;  i--) {		//min width abort  
				
				tmpVal=0;
				j=(sample_point.x);
				for ( ; (j>0) && (j > minScan[i+1]) && (tmpVal==0); j--) {	//scan left
					
					ptr = &CV_IMAGE_ELEM(mask,uchar,i,j);
					
					// pixel per address
						if (ptr[0] != 0) {
							ptr[0]= 0x00; //mask bug fix
						}else{
							tmpVal = 1;
							ptr[0]= 0xFF; 
						}
						
				}
				minScan[i]=j;
				
				tmpVal=0;
				j=(sample_point.x +1);
				for (; (j < mask_size.width) && (j < maxScan[i+1]) && (tmpVal==0); j++) { //scan right
					
					ptr = &CV_IMAGE_ELEM(mask,uchar,i,j);
					
					// pixel per address
						if (ptr[0] != 0) {
							ptr[0]= 0x00; 
						}else{
							tmpVal = 1;
							ptr[0]= 0xFF; 
						}
						
				}
				maxScan[i]=j;
			}

			minScan[0]=0;
			maxScan[0]=mask_size.width;
			tmpVal=0;
			for (i= sample_point.y ; ((i > 0) && (minScan[i] +5 <= maxScan[i])) &&
					((maxScan[i]-minScan[i]) > (lo_diff_gap+(sample_point.y-i)))  ;  i--) {

				if(minScan[i] > minScan[0]){	//find max of min
					minScan[0] = minScan[i];
				}
				
				if(maxScan[i] < maxScan[0]){	//find min of max
					maxScan[0] = maxScan[i];
				}

					j=((maxScan[i] - minScan[i])/2)+minScan[i];	
					ptr = &CV_IMAGE_ELEM(mask,uchar,i,j);		//mark the ideal path
				//	ptr[0]= 0xFF; 
				//	ptr[1]= 0x00; 
				//	ptr[2]= 0xFF; 
				//	ptr[3]= 0x00; 
				//	ptr[4]= 0xFF; 
					
				
				if(sample_point.x < j){		//vote left or right?
					tmpVal+=(j-sample_point.x );
				}else{
					tmpVal-=(sample_point.x-j );
				}
				  
			}
		
		
		char reply[256] = "";  // string buffer for reply
		char replyTmpArray[64] ="";
		if(robot_stop_distance < i){
			voterAvg[0]=1;	//reset command
			
			robot_throttle_control_backup=robot_throttle_control; 
			robot_throttle_control=0; 
			printf("robot_throttle_ABORTED=%i (Stop)\n",robot_throttle_control); 
				
			// Assemble the reply.
			strcpy(reply, "auto_throttle,");
			sprintf(replyTmpArray,"%i", robot_throttle_control);
			strcat(reply, replyTmpArray);	
				
			// Send the reply, checking for errors.
			sendHALCommand(reply);
			usleep(3000);
			
			ROBOT_FSM_MODE=ROBOT_STOPPED;
	
		}else{
//#define ROBOT_STOPPED 0
//#define ROBOT_DRIVE 1
//#define ROBOT_LOOK_LEFT 2
//#define ROBOT_LOOK_RIGHT 3
//#define ROBOT_LOOK_TURN 4
			
			if(ROBOT_FSM_MODE==ROBOT_STOPPED){
				ROBOT_FSM_MODE=ROBOT_DRIVE;
				// Assemble the reply.
				strcpy(reply, "auto_turret,");
				sprintf(replyTmpArray,"%i", (0));
				strcat(reply, replyTmpArray);	
				strcat(reply, ",");	
				sprintf(replyTmpArray,"%i", (0));
				strcat(reply, replyTmpArray);	
				// Send the reply, checking for errors.
				sendHALCommand(reply);
					
				robot_throttle_control=robot_throttle_control_backup; 
				robot_throttle_control_backup=0;
				// Assemble the reply.
				strcpy(reply, "auto_throttle,");
				sprintf(replyTmpArray,"%i", robot_throttle_control);
				strcat(reply, replyTmpArray);	
				// Send the reply, checking for errors.
				sendHALCommand(reply);
			}
			
			
		}

//store for top line marker			
		pt[4].y = i;	
		pt[4].x = 0;
		pt[5].y = i;
		pt[5].x = mask_size.width;		
			
			
			
		if(voterAvg[0] >= vote_avg_limit)
		{
			tmpVal=0;
			for(i=1;i<vote_avg_limit;i++)
			{
				tmpVal=(tmpVal+voterAvg[i])/2;
			}
			
//			printf("\n\nMin:%i Max:%i   \n",minScan[0],maxScan[0]);
			
			if(tmpVal > 20){
				
				//scale for tbrprobe
				tmpVal /=10;
				tmpVal += robot_steer_control_offset;

				if(tmpVal > 99){
					tmpVal=99;	//max right steering
				}
			
				printf("Voted %i:TURN RIGHT   \n",tmpVal);
				
			}
			//prevent robot_steer_control_offset bugs buy scanning other boundary

			if(tmpVal < -20){
				//scale for tbrprobe
				tmpVal /=10;
				tmpVal += robot_steer_control_offset;
				
				if(tmpVal < -99){
					tmpVal= -99;	//max left steering
				}
				
				printf("Voted %i:TURN LEFT   \n",tmpVal);
				
			}

			if((tmpVal < 20) && (tmpVal > -20)){
					tmpVal=0;	//center steering
					printf("Voted %i:NOP   \n",tmpVal);
			}
			
		char reply[256] = "";  // string buffer for reply
		char replyTmpArray[64] ="";

		// Assemble the reply.
		strcpy(reply, "auto_steering,");
		sprintf(replyTmpArray,"%i", (tmpVal));
		strcat(reply, replyTmpArray);	
			
		// Send the reply, checking for errors.
		sendHALCommand(reply);
			
			voterAvg[0]=1;
		}else{
			voterAvg[voterAvg[0]]=tmpVal;
			voterAvg[0]++;	
		}
		
		//------------------------------------------------------------------------------------------------------
		if(is_warp_perspective ){
				// COMPUTE THE FRONTAL PARALLEL OR BIRD'S-EYE VIEW:
				// USING HOMOGRAPHY TO REMAP THE VIEW
				//
				CV_MAT_ELEM(*H,float,2,2) = lo_Z_distort;
				//
				cvWarpPerspective(
					mask,
					mask,
					H,
					(CV_INTER_LINEAR | CV_WARP_FILL_OUTLIERS), //CV_WARP_INVERSE_MAP | 
					cvScalarAll(0) //color to fill in unknown data 
					);
				
				//(CV_INTER_LINEAR | CV_WARP_INVERSE_MAP | CV_WARP_FILL_OUTLIERS),
				//cvScalarAll(0) //color to fill in unknown data 
				cvShowImage( "mask", mask );
				cvZero(mask);
		}else{
				cvShowImage( "mask", mask);
				cvZero(mask);
		}
       }
     }

}

/////////////////////////////////////////////////////////////////////////
//GUI call back
void on_mouse( int event, int x, int y, int flags, void* param )
{

    if( !color_img )
        return;

    switch( event )
    {
    case CV_EVENT_LBUTTONDOWN:
        {
		sample_point_x = x;
		sample_point_y = y;
        }
        break;
    }
}

//main GUI control (hot zones interface or Exit)
void on_mouse_gui( int event, int x, int y, int flags, void* param )
{
	char reply[256] = "";  // string buffer for reply
	char replyTmpArray[64] ="";
	
	
     switch( event )
    {
    case CV_EVENT_LBUTTONDOWN:
        {
		x/=10;
		y/=10;
		printf("\n(*10pixel) X:%i Y:%i\n",x,y);
		
		
		if(  ((x > 6) && (y > 1)) &&
			((x < 13) && (y < 7)) 
		)
		{
			printf("\n/\\\n");
			robot_throttle_control +=5;
			if(robot_throttle_control > 99){
				robot_throttle_control=99;
			}
			printf("robot_throttle_control=%i(+)\n",robot_throttle_control); 
			// Assemble the reply.
			strcpy(reply, "auto_throttle,");
			sprintf(replyTmpArray,"%i", robot_throttle_control);
			strcat(reply, replyTmpArray);	
			
			// Send the reply, checking for errors.
			sendHALCommand(reply);
			return;
		}

		
		
		if(  ((x > 6) && (y > 9)) &&
			((x < 13) && (y < 15)) 
		)
		{
			printf("\n[]\n");
			
			robot_throttle_control=0; 
			robot_steer_control_offset=0;

			ROBOT_FSM_MODE=ROBOT_STOPPED;
			robot_throttle_control_backup=0;
			printf("robot_throttle_control=%i (Stop)\n",robot_throttle_control); 
		
			// Assemble the reply.
			strcpy(reply, "auto_throttle,");
			sprintf(replyTmpArray,"%i", robot_throttle_control);
			strcat(reply, replyTmpArray);	
				
			// Send the reply, checking for errors.
			sendHALCommand(reply);
			return;
		}
		
		
		if(  ((x > 6) && (y > 17)) &&
			((x < 13) && (y < 24)) 
		)
		{
			printf("\n\\/\n");	

			robot_throttle_control -=5;
			if(robot_throttle_control < -99){
				robot_throttle_control=-99;
			}
			printf("robot_throttle_control=%i(-)\n",robot_throttle_control); 
			// Assemble the reply.
			strcpy(reply, "auto_throttle,");
			sprintf(replyTmpArray,"%i", robot_throttle_control);
			strcat(reply, replyTmpArray);	
				
			// Send the reply, checking for errors.
			sendHALCommand(reply);
			return;
		}
		
		if(  ((x > 22) && (y > 6)) &&
			((x < 30) && (y < 9)) 
		)
		{
			printf("\n/\\Tilt\n");	
			
			turret_Pitch++;
			if(turret_Pitch > 99){
				turret_Pitch=99;
			}

			// Assemble the reply.
			strcpy(reply, "auto_turret,");
			sprintf(replyTmpArray,"%i", (turret_Pitch));
			strcat(reply, replyTmpArray);	
			strcat(reply, ",");	
			sprintf(replyTmpArray,"%i", (turret_Yaw));
			strcat(reply, replyTmpArray);	
							
			// Send the reply, checking for errors.
			sendHALCommand(reply);
			return;
		}
		
		if(  ((x > 23) && (y > 14)) &&
			((x < 30) && (y < 17)) 
		)
		{
			printf("\n\\/Tilt%i\n",turret_Pitch);	
			
			turret_Pitch--;
			if(turret_Pitch < -99){
				turret_Pitch=-99;
			}

			// Assemble the reply.
			strcpy(reply, "auto_turret,");
			sprintf(replyTmpArray,"%i", (turret_Pitch));
			strcat(reply, replyTmpArray);	
			strcat(reply, ",");	
			sprintf(replyTmpArray,"%i", (turret_Yaw));
			strcat(reply, replyTmpArray);	
							
			// Send the reply, checking for errors.
			sendHALCommand(reply);
			return;
		}
		
		if(  ((x > 22) && (y > 9)) &&
			((x < 28) && (y < 13)) 
		)
		{
			printf("\n<Pan\n");	
			
			turret_Yaw--;
			if(turret_Yaw < -99){
				turret_Yaw=-99;
			}

			// Assemble the reply.
			strcpy(reply, "auto_turret,");
			sprintf(replyTmpArray,"%i", (turret_Pitch));
			strcat(reply, replyTmpArray);	
			strcat(reply, ",");	
			sprintf(replyTmpArray,"%i", (turret_Yaw));
			strcat(reply, replyTmpArray);	
							
			// Send the reply, checking for errors.
			sendHALCommand(reply);
			return;
		}
		
		if(  ((x > 31) && (y > 9)) &&
			((x < 35) && (y < 13)) 
		)
		{
			printf("\n>Pan\n");	
			
			turret_Yaw++;
			if(turret_Yaw > 99){
				turret_Yaw=99;
			}

			// Assemble the reply.
			strcpy(reply, "auto_turret,");
			sprintf(replyTmpArray,"%i", (turret_Pitch));
			strcat(reply, replyTmpArray);	
			strcat(reply, ",");	
			sprintf(replyTmpArray,"%i", (turret_Yaw));
			strcat(reply, replyTmpArray);	
							
			// Send the reply, checking for errors.
			sendHALCommand(reply);
			return;
		}
		
		if(  ((x > 28) && (y > 9)) &&
			((x < 31) && (y < 13)) 
		)
		{
			printf("\n0 Pan\n");	
			
			turret_Yaw=0;
			turret_Pitch=0;
			// Assemble the reply.
			strcpy(reply, "auto_turret,");
			sprintf(replyTmpArray,"%i", (turret_Pitch));
			strcat(reply, replyTmpArray);	
			strcat(reply, ",");	
			sprintf(replyTmpArray,"%i", (turret_Yaw));
			strcat(reply, replyTmpArray);	
							
			// Send the reply, checking for errors.
			sendHALCommand(reply);
			return;
		}
		
		
		if(  ((x > 1) && (y > 9)) &&
			((x < 6) && (y < 15)) 
		)
		{
			printf("\n< steer bias:%i\n", robot_steer_control_offset);
			
			robot_steer_control_offset -=5;\
			if(robot_steer_control_offset < -59){
				robot_steer_control_offset=-59;
			}
			return;
		}
		
		
		if(  ((x > 14) && (y > 9)) &&
			((x < 19) && (y < 15)) 
		)
		{
			printf("\n> steer bias:%i\n", robot_steer_control_offset);
			robot_steer_control_offset +=5;\
			if(robot_steer_control_offset > 59){
				robot_steer_control_offset=59;
			}
			return;
		}	
		
		if(  (x > 38) && 
			((y > 6) && (y < 15)))
		{
		     if( is_mask )
		    {
			cvDestroyWindow( "mask" );
			is_mask = 0;
		    }
		    else
		    {
			cvNamedWindow( "mask", CV_WINDOW_AUTOSIZE );
			cvZero( mask );
			cvShowImage( "mask", mask );
			is_mask = 1;
		    }
			
		}
		
		if((x > 38) && (y < 6)){
		exit(0);
		}
		
		
        }
        break;
    }
}


/////////////////////////////////////////////////////////////////////////
//Main loop
int main( int argc, char** argv )
{
    CvCapture* capture = 0;
    IplImage *logoImage = 0;
	
    int c;
    CvFileStorage *fs;
    CvFileNode *param;
	char reply[256] = "";  // string buffer for reply
	char replyTmpArray[64] ="";
	
    voterAvg[0]=1;
	
    H  = &Hz[0];
	
   fs = cvOpenFileStorage ("H.xml", 0, CV_STORAGE_READ);
  param = cvGetFileNodeByName (fs, NULL, "H");
  H = (CvMat *) cvRead (fs, param,0);	
  cvReleaseFileStorage (&fs);
	
    storage = cvCreateMemStorage(0);
	
   // capture = cvCaptureFromCAM(CV_CAP_ANY);
    capture = cvCreateCameraCapture(CV_CAP_ANY);
  
//set capture resolution only works for linux
    cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH, WEBCAM_CAPTURE_WIDTH);
    cvSetCaptureProperty(capture,CV_CAP_PROP_FRAME_HEIGHT, WEBCAM_CAPTURE_HEIGHT);
    cvSetCaptureProperty(capture,CV_CAP_PROP_FPS, WEBCAM_CAPTURE_FRAMERATE);

    printf("\nCAMERA W:%f H:%f FPS:%f\n\n",
	    cvGetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH),
	    cvGetCaptureProperty(capture,CV_CAP_PROP_FRAME_HEIGHT),
	    cvGetCaptureProperty(capture,CV_CAP_PROP_FPS)
	);
	



	
    printf( "Hot keys: \n"
            "\tESC - quit the program\n"  
	    "\t\\ - robot throttle control stop\n"
	    "\t[ - robot throttle control increase\n"
	    "\t] - robot throttle control decrease\n" 
            "\tm - show mask\n"
	    "\td- dialate mask option is set\n"
            "\ts - use null-range floodfill\n"
            "\tf - use gradient floodfill with fixed(absolute) range\n"
            "\tg - use gradient floodfill with floating(relative) range\n"
            "\tl - use floodfill walls\n"
            "\t4 - use 4-connectivity mode\n"
            "\t8 - use 8-connectivity mode\n" 
	    "\tw- warp perspective mode\n");

	bindHALClient();	//open network socket
   
    cvNamedWindow( "image", CV_WINDOW_AUTOSIZE );
//Mouse lines
     cvSetMouseCallback( "image", on_mouse, 0 );  
     
 //GUI
    cvNamedWindow( "remote", CV_WINDOW_AUTOSIZE );
    cvSetMouseCallback( "remote", on_mouse_gui, 0 );  
     
     
    color_img0 = cvQueryFrame(capture);
     robot_stop_distance=color_img0->height ;
    cvNamedWindow( "robot", CV_WINDOW_AUTOSIZE );
    cvCreateTrackbar( "gap_limit", "robot", &lo_diff_gap, robot_stop_distance, NULL ); 
    cvCreateTrackbar( "stop_dist", "robot", &robot_stop_distance, robot_stop_distance, NULL ); 
    cvCreateTrackbar( "lo_diff", "robot", &lo_diff, 100, NULL );
    cvCreateTrackbar( "up_diff", "robot", &up_diff, 100, NULL );
    cvCreateTrackbar( "Z distortion", "robot", &lo_Z_distort, 100, NULL ); 
    
//load GUI instructions from jpg image?
if( (logoImage = cvLoadImage("quartz.jpg", 1)) != 0 )
{
	cvShowImage("remote", logoImage);
}


    if( capture )
    {
        for(;;)
        {
/*		if( !cvGrabFrame( capture ))
		{
		    break;
		}
		
                color_img0 = cvRetrieveFrame( capture,0 );	//DO not call cvReleaseImage on pointer to frame buffer
	    */
		color_img0 = cvQueryFrame(capture);
		if( !color_img0 )
		{ 
		    break;
		}
	    
		if( !color_img )
		{
		color_img = cvCreateImage( cvSize(color_img0->width,color_img0->height),
                                            IPL_DEPTH_8U, color_img0->nChannels );
	    
                color_img_warp = cvCreateImage( cvSize(color_img0->width,color_img0->height),
                                            IPL_DEPTH_8U, color_img0->nChannels );
		}
		
		if( color_img0->origin == IPL_ORIGIN_TL )
		{
			cvCopy( color_img0, color_img, 0 );
		}else{
                	cvFlip( color_img0, color_img, 0 );
		}

		if (!mask){
			mask = cvCreateImage( cvSize(color_img0->width + 2, color_img0->height + 2), 8, 1 );
	                cvShowImage( "mask", mask );
		}
		flood_path( sample_point_x, sample_point_y );

		c = cvWaitKey(10);
		switch( (char) c )
		{
		case '\x1b':
		    printf("Exiting ...\n");
		    goto exit_main;
		case 's':
		    printf("Simple floodfill mode is set\n");
		    ffill_case = 0;
		    break;
		case ' ':
			robot_throttle_control=0; 
			printf("robot_throttle_control=%i (Stop)\n",robot_throttle_control); 
		
			// Assemble the reply.
			strcpy(reply, "auto_throttle,");
			sprintf(replyTmpArray,"%i", robot_throttle_control);
			strcat(reply, replyTmpArray);	
				
			// Send the reply, checking for errors.
			sendHALCommand(reply);
		    break;
		case '[':
			robot_throttle_control +=5;
			if(robot_throttle_control > 99){
				robot_throttle_control=99;
			}
			printf("robot_throttle_control=%i(+)\n",robot_throttle_control); 
			// Assemble the reply.
			strcpy(reply, "auto_throttle,");
			sprintf(replyTmpArray,"%i", robot_throttle_control);
			strcat(reply, replyTmpArray);	
				
			// Send the reply, checking for errors.
			sendHALCommand(reply);
		    break;
		case 'l':
			if(show_flood_walls != 0){
				show_flood_walls = 0;
			}else{
				show_flood_walls =1;
			}
		    break;	
		case ']':
			robot_throttle_control -=5;
			if(robot_throttle_control < -99){
				robot_throttle_control=-99;
			}
			printf("robot_throttle_control=%i(-)\n",robot_throttle_control); 
			// Assemble the reply.
			strcpy(reply, "auto_throttle,");
			sprintf(replyTmpArray,"%i", robot_throttle_control);
			strcat(reply, replyTmpArray);	
				
			// Send the reply, checking for errors.
			sendHALCommand(reply);
		    break;
		case 'f':
		    printf("Fixed Range floodfill mode is set\n");
		    ffill_case = 1;
		    break;
		case 'g':
		    printf("Gradient (floating range) floodfill mode is set\n");
		    ffill_case = 2;
		    break;
		case '4':
		    printf("4-connectivity mode is set\n");
		    connectivity = 4;
		    break;
		case '8':
		    printf("8-connectivity mode is set\n");
		    connectivity = 8;
		    break;
		case 'd':
			if(is_mask_dialate)
			{		
				printf("d- dialate mask option is off\n");
				is_mask_dialate =0;
			}else{				
				printf("d- dialate mask option is set\n");
				is_mask_dialate =1;
			}
			break;
		case 'm':
		    if( is_mask )
		    {
			cvDestroyWindow( "mask" );
			is_mask = 0;
		    }
		    else
		    {
			cvNamedWindow( "mask", CV_WINDOW_AUTOSIZE );
			cvZero( mask );
			cvShowImage( "mask", mask );
			is_mask = 1;
		    }
		    break;
		case 'w':
		    if( is_warp_perspective )
		    {
			printf("w- warp perspective option is off\n");
			is_warp_perspective = 0;
		    }else{
			printf("w- warp perspective option is set\n");
			is_warp_perspective= 1;
		    }
		    break;
		     
		default:
			{}
		}
        }

        //cvReleaseImage(&color_img); // DO NOT deallocate color_img0, it is allocated and deallocated internally in the capture structure
	cvReleaseImage(&color_img_warp);
        cvReleaseCapture( &capture );
    }
    


exit_main:

    cvReleaseImage( &gray_img );
    cvReleaseImage( &gray_img0 );
    cvReleaseImage( &color_img );
    cvReleaseImage( &mask );
    cvReleaseCapture( &capture );
    cvDestroyAllWindows();
/*    cvDestroyWindow( "test" );
    cvDestroyWindow( "image" );
    cvDestroyWindow( "mask" );*/

    return 1;
}

#ifdef _EiC
main(1,"ffilldemo.c");
#endif
