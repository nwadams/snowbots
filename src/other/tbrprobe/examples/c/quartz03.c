//flood fill context nav with turret

//Loogitech Chat for skype
#define WEBCAM_CAPTURE_WIDTH 352
#define WEBCAM_CAPTURE_HEIGHT 288
#define WEBCAM_CAPTURE_FRAMERATE 10

//for win32 etc. the set video size does not work
//this will always work:
#define VIDEOSCALE 2


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
 

///////////////////////////////////////////////
//set the video mean weight vote scans
#define vote_avg_limit 2
int voterAvg[vote_avg_limit+1];

///amount to bias 
int robot_steer_control_offset = 0;
#define ROBOT_STOPPED 0
#define ROBOT_DRIVE 1
#define ROBOT_BACKUP 2
static int ROBOT_FSM_MODE=ROBOT_STOPPED;

#define ROBOT_LOOK_CENTER 1
#define ROBOT_CHECK_CENTER 2
#define ROBOT_LOOK_LEFT 3
#define ROBOT_CHECK_LEFT 4
#define ROBOT_LOOK_RIGHT 5
#define ROBOT_CHECK_RIGHT 6
#define ROBOT_PATH_PLAN 7
#define ROBOT_TURN_CENTER 8
#define ROBOT_TURN_LEFT 9
#define ROBOT_TURN_RIGHT 10
static int ROBOT_CONTEX_MODE=ROBOT_TURN_CENTER;
static int PlanPathRightValue=0;
static int PlanPathCenterValue=0;
static int PlanPathLeftValue=0;
 

//number of flood points to try
#define sample_point_scan_count 20

//track current throttle settings to send updates
int robot_throttle_control = 0;
int robot_throttle_control_backup = 0;
int robot_stop_distance = WEBCAM_CAPTURE_HEIGHT/VIDEOSCALE;
int robot_stop_distance_left = WEBCAM_CAPTURE_WIDTH/VIDEOSCALE;
int robot_stop_distance_right = 0;
int robot_side_nav_distance = (WEBCAM_CAPTURE_HEIGHT/VIDEOSCALE)/2;

IplImage* color_img = NULL;
IplImage* gray_img0 = NULL;
IplImage* gray_img = NULL;
int ffill_case = 1;
//int lo_diff = 21, up_diff = 32;
//int lo_diff = 69, up_diff = 72;    //highbay intel cs430
//int lo_diff = 28, up_diff = 36;    //low light logitech CHAT for skype
int lo_diff = 62, up_diff = 69;    //highbay logitech CHAT for skype

int lo_diff_gap =0;	//min pixel ratio gap to halt scan of fill
int connectivity = 8;
int is_color = 1;
int is_mask = 0;
int is_mask_dialate = 0;
int new_mask_val = 255;


static int sample_point_track =1;
static int sample_point_x =(WEBCAM_CAPTURE_WIDTH/VIDEOSCALE)/2;
static int sample_point_y =(WEBCAM_CAPTURE_HEIGHT/VIDEOSCALE)/2;
static int show_flood_walls = 1;

//turret vars
static int turret_Pitch=-13;
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
CvScalar color;	//image markers
int sz = 1;
CvPoint pt[21];
//find the image sizes
CvSize src_size;
CvSize mask_size;

CvConnectedComp comp;	//for dialate to remove pin holes
IplConvKernel *elementDialateKern;
	
//elementDialateKern = cvCreateStructuringElementEx (33,33,16,16,CV_SHAPE_RECT, NULL);

uchar* ptr;		//mask scanner
int minScan[WEBCAM_CAPTURE_HEIGHT+2];
int maxScan[WEBCAM_CAPTURE_HEIGHT+2];
#define scannerMinVar minScan[0]
#define scannerMaxVar maxScan[0]
int maxHeightVal=0;
int tmpVal = 0;

void flood_path(int x, int y)
{
	CvPoint seed = cvPoint(x,y);
	int i,j;
	
	int lo = ffill_case == 0 ? 0 : lo_diff;
	int up = ffill_case == 0 ? 0 : up_diff;
	int flags = connectivity + (new_mask_val << 8) +
                        (ffill_case == 1 ? CV_FLOODFILL_FIXED_RANGE : 0);

	
	if( !color_img ){ return; }
	
	if( is_mask )
	{
               cvThreshold( mask, mask, 1, 16, CV_ADAPTIVE_THRESH_MEAN_C );
  //             cvThreshold( mask, mask, 1, 16, CV_THRESH_BINARY );
	}

	//------------------------------------------------------------------------------------------------------
	if( is_color )
	{
                sz = 1;
		src_size = cvGetSize(color_img);
		mask_size = cvGetSize(mask);
		    
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
    
		if((show_flood_walls ==0)&&(ROBOT_CONTEX_MODE==ROBOT_TURN_CENTER)){    
		cvLine( color_img, pt[0], pt[1], CV_RGB(0, 255,0), 2, 0, 0);	//flood path stop edges
		cvLine( color_img, pt[2], pt[3], CV_RGB(0, 255,0), 2, 0, 0);	//flood path stop edges
		}
		pt[10].x = 1;
		pt[10].y = set_to_horizon;
		pt[11].x = src_size.width;
		pt[11].y =set_to_horizon;
		cvLine( color_img, pt[10], pt[11], CV_RGB(0, 255,0), 2, 0, 0);	//horizon relates to pitch


		//sample_point.x = (src_size.width/2)+(rand() % 43)-(rand() % 34);
		sample_point.x = (src_size.width/2);
		//sample_point.y +=40;
		sample_point.y =(src_size.height) - (sample_point_track+1);
		sample_point_track = ((sample_point_track + 2)% sample_point_scan_count);
		
                cvFloodFill( color_img, sample_point, color, CV_RGB( lo, lo, lo ),
                             CV_RGB( up, up, up ), &comp, flags, is_mask ? mask : NULL );
			     
		cvCircle(color_img, sample_point, 4, CV_RGB(128,128,128), 1,1,0);	//sample point
		
		//show stoplines
		pt[12].x = 0;
		pt[13].x =src_size.width;
		pt[12].y = robot_stop_distance;
		pt[13].y =robot_stop_distance;
		cvLine( color_img, pt[12], pt[13], CV_RGB(255,0,0), 2, 0, 0);	//stop line limit
		pt[14].x = robot_stop_distance_left;
		pt[15].x =robot_stop_distance_left;
		pt[14].y = 0;
		pt[15].y =src_size.height;
		cvLine( color_img, pt[14], pt[15], CV_RGB(255,0,0), 2, 0, 0);	//stop line limit
		pt[14].x = robot_stop_distance_right;
		pt[15].x =robot_stop_distance_right;
		pt[14].y = 0;
		pt[15].y =src_size.height;
		cvLine( color_img, pt[14], pt[15], CV_RGB(255,0,0), 2, 0, 0);	//stop line limit
		
		pt[16].x = 0;
		pt[17].x =src_size.width;
		pt[16].y = robot_side_nav_distance;
		pt[17].y =robot_side_nav_distance;
		cvLine( color_img, pt[16], pt[17], CV_RGB(255,0,255), 2, 0, 0);	//side nav ideal guide
		
		
		
		cvShowImage( "image", color_img);  //show frame data
			

		//------------------------------------------------------------------------------------------------------
		//in this part we choose what information about the mask is needed
		if( is_mask )
		{

			if(is_mask_dialate)
			{
				//size of patchwork to apply
				cvDilate (mask,mask, elementDialateKern, 1); 
			}
			
			minScan[sample_point.y+1]=0;
			maxScan[sample_point.y+1]=mask_size.width;
			
			for (i= sample_point.y ; ((i > 0) && (minScan[i+1] +5 <= maxScan[i+1])) 
			&&((maxScan[i+1]-minScan[i+1]) > (lo_diff_gap+(sample_point.y-i)))
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
				for (; (j < mask_size.width) && (j < maxScan[i+1]) &&
					 (tmpVal==0); j++) { //scan right
					
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
			maxHeightVal=i;	//probe height limit, and ignore sample point position
			
			minScan[0]=0;
			maxScan[0]=mask_size.width;
			tmpVal=0;
			for (i= sample_point.y ; ((i > 0) && (minScan[i] +5 <= maxScan[i])) &&			((maxScan[i]-minScan[i]) > (lo_diff_gap+(sample_point.y-i)))  ;  i--) {

				if(minScan[i] > scannerMinVar){	//find max of min
					scannerMinVar = minScan[i];
				}
				
				if(maxScan[i] < scannerMaxVar){	//find min of max
					scannerMaxVar = maxScan[i];
				}

				//j=((maxScan[i] - minScan[i])/2)+minScan[i];	//center of path
				//ptr = &CV_IMAGE_ELEM(mask,uchar,i,j);	//mark the ideal path
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

		//------------------------------------------------------------------------------------------------------
		cvShowImage( "mask", mask);
		cvZero(mask); 
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


//////////////////////////////////////////////////////////////
//helpers for more abstract steering use, convert to 
void Robot_Turn_To_Right(int directParam)
{
	char bufTemp[64];
	char reply[256] = "auto_steering,";  // string buffer for reply
	
	if(directParam < 0){
		directParam=-directParam;	
	}

	if(directParam > 99){
		directParam=99;	
	}

	// Assemble the reply.
	sprintf(bufTemp,"%i", (directParam));
	strcat(reply, bufTemp);	
	
	// Send the reply, checking for errors.
	sendHALCommand(reply);
}

void Robot_Turn_To_Left(int directParam)
{
	char bufTemp[64];
	char reply[256] = "auto_steering,-";  // string buffer for reply
	
	if(directParam < 0){
		directParam=-directParam;	
	}

	if(directParam > 99){
		directParam=99;	
	}

	// Assemble the reply.
	sprintf(bufTemp,"%i", (directParam));
	strcat(reply, bufTemp);	
	
	// Send the reply, checking for errors.
	sendHALCommand(reply);
}

void Robot_Turn_To_Center(void)
{
	char reply[256] = "auto_steering,0";  // string buffer for reply
	// Send the reply, checking for errors.
	sendHALCommand(reply);
}

//turret commands for context of vision commands
void Robot_Look_To_Right(void)
{
	char reply[256] = "auto_throttle,";  // string buffer for reply
	char replyTmpArray[64] ="";
			turret_Yaw=99;
			// Assemble the reply.
			strcpy(reply, "auto_turret,");
			sprintf(replyTmpArray,"%i", (turret_Pitch));
			strcat(reply, replyTmpArray);	
			strcat(reply, ",");	
			sprintf(replyTmpArray,"%i", (turret_Yaw));
			strcat(reply, replyTmpArray);	
							
			// Send the reply, checking for errors.
			sendHALCommand(reply);

}

void Robot_Look_To_Left(void)
{
	char reply[256] = "auto_throttle,";  // string buffer for reply
	char replyTmpArray[64] ="";
			turret_Yaw=-99;
			// Assemble the reply.
			strcpy(reply, "auto_turret,");
			sprintf(replyTmpArray,"%i", (turret_Pitch));
			strcat(reply, replyTmpArray);	
			strcat(reply, ",");	
			sprintf(replyTmpArray,"%i", (turret_Yaw));
			strcat(reply, replyTmpArray);	
							
			// Send the reply, checking for errors.
			sendHALCommand(reply);

}

void Robot_Look_To_Center(void)
{
	char reply[256] = "auto_throttle,";  // string buffer for reply
	char replyTmpArray[64] ="";
			turret_Yaw=0;
			// Assemble the reply.
			strcpy(reply, "auto_turret,");
			sprintf(replyTmpArray,"%i", (turret_Pitch));
			strcat(reply, replyTmpArray);	
			strcat(reply, ",");	
			sprintf(replyTmpArray,"%i", (turret_Yaw));
			strcat(reply, replyTmpArray);	
							
			// Send the reply, checking for errors.
			sendHALCommand(reply);

}

/////////////////////////////////////////////////////////////////////////
//robot driving state changers
void	Robot_Stop(void)		
{
	char reply[256] = "auto_throttle,";  // string buffer for reply
	char replyTmpArray[64] ="";
		if(ROBOT_FSM_MODE!=ROBOT_STOPPED){
			voterAvg[0]=1;	//reset command
			
			robot_throttle_control_backup=robot_throttle_control; 
			robot_throttle_control=0; 
			printf("robot_throttle_ABORTED_R=%i < %i(Stop)\n",robot_stop_distance, maxHeightVal); 
				
			// Assemble the reply.
			strcat(reply, "0"); 
			
			// Send the reply, checking for errors.
			sendHALCommand(reply);
			usleep(3000);
			
			ROBOT_FSM_MODE=ROBOT_STOPPED;
		} 
}


void	Robot_Go(void)		
{
	char reply[256] = "auto_throttle,";  // string buffer for reply
	char replyTmpArray[64] ="";
	if(ROBOT_FSM_MODE==ROBOT_STOPPED)
	{
				ROBOT_FSM_MODE=ROBOT_DRIVE;
				robot_throttle_control=robot_throttle_control_backup; 
				robot_throttle_control_backup=0;
				// Assemble the reply. 
				sprintf(replyTmpArray,"%i", robot_throttle_control);
				strcat(reply, replyTmpArray);	
				// Send the reply, checking for errors.
				sendHALCommand(reply);
	}
}

//helpers check if road perspective has violated stopline limit, and halts driving if true
void	Robot_Check_Forward_Stopline(void)		
{ 
		if(robot_stop_distance < maxHeightVal){
			Robot_Stop();
			usleep(3000); 
		}else{ 
			Robot_Go();
		}
}

void	Robot_Check_Left_Stopline(void)		
{ 
		if(robot_stop_distance_left > scannerMaxVar){
			Robot_Stop();
			usleep(3000); 
		}else{ 
			Robot_Go();
		}
}


void	Robot_Check_Right_Stopline(void)		
{ 
		if(robot_stop_distance_right < scannerMinVar){
			
			Robot_Stop();
			usleep(3000); 
		}else{ 
			Robot_Go();
		}
}

/////////////////////////////////////////////////////////////////////////
//when driving forward the robot will create a weighted average to guess the centre of the road		
void Robot_Check_Forward_Steer(void)
{
	int i;
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
}		

////////////////////////////
//steer while looking right
void Robot_Check_Right_Steer(void)
{
		voterAvg[0]=1;  //reset other context scanner
			
			if(maxHeightVal < (robot_side_nav_distance-5)){
				Robot_Turn_To_Right(robot_side_nav_distance-maxHeightVal);
				printf("RS %i:TURN RIGHT   \n",maxHeightVal);
				return;
			} 

			if(maxHeightVal > (robot_side_nav_distance+5)){
				Robot_Turn_To_Left(maxHeightVal-robot_side_nav_distance);
				printf("RS %i:TURN LEFT   \n",maxHeightVal);
				return;				
			}

//			if((maxHeightVal < (robot_side_nav_distance+5)) && 
//				(maxHeightVal > (robot_side_nav_distance-5)))
//			{
				 Robot_Turn_To_Center();
//				printf("RS %i:NOP   \n",maxHeightVal);
				return;
//			}

}		

////////////////////////////
//steer while looking left
void Robot_Check_Left_Steer(void)
{
		voterAvg[0]=1;  //reset other context scanner
			
			if(maxHeightVal > (robot_side_nav_distance+10)){
				Robot_Turn_To_Left(maxHeightVal-robot_side_nav_distance);
				printf("LS Voted %i:TURN LEFT   \n",maxHeightVal);
				return;
			} 

			if(maxHeightVal < (robot_side_nav_distance-10)){
				Robot_Turn_To_Right(robot_side_nav_distance-maxHeightVal);
				printf("LS Voted %i:TURN RIGHT   \n",maxHeightVal);
				return;				
			}

			if((maxHeightVal > (robot_side_nav_distance+10)) && 
				(maxHeightVal < (robot_side_nav_distance-10)))
			{
				 Robot_Turn_To_Center();
				printf("LS Voted %i:NOP   \n",maxHeightVal);
				return;
			}

}	

/////////////////////////////////////////////////////////////////////////
//main hub to control robot's motion etc.
void Robot_Brain(void)
{

	switch(ROBOT_CONTEX_MODE)			//drive mode
	{
		case(ROBOT_LOOK_CENTER):	//read path Center
		{
			Robot_Stop();
			Robot_Look_To_Center();
			usleep(3000); 
			ROBOT_CONTEX_MODE=ROBOT_CHECK_CENTER;
		break;
		}
		case(ROBOT_CHECK_CENTER):
		{
			PlanPathCenterValue=maxHeightVal;
			ROBOT_CONTEX_MODE=ROBOT_LOOK_LEFT;
		break;
		}
		case(ROBOT_LOOK_LEFT):		//read path left
		{
			Robot_Stop();
			Robot_Look_To_Left();
			usleep(3000); 
			ROBOT_CONTEX_MODE=ROBOT_CHECK_LEFT;
		break;
		}
		case(ROBOT_CHECK_LEFT):
		{
			PlanPathLeftValue=maxHeightVal;
			ROBOT_CONTEX_MODE=ROBOT_LOOK_RIGHT;
		break;
		}
		case(ROBOT_LOOK_RIGHT):		//read path right
		{
			Robot_Stop();
			Robot_Look_To_Right();
			usleep(3000); 
			ROBOT_CONTEX_MODE=ROBOT_CHECK_RIGHT;
		break;
		}
		case(ROBOT_CHECK_RIGHT):
		{
			PlanPathRightValue=maxHeightVal;
			ROBOT_CONTEX_MODE=ROBOT_PATH_PLAN;
		break;
		}
		case(ROBOT_PATH_PLAN):		//plan by choosing least restrictive path
		{
			
			
			if(( PlanPathRightValue> PlanPathCenterValue) &&
				(PlanPathRightValue > PlanPathLeftValue)) //right
			{
				Robot_Look_To_Right();
				ROBOT_CONTEX_MODE=ROBOT_TURN_RIGHT;
				Robot_Go();
				return;
			}
/*			
			if((PlanPathCenterValue > PlanPathRightValue) &&
				(PlanPathCenterValue > PlanPathLeftValue))		//fwd
			{
				Robot_Look_To_Center();
				ROBOT_CONTEX_MODE=ROBOT_TURN_CENTER;
				Robot_Go();
				return;
			}
			
			
			if((PlanPathLeftValue > PlanPathRightValue) &&
				( PlanPathLeftValue > PlanPathCenterValue))		//left
			{
				Robot_Look_To_Left();
				ROBOT_CONTEX_MODE=ROBOT_TURN_LEFT; 
				Robot_Go();
				return;
			}
			
			
//			todo: ROBOT_CONTEX_MODE=ROBOT_REVERSE_DEADEND;
			
			Robot_Look_To_Center();
			ROBOT_CONTEX_MODE=ROBOT_LOOK_CENTER;			//nop 
*/			
			
				ROBOT_CONTEX_MODE=ROBOT_LOOK_RIGHT; //demo
			usleep(6000);
			
		break;
		}
	//	case(ROBOT_TURN_RIGHT):
		default:
		{
			//Robot_Check_Right_Stopline();
			Robot_Check_Right_Steer();
			Robot_Look_To_Right();
			if(ROBOT_FSM_MODE==ROBOT_STOPPED){
				//ROBOT_CONTEX_MODE=ROBOT_LOOK_CENTER;
				
				ROBOT_CONTEX_MODE=ROBOT_LOOK_RIGHT; //demo
			}

			PlanPathCenterValue=0;	//demo to force planner
			PlanPathLeftValue=0;
			ROBOT_CONTEX_MODE=ROBOT_TURN_RIGHT;
			usleep(50000);
			
		break;
		}
/*
		case(ROBOT_TURN_LEFT):
		{
			Robot_Check_Left_Stopline();
			Robot_Check_Left_Steer();
			Robot_Look_To_Left();
			if(ROBOT_FSM_MODE==ROBOT_STOPPED){
				ROBOT_CONTEX_MODE=ROBOT_LOOK_CENTER;
			}
		break;
		}
		default://ROBOT_CONTEX_MODE==ROBOT_TURN_CENTER;
		{
			Robot_Check_Forward_Stopline();
			Robot_Check_Forward_Steer();
			Robot_Look_To_Center();
			if(ROBOT_FSM_MODE==ROBOT_STOPPED){
				ROBOT_CONTEX_MODE=ROBOT_LOOK_CENTER;
			}
		break;
		}
*/
	} 
}


/////////////////////////////////////////////////////////////////////////
//Main loop
int main( int argc, char** argv )
{
    CvCapture* capture = 0;
    IplImage *logoImage = 0;
	
    int c; 
	char reply[256] = "";  // string buffer for reply
	char replyTmpArray[64] ="";
	elementDialateKern=cvCreateStructuringElementEx (17,17,9,9,CV_SHAPE_RECT, NULL);
	color = CV_RGB( 0, 0, 255 );	//image markers
	
    voterAvg[0]=1;
	 
	
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
            "\t8 - use 8-connectivity mode\n" );

	bindHALClient();	//open network socket
   
    cvNamedWindow( "image", CV_WINDOW_AUTOSIZE );
//Mouse lines
     cvSetMouseCallback( "image", on_mouse, 0 );  
     
 //GUI
    cvNamedWindow( "remote", CV_WINDOW_AUTOSIZE );
    cvSetMouseCallback( "remote", on_mouse_gui, 0 );  
     
     
    color_img0 = cvQueryFrame(capture);
     robot_stop_distance=(WEBCAM_CAPTURE_HEIGHT/VIDEOSCALE)-10 ;
     robot_stop_distance_left=(WEBCAM_CAPTURE_WIDTH/VIDEOSCALE)-10;
     robot_stop_distance_right=10;
     robot_side_nav_distance=((WEBCAM_CAPTURE_HEIGHT/VIDEOSCALE)/2) + 10;
    cvNamedWindow( "robot", CV_WINDOW_AUTOSIZE );
    cvCreateTrackbar( "gap_limit", "robot", &lo_diff_gap, (WEBCAM_CAPTURE_HEIGHT/VIDEOSCALE) , NULL ); 
    cvCreateTrackbar( "side_nav", "robot", &robot_side_nav_distance, (WEBCAM_CAPTURE_HEIGHT/VIDEOSCALE) , NULL ); 
    cvCreateTrackbar( "stop_distC", "robot", &robot_stop_distance, (WEBCAM_CAPTURE_HEIGHT/VIDEOSCALE) , NULL ); 
    cvCreateTrackbar( "stop_distL", "robot", &robot_stop_distance_left, (WEBCAM_CAPTURE_WIDTH/VIDEOSCALE), NULL ); 
    cvCreateTrackbar( "stop_distR", "robot", &robot_stop_distance_right, (WEBCAM_CAPTURE_WIDTH/VIDEOSCALE), NULL ); 
    cvCreateTrackbar( "lo_diff", "robot", &lo_diff, 100, NULL );
    cvCreateTrackbar( "up_diff", "robot", &up_diff, 100, NULL ); 
    
	//load GUI instructions from jpg image?
	if( (logoImage = cvLoadImage("quartz.jpg", 1)) != 0 )
	{
		cvShowImage("remote", logoImage);
	}

	//reset robot state
	Robot_Turn_To_Center();
	Robot_Look_To_Center();
	
    if( capture )
    {
        for(;;)
        {
		color_img0 = cvQueryFrame(capture);
		if( !color_img0 )
		{ 
		    break;
		}
	    
		if( !color_img )
		{
			color_img = cvCreateImage( cvSize(WEBCAM_CAPTURE_WIDTH/VIDEOSCALE,WEBCAM_CAPTURE_HEIGHT/VIDEOSCALE),  IPL_DEPTH_8U, color_img0->nChannels);
	 		cvResize( color_img0, color_img, CV_INTER_LINEAR);
	 	}
		
		if( color_img0->origin == IPL_ORIGIN_TL )
		{
			//cvCopy( color_img0, color_img, 0 );
			cvResize( color_img0, color_img, CV_INTER_LINEAR); //also removes pin holes
			

		}else{
			cvResize( color_img0, color_img, CV_INTER_LINEAR); //also removes pin holes
                	cvFlip( color_img, color_img, 0 );
		}

		if (!mask){
			mask = cvCreateImage( cvSize(color_img->width + 2, color_img->height + 2), 8, 1 );
	                cvShowImage( "mask", mask );
		}
		
		//process image data
		flood_path( sample_point_x, sample_point_y );
		//do some stuff
		Robot_Brain();
		
		c = cvWaitKey(100);
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
		
			robot_throttle_control_backup=0;
			ROBOT_FSM_MODE=ROBOT_STOPPED;
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
		     
		default:
			{}
		}
        }

        //cvReleaseImage(&color_img); // DO NOT deallocate color_img0, it is allocated and deallocated internally in the capture structure
        cvReleaseCapture( &capture );
    }
    


exit_main:

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
main(1,"quartz02.c");
#endif
