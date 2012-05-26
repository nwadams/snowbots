/**
This program tests a basic bump and turn sonar based nav system for drag races.
**/

//Loogitech Chat for skype
#define WEBCAM_CAPTURE_WIDTH 352
#define WEBCAM_CAPTURE_HEIGHT 288
#define WEBCAM_CAPTURE_FRAMERATE 15

//for win32 etc. the set video size does not work
//this will always work in linux if the driver has the setting:
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
#include <pthread.h>

//includes
#include "tbrprobe_client.h"
#include "../../udp_constants.h"

#define ROBOT_STOPPED 0 
#define ROBOT_GO 1

//no booleans
#define True 1
#define False 0

//FSM globals

static int robot_run_state = ROBOT_STOPPED; 
//odometer
static int robot_started_flag = False;
static double Odemeter_Limit = 0;
static double Odemeter_Value = 0;

//how fast should Robot go?
static int robotVelocity = 0;
static int robotBackVelocity = 0;

//sonar vars
static double Sonar_minimum =0.20;	//20cm
static double Sonar_maximum =6.0;	//6.0 m , even though the srf10 may max at 11.0 m
static double Sonar_front_middle = 0;
static double Sonar_front_right = 0;
static double Sonar_front_left = 0;
static double Sonar_rear_middle = 0;

///////////////////////////////////////////////////////////////////////////////////////////
void choose_sonar_path( void ); 
int Get_Odemeter(void );
int Get_RAW_AD_VAL(int adpin);	//AD5 = 5

///////////////////////////////////////////////////////////////////////////////////////////
//capture terminal key press

static pthread_t GLOBAL_myAI_Thread_id;
static pthread_mutex_t POS_myAI_Write_Lock = PTHREAD_MUTEX_INITIALIZER;
	
void* myAIThread(void *arg)
{
	pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS,NULL);	

	
	printf ("\33[2J" );			//Clear terminal screen	
	printf ("\33[34;42mNight mode\n" ); 
	//test optics
	Robot_Optics_Night_Mode();
	usleep(1000000);
	Robot_Optics_Indoor_Mode();
	printf ("\33[37;44mIndoor mode\n" );
	usleep(1000000);
	Robot_Optics_Outdoor_Mode();
	printf ("\33[36;44mOutdor mode\n" ); 
	usleep(1000000);
	
for(;;)
{
	while(pthread_mutex_lock(&POS_myAI_Write_Lock) != 0)	//prevent thread lock up
	{		
			usleep(1060);	//odd number for timer
	}
	
printf ("\33[2J" );			//Clear terminal screen
	
	
printf ("\33[30;47m\n" ); 	//black text on white screen
 
	//Check robot state
	Sonar_front_middle = get_sonar_front_center();
	Sonar_front_right = get_sonar_front_right();
	Sonar_front_left = get_sonar_front_left();
	Sonar_rear_middle = get_sonar_rear_center();
	Odemeter_Value =  get_odometer_distance();
	
	printf("\33[1;1HSonar FR: %f\n", Sonar_front_right);
	printf("\33[2;1HSonar FM: %f\n", Sonar_front_middle);
	printf("\33[3;1HSonar FL: %f\n", Sonar_front_left);
	printf("\33[4;1HSonar RM: %f\n", Sonar_rear_middle);
	printf("\33[6;1HDistance: %f\n", Odemeter_Value);
	printf("\33[8;1HVelocity: %f\n", get_odometer_velocity());
	printf("\33[8;25HSetPoint: %i\n", robotVelocity);
			
//Check the switch for start command > 2.5v (held high)
//if(get_raw_AD_Value(dev_AD5) > 1){

		
	if((robot_started_flag == False) && (robot_run_state == ROBOT_STOPPED))
	{

		// set odometer countdown limit, set started flag, and start car.
		Odemeter_Limit = Odemeter_Value + 16; 	//16 Metres limit 
		robot_started_flag = True;	
	}else{
		
		//Check odemeter for distance traveled
		//if reached destination overshoot... then stop robot?
		if(Odemeter_Limit < Odemeter_Value)
		{
			robot_run_state = ROBOT_STOPPED;
		}else{
			robot_run_state = ROBOT_GO;	
		}
	}
//}else{
//	robot_run_state = ROBOT_STOPPED;
//}	




	//Check front left and right sonar 
	if(robot_run_state == ROBOT_GO)
	{
		choose_sonar_path();
		usleep(500000);
	}else{
		robot_run_state = ROBOT_STOPPED;
		Robot_Stop();
		usleep(3000000);	
	}
		
	pthread_mutex_unlock(&POS_myAI_Write_Lock);
}
}


///////////////////////////////////////////////////////////////////////////////////////////
//capture terminal key press
static int c;		//capture keypress 

void GetKeyPress(){
	c = cvWaitKey(0);	//get terminal char

		switch( (char)c )
		{
			case ('['):
			{
				robotVelocity +=5;
				if(robotVelocity > 99){
					robotVelocity=99;
				}
				
				cvCreateTrackbar( "fwdSpeed", "robot", &robotVelocity, 100, NULL );
			break;
			}
			case (']'):
			{
				robotVelocity -=5;
				if(robotVelocity < 0){
					robotVelocity=0;
				}

				cvCreateTrackbar( "fwdSpeed", "robot", &robotVelocity, 100, NULL );
			break;
			}
			case ('o'):
			{
				robotBackVelocity +=5;
				if(robotBackVelocity > 99){
					robotBackVelocity=99;
				}
				
				cvCreateTrackbar( "bckSpeed", "robot", &robotBackVelocity, 100, NULL );
			break;
			}
			case ('p'):
			{
				robotBackVelocity -=5;
				if(robotBackVelocity < 0){
					robotBackVelocity=0;
				}
				
				cvCreateTrackbar( "bckSpeed", "robot", &robotBackVelocity, 100, NULL );
			break;
			}
			
			case (' '):
			{
				robotVelocity=0;
				robotBackVelocity=0;
				cvCreateTrackbar( "fwdSpeed", "robot", &robotVelocity, 100, NULL );
				cvCreateTrackbar( "bckSpeed", "robot", &robotBackVelocity, 100, NULL );
			break;
			}
			case ('g'):
			{
				robotVelocity=30;
				robot_started_flag = False;
				robot_run_state = ROBOT_STOPPED;
				
				cvCreateTrackbar( "fwdSpeed", "robot", &robotVelocity, 100, NULL );
			break;
			}
			case ('x'):
			{
				
				cvDestroyAllWindows();
				exit(0);
				break;
			}
			case ('\x1b'):
			{
				
				cvDestroyAllWindows();
				exit(0);
				break;
			}
			default:
			{
				c=0;	//purge char buffer
			//no-op	
			}
		}
		
		if(c != 0)
		{
			printf("\33[8;25HSetPoint:fwd=%i | bck=%i                   \n", robotVelocity, robotBackVelocity);
		}
	c=0;	//purge char buffer

}

////////////////////////////////////////////////////////////////////////////////////

int main( int argc, char** argv )
{
cvNamedWindow( "robot", CV_WINDOW_AUTOSIZE );
cvCreateTrackbar( "fwdSpeed", "robot", &robotVelocity, 100, NULL );
cvCreateTrackbar( "bckSpeed", "robot", &robotBackVelocity, 100, NULL );
	
bindHALClient(); 	//open the tbrprobe_client network socket
   
// cvNamedWindow( "controls", CV_WINDOW_AUTOSIZE );
tclient_start( DEFAULT_LOCAL_ADDR, UDP_PORT );
while ( !(tclient_is_ready()) )
{
			printf(".");
			usleep(100000);
}


pthread_create(&GLOBAL_myAI_Thread_id, NULL, myAIThread, NULL );
pthread_mutex_unlock(&POS_myAI_Write_Lock);

for(;;){
GetKeyPress();	//go off and capture keypress
}

return 0;
}

///////////////////////////////////////////////////////////////////////////////////////////
//
void choose_sonar_path(void){


//clean up sonar results-------------------------------------------------------------
//check if path option exsists to the left
if ((Sonar_front_left <= 0 )||(Sonar_front_left > Sonar_maximum)){
//vote Left max (sonar has lost the ping -- as it should never be below 5cm)
		Sonar_front_left= Sonar_maximum;
}

//check if path option exsists to the Right
if ((Sonar_front_right <= 0 ) || (Sonar_front_right > Sonar_maximum)){
//vote Right max (sonar has lost the ping -- as it should never be below 5cm)
		Sonar_front_right= Sonar_maximum;
}
	
	
//collision sensor-------------------------------------------------------------
//wall panic, intend to stop	
if ( (Sonar_front_middle < Sonar_minimum) ||
	(Sonar_front_right < Sonar_minimum) ||
	(Sonar_front_left < Sonar_minimum) ){
		
	
	printf("\33[12;1HRobot_Panic_Mode\n");
	//roll stop robot
	Robot_Stop();
	//pause 3 seconds
	usleep(3000000);
		
	//reverse a few seconds?
	if (Sonar_rear_middle > Sonar_minimum) {
		Robot_Reverse(robotBackVelocity);	//always set robot to stop before reverse
		//reverse 
		usleep(1000000);
	}
		
	//stop car, and try again later
	Robot_Stop();
	return;
}


//Basic Nav system-------------------------------------------------------------
//check if front path option exsists
if ((Sonar_front_middle > Sonar_minimum)  &&
	(Sonar_front_middle > Sonar_front_right) &&
	(Sonar_front_middle > Sonar_front_left)  ){
//vote Center
Robot_Turn_To_Center();
Robot_Start(robotVelocity);
printf("\33[12;1HRobot_Turn_To_Center\n");
return;
}

//check if path option exsists  
if (Sonar_front_right > Sonar_front_left){
//vote Right
Robot_Turn_To_Right();	
Robot_Start(robotVelocity);
printf("\33[12;1HRobot_Turn_To_Right\n");
return;
}

//check if path option exsists  
if ( Sonar_front_left > Sonar_front_right){
//vote Left
Robot_Turn_To_Left();	
Robot_Start(robotVelocity);
printf("\33[12;1HRobot_Turn_To_Left\n");
return;
}

Robot_Stop();

printf("\33[12;1HRobot_Error\n");
return;
}

#ifdef _EiC
main(1,"quartz_drag_race.c");
#endif
