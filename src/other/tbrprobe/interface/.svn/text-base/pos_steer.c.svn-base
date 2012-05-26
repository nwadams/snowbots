#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <pthread.h>
#include <string.h>
//The Steer servo requires the defined macros in robot_config.h to operate
#include "../robot_config.h" 
#include "pos_steer.h" 

#define STEER_ANGLE_SCALE_RESOLUTION 100.0
//note GCC type casting limits % binops... thus we dupe...
#define STEER_ANGLE_SCALE_RESOLUTION_INT 100

//scale left and right (leave as vars for adaptive feedback config later)
static double POS_Steer_Scale_Left;
static double POS_Steer_Scale_Right;

//TODO: add servo setting confirm cycle

//current servo angle
static double POS_Steer_Current_Setting;
static double POS_Steer_Current_Abstract_Setting;


//////////
//These items are static globals to allow persistance during multi-threaded use
// In this case it is faster than inter thread meassage systems, but could be implemented better.
static pthread_mutex_t POS_Steer_Write_Lock  = PTHREAD_MUTEX_INITIALIZER;
	
//------------------------------------------------------------------------------------------------------------------------------------------------------
//setup
void Init_POS_Steer(void)	//reset the steering to default ready state
{
	//reset mutexs
	pthread_mutex_unlock(&POS_Steer_Write_Lock);
	
	//scale right
	#if (MOTOR_STEER_SERVO_MAX_RIGHT  > MOTOR_STEER_SERVO_CENTER)
		POS_Steer_Scale_Right = (MOTOR_STEER_SERVO_MAX_RIGHT-MOTOR_STEER_SERVO_CENTER)/STEER_ANGLE_SCALE_RESOLUTION;
	#else
		POS_Steer_Scale_Right = (MOTOR_STEER_SERVO_CENTER-MOTOR_STEER_SERVO_MAX_RIGHT)/STEER_ANGLE_SCALE_RESOLUTION;
	#endif
	
	//scale left
	#if (MOTOR_STEER_SERVO_MAX_LEFT  > MOTOR_STEER_SERVO_CENTER)
		POS_Steer_Scale_Left = (MOTOR_STEER_SERVO_MAX_LEFT-MOTOR_STEER_SERVO_CENTER)/STEER_ANGLE_SCALE_RESOLUTION;
	#else
		POS_Steer_Scale_Left = (MOTOR_STEER_SERVO_CENTER-MOTOR_STEER_SERVO_MAX_LEFT)/STEER_ANGLE_SCALE_RESOLUTION;
	#endif

	POS_Steer_Current_Setting = MOTOR_STEER_SERVO_CENTER;
	POS_Steer_Current_Abstract_Setting = 0;
	
}

//------------------------------------------------------------------------------------------------------------------------------------------------------
//get state
int Get_Steer_Angle(void)		//correct setting for control of Servo Steer device
{
return abs(POS_Steer_Current_Setting * 1);	//round to INT
}


//------------------------------------------------------------------------------------------------------------------------------------------------------
//set abstract state
void Set_Steer_Angle(int rawdata)		//setting abtract control of  Servo Steer device -50% to 0 to 50%
{
	while(pthread_mutex_lock(&POS_Steer_Write_Lock) != 0)	//prevent thread lock up
	{		
			usleep(1087+(rawdata%10));	//odd number for timer
	}
	
	//abstract input for steering may be inverted with STEER_ANGLE_INVERTED = -1 for some platforms like traxxas
	POS_Steer_Current_Abstract_Setting = (STEER_ANGLE_INVERTED*rawdata);

	//filter out bad data on input
	if(rawdata < -STEER_ANGLE_SCALE_RESOLUTION_INT){
	rawdata = -STEER_ANGLE_SCALE_RESOLUTION_INT;
	}
	
	if(rawdata > STEER_ANGLE_SCALE_RESOLUTION_INT){
	rawdata = STEER_ANGLE_SCALE_RESOLUTION_INT;
	}
	
	
	if(rawdata == 0)
	{
		POS_Steer_Current_Setting = MOTOR_STEER_SERVO_CENTER;
	}else{

		
			if(rawdata > 0) 		//Turn RIGHT
			{

					#if defined(duratrax)
						POS_Steer_Current_Setting = MOTOR_STEER_SERVO_CENTER + (POS_Steer_Scale_Right * (POS_Steer_Current_Abstract_Setting));
					#elif defined(traxxas) || defined(snowtires) || defined(snowdrift)
						//reversed steering
						POS_Steer_Current_Setting = MOTOR_STEER_SERVO_CENTER - (POS_Steer_Scale_Right * (POS_Steer_Current_Abstract_Setting));
					#elif defined(snowfury)
						POS_Steer_Current_Setting = MOTOR_STEER_SERVO_CENTER + (POS_Steer_Scale_Right * (POS_Steer_Current_Abstract_Setting));
					#endif	 
			}else{			//Turn LEFT
				
					POS_Steer_Current_Abstract_Setting *=(-1.0);		//flip sign...  [-1]   =>  [+1] 
				
					#if defined(duratrax)
						POS_Steer_Current_Setting = MOTOR_STEER_SERVO_CENTER - (POS_Steer_Scale_Left * (POS_Steer_Current_Abstract_Setting));
					#elif defined(traxxas) || defined(snowtires) || defined(snowdrift)
						//reversed steering
						POS_Steer_Current_Setting = MOTOR_STEER_SERVO_CENTER + (POS_Steer_Scale_Left * (POS_Steer_Current_Abstract_Setting)); 
					#elif defined(snowfury)
						POS_Steer_Current_Setting = MOTOR_STEER_SERVO_CENTER - (POS_Steer_Scale_Left * (POS_Steer_Current_Abstract_Setting));
					#endif 
			}
	}
	

	pthread_mutex_unlock(&POS_Steer_Write_Lock);	//enable other threads to write state
}
