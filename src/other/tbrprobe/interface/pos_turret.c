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
//The turret requires defined macros in robot_config.h to operate
#include "../robot_config.h" 
#include "pos_turret.h" 

#define TURRET_SCALE_RESOLUTION 100.0
//note GCC type casting limits % binops... thus we dupe...
#define TURRET_SCALE_RESOLUTION_INT 100

//////////
//cast constants
#define EVILMILLION 1000000.0
#define EVILKM 1000.0




//////////
//These items are static globals to allow persistance during multi-threaded use
// In this case it is faster than inter thread meassage systems, but could be implemented better.
	static pthread_mutex_t POS_Turret_Write_Lock = PTHREAD_MUTEX_INITIALIZER;

/////////////
//These items control the abstract throttle direction and the -100%-0-100% settings
	static int POS_Turret_Pitch_Percent;
	static int POS_Turret_Yaw_Percent;


//////////////////////////////////////////////////////////////////////////////////////////
//reset the tracker to default ready state
void Init_Turret(void)
{
	//reset mutexs
	pthread_mutex_unlock(&POS_Turret_Write_Lock);
	POS_Turret_Pitch_Percent=0;
	POS_Turret_Yaw_Percent=0;
}


//////////////////
void Set_Turret_Yaw_Setting(int rawdata)  //set abtract positon device -100% to 0 to 100%
{
 	while(pthread_mutex_lock(&POS_Turret_Write_Lock) != 0)	//prevent thread lock up
	{		
			usleep(39+(rawdata%16));	//odd number for timer
	}

	POS_Turret_Yaw_Percent=rawdata%100;

	pthread_mutex_unlock(&POS_Turret_Write_Lock);
	return;
}

/////////////////////
void Set_Turret_Pitch_Setting(int rawdata)  //set abtract positon device -100% to 0 to 100%
{
 	while(pthread_mutex_lock(&POS_Turret_Write_Lock) != 0)	//prevent thread lock up
	{		
			usleep(25+(rawdata%16));	//odd number for timer
	}
 	POS_Turret_Pitch_Percent=rawdata%100;

	pthread_mutex_unlock(&POS_Turret_Write_Lock);
	return;
}

/////////////////////
void Update_Turret(int pitch, int yaw)  //set abtract positon device -100% to 0 to 100%
{
 	while(pthread_mutex_lock(&POS_Turret_Write_Lock) != 0)	//prevent thread lock up
	{		
			usleep(29+(pitch%16));	//odd number for timer
	}
 
 	POS_Turret_Pitch_Percent=pitch%100;
	POS_Turret_Yaw_Percent=yaw%100;

	pthread_mutex_unlock(&POS_Turret_Write_Lock);
	return;
}
 	  
/////////////////////
int Get_Turret_Yaw_Setting(void) //Result in servo corrected setting to send
{
	
	if(POS_Turret_Yaw_Percent > 0)	//return a real setting
	{
		return (POS_Turret_Yaw_Percent * TURRET_YAW_FORDWARD_INDEX)
							+TURRET_YAW_SERVO_CENTER;
	}else{
		return (POS_Turret_Yaw_Percent * TURRET_YAW_BACKWARD_INDEX)
							+TURRET_YAW_SERVO_CENTER;
	}
	
	
		#if defined(DEBUG_LEVEL_0) 	
		printf ("\33[15;1HTurret: Pitch=%i%% Yaw=%i%%", 
				POS_Turret_Pitch_Percent,
				POS_Turret_Yaw_Percent
			);	
		#endif
	
	return 0;	//?dumb compiler
}


//////////////////////
int Get_Turret_Pitch_Setting(void) //Result in servo corrected setting to send
{
	
	if(POS_Turret_Pitch_Percent > 0)	//return a real setting
	{
		return (POS_Turret_Pitch_Percent * TURRET_PITCH_FORDWARD_INDEX)
							+TURRET_PITCH_SERVO_CENTER;
	}else{
		return (POS_Turret_Pitch_Percent * TURRET_PITCH_BACKWARD_INDEX)
							+TURRET_PITCH_SERVO_CENTER;
	}
	
	return 0;	//?dumb compiler
}






