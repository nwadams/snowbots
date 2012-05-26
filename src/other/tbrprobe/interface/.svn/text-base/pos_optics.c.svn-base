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
#include "pos_optics.h" 



//////////
//These items are static globals to allow persistance during multi-threaded use
// In this case it is faster than inter thread meassage systems, but could be implemented better.
	static pthread_mutex_t POS_Optics_Write_Lock = PTHREAD_MUTEX_INITIALIZER;

/////////////
//These items control the abstract throttle direction and the -100%-0-100% settings
	static int POS_Optical_state_Percent;


//////////////////////////////////////////////////////////////////////////////////////////
//reset the optics to default ready state

void Init_Optics(void)
{
	//reset mutexs
	pthread_mutex_unlock(&POS_Optics_Write_Lock);
	POS_Optical_state_Percent=0;

}


//////////////////
 void Set_Optics_Setting(int rawdata)  //set abstract positon of servo
{
 	while(pthread_mutex_lock(&POS_Optics_Write_Lock) != 0)	//prevent thread lock up
	{		
			usleep(34+(rawdata%16));	//odd number for timer
	}

	POS_Optical_state_Percent=rawdata%100;

	pthread_mutex_unlock(&POS_Optics_Write_Lock);
	return;
}
 

 	  
/////////////////////
int Get_Optics_Setting(void) //Result in servo corrected setting to send
{
	
	if(POS_Optical_state_Percent > 0)	//return a real setting
	{
		return (POS_Optical_state_Percent * OPTICS_FILTER_ARM_FORDWARD_INDEX)
							+ OPTICS_FILTER_ARM_SERVO_CENTER;
	}else{
		return (POS_Optical_state_Percent * OPTICS_FILTER_ARM_BACKWARD_INDEX)
							+OPTICS_FILTER_ARM_SERVO_CENTER;
	}
	
		#if defined(DEBUG_LEVEL_0) 	
		printf ("\33[10;1HOptics: FILTER=%i%%", 
				POS_Optical_state_Percent 
			);	
		#endif
	
	return 0;	//?dumb compiler
}

