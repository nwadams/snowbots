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
#include "../robot_config.h" 
#include "sonar_prox.h"

//add more zones as needed for a max 16 to reflect the srf bus limits
// 1cm per index number for the srf08/srf10 sonar modules
// 100 cm per meters
#define SONAR_PROX_SCALE_TRANSLATE_TO_METERS 0.01

static int var_sonar_prox_data[18];


//////////
//These items are static globals to allow persistance during multi-threaded use
// In this case it is faster than inter thread meassage systems, but could be implemented better.
static pthread_mutex_t Sonar_Prox_Write_Lock  = PTHREAD_MUTEX_INITIALIZER;
	
//------------------------------------------------------------------------------------------------------------------------------------------------------
//setup
void Init_Sonar_Prox(void)	//reset to default ready state
{
	int ii;
	//reset mutexs
	pthread_mutex_unlock(&Sonar_Prox_Write_Lock);

	for(ii=0; ii < 16;ii++){		//clear sonar samples to error state
	var_sonar_prox_data[ii]=-1;
	}
	
}

//------------------------------------------------------------------------------------------------------------------------------------------------------
//get state distance in meters
double Get_Sonar_Distance(int indexid) 
{
return ((var_sonar_prox_data[(indexid%17)] ) * SONAR_PROX_SCALE_TRANSLATE_TO_METERS); 
}




//------------------------------------------------------------------------------------------------------------------------------------------------------
void Print_Sonar_Data(void)
{
	while(pthread_mutex_lock(&Sonar_Prox_Write_Lock) != 0)	//prevent thread lock up
	{		
			usleep(1177);	//odd number for timer
	}

	#if defined(DEBUG_LEVEL_0) 	
		printf ("\33[14;1HSonars:" );		//move cursor to row 16 and  column 1
	
		int ii;
		ii=var_sonar_prox_data[SONAR_LOCPROXNW];
		printf ( "\33[15;1HNorthWest=%i cm",ii);
	
		ii=var_sonar_prox_data[SONAR_LOCPROXN];
		printf ( "\33[16;1HNorth=%i cm",ii);
	
		ii=var_sonar_prox_data[SONAR_LOCPROXNE];
		printf ( "\33[17;1HNorthEast=%i cm",ii);
	
		ii=var_sonar_prox_data[SONAR_LOCPROXW];
		printf ( "\33[18;1HWest=%i cm",ii);
	
		ii=var_sonar_prox_data[SONAR_LOCPROXE];
		printf ( "\33[19;1HEast=%i cm",ii);
	
		ii=var_sonar_prox_data[SONAR_LOCPROXSE];
		printf ( "\33[20;1HSouthEast=%i cm",ii);
		
		ii=var_sonar_prox_data[SONAR_LOCPROXS];
		printf ( "\33[21;1HSouth=%i cm",ii);
		
		ii=var_sonar_prox_data[SONAR_LOCPROXSW];
		printf("\33[22;1HSouthWest=%i cm", ii);
	#endif
	pthread_mutex_unlock(&Sonar_Prox_Write_Lock);	//enable other threads to write state
		
}

//------------------------------------------------------------------------------------------------------------------------------------------------------
/**
set abstract state as raw data is passed in
@param rawdata The raw data sent by the sonar
@param indexid The constant - defined in sonar_prox.h - that defines a sonar position.
    Do not confuse this with the sonar's bus id, which is programmed into the sonar firmware.
**/
void Set_Sonar_Data(int rawdata, char indexid)
{
	int bugfix =indexid;
	
	while(pthread_mutex_lock(&Sonar_Prox_Write_Lock) != 0)	//prevent thread lock up
	{		
			usleep(116);	//odd number for timer
	}
	
	if(rawdata > 0) 
	{
		var_sonar_prox_data[bugfix] = rawdata; 

	}else{	//if raw data < 0 then an error occured in the sonar's i2c bus, or COM link
			
		var_sonar_prox_data[bugfix] = 0; 	//TODO: respond the upper level with error sate as defined sonar is not working/assigned
	}
	

	pthread_mutex_unlock(&Sonar_Prox_Write_Lock);	//enable other threads to write state
}








