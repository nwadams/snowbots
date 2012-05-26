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
#include "raw_analog.h"

//bucket out PLD data
static double var_raw_analog_data[7];

//PLD Vref to convert into voltage approximation
#define PLD_A2D_Vref 5.0
#define PLD_A2D_step 1024.0

//////////
//These items are static globals to allow persistance during multi-threaded use
// In this case it is faster than inter thread meassage systems, but could be implemented better.
static pthread_mutex_t Raw_Analog_Write_Lock  = PTHREAD_MUTEX_INITIALIZER;
	
//------------------------------------------------------------------------------------------------------------------------------------------------------
//setup
void Init_Raw_Analog(void)
{
	int ii;
	//reset mutexs
	pthread_mutex_unlock(&Raw_Analog_Write_Lock);

	for(ii=0; ii < 6;ii++){		//clear samples to error state
	var_raw_analog_data[ii]=-1;
	}
	
}

//------------------------------------------------------------------------------------------------------------------------------------------------------
//get voltage
double Get_Raw_Analog(int indexid) 
{
return (var_raw_analog_data[(indexid%6)]);
}


//------------------------------------------------------------------------------------------------------------------------------------------------------
//set state as raw data is passed in
void Update_Raw_Analog(int rawdata, int indexid)
{
	while(pthread_mutex_lock(&Raw_Analog_Write_Lock) != 0)	//prevent thread lock up
	{		
			usleep(133+(rawdata%13));	//odd number for timer
	}
	
	if(rawdata > 0) 
	{
		var_raw_analog_data[indexid] = (rawdata*(PLD_A2D_Vref/PLD_A2D_step)); 	//convert to voltage

	}else{	//if raw data < 0 then an error occured in the sample(or grounded)			
		var_raw_analog_data[indexid] = 0; 
	}
	
	

	#if defined(DEBUG_LEVEL_0) 	
	//yes this will be 5 times as redundant....
		printf ("\33[12;1HRAW AD0=%.2f V(IR) AD1=%.2f V(IR) AD2=%.2f V(IR) \33[13;1H    AD3=%.2f V(?) AD4=%.2f V(?) AD5=%.2f V(Pause Switch)",
				var_raw_analog_data[AD0],
				var_raw_analog_data[AD1],
				var_raw_analog_data[AD2],
				var_raw_analog_data[AD3],
				var_raw_analog_data[AD4],
				var_raw_analog_data[AD5]
			);
	#endif
	
	pthread_mutex_unlock(&Raw_Analog_Write_Lock);	//enable other threads to write state
}
