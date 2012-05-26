#include "nimu_control.h"
#include "serial232.h"
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <pthread.h>
#include <time.h>

pthread_t nIMU_Thread_id;              //Thread parts
pthread_mutex_t nIMU_Mutex = PTHREAD_MUTEX_INITIALIZER;
void* nIMUThread();
static int nIMU_Port = -1;
static int nIMU_Port_fd = 0;
static int nIMU_TX = 0;
static int nIMU_RX = 0;
static char *nIMU_BufferInPtr;
static char *nIMU_BufferOutPtr;

void nIMU_Send_Command(char PktArr[]);

//////////////////////////////////////////////////////////////////////////////
//nIMU HAL

void* nIMUThread(void *arg)
{
	pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS,NULL);
	//int ii =0;
	char someGenericCommand[1000]={""};
	struct termios oldtermios;
	
	nIMU_Port = *((int *) arg);
	//	printf("Connecting to IMU on port %d.\n", nIMU_Port);

	usleep(5000000);						//needed to allow probe dispatch to close the port
	nIMU_Port_fd= Serial_Open(nIMU_Port,19200,&oldtermios);	//try to open port

	for(;(nIMU_Port != -1);)
	{
			
			 strcpy(someGenericCommand, "Z\0\0");
			 nIMU_Send_Command(someGenericCommand); 
			 printf("Response %s from nIMU Port ID:%i\n",someGenericCommand, nIMU_Port);

	}
	
	pthread_exit(0);
	return 0;
}


//////////////////////////////////////////////////////////////////////////////
//send to bound port

void nIMU_Send_Command(char PktArr[])
{
	char PktBuff[255]={""};
	int bufferScanIndex=0;
	int inputIndex=0;
	int timeout=0;
	

	
	if((nIMU_Port_fd < 1)&&(nIMU_Port < 0))
	{
		printf("No nIMU on port on ID:%i\n",nIMU_Port);
		nIMU_Port = -1;
		return;
	} 
	
		nIMU_BufferOutPtr=PktArr;				//Prepares a command packet to be sent
		Serial_Write(nIMU_Port_fd, nIMU_BufferOutPtr, strlen(PktArr), &nIMU_TX);		//send word to slave device

		if(nIMU_TX < 1)
		{
			printf("Error: can't write to nIMU Port ID:%i\n", nIMU_Port);
			nIMU_Port = -1;
			return;
		}
		
	
		nIMU_BufferInPtr=PktArr;				//Prepares for packet input
		inputIndex=0;
		bufferScanIndex=0;
		PktBuff[0]='\0';
		
		for(timeout = 10000; (timeout>0); timeout--)
		{
			nIMU_RX= Serial_Read(nIMU_Port_fd, PktBuff,127);	//ONE MUST ALWAYS FLUSH INPUT BUFFER
			PktBuff[nIMU_RX] = '\0';					//make buffer a string
		
			for(bufferScanIndex = 0; (bufferScanIndex < nIMU_RX); bufferScanIndex++)
			{ 
					if(((PktBuff[bufferScanIndex] >= '0') &&
					(PktBuff[bufferScanIndex] <= '9'))	||
					(PktBuff[bufferScanIndex] == ',') ||
					(PktBuff[bufferScanIndex] == '-') ||
					(PktBuff[bufferScanIndex] == nIMU_KeySig) )
					{	
						nIMU_BufferInPtr[inputIndex] = PktBuff[bufferScanIndex];	//scan in char
						inputIndex++;	//index global buffer		
					}	
					nIMU_BufferInPtr[inputIndex] = '\0';	//always a string
						
					//Scan buffer for known valid token END responses 
					if(PktBuff[bufferScanIndex] == nIMU_KeySig)
					{
						return;	
					}
			}
		
		}
		
	printf("No data response from nIMU on Port ID:%i\n", nIMU_Port);
	nIMU_Port = -1;
		
}




