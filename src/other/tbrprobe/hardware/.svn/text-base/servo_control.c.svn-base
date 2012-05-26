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
#include "servo_control.h" 
#include "serial232.h"
#include "../robot_config.h"  



pthread_t Servo_Thread_id;              //Thread parts
pthread_mutex_t Servo_Mutex  = PTHREAD_MUTEX_INITIALIZER;
void* ServoThread();
static int Servo_Port = -1;
static int Servo_Port_fd = 0;
static int Servo_TX = 0;
static int Servo_RX = 0;
static char *Servo_BufferInPtr;
static char *Servo_BufferOutPtr;
void Servo_Send_Command(char arr[]);		//send packet

//////////////////////////////////////////////////////////////////////////////
//SERVO HAL
void* ServoThread(void *arg)
{
	pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS,NULL);
	int ii =0; 
	char someGenericCommand[1000]={""};
	char someTmp[1000]={""};
	struct termios oldtermios;
	Servo_Port = *((int *) arg);
	
	usleep(5000000);						//needed to allow probe dispatch to close the port
	Servo_Port_fd= Serial_Open(Servo_Port,19200,&oldtermios);	//try to open port
	
	//config for device 1
	//17,0,0,0,0,0,0,0,0,0,0,250,250,S
	//read device 1
	//1,0,0,0,0,0,0,0,0,0,0,250,250,S
	
	//MOTOR_DRIVE_SERVO_ESC_CONFIG Setup   /////////////////////////////////////////////////////////
	//After a power cycle, the Duratrax ESC as must do an auto-setup...
	//Some ESC require a STOP signal upon power up to config the neutral logic 
	//Note: Green light on ESC will blink while waiting to boot. 
	//         (If already setup it willl show steady green)
	if(MOTOR_DRIVE_SERVO_ESC_CONFIG == 1)
	{
		strcpy(someGenericCommand, "0,0,0,0,0,0,0,0,0,0,0,\0");	//create message passer
		sprintf(someTmp, "%i",MOTOR_DRIVE_SERVO_STOP );	//neutral drive
		strcat(someTmp, ",\0");	//add comma
		strcat(someGenericCommand, someTmp);	
		sprintf(someTmp, "%i",MOTOR_STEER_SERVO_CENTER);	//neutral steering
		strcat(someTmp, ",\0");	//add comma
		strcat(someGenericCommand, someTmp);	
		strcat(someGenericCommand, "S\0");		//end of packet
		Servo_Send_Command(someGenericCommand); 
		//force user to power cycle the ESC?
//		printf("\n+++++++ TURN ON THE Duratrax ESC... THEN PRESS ENTER TO CONTINUE +++++++ ");
//		someChar = getchar();
		printf("\nNow booting the Duratrax ESC... ");
		usleep(5000000);						//needed to setup the config signal over 3 seconds
	}
	//End Setup ////////////////////////////////////////////////////////////////////////////////////////////
	
	
	for(;(Servo_Port != -1);)
	{
		strcpy(someGenericCommand, "0,0,0,0,0,0,0,0,0,0,0,\0");	//create message passer
		sprintf(someTmp, "%i",MOTOR_DRIVE_SERVO_STOP );	//neutral drive
		strcat(someTmp, ",\0");	//add comma
		strcat(someGenericCommand, someTmp);	
		sprintf(someTmp, "%i",MOTOR_STEER_SERVO_CENTER);	//neutral steering
		strcat(someTmp, ",\0");	//add comma
		strcat(someGenericCommand, someTmp);	
		strcat(someGenericCommand, "S\0");		//end of packet
		Servo_Send_Command(someGenericCommand); 
		
		printf("Response from Servo Port ID:%i\n%s \n",Servo_Port, someGenericCommand );
		usleep(5000000);	//needed
		
		
		// Start Drive Example code =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
		//forwards
		for(ii=MOTOR_DRIVE_SERVO_STOP ; (ii < MOTOR_DRIVE_SERVO_MAX_FWD) ; )
		{
			strcpy(someGenericCommand, "0,0,0,0,0,0,0,0,0,0,0,\0");	//create message passer

			sprintf(someTmp, "%i", ii);	//convert number to string
			strcat(someTmp, ",\0");	//add comma
			strcat(someGenericCommand, someTmp);	
			
			sprintf(someTmp, "%i",MOTOR_STEER_SERVO_CENTER);	//neutral steering
			strcat(someTmp, ",\0");	//add comma
			strcat(someGenericCommand, someTmp);	
			
			strcat(someGenericCommand, "S\0");		//end of packet
		
			printf("SEND %s \n",someGenericCommand);
			Servo_Send_Command(someGenericCommand); 
			printf("Response %s from Servo Port ID:%i\n",someGenericCommand, Servo_Port);
			usleep(200000);	//needed
			
			if(MOTOR_DRIVE_SERVO_MAX_FWD  > MOTOR_DRIVE_SERVO_STOP)
			{
				ii+=5;
			}else{
				ii-=5;
			}
		}
		
		//reverse
		for(ii=MOTOR_DRIVE_SERVO_STOP ; (ii > MOTOR_DRIVE_SERVO_MAX_REV) ; )
		{
			strcpy(someGenericCommand, "0,0,0,0,0,0,0,0,0,0,0,\0");	//create message passer

			sprintf(someTmp, "%i", ii);	//convert number to string
			strcat(someTmp, ",\0");	//add comma
			strcat(someGenericCommand, someTmp);	
			
			sprintf(someTmp, "%i",MOTOR_STEER_SERVO_CENTER);	//neutral steering
			strcat(someTmp, ",\0");	//add comma
			strcat(someGenericCommand, someTmp);	
			
			strcat(someGenericCommand, "S\0");		//end of packet
		
			printf("SEND %s \n",someGenericCommand);
			Servo_Send_Command(someGenericCommand); 
			printf("Response %s from Servo Port ID:%i\n",someGenericCommand, Servo_Port);
			usleep(200000);	//needed
			
			if(MOTOR_DRIVE_SERVO_MAX_REV  > MOTOR_DRIVE_SERVO_STOP)
			{
				ii+=5;
			}else{
				ii-=5;
			}
		}
		
	// Start STEER Example code =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
		//LEFT
		for(ii=MOTOR_STEER_SERVO_CENTER ; (ii < MOTOR_STEER_SERVO_MAX_LEFT) ; )
		{
			strcpy(someGenericCommand, "0,0,0,0,0,0,0,0,0,0,0,\0");	//create message passer

			sprintf(someTmp, "%i",MOTOR_DRIVE_SERVO_STOP);	//neutral steering
			strcat(someTmp, ",\0");	//add comma
			strcat(someGenericCommand, someTmp);	
			
			sprintf(someTmp, "%i", ii);	//convert number to string
			strcat(someTmp, ",\0");	//add comma
			strcat(someGenericCommand, someTmp);	
			
			strcat(someGenericCommand, "S\0");		//end of packet
		
			printf("SEND %s \n",someGenericCommand);
			Servo_Send_Command(someGenericCommand); 
			printf("Response %s from Servo Port ID:%i\n",someGenericCommand, Servo_Port);
			usleep(200000);	//needed
			
			if(MOTOR_STEER_SERVO_MAX_LEFT  > MOTOR_STEER_SERVO_CENTER)
			{
				ii+=2;
			}else{
				ii-=2;
			}
		}
		
		//RIGHT
		for(ii=MOTOR_STEER_SERVO_CENTER ; (ii > MOTOR_STEER_SERVO_MAX_RIGHT) ; )
		{
			strcpy(someGenericCommand, "0,0,0,0,0,0,0,0,0,0,0,\0");	//create message passer

			sprintf(someTmp, "%i",MOTOR_DRIVE_SERVO_STOP);	//neutral steering
			strcat(someTmp, ",\0");	//add comma
			strcat(someGenericCommand, someTmp);	
			
			sprintf(someTmp, "%i", ii);	//convert number to string
			strcat(someTmp, ",\0");	//add comma
			strcat(someGenericCommand, someTmp);	
			
			strcat(someGenericCommand, "S\0");		//end of packet
		
			printf("SEND %s \n",someGenericCommand);
			Servo_Send_Command(someGenericCommand); 
			printf("Response %s from Servo Port ID:%i\n",someGenericCommand, Servo_Port);
			usleep(200000);	//needed
			
			if(MOTOR_STEER_SERVO_MAX_RIGHT  > MOTOR_STEER_SERVO_CENTER)
			{
				ii+=2;
			}else{
				ii-=2;
			}
		}		
		
		
	}
	
	
	printf("\n ?????????????????????? EXIT SERVO! ???????????????????????? \n");
	pthread_exit(0);
	return 0;
}
	
//////////////////////////////////////////////////////////////////////////////
//send to bound port

void Servo_Send_Command(char PktArr[])
{
	char PktBuff[255]={""};
	int bufferScanIndex=0;
	int inputIndex=0;
	int timeout=0;
	
	
	if((Servo_Port_fd < 1)&&(Servo_Port < 0))
	{
		printf("No Servo on port on ID:%i\n", Servo_Port);
		Servo_Port = -1;
		return;
	} 
	
	
		Servo_BufferOutPtr=PktArr;				//Prepares a command packet to be sent
		Serial_Write(Servo_Port_fd,Servo_BufferOutPtr,strlen(PktArr), &Servo_TX);		//send word to slave device

		if(Servo_TX < 1)
		{
			printf("Error: can't write to Servo Port ID:%i\n", Servo_Port);
			Servo_Port = -1;
			return;
		}
		
	
		Servo_BufferInPtr=PktArr;				//Prepares for packet input
		inputIndex=0;
		bufferScanIndex=0;
		PktBuff[0]='\0';
		
		for(timeout = 20000; (timeout>0); timeout--)
		{
			Servo_RX= Serial_Read(Servo_Port_fd, PktBuff,127);	//ONE MUST ALWAYS FLUSH INPUT BUFFER
			PktBuff[Servo_RX] = '\0';					//make buffer a string
		
			for(bufferScanIndex = 0; (bufferScanIndex < Servo_RX); bufferScanIndex++)
			{ 
					if(((PktBuff[bufferScanIndex] >= '0') &&
					(PktBuff[bufferScanIndex] <= '9'))	||
					(PktBuff[bufferScanIndex] == ',') ||
					(PktBuff[bufferScanIndex] == '-') ||
					(PktBuff[bufferScanIndex] == Servo_KeySig) )
					{	
						Servo_BufferInPtr[inputIndex] = PktBuff[bufferScanIndex];	//scan in char
						inputIndex++;	//index global buffer			
					}
					Servo_BufferInPtr[inputIndex] = '\0';	//always a string
						
					//Scan buffer for known valid token END responses 
					if(PktBuff[bufferScanIndex] == Servo_KeySig)
					{
						return;	
					}
			}
		
		}
		
	printf("No data response from Servo on Port ID:%i\n", Servo_Port);
	Servo_Port = -1;
		
}
