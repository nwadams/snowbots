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
#include "motor_control.h"
#include "serial232.h"

pthread_t Motor_Thread_id;              //Thread parts
pthread_mutex_t Motor_Mutex = PTHREAD_MUTEX_INITIALIZER;
void* MotorThread();
static int Motor_Port = -1;
static int Motor_Port_fd = 0;
static int Motor_TX = 0;
static int Motor_RX = 0;
static int Stepper_X = 0;
static int Stepper_Y = 0;
static int Stepper_X_Buffer = 0;
static int Stepper_Y_Buffer = 0;
static char *Motor_BufferInPtr;
static char *Motor_BufferOutPtr;

int stepper(int* turn);		//convert current phase into steeper control output pattern

void Motor_Send_Command(char arr[]);		//send packet

	
//////////////////////////////////////////////////////////////////////////////
//Motor HAL

void* MotorThread(void *arg)
{
	pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS,NULL);
	int ii =0;	
	char someGenericCommand[1000]={""};
	struct termios oldtermios;
	Motor_Port = *((int *) arg);
	
	usleep(5000000);						//needed to allow probe dispatch to close the port
	Motor_Port_fd= Serial_Open(Motor_Port,19200,&oldtermios);	//try to open port

	strcpy(someGenericCommand, "0,1,15,1,15,X\0");	//create message passer
	Motor_Send_Command(someGenericCommand);		//poll
	usleep(100000);

	strcpy(someGenericCommand, "0,1,25,1,25,X\0");
	Motor_Send_Command(someGenericCommand);
	usleep(100000);
		
	strcpy(someGenericCommand, "0,0,0,0,0,X\0");
	Motor_Send_Command(someGenericCommand);
	usleep(100000);
		

	for(;(Motor_Port != -1);)
	{
		
		 printf("X");
		for(ii=0; ii < 100;ii++)
		{
			Stepper_X++;	//X rot forward
			Stepper_Y++;	//X rot forward
			Stepper_X_Buffer = stepper(&Stepper_X);		//get pattern for X 
			Stepper_Y_Buffer = stepper(&Stepper_Y);		//get pattern for Y
			Stepper_X_Buffer = ((Stepper_X_Buffer << 0x08) | Stepper_Y_Buffer);	//combine data  b'XXXXYYYY'
				
			 sprintf(someGenericCommand, "%i", Stepper_X_Buffer);	//convert to string
			 strcat(someGenericCommand, ",1,64,0,0,X");
			
			 Motor_Send_Command(someGenericCommand);
			 printf("Response %s from Motor Port ID:%i\n",someGenericCommand, Motor_Port);
			
		}

		
		for(ii=0; ii < 100;ii++)
		{
			Stepper_X--;	//X rot backwards
			Stepper_Y--;	//X rot backwards
			Stepper_X_Buffer = stepper(&Stepper_X);		//get pattern for X 
			Stepper_Y_Buffer = stepper(&Stepper_Y);		//get pattern for Y
			Stepper_X_Buffer = ((Stepper_X_Buffer << 0x08) | Stepper_Y_Buffer);	//combine data
				
			 sprintf(someGenericCommand, "%i", Stepper_X_Buffer);
			 strcat(someGenericCommand, ",0,0,1,64,X");
			
			 Motor_Send_Command(someGenericCommand); 
			 printf("Response %s from Motor Port ID:%i\n",someGenericCommand, Motor_Port);
				
		}

	}
	
	pthread_exit(0);
	return 0;
}



//////////////////////////////////////////////////////////////////////////////
//send to bound port

void Motor_Send_Command(char PktArr[])
{
	char PktBuff[255]={""};
	int bufferScanIndex=0;
	int inputIndex=0;
	int timeout=0; 
	
	if((Motor_Port_fd < 1)&&(Motor_Port < 0))
	{
		printf("No Motor on port on ID:%i\n",Motor_Port);
		Motor_Port = -1;
		return;
	} 
	
	
		Motor_BufferOutPtr=PktArr;				//Prepares a command packet to be sent
		Serial_Write(Motor_Port_fd,Motor_BufferOutPtr,strlen(PktArr), &Motor_TX);		//send word to slave device

		if(Motor_TX < 1)
		{
			printf("Error: can't write to Motor Port ID:%i\n", Motor_Port);
			Motor_Port = -1;
			return;
		}
		
	
		Motor_BufferInPtr=PktArr;				//Prepares for packet input
		inputIndex=0;
		bufferScanIndex=0;
		PktBuff[0]='\0';
		
		for(timeout = 10000; (timeout>0); timeout--)
		{	
			Motor_RX= Serial_Read(Motor_Port_fd, PktBuff,127);	//ONE MUST ALWAYS FLUSH INPUT BUFFER
			PktBuff[Motor_RX] = '\0';					//make buffer a string
		
			for(bufferScanIndex = 0; (bufferScanIndex < Motor_RX); bufferScanIndex++)
			{ 
					if(((PktBuff[bufferScanIndex] >= '0') &&
					(PktBuff[bufferScanIndex] <= '9'))	||
					(PktBuff[bufferScanIndex] == ',') ||
					(PktBuff[bufferScanIndex] == '-') ||
					(PktBuff[bufferScanIndex] == Motor_KeySig) )
					{	
						Motor_BufferInPtr[inputIndex] = PktBuff[bufferScanIndex];	//scan in char
						inputIndex++;	//index global buffer			
					}
					Motor_BufferInPtr[inputIndex] = '\0';	//always a string
						
					//Scan buffer for known valid token END responses 
					if(PktBuff[bufferScanIndex] == Motor_KeySig)
					{
						return;	
					}
			}
		
		}
		
	printf("No data response from Motor on Port ID:%i\n", Motor_Port);
	Motor_Port = -1;
		
}

//////////////////////////////////////////////////////////////////////////////
//convert step index to output pattern

int stepper(int* turn)	
{

if(*turn < 0)		//reverse rotate wrap around pattern
{
	*turn = 0x0007;
}

*turn = ((*turn)%0x0008);	//Forward the stepper value upon next update
	
	
switch(*turn)		//stepping test pattern...
{
	case(0):
	{
		return(0x0009);
		break;
	}
	case(1):
	{
		return(0x0001);
		break;
	}
	case(2):
	{
		return(0x0005);
		break;
	}
	case(3):
	{
		return(0x0004);
		break;
	}
	case(4):
	{
		return(0x0006);
		break;
	}
	case(5):
	{
		return(0x0002);
		break;
	}
	case(6):
	{
		return(0x000A);
		break;
	}
	case(7):
	{
		return(0x0008);
		break;
	}
	default:
	{
		return(0x0009);
	}
}

	
	return 0;
}
