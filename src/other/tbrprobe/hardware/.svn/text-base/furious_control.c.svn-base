/*
Notes:
1.)  strtok_r() is a POSIX thread safe version of strtok()
    one may have to import the function if it is not available in your Compiler.

2.) If one unplugs the USB cable it will put servos into free-move-mode and stop the ESC motors

3.)
Both tbrprobe and the RS232/USB devices will filter any character outside the marshaling syntax.
In this case only  the characters "01234456789,-G" with a base 10 numbering system is used. 
The single letter "G" is used to identify the device type, and key the byte stream for the driver.

4.)
*MAKE SURE ONLY ONE DEVICE IS ON THE i2c Bus AT A TIME FOR PROGRAMMING
*Give a few seconds before reading to allow the sonar to reboot
Example to config sonar device 1
command: 17,0,0,0,0,0,0,0,0,0,0,250,250,G
then to read device 1
command: 1,0,0,0,0,0,0,0,0,0,0,250,250,G

*/

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
//all macros for furious devices are the same for the servo_config based devices
#include "../robot_config.h" 
#include "../hardware/servo_control.h"
#include "../hardware/nimu_control.h"
#include "../hardware/motor_control.h"
#include "furious_control.h"
#include "serial232.h"
#include "../interface/pos_tracker.h"
#include "../interface/pos_steer.h" 
#include "../interface/pos_turret.h"
#include "../interface/ir_grade_axis.h"
#include "../interface/sonar_prox.h"
#include "../interface/pos_battery.h"
#include "../interface/raw_analog.h"
#include "../interface/pos_optics.h"


//////////////////////////////////////////////////////////////////////////////
//Furious HAL
void* FuriousThread(void *arg)
{
	pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS,NULL);
	//message to pass around
	char someGenericCommand[500]={""};
	//locals
	char someTmp[500]={""};
	char someLocalTmp[500]={""};
	char someLocalTmp2[500]={""};
	int Furious_Port = -1;
	int Furious_Port_fd = 0; 
	//this is where polled data should be parsed into
	int furiousModuleMachineState[24]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	char *parsePtrResult = NULL;
	char parseSplitIDtok[] = ",";	
	char *tokenContextSave;
	int indexScan =0;
	//config sonar Ids to scan (if i2c BUSID = 0 ends list)
	int furiousSonarTmp = 0;
	int tmpSonarContextVar =FURIOUS_NO_SONAR_NOOP ;	
									
	#if (MOTOR_DRIVE_SERVO_ESC_MANUAL_CONFIG_SWEEP == 1)
		char someChar;		//input buffer for key press capture
	#endif
									
	//import id from PnP scanner
	Furious_Port = *((int *) arg);	 
	Furious_Port_fd = Get_Serial_Port_Descriptor(Furious_Port); //localize port file descipter from scanner
	
	
	printf("\nFurious Driver has started on dev=%i with fd=%i...\n", 
						Furious_Port, Furious_Port_fd);

	if(Furious_Port == -1){
	printf("\n ?????????????????????? =P EXIT Furious! ???????????????????????? \n");
		pthread_exit(0);
		return 0;
	}
	
	//MOTOR_DRIVE_SERVO_ESC_CONFIG       Setup   
	/////////////////////////////////////////////////////////
	//After a power cycle, the Duratrax ESC as must do an auto-setup...
	//Some ESC require a STOP signal upon power up to config the neutral logic 
	//Note: Green light on ESC will blink while waiting to boot. 
	//         (If already setup it willl show steady green)
	#if (MOTOR_DRIVE_SERVO_ESC_CONFIG == 1)
	
		//Example: create message to init ESC and program srf10/srf08 to SONAR ID 1	
		//strcpy(someGenericCommand, "17,0,0,0,0,0,0,\0");
		//or id2
		//strcpy(someGenericCommand, "18,0,0,0,0,0,0,\0");
		//or id3
		//strcpy(someGenericCommand, "19,0,0,0,0,0,0,\0");
		//or id4
		//strcpy(someGenericCommand, "20,0,0,0,0,0,0,\0");
	
		//use 0 for default ESC init
		strcpy(someGenericCommand, "0,0,0,0,0,0,0,\0");
	
		//.....Shim to set maximum period.... 
		strcat(someGenericCommand, MAX_SERVO_CYCLE_SHIM); 		//firmware bug fix will repair this issue
		strcat(someGenericCommand, "0,0,0,\0");	//create message passer

		sprintf(someTmp, "%i",MOTOR_DRIVE_SERVO_STOP );	//neutral drive
		strcat(someTmp, ",\0");	//add comma
		strcat(someGenericCommand, someTmp);	
		sprintf(someTmp, "%i",MOTOR_STEER_SERVO_CENTER);	//neutral steering
		strcat(someTmp, ",\0");	//add comma
		strcat(someGenericCommand, someTmp);	
		strcat(someGenericCommand, "G\0");		//end of packet
		printf("SEND \n>%s \n",someGenericCommand);
		Furious_Send_Command(someGenericCommand, &Furious_Port_fd);  
		printf("\nNow booting the Electronic Speed Control...");
		usleep(5000000);	//needed to setup the config signal over 3 seconds
	#endif

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//The  ESC may need a MANUAL-setup by a human... 
	//You must complete this script at least ONCE with a new ESC.
	#if (MOTOR_DRIVE_SERVO_ESC_MANUAL_CONFIG_SWEEP == 1)

			/////////////////////////////////////////////////////////////////////
			//Some ESC like the traxxas XL5 Speed control may need to be calibrated
			#if  defined(traxxas) || defined(snowtires) || defined(snowdrift) || defined(snowdrift)
			printf("\nRange sweep setup of traxxas XL5 ESC:");
			
			//Stop
			strcpy(someGenericCommand, "0,0,0,0,0,0,0,\0");
			//.....Shim to set maximum period.... 
			strcat(someGenericCommand, MAX_SERVO_CYCLE_SHIM); 		//firmware bug fix will repair this issue
			strcat(someGenericCommand, "0,0,0,\0");	//create message passer
					
 			//Note the shim places the actual neutral off the real
			//  stop position to prevent the ESC Brake from turning on
			sprintf(someTmp, "%i", 
				(MOTOR_DRIVE_SERVO_STOP - MOTOR_DRIVE_SERVO_BRAKE_SHIM));	
			strcat(someTmp, ",\0");	//add comma
			strcat(someGenericCommand, someTmp);	
			sprintf(someTmp, "%i",MOTOR_STEER_SERVO_CENTER);//neutral steering
			strcat(someTmp, ",\0");	//add comma
			strcat(someGenericCommand, someTmp);	
			strcat(someGenericCommand, "G\0");		//end of packet
			Furious_Send_Command(someGenericCommand, &Furious_Port_fd); 
			
			printf("\n1.) Hold down traxxas XL5 ESC [Set] button until the LED turns green and then red... then release button.");
			usleep(100000);	//needed
			printf("\n   When the LED blinks RED ONCE then PRESS ENTER KEY TO CONTINUE.");
			someChar = getchar();

			//forwards
			strcpy(someGenericCommand, "0,0,0,0,0,0,0,\0");
			//.....Shim to set maximum period.... 
			strcat(someGenericCommand, MAX_SERVO_CYCLE_SHIM); 		//firmware bug fix will repair this issue
			strcat(someGenericCommand, "0,0,0,\0");	//create message passer
			
			sprintf(someTmp, "%i", MOTOR_DRIVE_SERVO_MAX_FWD);	 
			strcat(someTmp, ",\0");	//add comma
			strcat(someGenericCommand, someTmp);	
			sprintf(someTmp, "%i",MOTOR_STEER_SERVO_CENTER);//neutral steering
			strcat(someTmp, ",\0");	//add comma
			strcat(someGenericCommand, someTmp);	
			strcat(someGenericCommand, "G\0");		//end of packet
			Furious_Send_Command(someGenericCommand, &Furious_Port_fd); 
			usleep(100000);	//needed
			printf("\n2.) When the LED blinks RED TWICE then PRESS ENTER KEY TO CONTINUE.");
			someChar = getchar();

			//Reverse
			strcpy(someGenericCommand, "0,0,0,0,0,0,0,\0");
			//.....Shim to set maximum period.... 
			strcat(someGenericCommand, MAX_SERVO_CYCLE_SHIM); 		//firmware bug fix will repair this issue
			strcat(someGenericCommand, "0,0,0,\0");	//create message passer

			sprintf(someTmp, "%i", MOTOR_DRIVE_SERVO_MAX_REV);	 
			strcat(someTmp, ",\0");	//add comma
			strcat(someGenericCommand, someTmp);	
			sprintf(someTmp, "%i",MOTOR_STEER_SERVO_CENTER);//neutral steering
			strcat(someTmp, ",\0");	//add comma
			strcat(someGenericCommand, someTmp);	
			strcat(someGenericCommand, "G\0");		//end of packet
			Furious_Send_Command(someGenericCommand, &Furious_Port_fd); 
			usleep(100000);	//needed
			printf("\n3.) Release... Note when the LED turns solid GREEN then PRESS ENTER KEY.");
			someChar = getchar();
			
			//stop
			strcpy(someGenericCommand, "0,0,0,0,0,0,0,\0");
			//.....Shim to set maximum period.... 
			strcat(someGenericCommand, MAX_SERVO_CYCLE_SHIM); 		//firmware bug fix will repair this issue
			strcat(someGenericCommand, "0,0,0,\0");	//create message passer

			sprintf(someTmp, "%i", (MOTOR_DRIVE_SERVO_STOP - MOTOR_DRIVE_SERVO_BRAKE_SHIM));	 
			strcat(someTmp, ",\0");	//add comma
			strcat(someGenericCommand, someTmp);	
			sprintf(someTmp, "%i",MOTOR_STEER_SERVO_CENTER);//neutral steering
			strcat(someTmp, ",\0");	//add comma
			strcat(someGenericCommand, someTmp);	
			strcat(someGenericCommand, "G\0");		//end of packet
			Furious_Send_Command(someGenericCommand, &Furious_Port_fd); 
			
			printf("\nDone Setup! (You may now disable this compile time option for this unit.)\n");
			printf("\nRange sweep ESC in Stop..");
			usleep(500000);	//needed
			#endif

			////////////////////////////////////////////////////////////
			//Some ESC like the TAMIYA  TEU-101BK FET Speed control may
			// need to be calibrated upon a power cycle
			#if defined(snowfury)
			
			printf("\nRange sweep setup of TAMIYA ESC:");
			//Stop
			strcpy(someGenericCommand, "0,0,0,0,0,0,0,\0");
			//.....Shim to set maximum period.... 
			strcat(someGenericCommand, MAX_SERVO_CYCLE_SHIM); 		//firmware bug fix will repair this issue
			strcat(someGenericCommand, "0,0,0,\0");	//create message passer
					
 			//Note the shim places the actual neutral off the real
			//  stop position to prevent the ESC Brake from turning on
			sprintf(someTmp, "%i", 
				(MOTOR_DRIVE_SERVO_STOP - MOTOR_DRIVE_SERVO_BRAKE_SHIM));	
			strcat(someTmp, ",\0");	//add comma
			strcat(someGenericCommand, someTmp);	
			sprintf(someTmp, "%i",MOTOR_STEER_SERVO_CENTER);//neutral steering
			strcat(someTmp, ",\0");	//add comma
			strcat(someGenericCommand, someTmp);	
			strcat(someGenericCommand, "G\0");		//end of packet
			Furious_Send_Command(someGenericCommand, &Furious_Port_fd); 
			usleep(100000);	//needed
			printf("\n1.) PUSH ON THE TAMIYA ESC [Set] button until it beeps once...\n   THEN PRESS ENTER KEY TO CONTINUE.");
			someChar = getchar();

			//forwards
			strcpy(someGenericCommand, "0,0,0,0,0,0,0,\0");
			//.....Shim to set maximum period.... 
			strcat(someGenericCommand, MAX_SERVO_CYCLE_SHIM); 		//firmware bug fix will repair this issue
			strcat(someGenericCommand, "0,0,0,\0");	//create message passer
			
			sprintf(someTmp, "%i", MOTOR_DRIVE_SERVO_MAX_FWD);	 
			strcat(someTmp, ",\0");	//add comma
			strcat(someGenericCommand, someTmp);	
			sprintf(someTmp, "%i",MOTOR_STEER_SERVO_CENTER);//neutral steering
			strcat(someTmp, ",\0");	//add comma
			strcat(someGenericCommand, someTmp);	
			strcat(someGenericCommand, "G\0");		//end of packet
			Furious_Send_Command(someGenericCommand, &Furious_Port_fd); 
			usleep(100000);	//needed
			printf("\n2.) PUSH ESC [Set] button once again until it beeps...\n   THEN PRESS ENTER KEY TO CONTINUE.");
			someChar = getchar();

			//Reverse
			strcpy(someGenericCommand, "0,0,0,0,0,0,0,\0");
			//.....Shim to set maximum period.... 
			strcat(someGenericCommand, MAX_SERVO_CYCLE_SHIM); 		//firmware bug fix will repair this issue
			strcat(someGenericCommand, "0,0,0,\0");	//create message passer

			sprintf(someTmp, "%i", MOTOR_DRIVE_SERVO_MAX_REV);	 
			strcat(someTmp, ",\0");	//add comma
			strcat(someGenericCommand, someTmp);	
			sprintf(someTmp, "%i",MOTOR_STEER_SERVO_CENTER);//neutral steering
			strcat(someTmp, ",\0");	//add comma
			strcat(someGenericCommand, someTmp);	
			strcat(someGenericCommand, "G\0");		//end of packet
			Furious_Send_Command(someGenericCommand, &Furious_Port_fd); 
			usleep(100000);	//needed
			printf("\n3.) Lastly, PUSH [Set] button one more time until it beeps once...\n   THEN PRESS ENTER KEY.");
			someChar = getchar();
			
			printf("\nDone Setup! (You may now disable this compile time option for this unit.)\n");
			//stop
			strcpy(someGenericCommand, "0,0,0,0,0,0,0,\0");
			//.....Shim to set maximum period.... 
			strcat(someGenericCommand, MAX_SERVO_CYCLE_SHIM); 		//firmware bug fix will repair this issue
			strcat(someGenericCommand, "0,0,0,\0");	//create message passer

			sprintf(someTmp, "%i", MOTOR_DRIVE_SERVO_STOP);	 
			strcat(someTmp, ",\0");	//add comma
			strcat(someGenericCommand, someTmp);	
			sprintf(someTmp, "%i",MOTOR_STEER_SERVO_CENTER);//neutral steering
			strcat(someTmp, ",\0");	//add comma
			strcat(someGenericCommand, someTmp);	
			strcat(someGenericCommand, "G\0");		//end of packet
			Furious_Send_Command(someGenericCommand, &Furious_Port_fd); 
			printf("\nRange sweep ESC in Stop..");
			usleep(500000);	//needed
			#endif	
	#endif
	
	//End Setup ///////////////////////////////////////////////////////////////////////////////////
	


//======================================================================================
	printf ("\33[44;37m\n" );       //white text on blue screen
						//this  shows if another thread clones
//======================================================================================
//begin control poll
//======================================================================================
	for(;(Furious_Port != -1);)
	{ 
		
 			sprintf(someGenericCommand, "%i", tmpSonarContextVar);	//PING ID

			//.....legacy stuff... may do stuff later... ignore for now...
			strcat(someGenericCommand, ",0,0,0,0,"); //
			
			//read pan/tilt unit settings
			#if defined(TURRET_ENABLED)
			sprintf(someTmp, "%i", Get_Turret_Yaw_Setting());//set Yaw	 
			strcat(someTmp, ",\0");	//add comma
			strcat(someGenericCommand, someTmp);	

			sprintf(someTmp, "%i", Get_Turret_Pitch_Setting());//set Pitch	 
			strcat(someTmp, ",\0");	//add comma
			strcat(someGenericCommand, someTmp);	
			#else
			strcat(someGenericCommand, "0,0,\0"); //Disables servos
			#endif
			
			//.....Shim to set maximum period.... 
			strcat(someGenericCommand, MAX_SERVO_CYCLE_SHIM); 		//firmware bug fix will repair this issue

			//assigns the 5th servo to control an optical filter arm
			#if defined(OPTICS_FILTER_ARM_SERVO_CENTER)
			sprintf(someTmp, "%i", Get_Optics_Setting());			//set optics arm with filter	 
			strcat(someTmp, ",\0");	//add comma
			strcat(someGenericCommand, someTmp);	
			#else
			strcat(someGenericCommand, "0,\0"); 				//ignore optics servo
			#endif
			
			//read Left and rigth track motor thottle settings...
			strcat(someGenericCommand, "0,0,\0"); //TODO:Add skid steer api

			//read motor setting
			sprintf(someTmp, "%i", Get_Throttle_Setting());	 
			strcat(someTmp, ",\0");	//add comma
			strcat(someGenericCommand, someTmp);	
			
			//setup steering
			sprintf(someTmp, "%i", Get_Steer_Angle());	//neutral steering?
			strcat(someTmp, ",\0");	//add comma
			strcat(someGenericCommand, someTmp);
			
			strcat(someGenericCommand, "G\0");		//end of packet

						
			strcpy(someLocalTmp2, someGenericCommand);	//for TX debug feedback			
			
			Furious_Send_Command(someGenericCommand, &Furious_Port_fd); 
			
			strcpy(someLocalTmp, someGenericCommand);		//for RX debug feedback
			
			//update local machine state vars
			parsePtrResult = strtok_r(someGenericCommand,
						 parseSplitIDtok,&tokenContextSave);//first line
			for(indexScan=0; 
				((parsePtrResult != NULL)&&(indexScan < 21)&&
					(parsePtrResult[0] != Furious_KeySig)); 
						indexScan++)
			{
				strcpy(someTmp, parsePtrResult);//copy out sub strings		
				furiousModuleMachineState[indexScan] = atoi(someTmp);
				parsePtrResult = strtok_r(NULL, parseSplitIDtok,&tokenContextSave);//next line
			}
			
			//check for valid parse index
			if(indexScan > 20){
				
			//stop the scroll man
			printf ("\33[2J" );	//Clear terminal screen
			printf("\33[1;30H");	//Name/config?
			printf(ROBOT_ID_NAME);	//debug com problems

			printf("\33[2;1HG(%i)TX: %s\n",Furious_Port,someLocalTmp2);	
			//debug com problems	
			printf ("\33[3;1HG(%i)RX: %s", Furious_Port, someLocalTmp);
			
			
			// - -  - -  - -  - -  - -  - -  - -  - -  - -  - -  - -  - -  - -   
			#if defined(duratrax) || defined(traxxas) || defined(snowfury) || defined(snowtires) || defined(snowdrift)
			//check if no odometer on unit: 
			Update_Odometer(furiousModuleMachineState[FURIOUS_ODOMETER_INDEXPOS]);
			//stamps data timers 
			#endif 
			
			// - -  - -  - -  - -  - -  - -  - -  - -  - -  - -  - -  - -  - -  -  	//Each robot may have diff sonar setups, but will share the same api... 
			
			//Check if no sonars (all IDs = 0)
			#if (FURIOUS_SONAR_PING_FRONT_LEFT_BUSID != FURIOUS_SONAR_PING_FRONT_RIGHT_BUSID)
			furiousSonarTmp =  furiousModuleMachineState[FURIOUS_SONAR_INDEXPOS];
			
			switch(tmpSonarContextVar)
			{	
				case(FURIOUS_SONAR_PING_FRONT_LEFT_BUSID):
				{ 
					tmpSonarContextVar=SONAR_LOCPROXNW;			//translate BUSID into abstract ID for API
					Set_Sonar_Data(furiousSonarTmp,tmpSonarContextVar);  	//pass sonar data in cm up to API
					//printf("\33[2;10H%i",tmpSonarContextVar);	//Name/config?

					tmpSonarContextVar=FURIOUS_SONAR_PING_FRONT_RIGHT_BUSID;	//config next sonar to call
				break;
				}
				case(FURIOUS_SONAR_PING_FRONT_RIGHT_BUSID):
				{ 
					tmpSonarContextVar=SONAR_LOCPROXNE;
					Set_Sonar_Data(furiousSonarTmp,tmpSonarContextVar);  			
					//printf("\33[2;10H%i",tmpSonarContextVar);	//Name/config?

					tmpSonarContextVar=FURIOUS_SONAR_PING_FRONT_MID_BUSID;
				break;
				}
				case(FURIOUS_SONAR_PING_FRONT_MID_BUSID):
				{
					tmpSonarContextVar=SONAR_LOCPROXN;
					Set_Sonar_Data(furiousSonarTmp,tmpSonarContextVar);  
					//printf("\33[2;10H%i",tmpSonarContextVar);	//Name/config?

					tmpSonarContextVar=FURIOUS_SONAR_PING_REAR_BUSID;
				break;
				}
				case(FURIOUS_SONAR_PING_REAR_BUSID):
				{ 
					tmpSonarContextVar=SONAR_LOCPROXS;
					Set_Sonar_Data(furiousSonarTmp,tmpSonarContextVar);  
					//printf("\33[2;10H%i",tmpSonarContextVar);	//Name/config?

					tmpSonarContextVar=FURIOUS_SONAR_PING_FRONT_LEFT_BUSID;
				break;
				}
				default:
				{ 
					tmpSonarContextVar=FURIOUS_SONAR_PING_FRONT_LEFT_BUSID ;	//to ensure round robin bootstrap
					//or to hault set to: tmpSonarContextVar=FURIOUS_NO_SONAR_NOOP;
					
				break;
				}
			}
			
			#endif 
			Print_Sonar_Data();
			
			
			// - -  - -  - -  - -  - -  - -  - -  - -  - -  - -  - -  - -
 			#if  defined(duratrax) || defined(snowfury) 
			Update_IR_Grade( 
			  furiousModuleMachineState[FURIOUS_SHARP_IR_FRONT_MID_INDEXPOS],
			  furiousModuleMachineState[FURIOUS_SHARP_IR_FRONT_LEFT_INDEXPOS],
			  furiousModuleMachineState[FURIOUS_SHARP_IR_FRONT_RIGHT_INDEXPOS],
			  furiousModuleMachineState[FURIOUS_SHARP_IR_BACK_MID_INDEXPOS]);
			#endif	
			
			// - -  - -  - -  - -  - -  - -  - -  - -  - -  - -  - -  - -
 			#if defined(duratrax) || defined(traxxas) || defined(snowfury) || defined(snowtires) || defined(snowdrift)
			Update_Raw_Analog( furiousModuleMachineState[0], AD0 ); //ir copy
			Update_Raw_Analog( furiousModuleMachineState[1], AD1 ); //ir copy
			Update_Raw_Analog( furiousModuleMachineState[2], AD2 ); //ir copy
			Update_Raw_Analog( furiousModuleMachineState[3], AD3 ); //?
			Update_Raw_Analog( furiousModuleMachineState[4], AD4 ); //?
			Update_Raw_Analog( furiousModuleMachineState[5], AD5 ); //Pause Switch
			#endif	
			
			// - -  - -  - -  - -  - -  - -  - -  - -  - -  - -  - -  - -
 			#if defined(duratrax) || defined(traxxas) || defined(snowfury) || defined(snowtires) || defined(snowdrift)
			Update_Battery_Level( 
			  furiousModuleMachineState[FURIOUS_MOTOR_BATTERY_INDEXPOS],
			  furiousModuleMachineState[FURIOUS_LOGIC_BOARD_BATTERY_INDEXPOS] );
			#endif				
			}

			printf ("\33[24;80H\n" );		//flush text buffer to screen
			//do NOT mod this or the index id will be BUGGY... 
	}
//end control poll
//======================================================================================
//======================================================================================
	
	
	printf("\n ?????????????????????? EXIT Furious! ???????????????????????? \n");
	pthread_exit(0);
	return 0;
}


//////////////////////////////////////////////////////////////////////////////
//send to bound port

void Furious_Send_Command(char PktArr[], int *Port_fd)
{
	char PktBuff[512]={""};
	int bufferScanIndex=0;
	int inputIndex=0;
	int timeout=0;
	char *Furious_BufferInPtr;
	char *Furious_BufferOutPtr;
	int Furious_TX = 0;
	int Furious_RX = 0;
	
	if(*Port_fd < 1)
	{
		printf("\nNo Furious on port fd:%i\n", *Port_fd);
		return;
	} 
	
	 // flush the buffers to dump PnP chatter and corruption etc...
	tcflush (*Port_fd, TCIOFLUSH);  //flushes both the input and output queue
	
	
	Furious_BufferOutPtr=PktArr;	//Prepares a command packet to be sent
	Serial_Write(*Port_fd , Furious_BufferOutPtr, strlen(PktArr), &Furious_TX);		//send word to slave device

//	printf("\n Write %i to Furious fd:%i\n", Furious_TX, *Port_fd );	//debug
	
	if(Furious_TX < 1)
	{
		printf("\nError: can't write %i to Furious fd:%i\n", Furious_TX, *Port_fd );
		return;
	}
		
		
	Furious_BufferInPtr=PktArr;	//Prepares for packet input
	inputIndex=0;
	bufferScanIndex=0;
	PktBuff[0]='\0';
	
		for(timeout = 20000; (timeout>0); timeout--)
		{
			Furious_RX= Serial_Read(*Port_fd, PktBuff,1);	
//>>>>>>>>>>>>ONE MUST ALWAYS EMPTY INPUT BUFFER<<<<<<<<<<<<<<<
			usleep(10);
			PktBuff[Furious_RX] = '\0'; //make buffer a string
			
			for(bufferScanIndex = 0; 
				(bufferScanIndex < Furious_RX); 
						bufferScanIndex++)
			{ 
					if(((PktBuff[bufferScanIndex] >= '0') &&
					(PktBuff[bufferScanIndex] <= '9'))	||
					(PktBuff[bufferScanIndex] == ',') ||
					(PktBuff[bufferScanIndex] == '-') ||
					(PktBuff[bufferScanIndex] == Furious_KeySig) )
					{	
					Furious_BufferInPtr[inputIndex] = 
								PktBuff[bufferScanIndex];
								//scan in char
						inputIndex++;	//index buffer		
					}	
					
					Furious_BufferInPtr[inputIndex] = '\0';	
					//always a string
						
					//Scan buffer for known valid token END responses 
					if(PktBuff[bufferScanIndex] == Furious_KeySig)
					{
						return;	
					}
			}
		
		}
		
	printf("No data response from Furious on fd:%i\n", *Port_fd);
		
}

