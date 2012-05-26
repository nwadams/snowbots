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
#include "robot_config.h"
#include "udp_thread.h"
#include "hardware/servo_control.h"
#include "hardware/nimu_control.h"
#include "hardware/motor_control.h"
#include "hardware/furious_control.h"
#include "hardware/serial232.h"
#include "interface/pos_tracker.h"
#include "interface/pos_steer.h" 
#include "interface/pos_turret.h"
#include "interface/ir_grade_axis.h"
#include "interface/sonar_prox.h"
#include "interface/pos_battery.h"
#include "interface/raw_analog.h"
#include "interface/pos_optics.h"

/* This program is designed to provide a staging area for a message passing Hardware Abstraction Layer */
/* PnP scan emulation over rs232, each device has a unique ID */

/****  Functions  ****/
// probeForDevices: search for device ports.
int probeForDevices( int startOnPortId );
// spawnDeviceThread: when a device is found, start a thread to control it.
void spawnDeviceThread( 
	int port_number, 
	int port_fd,
	struct termios *oldtermios_p, 
	void *(*thread_func_p)(void *), 
	char *device_name );

/****  GLOBAL vars  ****/
// GLOBAL_tbrprobe_Thread_id: Each device is run by its own thread.  Pointers
// to those threads are stored in this array, so we can selectively kill a
// given thread.  Note there is a limit of 127 threads, and therefore 127
// USB devices.
static pthread_t GLOBAL_tbrprobe_Thread_id[ 128 ];
// GLOBAL_tbrprobe_Thread_index: keeps count of running total of threads we
// have spawned.
static int GLOBAL_tbrprobe_Thread_index = 0;


int main ( int argc, char *argv[] )
{
#if defined(fliteTTSvoiceEnable)
char msgsVoice[1024];
char msgsTmp[24];
system("flite -t \"the system is booting, please standby until I am ready to accept The A I commands\"");
#endif
	
//	printf ("\33[7m\n" );		//invert video 
	printf( "\n-----------------------------------------------------\n" ); 
	//note the set text colour will fail unless some text is already on screen
	printf ("\33[45;37m\n" ); //white text on magenta screen
	
	if ( argc < 2 )
	{
		printf( "\n\ntbrprobe usage to start at a specific port: tbrprobe 4\n\n" );
		usleep( 3000000 );
		probeForDevices( 4 );	//probe for devices to populate port IDs
	}
	else
	{
		printf( "\n-----------------------------------------------------\n" );
		printf( "Robot probes set for: " );
		printf( ROBOT_ID_NAME );
		printf( "\n-----------------------------------------------------" );

		printf( "\n\t\t./tbrprobe %i\n\t\tResults of scan...", atoi( argv[ 1 ] ) );
		printf( "\n.....................................................\n" );
		probeForDevices( atoi( argv[ 1 ] ) - 1 );	//probe for devices to populate port IDs
	}

	if (GLOBAL_tbrprobe_Thread_index < 1)
	{
		printf("No devices were found.  Exiting...\n\n");
		exit(1);
	}

	Init_POS_Battery();	//reset the battery tracker
	Init_POS_Steer();	//setup steer servo etc. to center
	Init_POS_Tracker();	//setup throttle tracker statics
	Init_IR_Grade();		//setup axis tracking ( need to calculate)
	Init_Sonar_Prox();	//reset the sonar sensors to default state
 	Init_Turret();		//reset the tracker to default 0',0'ready state
 	Init_Raw_Analog();	//reset the tracker to default 0',0'ready state
 	Init_Optics();		//reset the filter arm to default


	// spawn thread for command input via UDP
	printf( "\nAttempting to open UDP thread.\n" );
	if (! udpThreadIsReady() )
	{  // not connected yet
		usleep(8000000);
		printf( "\nCreating UDP thread.\n" );
		pthread_create( &udpThreadId, NULL, &udpThread, NULL );
	}

	for ( ;; )
	{
		printf( "." );
		fflush( stdout );

		#if defined(fliteTTSvoiceEnable)
		strcpy(msgsVoice, "flite -t \"The Logic Battery level is at ");
		sprintf(msgsTmp,"%i",Get_Battery_Level());
		strcat(msgsVoice, msgsTmp);
		strcat(msgsVoice, " percent.. Motor Battery level is at ");
		sprintf(msgsTmp,"%i",Get_Battery_Level_Motor());
		strcat(msgsVoice, msgsTmp);
		strcat(msgsVoice, " percent.. Robot has travled ");
		sprintf(msgsTmp,"%.1f",Get_Distance());
		strcat(msgsVoice, msgsTmp);
		strcat(msgsVoice, " meters.. Your laptop power status is \"\n");
		system(msgsVoice);
		system("acpi -a | flite");
		#endif
		
		usleep( 5000000 );		//poll every 5 seconds
		//TODO: add port watch dog to shutdown the controls (collect threads), and reprobe for devices on the fly.
	}

	return 0 ;
}

/****************************************************************************************/

int probeForDevices( int startOnPortId )
{
	//rs232.h holds the real global index of file handles	
	int  st1Port,st1TX, st1RX;
	char *ip;
	char *op;
	char out_buffer[ 150 ] = {"\0"};  // TODO: is this useful?
	char in_buffer[ 150 ] = {"\0"};  // TODO: is this useful?
	char Command_Probe_Packet[ 500 ] = {"\0"};
	int size;
	char repet;
	char successBreak;  //prob vars
	char bufferScan;
	int bufferScanIndex;
	struct termios oldtermios;
	int port_number_scanner = startOnPortId;  //to force only USB adapter device scan set to 4
	char* filterResultForDumbEcho;	//filter to skip loopback ports
	

	//Note other devices will ignore invalid command chars
	op = ( char * ) memset( out_buffer, '\0', 100 );  //Creates an RX buffer

	/****  Loop 1: scan all ports  ****/
	while ( port_number_scanner < MAX_NUM_PORTS )  //no devices found yet?
	{
		port_number_scanner++;  //next port to scan

		// Try to open port. 
		st1Port = Serial_Open( port_number_scanner, 115200, &oldtermios );
		//st1Port = Serial_Open( port_number_scanner, 19200, &oldtermios );

		// Force motor device to all stop, and to echo data.
		// Why do we send the probe packet twice?  The first one is used to
		// identify the device we're talking to, and the second one forces the 
		// hardware to reset to zero. 
		strcpy( Command_Probe_Packet, "0,0,0,0,0,0,0,0,0,0,0,0,0,GSXZ0,0,0,0,0,0,0,0,0,0,0,0,0,GSXZ" );
		if ( st1Port < 1 )
		{
			printf( "No port on ID:%i\n", port_number_scanner );
		}
		else
		{
			printf( "Scanning for Units on:%i (fd=%i)\n", port_number_scanner,st1Port );
			printf( "Sending:%s\n", Command_Probe_Packet );

			successBreak = 0;
			ip = Command_Probe_Packet;  //Prepares a command packet to be sent
			size = strlen( ip );
			usleep( 100000 );
			Serial_Write( st1Port, ip, size, &st1TX );  //send word to slave device

			if ( st1TX < 1 )
			{
				printf( "Error can't write to Port ID:%i\n", port_number_scanner );
			}
			else
			{
				printf( "\nStandby to Receive packets...\n" );

				in_buffer[ 0 ] = '\0';
				size = 120;
				
				/****  Loop 2: make 10 attempts to achieve communication with
				 ****  the device on the port.  ****/
				repet = 0;
				while ( ( repet < 10 ) && ( successBreak == 0 ) )
				{
					usleep( 100000 );  //  I/O Delay to prevent bus problems and sample error	(allows buffer fills)
					st1RX = Serial_Read( st1Port, op, size );  //record data
					op[ st1RX ] = '\0';  //make buffer a string
					
					printf("Response is %s\n", op);

					//filter out any loop back ports
					filterResultForDumbEcho = strstr(  op, "GSXZ" );	//does device contain a more than one key value? 
					if (( st1RX > 0 ) && ( filterResultForDumbEcho == NULL ))	//than skip if filter gets a hit...
					{
						/****  Loop 3:  scan each character of data read in
						 ****  from the serial port and identify the device
						 ****  attached.  ****/
						for ( bufferScanIndex = 0; ( bufferScanIndex < 120 ) && ( successBreak == 0 ); bufferScanIndex++ )
						{
							/* We need to remember the name and thread function
							 * of whatever device we found. */
							char *device_name;
							void *(*thread_func)(void *);
							
							bufferScan = op[ bufferScanIndex ];  //scanned char
							in_buffer[ 0 ] = bufferScan;  //debug only
							in_buffer[ 1 ] = '\0';
							printf( "%s", in_buffer );

							//Scan buffer for known valid token responses -- Note loop back device will be caught in part II
							//someDataPointer = &port_number_scanner;  // IFP - removed; see declaration above

							/* See if this character matches the key signature
							 * for any of the devices we support. */
							if ( bufferScan == Motor_KeySig )
							{
								thread_func = &MotorThread;
								device_name = "Motor Control";
								successBreak = 1;  //Exit buffer scan
							}
							else if ( bufferScan == nIMU_KeySig )
							{
								thread_func = &nIMUThread;
								device_name = "nIMU";
								successBreak = 1;  //Exit buffer scan
							}
							else if ( bufferScan == Servo_KeySig )
							{
								thread_func = &ServoThread;
								device_name = "Servo";
								successBreak = 1;  //Exit buffer scan
							}
							else if ( bufferScan == Furious_KeySig )
							{
								thread_func = &FuriousThread; 
								device_name = "Furious";
								//port_number_scanner = MAX_NUM_PORTS;  // TEMP HACK TO FIX DUPE
								successBreak = 1;  //Exit buffer scan
							}
							
							/* If we've got a device... */
							if ( successBreak )
							{
								/* ... say that we've found a device */
								printf( "\n\n----------------------------------------\n" );
								printf( 
									"%s Device Found on Port ID %i\n", 
									device_name, port_number_scanner );
								
								 Set_Serial_Port_Descriptor(port_number_scanner,st1Port);
	
								
								/* ... close the serial port file descriptor
								 * (the control thread for the device will
								 * reopen this to communicate with the device) */
								
								//Serial_Close( st1Port, &oldtermios );
								//usleep( 3000000 );  //delay seconds
								
								/* ... spawn a thread to control it. */
								//pthread_create(&GLOBAL_tbrprobe_Thread_id[GLOBAL_tbrprobe_Thread_index], NULL, &MotorThread, (void *) someDataPointer) ;
								pthread_create(
									&GLOBAL_tbrprobe_Thread_id[ GLOBAL_tbrprobe_Thread_index ],
									NULL,
									thread_func,
									( void * ) &port_number_scanner );
								
								
								printf ("\33[30;47m\n" ); //black text on white screen
								usleep( 5000000 );  //delay to allow new thread to load
								
								GLOBAL_tbrprobe_Thread_index++;
								printf( "\n----------------------------------------\n" );
							}
						} /* end Loop 3: character scan */
						
					}else{
						if(filterResultForDumbEcho != NULL)	//remove the filter hit if set 
						{
							free(filterResultForDumbEcho);	//this is C not a C++ delete() call
						}
					}
					repet++;
				} /* end Loop 2: 100 repetitions */

				if ( successBreak )
				{
#if defined(HALT_PROBE_AFTER_FIRST_MODULE_FOUND)
					//port_number_scanner = MAX_NUM_PORTS;
					break;  // Break out of port scan loop.
#endif
				}
				else
				{
					printf( "Warning: NO known device response on Port ID:%i\n", port_number_scanner );
					Serial_Close( st1Port, &oldtermios );
				}
			}
		}
	} /* end Loop 1: port scan */

	return 0;
}
