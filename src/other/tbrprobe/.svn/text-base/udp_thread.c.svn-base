/** @file udp_thread.c

This file implements the UDP communication layer for tbrprobe.  Clients that
wish to control a robot that is running tbrprobe in the control layer can
send commands to tbrprobe using a set of commands documented herein.

Also see examples/c/tbrclient/ for a C library that simplifies the writing
of client programs.

Notes:
1.)  strtok_r() is a POSIX thread safe version of strtok()
    one may have to import the function if it is not available in your Compiler.

**/

#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <pthread.h>
#include "udp_thread.h"
#include "udp_constants.h"

//robot state i/o
#include "robot_config.h"
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
#include "interface/raw_analog.h"
#include "interface/pos_optics.h"

/** message macros to help make code scale */
typedef enum { NO_OP, STEERING, THROTTLE, TURRET, SENSOR, OPTICS } MessageType;

/** The system can be under automatic (AI) or manual control. */
typedef enum { AUTO, MANUAL } ControlMode;

static char DELIM_CHAR[] = ",";
static ControlMode Control_Mode_State;
static int udp_sock_fd = -1;  // initialize the socket fd

static pthread_mutex_t udp_thread_Write_Lock  = PTHREAD_MUTEX_INITIALIZER;


/////////////////////////////////////////////////////////////////////
// PRIVATE FUNCTIONS
/////////////////////////////////////////////////////////////////////

static void
initializeSocket( struct sockaddr_in *my_addr )
{
	int bind_status;
	udp_sock_fd = socket( AF_INET, SOCK_DGRAM, 0 );
	if ( udp_sock_fd < 0 )
	{
		perror( "socket" );
		exit( 1 );
	}

	printf( "udp_sock_fd = %i\n", udp_sock_fd );

	my_addr->sin_family = AF_INET;         // host byte order
	my_addr->sin_port = htons( UDP_PORT );   // short, network byte order
	my_addr->sin_addr.s_addr = INADDR_ANY; // automatically fill with my IP
	memset( my_addr->sin_zero, '\0', sizeof my_addr->sin_zero );

	bind_status = bind( udp_sock_fd, ( struct sockaddr * ) my_addr, sizeof *my_addr );
	if ( bind_status < 0 )
	{
		perror( "bind" );
		exit( 1 );
	}
}



/**
 * Parses the given command string and returns the command macro id (see
 * above).
**/
static MessageType 
parseCommand( const char *command )
{
	// Parse the command to see what kind of command it is...

	/****  AI commands  ****/
	// Usually, commands are instructions from the AI to turn or drive.
	if ( strstr( command, CMD_AUTO_STEERING ) &&
	        ( Control_Mode_State == AUTO ) ) 	//AI
	{
		return STEERING;
	}
	else if ( strstr( command, CMD_AUTO_THROTTLE ) &&
	          ( Control_Mode_State == AUTO ) )  //AI
	{
		return THROTTLE;
	}
	else if ( strstr( command, CMD_AUTO_TURRET ) &&
	          ( Control_Mode_State == AUTO ) )  //AI
	{
		return TURRET;
	}
	else if ( strstr( command, CMD_AUTO_OPTICS ) &&
	          ( Control_Mode_State == AUTO ) )  //Camera Optics
	{
		return OPTICS;
	}
	
	/****  Requests for data  ****/
	// And sometimes, the AI is requesting information about the state of the
	// robot from tbrprobe.
	else if ( strstr( command, CMD_GET_SENSOR ) )
	{
		return SENSOR;
	}

	/****  Manual override commands  ****/
	// Sometimes, though, we are using a remote control to test the vehicle
	// or prevent it from going on an autonomous killing rampage.
	else if ( strstr( command, CMD_MANUAL_STEERING ) &&
	          ( Control_Mode_State == MANUAL ) )  //Wii Remote
	{
		return STEERING;
	}
	else if ( strstr( command, CMD_MANUAL_THROTTLE ) &&
	          ( Control_Mode_State == MANUAL ) )  //Wii Remote
	{
		return THROTTLE;
	}
	else if ( strstr( command, CMD_MANUAL_ON ) )
	{
		Control_Mode_State = MANUAL;
	}
	else if ( strstr( command, CMD_MANUAL_OFF ) )
	{
		Control_Mode_State = AUTO;
	}
	
	return NO_OP;
}


/**
 * Takes the id of a sensor whose data we want, the ip address and port
 * of the client who requested the data, and returns the value of that
 * sensor to the client via UDP.
 */
static void answerDataRequest( int sensor_id, struct sockaddr_in *their_addr )
{
	char reply[ UDP_BUF_LEN ] = "";  // string buffer for reply
	double replyTmpValue = 0;
	char replyTmpArray[ 64 ] = "";
	int numbytes;

	// Assemble the reply.
	sprintf( reply, "virtual_sensor,%i,", sensor_id );

	switch ( sensor_id )   //generate and append state info
	{
	case( dev_noop ) :
		{
			strcat( reply, "0" );
			break;
		}
	case( dev_AD0 ) :
	{
			
			replyTmpValue = Get_Raw_Analog(AD0);
			//abstract device defined in "sonar_prox.h"
			sprintf( replyTmpArray, "%f", replyTmpValue );
			strcat( reply, replyTmpArray );
			break;
	}	
	case( dev_AD1 ) :
	{
			
			replyTmpValue = Get_Raw_Analog(AD1);
			//abstract device defined in "sonar_prox.h"
			sprintf( replyTmpArray, "%f", replyTmpValue );
			strcat( reply, replyTmpArray );
			break;
	}	
	case( dev_AD2 ) :
	{
			
			replyTmpValue = Get_Raw_Analog(AD2);
			//abstract device defined in "sonar_prox.h"
			sprintf( replyTmpArray, "%f", replyTmpValue );
			strcat( reply, replyTmpArray );
			break;
	}	
	case( dev_AD3 ) :
	{
			
			replyTmpValue = Get_Raw_Analog(AD3);
			//abstract device defined in "sonar_prox.h"
			sprintf( replyTmpArray, "%f", replyTmpValue );
			strcat( reply, replyTmpArray );
			break;
	}	
	case( dev_AD4 ) :
	{
			
			replyTmpValue = Get_Raw_Analog(AD4);
			//abstract device defined in "sonar_prox.h"
			sprintf( replyTmpArray, "%f", replyTmpValue );
			strcat( reply, replyTmpArray );
			break;
	}	
	case( dev_AD5 ) :
	{
			
			replyTmpValue = Get_Raw_Analog(AD5);
			//abstract device defined in "sonar_prox.h"
			sprintf( replyTmpArray, "%f", replyTmpValue );
			strcat( reply, replyTmpArray );
			break;
	}		
	case( dev_sonar_front_left ) :
		{
			replyTmpValue = Get_Sonar_Distance( SONAR_LOCPROXNW );
			//abstract device defined in "sonar_prox.h"
			sprintf( replyTmpArray, "%f", replyTmpValue );
			strcat( reply, replyTmpArray );
			break;
		}

	case( dev_sonar_front_center ) :
		{
			replyTmpValue = Get_Sonar_Distance( SONAR_LOCPROXN );
			//abstract device defined in "sonar_prox.h"
			sprintf( replyTmpArray, "%f", replyTmpValue );
			strcat( reply, replyTmpArray );
			break;
		}
	case( dev_sonar_front_right ) :
		{
			replyTmpValue = Get_Sonar_Distance( SONAR_LOCPROXNE );
			//abstract device defined in "sonar_prox.h"
			sprintf( replyTmpArray, "%f", replyTmpValue );
			strcat( reply, replyTmpArray );
			break;
		}
	case( dev_sonar_rear_center ) :
		{
			replyTmpValue = Get_Sonar_Distance( SONAR_LOCPROXS );
			//abstract device defined in "sonar_prox.h"
			sprintf( replyTmpArray, "%f", replyTmpValue );
			strcat( reply, replyTmpArray );
			break;
		}
	case( dev_odometer ) :
		{
			sprintf( 
				replyTmpArray, 
				"%f,%f,%f,%f",
				//Result in Meters (from  "pos_tracker.h")
				Get_Distance( ),
				//Result in Meters Per Second
				Get_Velocity( ),
				//Result in Meters Per Second Per Second
				Get_Acceleration( ),
				//Result in Meters Per Second Per Second Per Second
				Get_Jerk( )
			);
			strcat( reply, replyTmpArray );
			break;
		}
	case( dev_ir_orientation ) :
		{
			sprintf( 
				replyTmpArray, 
				"%i,%i,%i,%i",
				//get the "ir_grade_axis.h" state info
				Get_ROLL( ),
				Get_YAW( ), 
				Get_PITCH( ),
				Get_Barrier( ) //distance to nearest obstacle
			);
			strcat( reply, replyTmpArray );
			break;
		}
	case( dev_ir_vector ) :
		{
			strcat( reply, "0... ADD JON's Code" );
			break;
		}
	default:
		{
			strcat( reply, "0" );  //default as a noop
			break;
		}
	}

	// Send the reply, checking for errors.
	printf( "\33[5;1HSending UDP reply: %s\n", reply );
	if ( ( numbytes = sendto( udp_sock_fd,
	                          reply,
	                          strlen( reply ),
	                          0,
	                          ( struct sockaddr * ) their_addr,
	                          sizeof *their_addr ) ) == -1 )
	{
		perror( "sendto" );
		exit( 1 );
	}
}

/**********************************************************************
 PUBLIC FUNCTIONS
***********************************************************************/

/**
 * This routine begins a thread which listens on a UDP port for incoming
 * commands.  See parseCommand() for a description of how such commands are
 * formatted.  This code is largely derived from Beej's Guide to Network
 * Programming <http://beej.us/guide/bgnet/>.  Sorry, Beej.
**/
void* udpThread( void *arg )
{
	struct sockaddr_in my_addr;    // my address information
	struct sockaddr_in their_addr; // connector's address information
	socklen_t addr_len;
	char udpBuf[ UDP_BUF_LEN ];
	char *tokenContextSave;
	
	/****  Initialization and setup  ****/

	//reset mutexs
	pthread_mutex_unlock(&udp_thread_Write_Lock);

	//mode to get commands from AI
	Control_Mode_State = AUTO;
	initializeSocket( &my_addr );
	addr_len = sizeof their_addr;


	/****  Main thread loop  ****/
	for ( ;; )
	{
		char *command_token;  // for breaking the command into parts
		MessageType command_mode;
		int command_value;  // Store the command id and value
		int numbytes;  // number of bytes received from client

		numbytes = recvfrom(
		               udp_sock_fd,
		               udpBuf,
		               UDP_BUF_LEN - 1 ,
		               0,
		               ( struct sockaddr * ) & their_addr,
		               &addr_len );
		if ( numbytes < 0 )
		{
			perror( "recvfrom" );
			exit( 1 );
		}

		// prevent thread lock up
		while(pthread_mutex_lock(&udp_thread_Write_Lock) != 0)
		{		
			usleep(1235);	//odd number for timer
		}

		printf( "\33[6;1HUDP packet (%d bytes) from %s", numbytes, inet_ntoa( their_addr.sin_addr ) );
		udpBuf[ numbytes ] = '\0';
//		printf( "\33[5;1HUDP packet contains \"%s\"", udpBuf );
		
		// first token: command
		command_token = strtok_r( udpBuf, DELIM_CHAR,&tokenContextSave );
		printf( "\33[5;1H module name '%s' now ", command_token );
		command_mode = parseCommand( command_token );
		printf( " contains \"%s\"\n", udpBuf );
		
		// calling strtok again with a NULL string will get the next token from the LAST
		// CALL to strtok that had a valid string argument
		command_token = strtok_r( NULL, DELIM_CHAR, &tokenContextSave ); 
		if ( command_token != NULL )
		{
//			printf( "token is %s\n", command_token);
			command_value = atoi(command_token);  // convert to int
		}

		switch ( command_mode )
		{
		case( STEERING ) :
			{
				//set local state in pos_steer.c
				Set_Steer_Angle( command_value );
				break;
			}
		case( THROTTLE ) :
			{
				//set local state in pos_tracker.c
				Set_Throttle_Setting( command_value );
				break;
			}
		case( OPTICS ) :
			{
				//set local state in pos_optics.c
				Set_Optics_Setting( command_value );
				break;
			}
		case( TURRET ) :
			{
				int pitch, yaw;
				pitch = command_value;
				command_token = strtok_r( NULL, DELIM_CHAR,&tokenContextSave);  //parse YAW value
				yaw = atoi( command_token );
				//set local state in pos_turret.c
				Update_Turret( pitch, yaw );
				break;
			}
		case( SENSOR ) :
			{
				answerDataRequest( command_value, &their_addr );
				break;
			}
		default:
			{
				printf("\33[5;1HUDP Command %s requires no action!!!", udpBuf);
			}
		}
		//enable other threads to write state
		pthread_mutex_unlock(&udp_thread_Write_Lock);
	}

	close( udp_sock_fd );

	return 0;
}


/**
 * Returns true if the udpThread is initialized and ready to send/receive
 * commands.
**/
int udpThreadIsReady( void )
{
	return ( udp_sock_fd >= 0 );
}


