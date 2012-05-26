#include <assert.h>
#include <math.h>
#include <float.h>
#include <limits.h>
#include <time.h>
#include <ctype.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>
#include "tbrprobe_client.h"
#include "../../udp_constants.h"



/** buffer for commands to send to tbrprobe */
static char command[UDP_BUF_LEN];

/** buffer for answers received from tbrprobe */
static char answer[UDP_BUF_LEN];

/** socket file descriptor */
static int sockfd = -1;
static int sockfds = -1;


/** Represents a reading from the odometer.  Contains four members, all 
    of which are doubles: distance, velocity, acceleration, and jerk. */
typedef struct {
	double distance;
	double velocity;
	double acceleration;
	double jerk;
} OdometerReading;

/** stores the server's address information */
static struct sockaddr_in their_addr;

static int send_cmd(char *command) {
	int numbytes = sendto(
		sockfds, 
		command, 
		strlen(command), 
		0,
		(struct sockaddr *) & their_addr, 
		sizeof their_addr );
	
	if ( numbytes < 0 )
	{
		perror("error sending command");
		exit(1);
	}
	return numbytes;
}

static int receive()
{
	socklen_t addr_len = sizeof their_addr;
	int numbytes = recvfrom(
		sockfds, 
		answer, 
		UDP_BUF_LEN - 1 , 
		0,
		NULL, NULL );
	
	if ( numbytes < 0 )
	{
		perror("error receiving answer");
		exit(1);
	}
	return numbytes;
}

static double parse_double()
{
	int device_id;
	double value;
	sscanf(answer, "virtual_sensor,%i,%lf", &device_id, &value);
	return value;
}

static OdometerReading parse_odometer_reading()
{
	int device_id;
	OdometerReading values;
	sscanf(answer, "virtual_sensor,%i,%lf,%lf,%lf,%lf",
		&device_id,
		&values.distance,
		&values.velocity,
		&values.acceleration,
		&values.jerk );
	return values;
}

static void get_virtual_sensor(int device_id)
{
	sprintf(command, "%s,%i", CMD_GET_SENSOR, device_id); 
	send_cmd(command);
	receive();  // Reply is written into "answer" variable
}

/* * * PUBLIC MEMBERS * * */
void tclient_start(const char *addr, int port)
{
	if ((sockfds = socket(AF_INET, SOCK_DGRAM, 0)) == -1)
	{
		perror("unable to open UDP socket");
	}
	else
	{
		their_addr.sin_family = AF_INET;     // host byte order
		their_addr.sin_port = htons(port); // short, network byte order
		their_addr.sin_addr.s_addr = inet_addr(addr);
		memset(their_addr.sin_zero, '\0', sizeof their_addr.sin_zero);
	}
}

void tclient_stop(void) 
{
	close(sockfds);
	close(sockfd);
	sockfd = -1;
	sockfds = -1;
}

int tclient_is_ready() {
	return (sockfd > 0);
}


double get_raw_AD_Value(int pin)
{
	//will translate abstract. or verbose requests
	switch(pin)
	{
		case(0):
		{
			get_virtual_sensor(dev_AD0);
			break;
		}	
		case(dev_AD0):
		{
			get_virtual_sensor(dev_AD0);
			break;
		}	

		case(1):
		{
			get_virtual_sensor(dev_AD1);
			break;
		}	
		case(dev_AD1):
		{
			get_virtual_sensor(dev_AD1);
			break;
		}	
		case(2):
		{
			get_virtual_sensor(dev_AD2);
			break;
		}	
		case(dev_AD2):
		{
			get_virtual_sensor(dev_AD2);
			break;
		}	
		case(3):
		{
			get_virtual_sensor(dev_AD3);
			break;
		}	
		case(dev_AD3):
		{
			get_virtual_sensor(dev_AD3);
			break;
		}	
		case(4):
		{
			get_virtual_sensor(dev_AD4);
			break;
		}	
		case(dev_AD4):
		{
			get_virtual_sensor(dev_AD4);
			break;
		}	
		default:
		{
			get_virtual_sensor(dev_AD5);
		}
}
	return parse_double();
}

double get_sonar_front_right(void)
{
	get_virtual_sensor(dev_sonar_front_right);
	return parse_double();
}

double get_sonar_front_center(void)
{
	get_virtual_sensor(dev_sonar_front_center);
	return parse_double();
}

double get_sonar_front_left(void)
{
	get_virtual_sensor(dev_sonar_front_left);
	return parse_double();
}

double get_sonar_rear_center(void)
{
	get_virtual_sensor(dev_sonar_rear_center);
	return parse_double();
}

double get_odometer_distance(void)
{
	get_virtual_sensor(dev_odometer);
	OdometerReading reading = parse_odometer_reading();
	return reading.distance;
}

double get_odometer_velocity(void)
{
	get_virtual_sensor(dev_odometer);
	OdometerReading reading = parse_odometer_reading();
	return reading.velocity;
}



//////////////////////////////////////////////////////////////
//must bind in main init code 
static struct sockaddr_in their_addr; // client's address information
static int numbytes =0 ;

void bindHALClient(void){
	// Open a UDP socket for sending the requested sensor data
	if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) == -1)
	{
		perror("socket");
		exit(1);
	}

			// Set up the address/port info that we will send the reply to.
	their_addr.sin_family = AF_INET;     // host byte order
	their_addr.sin_port = htons(TBRPROBE_PORT); // short, network byte order
	their_addr.sin_addr.s_addr = inet_addr(TBRPROBE_IP);
	memset(their_addr.sin_zero, '\0', sizeof their_addr.sin_zero);
}
	
//sent client command to HAL
void sendHALCommand(char Msg[]){	

		// Send the reply, checking for errors.
	if ((numbytes = sendto(sockfd,
	                       Msg, 
	                       strlen(Msg),
	                       0,
	                       (struct sockaddr *) &their_addr,
	                       sizeof their_addr)) == -1)
	{
		perror("sendto");
		exit(1);
	}

	
}


//////////////////////////////////////////////////////////////
//helpers for more abstract steering use
void Robot_Turn_To_Right(void)
{
	char reply[256] = "auto_steering,99";  // string buffer for reply
	sendHALCommand(reply);
}

void Robot_Turn_To_Left(void)
{
	char reply[256] = "auto_steering,-99";  // string buffer for reply
	sendHALCommand(reply);
}

void Robot_Turn_To_Center(void)
{
	char reply[256] = "auto_steering,0";  // string buffer for reply
	sendHALCommand(reply);
}


void Robot_Turn_Right(int directParam)
{
	char bufTemp[64];
	char reply[256] = "auto_steering,";  // string buffer for reply
	
	if(directParam < 0){
		directParam=-directParam;	
	}

	if(directParam > 99){
		directParam=99;	
	}

	// Assemble the reply.
	sprintf(bufTemp,"%i", (directParam));
	strcat(reply, bufTemp);	
	
	// Send the reply, checking for errors.
	sendHALCommand(reply);
}

void Robot_Turn_Left(int directParam)
{
	char bufTemp[64];
	char reply[256] = "auto_steering,-";  // string buffer for reply
	
	if(directParam < 0){
		directParam=-directParam;	
	}

	if(directParam > 99){
		directParam=99;	
	}

	// Assemble the reply.
	sprintf(bufTemp,"%i", (directParam));
	strcat(reply, bufTemp);	
	
	// Send the reply, checking for errors.
	sendHALCommand(reply);
}



/////////////////////////////////////////////////////////////////////////
//turret vars
static int turret_Pitch=-13;
static int turret_Yaw=0;
//turret commands for context of vision commands
void Robot_Look_To_Right(void)
{
	char reply[256] = "auto_throttle,";  // string buffer for reply
	char replyTmpArray[64] ="";
			turret_Yaw=99;
			// Assemble the reply.
			strcpy(reply, "auto_turret,");
			sprintf(replyTmpArray,"%i", (turret_Pitch));
			strcat(reply, replyTmpArray);	
			strcat(reply, ",");	
			sprintf(replyTmpArray,"%i", (turret_Yaw));
			strcat(reply, replyTmpArray);	
							
			// Send the reply, checking for errors.
			sendHALCommand(reply);

}

void Robot_Look_To_Left(void)
{
	char reply[256] = "auto_throttle,";  // string buffer for reply
	char replyTmpArray[64] ="";
			turret_Yaw=-99;
			// Assemble the reply.
			strcpy(reply, "auto_turret,");
			sprintf(replyTmpArray,"%i", (turret_Pitch));
			strcat(reply, replyTmpArray);	
			strcat(reply, ",");	
			sprintf(replyTmpArray,"%i", (turret_Yaw));
			strcat(reply, replyTmpArray);	
							
			// Send the reply, checking for errors.
			sendHALCommand(reply);

}

void Robot_Look_To_Center(void)
{
	char reply[256] = "auto_throttle,";  // string buffer for reply
	char replyTmpArray[64] ="";
			turret_Yaw=0;
			// Assemble the reply.
			strcpy(reply, "auto_turret,");
			sprintf(replyTmpArray,"%i", (turret_Pitch));
			strcat(reply, replyTmpArray);	
			strcat(reply, ",");	
			sprintf(replyTmpArray,"%i", (turret_Yaw));
			strcat(reply, replyTmpArray);	
							
			// Send the reply, checking for errors.
			sendHALCommand(reply);

}

/////////////////////////////////////////////////////////////////////////
//robot driving state changers
void	Robot_Stop(void)		
{
	Robot_Start(0);	
}

void Robot_Reverse(int robot_throttle_control)
{
	Robot_Start((-robot_throttle_control));	
	
}

void	Robot_Start(int robot_throttle_control)		
{
	char reply[256] = "auto_throttle,";  // string buffer for reply
	char replyTmpArray[64] ="";

	// Assemble the reply. 
	sprintf(replyTmpArray,"%i", robot_throttle_control);
	strcat(reply, replyTmpArray);	
	// Send the reply, checking for errors.
	sendHALCommand(reply);
}



//////////////////////////////////////////////////////////////
//helpers for more abstract optics use
void Robot_Optics_Outdoor_Mode(void)
{
	char reply[256] = "auto_optics,99";  // string buffer for reply
	sendHALCommand(reply);
}

void Robot_Optics_Indoor_Mode(void)
{
	char reply[256] = "auto_optics,0";  // string buffer for reply
	sendHALCommand(reply);
}

void Robot_Optics_Night_Mode(void)
{
	char reply[256] = "auto_optics,-99";  // string buffer for reply
	sendHALCommand(reply);
}



