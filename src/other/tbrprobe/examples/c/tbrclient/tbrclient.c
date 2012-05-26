#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>

#include <udp_constants.h>
#include "tbrclient.h"


/* * * PRIVATE MEMBERS * * */

/** buffer for commands to send to tbrprobe */
static char command[UDP_BUF_LEN];

/** buffer for answers received from tbrprobe */
static char answer[UDP_BUF_LEN];

/** socket file descriptor */
static int sockfd = -1;

/** stores the server's address information */
static struct sockaddr_in their_addr;

static int send_cmd(char *command) {
	int numbytes = sendto(
		sockfd, 
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
		sockfd, 
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
	if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) == -1)
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
	close(sockfd);
	sockfd = -1;
}

int tclient_is_ready() {
	return (sockfd > 0);
}


double get_sonar_front_right(void)
{
	get_virtual_sensor(dev_sonar_front_left);
	return parse_double();
}
double get_sonar_front_left(void)
{
	get_virtual_sensor(dev_sonar_front_right);
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

