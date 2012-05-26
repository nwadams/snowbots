#include <assert.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "furious_driver.h"
#include "serial232.h"

FuriousDriver *fdCreateDriver(const char *portname)
{
	FuriousDriver *dr;
	int servo, state;

	// Allocate memory
	dr = (FuriousDriver *) malloc(sizeof(FuriousDriver));

	// Initialize members
	dr->serial_fd = serial_open(
		portname, FD_BAUDRATE, &(dr->old_termios)
	);
	for (servo = 0; servo < FD_NUM_SERVOS; servo++)
	{
		dr->servos[servo] = 0;
	}
	for (state = 0; state < FD_NUM_STATE_VARS; state++)
	{
		dr->state[state] = 0;
	}
	return dr;
}

void fdReleaseDriver(FuriousDriver *dr)
{
	if ( fdIsReady(dr) )
	{
		serial_close(dr->serial_fd, &(dr->old_termios));
	}
	free(dr);
}

int fdIsReady(FuriousDriver *dr)
{
	return (dr->serial_fd > 0);
}

void fdUpdate(FuriousDriver *dr)
{
	// for storing the command to be sent
	char command[256];
	char cmd_buf[8];
	// for storing the response received
	char response[256];
	char resp_buf[2];
	// for parsing the response
	char *token_ptr, *r_ptr;
	// loop counters
	int servo, state, i;

	assert(fdIsReady(dr));

	// assemble command
	command[0] = '\0';
	for (i = 0; i < 6; i++)
	{
		strcat(command, "0,");
	}
	for (servo = 0; servo < FD_NUM_SERVOS; servo++ )
	{
		sprintf(cmd_buf, "%i,", dr->servos[servo]);
		strcat(command, cmd_buf);
	}
	strcat(command, FD_DELIM_CHAR);

	// send command
	serial_write(dr->serial_fd, command, strlen(command));

	// receive response
	response[0] = '\0';
	do {
		serial_read(dr->serial_fd, resp_buf, 1);
		resp_buf[1] = '\0';
		strcat(response, resp_buf);
	} while ( strcmp(resp_buf, FD_DELIM_CHAR) != 0 );

	// parse response
	token_ptr = strtok_r(response, ",", &r_ptr);
	assert(token_ptr != NULL);
	dr->state[0] = atoi(token_ptr);
	for (state = 1; state < FD_NUM_STATE_VARS; state++)
	{
		token_ptr = strtok_r(NULL, ",", &r_ptr);
		assert(token_ptr != NULL);
		dr->state[state] = atoi(token_ptr);
	}
}


void fdSetServo(FuriousDriver *dr, int servo_id, int value)
{
	assert(servo_id < FD_NUM_SERVOS);
	dr->servos[servo_id] = value;
}

int fdGetAnalog(FuriousDriver *dr, int analog_id)
{
	assert(analog_id < FD_NUM_STATE_VARS);
	return dr->state[analog_id];
}


#ifdef TESTING
FuriousDriver *g_board = NULL;
#define FREQ 20

void shutdown(int signal)
{
	printf("Shutting down...\n");
	fdReleaseDriver(g_board);
	exit(signal);
}

void term_handler(int sigtype)
{
	printf("\nSignal caught.  ");
	shutdown(sigtype);
}


/** for testing */
int main(int argc, char **argv)
{
	char *port = "/dev/ttyACM0";
	int alg_id;

	// signal handlers
	signal(SIGINT, term_handler);
	signal(SIGTERM, term_handler);

	// process command line args
	if ( argc > 1 )
	{
		port = argv[1];
	}

	// main loop action
	g_board = fdCreateDriver(port);
	while (1)
	{
		fdUpdate(g_board);
		for (alg_id = 0; alg_id < FD_NUM_STATE_VARS; alg_id++)
		{
			printf("%i ", fdGetAnalog(g_board, alg_id));
		}
		printf("\n");
		usleep(1000 * 1000 / FREQ);
	}
	// Will never get here
	shutdown(0);
	return 0;
}
#endif
