#include <assert.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "Furious.h"
#include "serial232.h"

const char *FuriousBoard::CMD_DELIM = "G";

int FuriousBoard::connect(const char *portname)
{
	this->serial_fd = serial_open(
		portname, FuriousBoard::BAUDRATE, &(this->old_termios)
	);
	for (int servo = 0; servo < FuriousBoard::NUM_SERVOS; servo++)
	{
		this->servos[servo] = 0;
	}
	for (int state = 0; state < FuriousBoard::NUM_STATE_VARS; state++)
	{
		this->state[state] = 0;
	}
	return this->serial_fd;
}

void FuriousBoard::disconnect()
{
	if ( this->isReady() )
	{
		serial_close(this->serial_fd, &(this->old_termios));
	}
}

bool FuriousBoard::isReady()
{
	return (this->serial_fd > 0);
}

void FuriousBoard::update()
{
	// for storing the command to be sent
	char command[256];
	char cmd_buf[8];
	// for storing the response received
	char response[256];
	char resp_buf[2];
	// for parsing the response
	char *token_ptr, *r_ptr;

	assert(this->isReady());

	// assemble command
	command[0] = '\0';
	for (int i = 0; i < 6; i++)
	{
		strcat(command, "0,");
	}
	for (int servo = 0; servo < FuriousBoard::NUM_SERVOS; servo++ )
	{
		sprintf(cmd_buf, "%i,", this->servos[servo]);
		strcat(command, cmd_buf);
	}
	strcat(command, FuriousBoard::CMD_DELIM);

	// send command
	serial_write(this->serial_fd, command, strlen(command));

	// receive response
	response[0] = '\0';
	do {
		serial_read(this->serial_fd, resp_buf, 1);
		resp_buf[1] = '\0';
		// TODO: validate the character that was just read
		strcat(response, resp_buf);
	} while ( strcmp(resp_buf, FuriousBoard::CMD_DELIM) != 0 );

	// parse response
	token_ptr = strtok_r(response, ",", &r_ptr);
	assert(token_ptr != NULL);
	this->state[0] = atoi(token_ptr);
	for (int state = 1; state < FuriousBoard::NUM_STATE_VARS; state++)
	{
		token_ptr = strtok_r(NULL, ",", &r_ptr);
		assert(token_ptr != NULL);
		this->state[state] = atoi(token_ptr);
	}
}


void FuriousBoard::setServo(int servo_id, int value)
{
	assert(servo_id < FuriousBoard::NUM_SERVOS);
	this->servos[servo_id] = value;
}

int FuriousBoard::getAnalog(int analog_id)
{
	assert(analog_id < FuriousBoard::NUM_STATE_VARS);
	return this->state[analog_id];
}

int FuriousBoard::getLogicBattery()
{
	assert(analog_id < FuriousBoard::NUM_STATE_VARS);
	static const int LOGIC_BATT_POS = 6;
	return this->state[LOGIC_BATT_POS];
}

int FuriousBoard::getMotorBattery()
{
	assert(analog_id < FuriousBoard::NUM_STATE_VARS);
	static const int MOTOR_BATT_POS = 7;
	return this->state[MOTOR_BATT_POS];
}

int FuriousBoard::getSonarValue()
{
	assert(analog_id < FuriousBoard::NUM_STATE_VARS);
	static const int SONAR_POS = 8;
	return this->state[SONAR_POS];
}

int FuriousBoard::getOdometer()
{
	assert(analog_id < FuriousBoard::NUM_STATE_VARS);
	static const int ODOMETER_POS = 9;
	return this->state[ODOMETER_POS];
}

#ifdef TESTING
FuriousBoard *g_board = NULL;
#define FREQ 20

void shutdown(int signal)
{
	printf("Shutting down...\n");
	g_board->disconnect();
	delete g_board;
	//printf("g_board is %x", g_board);
	exit(signal);
}

void term_handler(int sigtype)
{
	printf("\nSignal caught.  ");
	shutdown(sigtype);
}


/** for testing */
int main(int argc, const char **argv)
{
	const char *port = "/dev/ttyACM0";

	// signal handlers
	signal(SIGINT, term_handler);
	signal(SIGTERM, term_handler);

	// process command line args
	if ( argc > 1 )
	{
		port = argv[1];
	}

	// main loop action
	g_board = new FuriousBoard();
	g_board->connect(port);
	while ( g_board->isReady() )
	{
		g_board->update();
		for (int alg_id = 0; alg_id < FuriousBoard::NUM_STATE_VARS; alg_id++)
		{
			printf("%i ", g_board->getAnalog(alg_id));
		}
		printf("\n");
		usleep(1000 * 1000 / FREQ);
	}
	// Will never get here
	shutdown(0);
	return 0;
}
#endif
