/**  @file furious_driver.h
 * Implements the low-level interface to the Furious microcontroller board.
 * Values sent to and returned from these functions are the raw values (eg,
 * PPM signal and analog-to-digital) that the Furious board deals with 
 * directly.
 * @author Ian Phillips <ianfp@freeshell.org>
**/

#ifndef FURIOUS_DRIVER_H
#define FURIOUS_DRIVER_H

#include <termios.h>

/**  The baud rate that the Furious module uses for serial communication. */
#define FD_BAUDRATE 19200

/**  The delimiter that marks the end of a Furious serial command. */
#define FD_DELIM_CHAR "G"

/**  The number of servo ports on the Furious board. */
#define FD_NUM_SERVOS 8

/**  The number of state variables that the Furious maintains. */
#define FD_NUM_STATE_VARS 11

/**  Represents a single Furious board. */
typedef struct {
	int serial_fd;
	int servos[FD_NUM_SERVOS];
	int state[FD_NUM_STATE_VARS];
	struct termios old_termios;
} FuriousDriver;

/**  Creates and initializes a new FuriousDriver. */
FuriousDriver *fdCreateDriver(const char *portname);

/**  Destroys the given FuriousDriver.  Closes its serial port and
 * frees up the memory it used. */
void fdReleaseDriver(FuriousDriver **dr);

/**  Returns true if the given FuriousDriver's serial port is open. */
int fdIsReady(FuriousDriver *dr);

/**  Sends the current servo values to the physical board, then awaits
 * a response that updates the state of the FuriousDriver.  Typically,
 * you would call this function in each iteration of your main control
 * loop to send all pending commands and receive all state updates. */
void fdUpdate(FuriousDriver *dr);

/**  Sets the value of the given servo.  Note that this does not update
 * the physical board immediately; it just queues up the new value.  The
 * physical board will be updated on the next call to fdUpdate(). */
void fdSetServo(FuriousDriver *dr, int servo_id, int value);

/**  Gets the raw analog value for the given analog port.  Note that if
 * fdUpdate() has not been called recently, the value returned might be
 * out-of-date. */
int fdGetAnalog(FuriousDriver *dr, int analog_id);

#endif // FURIOUS_DRIVER_H
