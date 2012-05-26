/**  @file serial232.h
 * Implements communication over a serial port.
**/


#ifndef FURIOUS_DRIVER_SERIAL232_H
#define FURIOUS_DRIVER_SERIAL232_H


/**  Takes the port number and the baud and returns an open file 
 * descriptor, or -1 = failed, -3 = other error. */
int serial_open(const char *name, int baud, struct termios *oldtermios);

/**  Resets then closes port of file descriptor fd. */
int serial_close(int fd, struct termios *oldtermios);

/**  Reads up to n chars and puts it into buf. */
int serial_read(int fd, char *buf, int n);

/**  Writes the string pointed to by buf, up to n chars.  Returns the 
 * number of bytes written. */
int serial_write(int fd, const char *buf, int n);


#endif // FURIOUS_DRIVER_SERIAL232_H
