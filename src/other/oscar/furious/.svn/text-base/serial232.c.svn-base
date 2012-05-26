#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
/*
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <sys/stat.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <pthread.h>
#include <time.h>
#include <dirent.h>
#include <strings.h>
#include "serial232.h"
*/


/*
Unknown author's GPL code for linux com port access.
file-scope variable to save terminal settings
*/

// THE FOLLOWING IS TWEAKED GPL CODE FROM AN UNKNOWN SOURCE //

int serial_write( int fd, const char *buf, int n )
{
	int bytes_written = 0;
	int index = 0;
	char nbuf[ 2 ];

	for ( index = 0; index <= n; index++ )
	{
		if ( index % 2 == 0 )
		{
			nbuf[ 1 ] = '\0';
		}

		nbuf[ 0 ] = buf[ index ];
		bytes_written = write ( fd, nbuf, 1 );
		bytes_written++;
	}
	return bytes_written;
}


int serial_read( int fd, char *buf, int n )
{
	return read( fd, buf, n );
}

int serial_open( const char *name, int baud, struct termios *oldtermios )
{
	int fd;
	int b_baud;
	struct termios t; /* struct for port settings */

	/* convert baud input */
	switch ( baud )
	{
	case 4800:
		b_baud = B4800;
		break;
	case 9600:
		b_baud = B9600;
		break;
	case 19200:
		b_baud = B19200;
		break;
	case 38400:
		b_baud = B38400;
		break;
	case 57600:
		b_baud = B57600;
		break;
	case 115200:
		b_baud = B115200;
		break;
	default:
		b_baud = B9600;
		break;
	}

	// printf("Opening %s\n", name);
	fd = open ( name, O_RDWR | O_NOCTTY );

	if ( fd < 0 )
	{
		return -1;
	} /* open failed */
	if ( tcgetattr ( fd, &t ) != 0 )
	{
		return ( -3 );
	}


	/*
	save old settings
	*/
	*oldtermios = t;

	/*
	do settings
	*/
	t.c_cc[ VMIN ] = 32;
	t.c_cc[ VTIME ] = 1;

	/* &= ~ disables bits, |= enables bits */
	t.c_iflag &= ~( BRKINT | IGNPAR | PARMRK | INPCK | ISTRIP | INLCR | IGNCR | ICRNL | IXON );
	t.c_oflag &= ~( OPOST );

	/* non-canonical */
	t.c_lflag &= ~( ECHO | ECHOE | ECHOK | ECHONL | ICANON | ISIG | NOFLSH | TOSTOP );
	t.c_cflag &= ~( CSIZE | CSTOPB | HUPCL | PARENB );

	t.c_cflag |= CLOCAL | CREAD | CS8;

	/* copy data rates into termios struct */
	if ( cfsetispeed ( &t, b_baud ) == -1 )
	{
		return ( -3 );
	}

	if ( cfsetospeed ( &t, b_baud ) == -1 )
	{
		return ( -3 );
	}

	//this will not compile in cycgwin, but may improve data rates 
	/* set up raw terminal  for real rs232 devices */
	//if ( cfmakeraw(&t) == -1 )
	//{
	//	return ( -3 );
	//}
	
	
	//Note some BSD systems report this kills the USB port too
	//tcflush (fd, TCIOFLUSH);                        // flush buffers
	
	/* throw away any input data / noise */
	if ( tcflush ( fd, TCIFLUSH ) == -1 )
	{
		return ( -3 );
	}

	/* Now, set the terminal attributes */
	if ( tcsetattr ( fd, TCSANOW, &t ) == -1 )
	{
		return ( -3 );
	}

	return fd;
}


int serial_close( int fd, struct termios *oldtermios )
{
	/* reset port to old settings and return */
	tcsetattr ( fd, TCSANOW, oldtermios );
	close ( fd );
	return 0;
}





