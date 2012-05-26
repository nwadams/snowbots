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
#include <dirent.h>
#include <strings.h>
#include "serial232.h"



// The port_number_scanner in probeForDevices() indexes for a device and opens the port >> once << 
// it is used by external threads only, please ignore the GCC warning =P
static int GLOBAL_Serial_Port_Descriptor_Index[ 128 ];


void Set_Serial_Port_Descriptor(int index, int fd)
{
	GLOBAL_Serial_Port_Descriptor_Index[(index%128)] = fd; 
}

int Get_Serial_Port_Descriptor(int index)
{
	return GLOBAL_Serial_Port_Descriptor_Index[(index%128)]; 
}


//list dir helper from gcc doc
static int
one ( const struct dirent *unused )
{
	return 1;
}

//list dir code from gcc doc... modded to scan MacOS dev dir for mounted control modules
void
scanDevIDs( char name[], int skipIndex )
{
	struct dirent **eps;
	int n;
	int cnt;
	char* filterResult;

	name[ 0 ] = '\0';

	n = scandir ( "/dev", &eps, one, alphasort );	//open dir struct

	if ( n >= 0 )
	{
		for ( cnt = 0; cnt < n; ++cnt )
		{
			strcpy( name, eps[ cnt ]->d_name );	//list dir file

			filterResult = strstr( name, "cu.usbmodem" );	//does device contain a modem in the cu list?
			if ( filterResult != NULL )
			{
				if ( skipIndex < 1 ) 	//write device name and path after skipping prior devices on list
				{
					strcpy( name, "/dev/" );	//list path
					strcat( name, eps[ cnt ]->d_name );  //list dev
					printf( "\n<-- %s -->\n", name );	//debug dir list filter???
					return ;
				}
				skipIndex--;
			}
			else
			{
				strcpy( name, "/dev/ttyACM0" );	//clobber file name
			}
		}
	}
	else
	{
		strcpy( name, "/dev/ttyACM0" );
		perror ( "Couldn't open the directory" );
	}

}




/*
Unknown author's GPL code for linux com port access.
file-scope variable to save terminal settings
*/

// THE FOLLOWING IS TWEAKED GPL CODE FROM AN UNKNOWN SOURCE //

/*
serial_bytes_waiting () returns the number of bytes waiting to be read
at the port described by fd.
*/

int Serial_Bytes_Waiting( int fd )
{
	int bytes;
	//ioctl (fd, FIONREAD, &bytes); /*read number of bytes in input buffer*/
	return bytes;
}


/*
The serial_read () and serial_write () are simple wrappers.
serial_write () writes the string pointed to by buf, up to n chars.
serial_read () reads up to n chars and puts it into buf.
*/

void Serial_Write( int fd, char *buf, int n, int *counter )
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
	*counter = bytes_written;
}


int Serial_Read( int fd, char *buf, int n )
{
	return read ( fd, buf, n );
}

/*
Function implementations:
Serial_open () takes the port number and the baud and returns
an open file descriptor, or -1 = failed, -3 = other error.
*/
int Serial_Open( int port_no, int baud, struct termios *oldtermios )
{
	int fd;
	char *name = "/dev/ttyACM0";
	char bufferString[ 255 ];
	int b_baud;
	struct termios t; /* struct for port settings */

	/* convert to port and baud input */
	switch ( port_no )
	{
	case 1:
		{
			name = "/dev/ttyS4";
			break;
		}
	case 2:
		{
			name = "/dev/ttyS3";
			break;
		}
	case 3:
		{
			name = "/dev/ttyS2";
			break;
		}
	case 4:
		{
			name = "/dev/ttyS1";
			break;
		}
	case 5:
		{
			name = "/dev/ttyACM4";		//new cdc-acm driver
			break;
		}
	case 6:
		{
			name = "/dev/ttyACM3";		//new cdc-acm driver
			break;
		}
	case 7:
		{
			name = "/dev/ttyACM2";		//new cdc-acm driver
			break;
		}
	case 8:
		{
			name = "/dev/ttyACM1";		//new cdc-acm driver
			break;
		}
	case 9:
		{
			name = "/dev/ttyACM0";		//new cdc-acm driver
			break;
		}
	case 10:
		{
			name = "/dev/ttyUSB4";		//older setserial driver
			break;
		}
	case 11:
		{
			name = "/dev/ttyUSB3";		//older setserial driver
			break;
		}
	case 12:
		{
			name = "/dev/ttyUSB2";		//older setserial driver
			break;
		}
	case 13:
		{
			name = "/dev/ttyUSB1";		//older setserial driver
			break;
		}
	case 14:
		{
			name = "/dev/ttyUSB0";		//older setserial driver
			break;
		}
	case 15:
		{
			name = "/dev/com9";			//cygwin com ports -- some usb devices will not detect as tty devices
			break;
		}
	case 16:
		{
			name = "/dev/com8";			//cygwin com port
			break;
		}
	case 17:
		{
			name = "/dev/com7";			//cygwin com port
			break;
		}
	case 18:
		{
			name = "/dev/com6";			//cygwin com port
			break;
		}
	case 19:
		{
			name = "/dev/com5";			//cygwin com port
			break;
		}
	case 20:
		{
			name = "/dev/com4";			//cygwin com port
			break;
		}
	case 21:
		{
			name = "/dev/com3";			//cygwin com port
			break;
		}
	case 22:
		{
			name = "/dev/com2";			//cygwin com port
			break;
		}
	case 23:
		{
			name = "/dev/com1";			//cygwin com port
			break;
		}
	case 24:
		{
			name = "/dev/com0";			//cygwin com port
			break;
		}
	case 25:
		{
			scanDevIDs( bufferString, 3 ); 			//Probe for 1st Mac OS cu.modem
			name = bufferString;
			break;
		}
	case 26:
		{
			scanDevIDs( bufferString, 2 ); 			//Probe for 2nd Mac OS cu.modem
			name = bufferString;
			break;
		}
	case 27:
		{
			scanDevIDs( bufferString, 1 ); 			//Probe for 3rd Mac OS cu.modem
			name = bufferString;
			break;
		}
	case 28:
		{
			scanDevIDs( bufferString, 0 ); 			//Probe for 4th Mac OS cu.modem
			name = bufferString;
			break;
		}
	default:
		{
			name = "/dev/ttyS0";
			break;
		}
	}

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


/*
serial_close () resets then closes port of file descriptor fd.
*/

int Serial_Close( int fd, struct termios *oldtermios )
{
	/* reset port to old settings and return */
	tcsetattr ( fd, TCSANOW, oldtermios );
	close ( fd );
	return 0;
}





