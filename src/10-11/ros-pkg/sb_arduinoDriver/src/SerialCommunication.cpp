/*
 File: Serial_Communication.cpp
 
 Author: Nick Adams and Jarek Ignasmenzies
 Last Edited: September 11, 2010
 Version 1.0
 License: GNU
 
 Class for serial communication using termios
 
*/

#include <stdio.h> //standard c library
#include <iostream>//standard cpp library
#include <unistd.h> // allows for unix terminal functions
#include <fcntl.h>  //used for the function open()
#include <termios.h> //unix serial setup
#include <string>//controlling strings
#include "SerialCommunication.h"
#include <stdlib.h>

using namespace std;

SerialCommunication::SerialCommunication(unsigned int baud, string port)
{
	fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
	
	if (fd == -1)
	{
		cout << "unable to open port" << endl;
		exit(0);
	}
	
	init(baud);
}

SerialCommunication::SerialCommunication()
{
    
}

bool SerialCommunication::connect(unsigned int baud, string port)
{
	fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);	
	
	if (fd == -1)
	{
		return false;
	}
	
	init(baud);
	return true;
}

bool SerialCommunication::isActive()
{
    if(fd == -1)
    {
        return false;
    } else {
        return true;
    }
}

SerialCommunication::~SerialCommunication()
{
	close (fd);
}

string SerialCommunication::readData(int n)
{
	char buf[n];
	read(fd, buf, n);
	return buf;
}

int SerialCommunication::writeData(string output, unsigned int n)
{
	int buf[1];
	unsigned int i;
	
	if (n == 0 )
	    n = output.length();
	    
	for(i = 0; i < n; i++)
	{
		buf[0] = output[i];
		if(!write(fd, buf, 1))
			return i;
	}
	return i;
}

void SerialCommunication::clearBuffer()
{
	tcflush ( fd, TCIFLUSH );
	return ;
}


void SerialCommunication::init(unsigned int baud)
{
    tcgetattr(fd,&options);
    switch(baud) //sets the baud rate I'll add more options later but works
    { 
		case 9600: cfsetispeed(&options,B9600); 
			cfsetospeed(&options,B9600); 
			break; 
		case 19200: cfsetispeed(&options,B19200); 
			cfsetospeed(&options,B19200); 
			break; 
		case 38400: cfsetispeed(&options,B38400); 
			cfsetospeed(&options,B38400); 
			break; 
		case 115200: cfsetispeed(&options,B115200); 
			cfsetospeed(&options,B115200); 
			break;
		default:cfsetispeed(&options,B9600); 
			cfsetospeed(&options,B9600); 
			break; 
    } 

    options.c_cflag |= (CLOCAL | CREAD); 
    options.c_cflag &= ~PARENB; 
    options.c_cflag &= ~CSTOPB; 
    options.c_cflag &= ~CSIZE; 
    options.c_cflag |= CS8; 
    tcsetattr(fd,TCSANOW,&options); 
}



