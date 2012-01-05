/*
 File: Serial_Communication.h
 
 Author: Nick Adams and Jarek Ignasmenzies
 Last Edited: September 11, 2010
 Version 1.0
 License: GNU
  
 Class for serial communication using termios

*/
#ifndef SERIALCOMMUNICATION
#define SERIALCOMMUNICATION

#include <stdio.h> //standard c library
#include <iostream>//standard cpp library
#include <unistd.h> // allows for unix terminal functions
#include <fcntl.h>  //used for the function open()
#include <termios.h> //unix serial setup
#include <string>//controlling strings
#include <stdlib.h>


using namespace std;

class SerialCommunication
{
public:
	
	//constructor that does nothing
	SerialCommunication();
	
	//constructor. does initialization
	SerialCommunication(unsigned int baud, string port);
	
	//initializes serial link
	//returns true if successful, false otherwise
	bool connect(unsigned int baud, string port);
	
	//checks to see if the serial link is open
	//returns true if open, false otherwise
	bool isActive();
	
	//reads n byte of data from serial buffer
	//returns string
	string readData(int n=100);
	
	//writes data to serial buffer
	//returns bytes written
	int writeData(string output, unsigned int n = 0);
	
	//clears buffer
	void clearBuffer();
	
	//destructor, clears memory
	~SerialCommunication();
	
	
private:
    
    //does initialization stuff
	void init(unsigned int Baud);
	
	int fd;
	
	struct termios options;
	
};

#endif
