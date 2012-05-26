/**
 * rSerial is a library to communicate via USB with an Arduino
 * microcontroller using embedded python.
 * @author Matthew Baumann
*/

#ifndef SERIALBRIDGE_H
#define SERIALBRIDGE_H

#include <ostream>
#include <iostream>
#include <stdio.h>
#include <Python.h>


#define STRING_SIZE 512

namespace rSerial{

	class serialBridge{
		public:
		
		serialBridge();
		~serialBridge();
		
		bool init(char* path, int baud);
		bool close();
		
		bool sendString(char* str);
		bool delay(double seconds);
		bool ready();
		bool setObserverStream(std::ostream* observer);
		
		private:
		char portpath[STRING_SIZE];
		int baudrate;
		bool state;
		std::ostream* obs;
	};

}

#endif
