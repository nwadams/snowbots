/**
 * @date May 21 2009
 * @author Ian Phillips
 * @brief This is a SWIG interface file that will generate a Python wrapper
 * around tbrclient.  This allows the C++ implementation of tbrclient to
 * be used in a Python program.
**/
%module tbrclient

%{
#define SWIG_FILE_WITH_INIT

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <iostream>
#include <ostream>

#include <tbrprobe/udp_constants.h>
#include <tbrclient/tbrclient.h>

#define UDP_PORT 1225
#define LOCALHOST_IP "127.0.0.1"
#define BUF_SIZE 100
#define SENSOR_ERROR_VAL -1.0
%}

namespace tbrprobe07{
class tbrclient{
	public:
		tbrclient();
		~tbrclient();

		bool initialize(int port, const char* addr);
		bool finalize();

		bool ready();

		bool setSteering(int value);
		bool setThrottle(int value);
		
		int getSteering();
		int getThrottle();
                        
		bool setTurret(int tilt, int pan);
		int getPan();
		int getTilt();

		double getSonarN();
		double getSonarNW();
		double getSonarNE();
		double getSonarS();

		double getOdometerDistance();
		double getOdometerVelocity();
		double getOdometerAcceleration();
		double getInfraredN();
		double getInfraredNW();
		double getInfraredNE();
		double getInfrared(int device_id);
		double getKillSwitchVal();
};
} /* end namespace */
