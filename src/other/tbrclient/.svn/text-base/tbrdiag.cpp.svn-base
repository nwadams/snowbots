#include <stdio.h>
#include <stdlib.h>
#include "tbrclient.h"
#define SLEEP_TIME 500 * 1000
#define DEFAULT_HOST "127.0.0.1"
#define DEFAULT_PORT 1225

int main( int argc, char* argv[] )
{
	tbrprobe07::tbrclient TC;
	std::string host = DEFAULT_HOST;
	int port = DEFAULT_PORT;
	
	/* Process command line arguments */
	for ( int i = 1; i < argc; i++ )
	{
		std::string current_arg = argv[i];
		if ( current_arg == "-d" ) {
			// enable debugging
			TC.setObserver(&std::cout);
		}
		else if ( current_arg == "-h" ) {
			// change default host
			i++;
			host = argv[i];
		}
		else if ( current_arg == "-p" ) {
			// change default  port
			i++;
			port = atoi(argv[i]);
		}
	}
	
	TC.initialize(port, host.c_str());
	if ( TC.ready() )
	{
		while (1) {
			std::cout << "SonarNE: " << TC.getSonarNE() << std::endl;
			std::cout << "SonarN:  " << TC.getSonarN()  << std::endl;
			std::cout << "SonarNW: " << TC.getSonarNW() << std::endl;
			std::cout << "SonarS:  " << TC.getSonarS()  << std::endl;
			std::cout << std::endl;
			std::cout << "Distance: " << TC.getOdometerDistance() << std::endl;
			std::cout << "Velocity: " << TC.getOdometerVelocity() << std::endl;
			std::cout << std::endl;
			
			usleep(SLEEP_TIME);
		}
	}
	else
	{
		printf( "Fail to connect to server\n" );
	}
	return 0;
}
