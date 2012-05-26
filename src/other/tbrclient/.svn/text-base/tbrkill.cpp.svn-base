//#include <stdio.h>
#include <stdlib.h>
#include <string>
#include "tbrclient.h"

/** Wait time between attempts to stop the robot. **/
#define SLEEP_TIME 500 * 1000
/** Number of times to attempt to stop the robot. **/
#define NUM_ATTEMPTS 5
/** Default host on which tbrprobe is running. **/
#define DEFAULT_HOST "127.0.0.1"
/** Default port on which tbrprobe is listening. **/
#define DEFAULT_PORT 1225


int main( int argc, char* argv[] )
{
	/* instantiate tbrclient and other local vars*/
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
	
	/* Initialize connection to tbrprobe and begin main control loop */
	TC.initialize(port, host.c_str());
	if ( TC.ready() )
	{
		// Try several times to stop the robot
		for ( int i = 0; i < NUM_ATTEMPTS; i++ )
		{
			TC.setSteering(0);
			TC.setThrottle(0);
			// Wait before trying again
			usleep(SLEEP_TIME);
		}
	}
	else
	{
		std::cout << "Fail to connect to server." << std::endl;
	}
	return 0;
}
