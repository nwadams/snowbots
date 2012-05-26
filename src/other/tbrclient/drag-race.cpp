#include <stdio.h>
#include <stdlib.h>
#include "tbrclient.h"
#define SLEEP_TIME 50 * 1000
#define DEFAULT_ADDR "127.0.0.1"
#define DEFAULT_PORT 1225
#define DEFAULT_CUTOFF 0.3
#define DEFAULT_MAX_THROTTLE 30

int main( int argc, char* argv[] )
{
	double cutoff = DEFAULT_CUTOFF;
	int max_throttle = DEFAULT_MAX_THROTTLE;
	tbrprobe07::tbrclient TC;
	
	/* Process command line arguments */
	for ( int i = 1; i < argc; i++ )
	{
		if ( strcmp(argv[i], "-d") == 0) {
			// enable debugging
			TC.setObserver(&std::cout);
		}
		else if ( strcmp(argv[i], "-c") == 0 ) {
			// change default sonar distance cutoff
			i++;
			cutoff = atof(argv[i]);
		}
		else if ( strcmp(argv[i], "-t") == 0 ) {
			// change default max throttle
			i++;
			max_throttle = atoi(argv[i]);
			if ( max_throttle > 100 ) max_throttle = 100;
		}
	}
	
	/* Initialize connection to tbrprobe and begin main control loop */
	TC.initialize(DEFAULT_PORT, DEFAULT_ADDR);
	if ( TC.ready() )
	{
		// Get the machine moving a bit.
		TC.setThrottle(max_throttle * 2);
		usleep(100 * 1000);

		// Ignore sonar readings greater than 3 meters.
		const double IGNORE_DIST = 3.0;
		// Initialize both sonar values to be ignored
		double sonarNE = IGNORE_DIST, sonarNW = IGNORE_DIST;
		while ( true )
		{
			int steering, throttle;
			// Get sonar readings and use them if they are within the allowable range.
			double ne = TC.getSonarNE();
			double nw = TC.getSonarNW();
			if ( ne < IGNORE_DIST ) sonarNE = ne;
			if ( nw < IGNORE_DIST ) sonarNW = nw;
			
			// Determine speed based on sonars
			if ( sonarNW < cutoff ) {
				steering = -50;
				throttle = max_throttle / 2;
			}
			else if ( sonarNE < cutoff ) {
				steering = 50;
				throttle = max_throttle / 2;
			}
			else {
				steering = 0;
				throttle = max_throttle;
			}
			
			TC.setSteering(steering);
			TC.setThrottle(throttle);
			
			usleep(SLEEP_TIME);
		}
	}
	else
	{
		printf( "Fail to connect to server\n" );
	}
	return 0;
}
