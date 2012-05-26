#include <stdio.h>
#include <stdlib.h>
#include <udp_constants.h>
#include "tbrclient.h"
#define Zzz 1000000
#define DEFAULT_ADDR "127.0.0.1"


/*print value on screen*/
int main( void )
{
	tclient_start( DEFAULT_ADDR, UDP_PORT );
	if ( tclient_is_ready() )
	{
		while ( 1 )
		{
			printf("Sonar FR: %f\n", get_sonar_front_right());
			printf("Sonar FL: %f\n", get_sonar_front_left());
			printf("Distance: %f\n", get_odometer_distance());
			printf("Velocity: %f\n", get_odometer_velocity());
			usleep( Zzz );
		}
	}
	else
	{
		printf( "Fail to connect to server\n" );
	}
	system( "PAUSE" );
	return 0;
}
