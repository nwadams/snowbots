/** @file tbrclient.h

 A library that can be used to write a client for the tbrprobe server.
 Defines basic functions for sending commands to and receving data from 
 tbrprobe.
**/


/** Represents a reading from the odometer.  Contains four members, all 
    of which are doubles: distance, velocity, acceleration, and jerk. */
typedef struct {
	double distance;
	double velocity;
	double acceleration;
	double jerk;
} OdometerReading;

/** Initializes a network socket to the IP address and port given. */
void tclient_start(const char *addr, int port);

/** Does any necessary shutdown cleanup. */
void tclient_stop(void);

/** Returns true if the client is ready for communication with the tbrprobe
    server. */
int tclient_is_ready(void);

/** Returns the value from the front-right sonar as a double. */
double get_sonar_front_right(void);

/** Returns the value from the front-left sonar as a double. */
double get_sonar_front_left(void);

/** Returns the odometer's distance value as a double. */
double get_odometer_distance(void);

/** Returns the odometer's velocity value as a double. */
double get_odometer_velocity(void);

