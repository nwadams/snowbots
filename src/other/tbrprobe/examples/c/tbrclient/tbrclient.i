/** @file tbrclient.i

 A SWIG wrapper file that tells swig how to convert tbrclient into a 
 Python module.  See www.swig.org for more info.
**/

%module tbrclient

%{
/** The default IP address of the tbrprobe server. */
#define DEFAULT_ADDR "127.0.0.1"

/** The default port on which the tbrprobe server listens. */
#define DEFAULT_PORT 1225
%}

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

