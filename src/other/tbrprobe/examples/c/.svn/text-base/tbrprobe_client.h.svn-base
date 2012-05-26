/////////////////////////////////////////////////////////////////////////
//network comm layer for TBRPROBE 09
#define TBRPROBE_PORT 1225
#define TBRPROBE_IP "127.0.0.1"
#define DEFAULT_LOCAL_ADDR "127.0.0.1"


//network tbrprobe client to send commands
void bindHALClient(void);
void sendHALCommand(char Msg[]);

//network tbrprobe client to get robot state
/** Initializes a network socket to the IP address and port given. */
void tclient_start(const char *addr, int port);

/** Does any necessary shutdown cleanup. */
void tclient_stop(void);

/** Returns true if the client is ready for communication with tbrprobe */
int tclient_is_ready(void);

//robot commands to call after 
void Robot_Turn_To_Right(void);
void Robot_Turn_To_Left(void);
void Robot_Turn_To_Center(void);
void Robot_Turn_Right(int directParam);
void Robot_Turn_Left(int directParam);
void	Robot_Stop(void);
void	Robot_Reverse(int robot_throttle_control);
void	Robot_Start(int robot_throttle_control);

//turret
void Robot_Look_To_Right(void);
void Robot_Look_To_Left(void);
void Robot_Look_To_Center(void);

//optics arm
void Robot_Optics_Outdoor_Mode(void);
void Robot_Optics_Night_Mode(void);
void Robot_Optics_Indoor_Mode(void);

/** Returns the value from the sonar as a double. */
double get_sonar_front_right(void);
double get_sonar_front_center(void);
double get_sonar_front_left(void);
double get_sonar_rear_center(void);

/* get AD value as 0-5 Volts  */
double get_raw_AD_Value(int pin);

/** Returns the odometer's distance value as a double. */
double get_odometer_distance(void);

/** Returns the odometer's velocity value as a double. */
double get_odometer_velocity(void);

