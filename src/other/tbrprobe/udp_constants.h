/** @file udp_constants.h

 Defines constant values used by the UDP communication system.
 
 GROUP 1: DEVICE IDS

 These devices are enumerated queries that get handled by the HAL
 these are abstract device IDs for sensor reports of machine state

 they are not intended to be used as config values as other programs will 
 use these enum-codes add to the list, but please avoid changing any values
 
 GROUP 2: COMMAND STRINGS
 
 These are the valid command strings a client can send to tbrprobe to 
 request data and send instructions.  Clients
 
 GROUP 3: MISC
 
 UDP port information, etc.
 
**/


/////////////////////////////////////////////////////////////////////
// DEVICE IDS
/////////////////////////////////////////////////////////////////////

/** to test if connection is alive **/
#define dev_noop 0

/** these raw sensors may be queried individually **/
#define dev_sonar_front_left 1
#define dev_sonar_front_center 2
#define dev_sonar_front_right 3
#define dev_sonar_rear_left 4
#define dev_sonar_rear_center 5
#define dev_sonar_rear_right 6
#define dev_sonar_side_left 7
#define dev_sonar_side_right 8


/** returns raw pin voltage 0.1 to Vcc +4.9 **/
#define dev_AD0 10
#define dev_AD1 11
#define dev_AD2 12
#define dev_AD3 13
#define dev_AD4 14
#define dev_AD5 15


/** virtual sensor to get approximate distance
 returns total_distance,current_velocity,current_acceleration,current_jerk **/
#define dev_odometer 100

/** the auto calculated plane virtual sensor created from ir raw information
 returns proximity,pitch,yaw,roll **/
#define dev_ir_orientation 200

//Jon's code
/** returns proximity,x_hat,y_hat,z_hat **/
#define dev_ir_vector 201


/////////////////////////////////////////////////////////////////////
// COMMAND STRINGS
/////////////////////////////////////////////////////////////////////

/** This command string is sent, followed by a comma and the device
id (see above) of the sensor whose value is requested. */
#define CMD_GET_SENSOR "get_virtual_sensor"


/** This command string is sent by AI control systems to change the
camera lens filter(s) positions.  */
#define CMD_AUTO_OPTICS "auto_optics"


/** This command string is sent by AI control systems to steer the vehicle.
It is ignored if the system is in manual override mode. */
#define CMD_AUTO_STEERING "auto_steering"

/** This command string is sent by AI control systems to set the vehicle's
throttle.  It is ignored if the system is in manual override mode. */
#define CMD_AUTO_THROTTLE "auto_throttle"

/** This command string is sent by AI control systems to control the vehicle's
pan-tilt unit (usually where the camera is mounted).  It is ignored if the 
system is in manual override mode. */
#define CMD_AUTO_TURRET "auto_turret"

/** This command string switches the system into manual override mode. */
#define CMD_MANUAL_ON "manual_control_on"

/** This command string switches the system into standard (AI) control mode. */
#define CMD_MANUAL_OFF "manual_control_off"

/** This command string is sent by manual control systems to steer the vehicle.
It is ignored if the system is in standard (AI) control mode. */
#define CMD_MANUAL_STEERING "manual_steering"

/** This command string is sent by manual control systems to set the vehicle's
throttle.  It is ignored if the system is in standard (AI) control mode. */
#define CMD_MANUAL_THROTTLE "manual_throttle"



/////////////////////////////////////////////////////////////////////
// MISC
/////////////////////////////////////////////////////////////////////

/** The UDP port number that tbrprobe will use to communicate with clients. */
#define UDP_PORT 1225

/** The standard buffer length used for sending/receiving UDP strings. */
#define UDP_BUF_LEN 100
