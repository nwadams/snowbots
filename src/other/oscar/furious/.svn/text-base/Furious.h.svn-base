/**  @file Furious.h
 * Header file for the FuriousBoard low-level driver class and 
 * FuriousCar high-level robot class.
 * @author Ian Phillips <ianfp@freeshell.org>
**/

#ifndef FURIOUS_H
#define FURIOUS_H

#include <termios.h>

/**
 * Represents a single Furious board. 
 */
class FuriousBoard
{
public:
	/**  The baud rate that the Furious module uses for serial communication. */
	static const int BAUDRATE = 19200;

	/**  The delimiter that marks the end of a Furious serial command. */
	static const char *CMD_DELIM;

	/**  The number of servo ports on the Furious board. */
	static const int NUM_SERVOS = 8;

	/**  The number of state variables that the Furious maintains. */
	static const int NUM_STATE_VARS = 11;

	/** Connect to the physical board attached to the given port
	 * (usually /dev/ttyACM0). */
	int connect(const char *portname);

	/**  Disconnect from the physical board. */
	void disconnect();

	/**  Returns true if this FuriousBoard's serial port is open. */
	bool isReady();

	/**  Sends the current servo values to the physical board, then awaits
	 * a response that updates the state of this object.  Typically,
	 * you would call this function in each iteration of your main control
	 * loop to send all pending commands and receive all state updates. */
	void update();

	/**  Sets the value of the given servo.  Note that this does not update
	 * the physical board immediately; it just queues up the new value.  The
	 * physical board will be updated on the next call to update(). */
	void setServo(int servo_id, int value);

	/**  Gets the raw analog value for the given analog port.  Note that if
	 * update() has not been called recently, the value returned might be
	 * out-of-date. */
	int getAnalog(int analog_id);

	/** Returns the raw value of the logic battery state. */
	int getLogicBattery();

	/** Returns the raw value of the motor battery state. */
	int getMotorBattery();

	/** Returns the raw value of the last sonar request. */
	int getSonarValue();

	/** Returns the number of new odometer ticks since the board
	 * was last polled. */
	int getOdometer();

private:
	int serial_fd;
	int servos[FuriousBoard::NUM_SERVOS];
	int state[FuriousBoard::NUM_STATE_VARS];
	struct termios old_termios;
};



/** 
 * Describes the configuration settings of a servo. 
 */
struct ServoConfig
{
	/** The id of the servo port. **/
	int servo_id;
	/** The PPM value for full forward throttle or full right turn. **/
	int max_ppm;
	/** The PPM value for no throttle or no turn.  **/
	int center_ppm;
	/** The PPM value for full reverse throttle or full left turn. **/
	int min_ppm;
	/** Constructor.  The default servo_id (0) means that this
	 * servo has not been configured yet. */
	ServoConfig() : 
		servo_id(-1),
		max_ppm(0),
		center_ppm(0),
		min_ppm(0) {};
};



/** 
 * Describes the maximum and minimum levels of a battery. 
 */
struct BatteryLimits
{
	/** The raw value that indicates the battery is full. **/
	int full;
	/** The raw value that indicates the battery is empty. **/
	int empty;
	/** Constructor.  Default initial values can (and probably
	 * should) be changed. */
	BatteryLimits(int f = 150, int e = 125) :
		full(f), empty(e) {};
};




/**
 * Represents a car controlled by a single Furious board.
 */
class FuriousCar
{
public:
	/** Constructor. */
	FuriousCar() : odometer_ticks(0), odometer_meters_per_tick(0.33) {};

	/** Connects the given car to the board on the given port. **/
	int connect(const char *portname);

	/** Disconnects the given car from its board. **/
	void disconnect();

	/** Returns true if the given car is connected to the board. **/
	bool isReady();

	/** Configures the throttle servo with the given settings. **/
	void configureThrottle(int id, int max, int center, int min);

	/** Configures the steering servo with the given settings. **/
	void configureSteering(int id, int max, int center, int min);

	/** Sets the car's steering to a value between -100 (full left)
	 * and 100 (full right). **/
	void setSteering(int value);

	/** Sets the car's throttle to a value between -100 (full reverse)
	 * and 100 (full forward). **/
	void setThrottle(int value);

	/** Sets the car's throttle and steering to zero. **/
	void stop();

	/** Returns a pointer to the name of the port to which the car
	 * is attached. **/
	const char *getPortName();

	/** Returns the charge level (0 - 100%) of the logic battery. **/
	int getLogicBattery();

	/** Returns the charge level (0 - 100%) of the motor battery. **/
	int getMotorBattery();

	/** Returns the odometer reading, in meters. **/
	double getOdometer();

	/** Returns the voltage (between 0 and 5.0v) coming from the 
	 * specified analog port. **/
	double getAnalog(int analog_id);

	/** Returns the distance (in meters) from the given IR sensor. **/
	double getIR(int ir_id);

	/** Sends all pending commands to and receives all updates from 
	 * the board. **/
	void refresh();

private:
	/** The driver for the FuriousBoard in the car. **/
	FuriousBoard board;
	/** The configurations for the throttle and steering servos. **/
	ServoConfig throttle, steering;
	/** The charge limits for the motor and logic batteries. **/
	BatteryLimits motor_battery, logic_battery;
	/** The port (eg, /dev/ttyACM0) that the Furious board is 
	 * connected to. **/
	char port_name[PORT_NAME_BUF_SIZE];
	/** The number of odometer ticks the car has accumulated so far. **/
	unsigned const long odometer_ticks;
	/** The number of meters the vehicle moves per odometer tick. **/
	const double odometer_meters_per_tick;
};

#endif // FURIOUS_H
