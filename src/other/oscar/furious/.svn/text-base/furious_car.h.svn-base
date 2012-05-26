/**  @file furious_car.h
 * A high-level interface that wraps furious_driver.h.  This
 * uses configuration settings to translate the raw data sent
 * and received in furious_driver into a human-friendly 
 * interface for a car-like vehicle.
 * @author Ian Phillips <ianfp@freeshell.org>
**/

#ifndef FURIOUS_CAR_H
#define FURIOUS_CAR_H

#include "furious_driver.h"

#define PORT_NAME_BUF_SIZE 100

/** Contains all of the configuration settings that FuriousCar
 * will need. **/
typedef struct {
	/** The id of the servo port to which the steering is attached. **/
	int steer_servo_id;
	/** The PPM value for a full right turn. **/
	int steer_max_right;
	/** The PPM value for no turn (ie, straight).  **/
	int steer_center;
	/** The PPM value for a full left turn. **/
	int steer_max_left;
	/** The id of the servo port to which the throttle is attached. **/
	int throttle_servo_id;
	/** The PPM value for full forward throttle. **/
	int throttle_max_fwd;
	/** The PPM value for throttle off (ie, stopped). **/
	int throttle_stop;
	/** The PPM value for full reverse throttle. **/
	int throttle_max_rev;
	int battery_logic_full;
	int battery_logic_empty;
	int battery_motor_full;
	int battery_motor_empty;
	/** The number of meters the vehicle moves per odometer tick. **/
	double odometer_meters_per_tick;
} FuriousCarConfig;

/** Represents a car controlled by a single Furious board. **/
typedef struct {
	FuriousDriver *driver;
	FuriousCarConfig *config;
	char port_name[PORT_NAME_BUF_SIZE];
	int odometer_ticks;
} FuriousCar;

/** Allocates and initializes a new FuriousCar object. **/
FuriousCar *fcCreateCar();

/** Deallocates the given car and sets the car's pointer to NULL. **/
void fcReleaseCar(FuriousCar **car);

/** Connects the given car to the board on the given port. **/
int fcConnect(FuriousCar *car, const char *portname);

/** Disconnects the given car from its board. **/
void fcDisconnect(FuriousCar *car);

/** Returns true if the given car is connected to the board. **/
int fcIsReady(FuriousCar *car);

/** Sets the car's steering to a value between -100 (full left)
 * and 100 (full right). **/
void fcSetSteering(FuriousCar *car, int value);

/** Sets the car's throttle to a value between -100 (full reverse)
 * and 100 (full forward). **/
void fcSetThrottle(FuriousCar *car, int value);

/** Sets the car's throttle and steering to zero. **/
void fcStop(FuriousCar *car);

/** Returns a pointer to the name of the port to which the car
 * is attached. **/
const char *fcGetPortName(FuriousCar *car);

/** Returns the charge level (0 - 100%) of the logic battery. **/
int fcGetLogicBattery(FuriousCar *car);

/** Returns the charge level (0 - 100%) of the motor battery. **/
int fcGetMotorBattery(FuriousCar *car);

/** Returns the odometer reading, in meters. **/
double fcGetOdometer(FuriousCar *car);

/** Returns the reading of the given sonar, in meters. **/
double fcGetSonar(FuriousCar *car, int sonar_id);

/** Returns the voltage (between 0 and 5.0v) coming from the 
 * specified analog port. **/
double fcGetAnalog(FuriousCar *car, int analog_id);

/** Returns the distance (in meters) from the given IR sensor. **/
double fcGetIR(FuriousCar *car, int ir_id);

/** Sends all pending commands to and receives all updates from 
 * the board. **/
void fcRefresh(FuriousCar *car);

#endif // define FURIOUS_CAR_H
