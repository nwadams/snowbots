#include "furious_car.h"

FuriousCar *fcCreateCar()
{
	FuriousCar *newcar = (FuriousCar *) malloc(sizeof(FuriousCar));
	newcar->config = (FuriousCarConfig *) 
		malloc(sizeof(FuriousCarConfig));
}

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
