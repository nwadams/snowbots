#include <cmath> // for abs(), pow()
#include <util/robot_utils.h> // for interpolate()
#include "Furious.h"

/** Connects the given car to the board on the given port. **/
int FuriousCar::connect(const char *portname)
{
	strcpy(this->port_name, portname);
	this->board.connect(this->port_name);
}

/** Disconnects the given car from its board. **/
void FuriousCar::disconnect()
{
	this->board.disconnect();
}

/** Returns true if the given car is connected to the board. **/
bool FuriousCar::isReady()
{
	return this->board.isReady() && 
		this->throttle.servo_id > -1 &&
		this->steering.servo_id > -1;
}

/** Sets the car's steering to a value between -100 (full left)
 * and 100 (full right). **/
void FuriousCar::setSteering(int value)
{
	int raw_val;
	if ( value < 0 ) {
		raw_val = interpolate(value, -100, 0, 
				this->steering.min_ppm, this->steering.center_ppm);
	}
	else {
		raw_val = interpolate(value, 0, 100, 
				this->steering.center_ppm, this->steering.max_ppm);
	}
	this->board.setServo(this->steering.servo_id, raw_val);
}

/** Sets the car's throttle to a value between -100 (full reverse)
 * and 100 (full forward). **/
void FuriousCar::setThrottle(int value)
{
	int raw_val;
	if ( value < 0 ) {
		raw_val = interpolate(value, -100, 0, 
				this->throttle.min_ppm, this->throttle.center_ppm);
	}
	else {
		raw_val = interpolate(value, 0, 100, 
				this->throttle.center_ppm, this->throttle.max_ppm);
	}
	this->board.setServo(this->throttle.servo_id, raw_val);
}

/** Immediately sets the car's throttle and steering to zero. **/
void FuriousCar::stop()
{
	this->setSteering(0);
	this->setThrottle(0);
	this->refresh();
}

/** Returns a pointer to the name of the port to which the car
 * is attached. **/
const char * FuriousCar::getPortName()
{
	return this->port_name;
}

/** Returns the charge level (0 - 100%) of the logic battery. **/
int FuriousCar::getLogicBattery()
{
	return interpolate(this->board.getLogicBattery(), 
			this->logic_battery.empty, this->logic_battery.full,
			0, 100 );
}

/** Returns the charge level (0 - 100%) of the motor battery. **/
int FuriousCar::getMotorBattery()
{
	return interpolate(this->board.getMotorBattery(), 
			this->motor_battery.empty, this->motor_battery.full,
			0, 100 );
}

/** Returns the odometer reading, in meters. **/
double FuriousCar::getOdometer()
{
	this->odometer_ticks += this->board.getOdometer();
	return this->odometer_ticks * this->odometer_meters_per_tick;
}

/** Returns the voltage (between 0 and 5.0v) coming from the 
 * specified analog port. **/
double FuriousCar::getAnalog(int analog_id)
{
	return this->board.getAnalog(analog_id) * 5.0 / 1024.0;
}

/** Returns the distance (in meters) from the given IR sensor. **/
double FuriousCar::getIR(int ir_id)
{
    /* The raw values returned by the sensor describe an exponential
     * curve with respect to distance.  This 4th-order polynomial equation
     * roughly fits that response curve.  (Credit: Matt Baumann) */
    int val = this->getAnalog(ir_id)
	static const int coefficient[] = {
		1.8929, -4.0081, 3.5853, -1.4103, 0.2006 };
	double distance = 0.0;
	for ( int power = 0; power < 5; power++ )
	{
		distance += coefficient[power] * pow(val, power);
	}
	return distance;
}

/** Sends all pending commands to and receives all updates from 
 * the board. **/
void FuriousCar::refresh()
{
	this->board.update();
}

