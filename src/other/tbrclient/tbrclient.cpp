/**
 * @author Matthew Baumann
 * @brief tbrclient is a class that can be inserted into a C++ program that will handle
 * UDP communication with tbrprobe.  Thus, any client program can treat this module as an
 * abstraction of the tbrprobe-controlled hardware.
 */

#include "tbrclient.h"

namespace tbrprobe07{
	
tbrclient::tbrclient(){
	sprintf(command,"");
	state = -1;
	observer = NULL;
	current_steering = 0;
	current_throttle = 0;
    current_pan = 0;
    current_tilt = 0;
}

tbrclient::~tbrclient(){
	if(state == 0)
	{
		finalize();
	}
}

bool tbrclient::initialize(int port, const char* addr) {
	if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) == -1)
	{
		if(observer != NULL) {
			*observer << "Error: failed to open socket." << std::endl;
		}
		return false;
	}
	if ( observer != NULL ) {
		*observer << "Connecting to " << addr << " on port " << port << std::endl;
	}
	
	their_addr.sin_family = AF_INET;     // host byte order
	their_addr.sin_port = htons(port); // short, network byte order
	their_addr.sin_addr.s_addr = inet_addr(addr);
	memset(their_addr.sin_zero, '\0', sizeof their_addr.sin_zero);
	
	state = 0;
	
	return true;
}

bool tbrclient::finalize() {
	close(sockfd);
	state = -1;
}

bool tbrclient::ready() {
	return (state == 0);
}

bool tbrclient::setSteering(int value) {
	current_steering = value;
	sprintf(command, "%s,%d", CMD_AUTO_STEERING, value);
	return sendCommand(command);
}

bool tbrclient::setThrottle(int value) {
	current_throttle = value;
	sprintf(command, "%s,%d", CMD_AUTO_THROTTLE, value);
	return sendCommand(command);
}

int tbrclient::getSteering() { return current_steering; }
int tbrclient::getThrottle() { return current_throttle; }

bool tbrclient::setTurret(int tilt, int pan){
    current_tilt = tilt;
    current_pan = pan;
	sprintf(command, "%s,%d,%d", CMD_AUTO_TURRET, tilt, pan);
	return sendCommand(command);
}

int tbrclient::getPan()  { return current_pan; }
int tbrclient::getTilt() { return current_tilt; }

double tbrclient::getSonarN() {
	return requestSonarData(dev_sonar_front_center);
}
double tbrclient::getSonarNW() {
	return requestSonarData(dev_sonar_front_left);
}
double tbrclient::getSonarNE() {
	return requestSonarData(dev_sonar_front_right);
}
double tbrclient::getSonarS() {
	return requestSonarData(dev_sonar_rear_center);
}

//request odometer information form the server
double tbrclient::getInfraredN(){
	return correctIRData(requestADData(1));
}

double tbrclient::getInfraredNW(){
	return correctIRData(requestADData(0));
}

double tbrclient::getInfraredNE(){
	return correctIRData(requestADData(2));
}

double tbrclient::getInfrared(int device_id){
	return correctIRData(requestADData(device_id));
}

double tbrclient::getKillSwitchVal(){
	return requestADData(5);
}

double tbrclient::getOdometerDistance() {
	if (! requestOdometerData() ) return SENSOR_ERROR_VAL;
	return odometer.distance;
}
double tbrclient::getOdometerVelocity() {
	if (! requestOdometerData() ) return SENSOR_ERROR_VAL;
	return odometer.velocity;
}
double tbrclient::getOdometerAcceleration() {
	if (! requestOdometerData() ) return SENSOR_ERROR_VAL;
	return odometer.acceleration;
}


void tbrclient::setObserver(std::ostream* obs) {
	observer = obs;
}

// Private methods
bool tbrclient::sendCommand(char *cmd) {
	if(observer != NULL) {
		*observer << "Sending: \"" << cmd << "\"" <<std::endl;
	}
	
	if ((numbytes = sendto(sockfd, cmd, strlen(cmd), 0,
	                       (struct sockaddr *) & their_addr, sizeof their_addr)) == -1)
	{
		if(observer != NULL) {
			*observer << "Error: sending command \""<<cmd<<"\" failed." << std::endl;
		}
		return false;
	}
	return true;
}

bool tbrclient::receiveSensorData() {
	if (observer != NULL) {
		*observer << "Awaiting sensor data" << std::endl;
	}
	int numbytes = recvfrom(sockfd, reply, BUF_SIZE, 0, NULL, NULL);
	if ( numbytes == -1 ) {
		if ( observer != NULL ) {
			*observer << "Error: receiving data failed." << std::endl; 
		}
		return false;
	}
	reply[numbytes] = '\0';  // terminate the reply string with a null character
	if ( observer != NULL ) {
		*observer << "Received '" << reply << "'." << std::endl;
	}
	return true;
}

double tbrclient::requestSonarData(int sonar_device_id) {
	sprintf(command, "%s,%d", CMD_GET_SENSOR, sonar_device_id);
	if (! sendCommand(command) ) return SENSOR_ERROR_VAL;
	else if (! receiveSensorData() ) return SENSOR_ERROR_VAL;
	else {
		int device_id;
		double value;
		sscanf(reply, "virtual_sensor,%i,%lf", &device_id, &value);
		return value;
	}
}

bool tbrclient::requestOdometerData() {
	sprintf(command, "%s,%d", CMD_GET_SENSOR, dev_odometer);
	if (! sendCommand(command) ) return false;
	else if (! receiveSensorData() ) return false;
	else {
		int device_id;
		sscanf(reply, "virtual_sensor,%i,%lf,%lf,%lf,%lf",
			&device_id,
			&odometer.distance,
			&odometer.velocity,
			&odometer.acceleration,
			&odometer.jerk );
		return true;
	}
}

double tbrclient::requestADData(int ir_device_ID){

	int dev_id = dev_AD0;
	switch(ir_device_ID){
		default:
		case 0:
			dev_id = dev_AD0;
		break;
		case 1:
			dev_id = dev_AD1;
		break;
		case 2:
			dev_id = dev_AD2;
		break;
		case 3:
			dev_id = dev_AD3;
		break;
		case 4:
			dev_id = dev_AD4;
		break;
		case 5:
			dev_id = dev_AD5;
		break;
	}

	sprintf(command, "%s,%d", CMD_GET_SENSOR, dev_id);
	if (! sendCommand(command) ) return SENSOR_ERROR_VAL;
	else if (! receiveSensorData() ) return SENSOR_ERROR_VAL;
	else {
		int device_id;
		double value;
		sscanf(reply, "virtual_sensor,%i,%lf", &device_id, &value);
		return value;
	}
}

double tbrclient::correctIRData(double val){
	//if(val > 82)
	//{
	//	return (6787.0 / (val - 3.0)) - 4.0;  //convert to meters
	//}else{
	//	return 81;	//max range of GP2D12 set to 80 cm
	//}
	
	// Performed a 4-th order polynomial fit to data.  Matt made this
	// up by manually testing the output at various distances and then
	// using Excel to generate the polynomial fit.
	return 0.2006*val*val*val*val - 1.4103*val*val*val + 3.5853*val*val - 4.0081*val + 1.8929;
}
	
}
