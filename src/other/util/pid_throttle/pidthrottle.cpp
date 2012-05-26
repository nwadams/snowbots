#include "pidthrottle.h"

namespace vision{

PIDthrottle::PIDthrottle(){
	forwardLimit = 100;
	reverseLimit = -100;
	state = 1;
	setPIDParams(0.1, 0.01, 0.01);
	controller.setPoint(0);
}

PIDthrottle::PIDthrottle(double Kp, double Ki, double Kd){
	forwardLimit = 100;
	reverseLimit = -100;
	state = 1;
	setPIDParams(Kp,Ki,Kd);
	controller.setPoint(0);
}

PIDthrottle::PIDthrottle(double Kp, double Ki, double Kd, double fwdlimit, double revlimit){
	forwardLimit = fwdlimit;
	reverseLimit = revlimit;
	state = 1;
	setPIDParams(Kp,Ki,Kd);
	controller.setPoint(0);
}

void PIDthrottle::setTargetSpeed(double vel){
	controller.setPoint(vel);	
}

void PIDthrottle::pause(){
	state = 0;
}
void PIDthrottle::resume(){
	state = 1;
}

void PIDthrottle::stop(){
	controller.setPoint(0);
}

void PIDthrottle::setPIDParams(double Kp, double Ki, double Kd){
	controller.setK(Kp,Ki,Kd);
}

void PIDthrottle::setKp(double Kp){
	controller.setKp(Kp);
}

void PIDthrottle::setKi(double Ki){
	controller.setKi(Ki);
}

void PIDthrottle::setKd(double Kd){
	controller.setKd(Kd);
}

double PIDthrottle::getKp(){
	return controller.getKp();
}

double PIDthrottle::getKi(){
	return controller.getKi();
}

double PIDthrottle::getKd(){
	return controller.getKd();
}

void PIDthrottle::setForwardLimit(double limit){
	forwardLimit = limit;
}

void PIDthrottle::setReverseLimit(double limit){
	reverseLimit = limit;
}

double PIDthrottle::getThrottle(double vel, double dt){
	
	switch(state){
		default:
		case 0:
			//the controller is paused, so always return 0
			return 0;
		break;
		case 1:
			double t = controller.step(vel,dt);
			if(t > forwardLimit){return forwardLimit;}
			else if(t < reverseLimit){return reverseLimit;}
			else {return t;}
		break;
	}
}

}