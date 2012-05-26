#include "pid.h"
namespace controller{

PID::PID(){
	last_e = 0;		//the error at the last step
	integral = 0;	//the cumulative integral
	
	//tuning parameters
	Kp = 1.0;
	Ki = 1.0;
	Kd = 1.0;

	MV = 0;
}

PID::PID(double Kp, double Ki, double Kd, double SP){
	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;
	
	this->SP = SP;
	
	MV = 0;
}

void PID::setKp(double Kp){
	this->Kp = Kp;
}

void PID::setKi(double Ki){
	this->Ki = Ki;
}

void PID::setKd(double Kd){
	this->Kd = Kd;
}

void PID::setK(double Kp, double Ki, double Kd){
	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;
}

double PID::getKp() const{
	return Kp;
}

double PID::getKi() const{
	return Ki;
}

double PID::getKd() const{
	return Kd;
}

void PID::setPoint(double SP){
	this->SP = SP;
}

double PID::getSetPoint() const{
	return SP;
}

double PID::getMV() const{
	return MV;
}

double PID::step(double PV, double dt){
	//compute the error
	double e = PV - SP;
	
	//compute Pout
	double Pout = Kp * e;
	
	//compute Iout
	integral += e*dt;
	double Iout = Ki * integral;
	
	//compute Dout
	double Dout = Kd * (e - last_e);
	last_e = e;
	
	//compute MV(t)
	MV = Pout + Iout + Dout;
	
	//printf("PV = %#1.3f, Mv = %#1.3f = %#1.3f + %#1.3f + %#1.3f, error = %#1.3f \n",PV,MV,Pout,Iout,Dout,e);
	
	return MV;
}

}