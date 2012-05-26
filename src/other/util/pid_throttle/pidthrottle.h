

#ifndef PID_THROTTLE_H
#define PID_THROTTLE_H

#include <stdio.h>
#include <iostream>

#include "pid.h"

namespace vision{
	
	class PIDthrottle{
		public:
		
			PIDthrottle();
			PIDthrottle(double Kp, double Ki, double Kd);
			PIDthrottle(double Kp, double Ki, double Kd, double fwdlimit, double revlimit);
		
			double getTargetSpeed();
			void setTargetSpeed(double vel);
			void pause();		//deactivate
			void resume();		//reactivate
			void stop();		//set the target speed to zero, but remain active
			
			void setPIDParams(double Kp, double Ki, double Kd);
			void setKp(double Kp);
			void setKi(double Ki);
			void setKd(double Kd);
			
			double getKp();
			double getKi();
			double getKd();
		
			void setForwardLimit(double limit);
			void setReverseLimit(double limit);
			
			double getThrottle(double vel, double dt = 1.0);
			
		protected:
			double forwardLimit;
			double reverseLimit;
			
			int state;	//0 is paused, 1 is active
			
			PID controller;
	};
	
}

#endif