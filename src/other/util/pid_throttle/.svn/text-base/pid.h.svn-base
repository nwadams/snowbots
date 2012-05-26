

#ifndef MY_CONTROLLERS_H
#define MY_CONTROLLERS_H

#include <stdio.h>

namespace vision{

	class PID{
		public:
			PID();
			PID(double Kp, double Ki, double Kd, double SP);
			
			void setKp(double Kp);
			void setKi(double Ki);
			void setKd(double Kd);
			void setK(double Kp, double Ki, double Kd);
			
			double getKp() const;
			double getKi() const;
			double getKd() const;
			
			void setPoint(double SP);
			double getSetPoint() const;
			
			double getMV() const;
			
			double step(double PV, double dt);
			
		protected:
			
			double last_e;		//the error at the last step
			double integral;	//the cumulative integral
			
			//set point (target value)
			double SP;
			
			//manipulated variable
			double MV;
			
			//tuning parameters
			double Kp;
			double Ki;
			double Kd;
			
	
	};


}

#endif
