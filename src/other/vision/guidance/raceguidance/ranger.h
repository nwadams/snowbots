
#ifndef VISION_RANGER_H
#define VISION_RANGER_H

#include <stdio.h>
#include <time.h>
#include <string>
#include <math.h>
#include <vector>

#include <cv.h>
#include <highgui.h>

#include "util.h"

#define VISION_RANGER_SENSORCOUNT 3

using namespace std;

namespace vision{
	
	
	class fuzzyController{
		public:
			struct inputVec{
				double val[VISION_RANGER_SENSORCOUNT];
			};
			
			struct outputVec{
				double steering;
				double throttle;
				double confidence;
			};
			
			fuzzyController();
			fuzzyController(double min, double max);
			
			void setMinMaxRange(double minrange, double maxrange);
			
			void addExample(double nw, double n, double ne, double st, double th, double cf);
			void addExample(inputVec in, outputVec out);
			
			void evaluate(inputVec v,outputVec& result);
		
		
		private:
			
			double dist(inputVec v1, inputVec v2);
		
			double minval;
			double maxval;
		
			vector<inputVec> exampleInput;
			vector<outputVec> exampleOutput;
	};
	
	class ranger{
		public:
			///constructor
			ranger(double minrange, double maxrange, double conf_exponent = 1.0);
		
			///input new range data
			void setRanges(double nw,double n, double ne, double mix);
			
			void setMinMaxRange(double minrange, double maxrange);
			
			///set the confidence exponential - attenuates confidence.  range (0,DBL_MAX], recommend 0.5-2.0
			///LOW EXPONENTS -> HIGHER CONFIDENCE!!!! 0.1^2 = 0.01,  0.1^0.5 = 
			void setConfExponent(double exponent);
		
			///get the value of the average
			double getSmoothRange(int index);
			//get the derivative
			double getSmoothRangeD(int index);
		
			///invoke the ranger's estimate of how the vehicle should behave
			double getSteering();
			double getThrottle();
			double getConfidence();
		
		private:
			
			///exponential averages of range values and their derivatives
			double range_avg[VISION_RANGER_SENSORCOUNT];
			double d_range_avg[VISION_RANGER_SENSORCOUNT];
			
			//the last range value
			double last_range[VISION_RANGER_SENSORCOUNT];
		
			///minimum and maximum rangefinder readings that are considered valid
			double minrange;
			double maxrange;
		
			double steering;
			double throttle;
		
			///rating of how intense the signal is
			///things that increase the intensity:
			///low range values (near objects)
			///multiple low range values
			///large negative derivatives: rapidly approaching objects
			double confidence;
			
			///exponent on the confidence: allows you to vary the 
			///sensitivity.  Defaults to 1.0
			double conf_exp;
			
			///fuzzy logic system for controlling response
			fuzzyController FC;
		
	
	};
	
	

}

#endif
