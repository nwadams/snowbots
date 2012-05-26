#include "ranger.h"

namespace vision{

fuzzyController::fuzzyController(){
	minval = 0.1;
	maxval = 2.0;

}


fuzzyController::fuzzyController(double min, double max){
	minval = min;
	maxval = max;

}

void fuzzyController::addExample(double nw, double n, double ne, double st, double th, double cf){
	inputVec in;
	in.val[0] = nw;
	in.val[1] = n;
	in.val[2] = ne;
	outputVec out;
	out.steering = st;
	out.throttle = th;
	out.confidence = cf;
	
	addExample(in,out);
}

void fuzzyController::setMinMaxRange(double minrange, double maxrange){
	minval = minrange;
	maxval = maxrange;
}

void fuzzyController::addExample(inputVec in, outputVec out){
	exampleInput.push_back(in);
	exampleOutput.push_back(out);
}

void fuzzyController::evaluate(inputVec v, outputVec& out){
	outputVec sum;
	sum.steering = 0;
	sum.throttle = 0;
	sum.confidence = 0;
	
	double totalweight = 0;
	
	for(int i = 0; i < (int) exampleInput.size(); i++){
		//for each example:
		
		//get the distance
		double d = dist(v,exampleInput[i]);
		
		//compute the weighting (normalized)
		double weight = saturation(d,0,1.0,1.0,0.0);
		//printf("dist= %f, weight = %f\n",d,weight);
		
		//add the weighted value to the sum
		sum.steering += weight * exampleOutput[i].steering;
		sum.throttle += weight * exampleOutput[i].throttle;
		sum.confidence += weight * exampleOutput[i].confidence;
		
		totalweight += weight;
		
	}
	if(totalweight > 0){
		sum.steering /= totalweight;
		sum.throttle /= totalweight;
		sum.confidence /= totalweight;
	}
	out.steering = sum.steering;
	out.throttle = sum.throttle;
	out.confidence = sum.confidence;
}

double fuzzyController::dist(inputVec v1, inputVec v2){
	double d = sqrt( 
		(v1.val[0]-v2.val[0])*(v1.val[0]-v2.val[0]) +
		(v1.val[1]-v2.val[1])*(v1.val[1]-v2.val[1]) +
		(v1.val[2]-v2.val[2])*(v1.val[2]-v2.val[2]) );
	//printf("|| [%f,%f,%f] - [%f,%f,%f] || = %f\n",v1.val[0],v1.val[1],v1.val[2],v2.val[0],v2.val[1],v2.val[2],d);
	return d;
}




	
//----------------------------------------------------------------------
	
ranger::ranger(double minrange, double maxrange, double conf_exponent){
	
	this->minrange = minrange;
	this->maxrange = maxrange;
	
	for(int i = 0; i < VISION_RANGER_SENSORCOUNT; i++){
		range_avg[i]=maxrange*2;
		d_range_avg[i]=0;
	}
	
	conf_exp = conf_exponent;
	
	
	steering = 0;
	throttle = 0;
	confidence = 0;
	
	FC.setMinMaxRange(minrange,maxrange);
	
	FC.addExample(maxrange,maxrange,maxrange,0,1.0,0.1);
	
	FC.addExample(maxrange,maxrange,maxrange*0.5,-0.75,0.5,0.4);
	FC.addExample(maxrange*0.5,maxrange,maxrange,0.75,0.5,0.4);
	
	FC.addExample(0,maxrange,maxrange,1.0,0.5,0.5);
	FC.addExample(maxrange,maxrange,0,-1.0,0.5,0.5);
	
	FC.addExample(maxrange*0.5,0,maxrange,1.0,0.25,0.7);
	FC.addExample(maxrange,0,maxrange*0.5,-1.0,0.25,0.7);
	
	FC.addExample(maxrange*0.75,maxrange*0.5,maxrange,0.5,0.25,0.5);
	FC.addExample(maxrange,maxrange*0.5,maxrange*0.75,-0.5,0.25,0.5);
	
	//FC.addExample(0,0,0,1.0,0.25,0.9);
	//FC.addExample(0,0,maxrange,-1.0,-0.5,0.6);
	//FC.addExample(maxrange,0,0,1.0,-0.5,0.6);	
}
	
void ranger::setRanges(double nw,double n, double ne, double mix){
	
	//nw = 0, n = 1, ne = 2
	
	//compute the derivative and store it in the exponential average
	//this means that d_range_avg contains a smoothed approximation of
	//the rate of change in the range values
	d_range_avg[0] = mix*(nw-last_range[0]) + (1.0-mix)*d_range_avg[0];
	d_range_avg[1] = mix*(n-last_range[1]) + (1.0-mix)*d_range_avg[1]; 
	d_range_avg[2] = mix*(ne-last_range[2]) + (1.0-mix)*d_range_avg[2];
	
	//compute the exponential average of the 
	range_avg[0] = mix*nw + (1.0-mix)*range_avg[0];
	range_avg[1] = mix*n + (1.0-mix)*range_avg[1];
	range_avg[2] = mix*ne + (1.0-mix)*range_avg[2];
	
	//set the memory of the last values
	last_range[0] = nw;
	last_range[0] = n;
	last_range[0] = ne;
	
	fuzzyController::inputVec in;
	in.val[0] = clamp(range_avg[0],minrange,maxrange);
	in.val[1] = clamp(range_avg[1],minrange,maxrange);
	in.val[2] = clamp(range_avg[2],minrange,maxrange);
	fuzzyController::outputVec result;
	FC.evaluate(in,result);
	
	steering = result.steering;
	throttle = result.throttle;
	confidence = result.confidence;
	
}

///get the value of the average
double ranger::getSmoothRange(int index){
	return range_avg[index];
}

//get the derivative
double ranger::getSmoothRangeD(int index){
	return d_range_avg[index];
}

void ranger::setMinMaxRange(double minrange, double maxrange){
	this->minrange = minrange;
	this->maxrange = maxrange;
}

void ranger::setConfExponent(double exponent){
	conf_exp = exponent;
}

double ranger::getSteering(){
	return steering;
}

double ranger::getThrottle(){
	return throttle;
}

double ranger::getConfidence(){
	return confidence;
}





//----------------------------------------------------------------------

}
