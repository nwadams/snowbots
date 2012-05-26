#include "util.h"

namespace vision{

double clamp(double val, double minval, double maxval){
	return val < maxval ? (val > minval ? val : minval ) : maxval; 
}

double interpolate(double x, double x0, double x1, double y0, double y1){
	return y0 + (x-x0)*(y1-y0)/(x1-x0);
}

double double_interpolate(double x, double x0, double x1, double x2, double y0, double y1, double y2){
	if(x < x1){
		return interpolate(x,x0,x1,y0,y1);
	}
	else{
		return interpolate(x,x1,x2,y1,y2);
	}
}

double saturation(double x, double x0, double x1, double y0, double y1){
	if(x < x0){return y0;}
	else if(x > x1){return y1;}
	else{return interpolate(x,x0,x1,y0,y1);}
}

}

