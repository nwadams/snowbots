#include "robot_utils.h"

double interpolate(double val, 
		double old_low, double old_high, 
		double new_low, double new_high)
{
	return  new_low + (
		( abs(val - old_low) / abs(old_high - old_low) ) *
		( new_high - new_low ) ) ;
}

int interpolate(int val, 
		int old_low, int old_high, 
		int new_low, int new_high)
{
	return int(
		interpolate( double(val), double(old_low), double(old_high),
			double(new_low), double(new_val) ) );
}
