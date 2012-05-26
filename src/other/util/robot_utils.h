#ifndef ROBOT_UTILS_H
#define ROBOT_UTILS_H

/** Returns a new value between new_low and new_high that is proportional
 * to val's position between old_low and old_high. */
double interpolate(double val, 
		double old_low, double old_high, 
		double new_low, double new_high);

int interpolate(int val, 
		int old_low, int old_high, 
		int new_low, int new_high);

#endif // ROBOT_UTILS_H
