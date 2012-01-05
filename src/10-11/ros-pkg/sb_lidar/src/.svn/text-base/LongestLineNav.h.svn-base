/* 
Andy Tan
Created: Dec 11th 2010

LongestLineNav is a class that sees cones on the left and right wall, then finds the longest straight line to navigate the section 
of the track.
*/

#ifndef LONGESTLINENAV_H
#define LONGESTLINENAV_H
#include "ConePosition.h"
#include <vector>

using namespace std;

class LongestLineNav {
public:
	LongestLineNav ( const vector<Cone> leftwall, const vector<Cone> rightwall); //insert argument later
	double findLineDirection(); //direction in radians. 0 is straight, positive is to right, negative to left.
	void printInfo();

private:
	double angleOffset (double distance); //adds an offset angle to avoid hitting a cone.
	vector<Cone> LeftWall;
	vector<Cone> RightWall;
};
#endif
