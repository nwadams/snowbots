	/* 
Andy Tan
Created: Dec 11th 2010

LongestLineNav is a class that sees cones on the left and right wall, then finds the longest straight line to navigate the section 
of the track.
*/
#include <iostream>
#include "LongestLineNav.h"
#include <math.h>

using namespace std;

LongestLineNav::LongestLineNav( const vector<Cone> leftwall, const vector<Cone> rightwall)
{
	LeftWall = leftwall;
	RightWall = rightwall;
}

double LongestLineNav::findLineDirection() 
{

	for ( int l = 0; l < (LeftWall.size()-1); l++ )  //check for sharp left turn.
	{
		if (LeftWall[l+1].angle < LeftWall[l].angle)
			return (LeftWall[l].angle + angleOffset(LeftWall[l].distance));
	}

	for ( int r = 0; r < (RightWall.size()-1); r++ )  //check for sharp right turn.
	{
		if (RightWall[r+1].angle > RightWall[r].angle)
			return (RightWall[r].angle + angleOffset(RightWall[r].distance));
	}

	return ((RightWall.back().angle + LeftWall.back().angle)/2);  //else the road is fairly straight and we'll go down its middle.
}

double LongestLineNav::angleOffset(double distance)
{
	return atan(0.1/distance);
}

void LongestLineNav::printInfo()
{
	cout << "Items in left wall are\n";
	cout << "i \t x \t y \t angle \t distance\n";
	for ( int i = 0; i < LeftWall.size(); i++)
	{
		cout << i << '\t' << LeftWall[i].x << '\t' << LeftWall[i].y << '\t' << LeftWall[i].angle << '\t' << LeftWall[i].distance << endl;
	}
	
	cout << "Items in right wall are\n";
	cout << "i \t x \t y \t angle \t distance\n";
	for ( int i = 0; i < RightWall.size(); i++)
	{
		cout << i << '\t' << RightWall[i].x << '\t' << RightWall[i].y << '\t' << RightWall[i].angle << '\t' << RightWall[i].distance << endl;
	}
	
	cout << "\nDirectional heading is " << this->findLineDirection() << " radians" << endl;
}
