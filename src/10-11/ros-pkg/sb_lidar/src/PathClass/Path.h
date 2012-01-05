/* 
Andy Tan
Created: Nov 13th 2010

Contains the declarations for the Path class.
Path is able to return the center path of the track. It takes an array of x,y positions for each wall of the track.
*/


#ifndef PATH_H
#define PATH_H
#include "ConePosition.h"

class Path {
	public:
	//Constructor
	Path ( ConePosition leftcones, ConePosition rightcones, int size);
	
	//Mutator
	//POST: modifies pathToTake to apropriate values
	//Will eventually replace the big ugly block in the constructor
	//void setPathToTake (ConePosition leftcones, ConePosition rightcones, int size);
	//NOT YET IMPLEMENTED
	
	//Accessor
	//PRE: Path knows the position of the robot and the cones that it can see.
	//POST: returns an angle in radians with 0 being straight, negative being left and positive being right
	//that is valid for the robot to take for the next second or so
	//The cone positions used will be relative to the robot.
	double getDirection( );
	
	//Accessor
	//POST:prints member variables on the screen 
	void printInfo();
	
	private:
	static const int SIZE = 10; //size of distanceToConesSquared
	int inputArraySize; //size of array of input ConePositions
	double forecastRange; //takes cones up to forecastRange metres away into account when calculating directional vector in getDirection()

	ConePosition pathToTake; //contains an array of x,y directional vectors corresponding to the average of the directional
								//vectors at the ith position of leftcones and rightcones.
	double distanceToConesSquared [SIZE]; //array of the distance squared of cones at the ith position from robot. This array should be at least of size inputArraySize
											

};

#endif