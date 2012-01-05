#include "Path.h"
#include <math.h>
#include <iostream>


/* 
Andy Tan
Created: Nov 13th 2010

Contains the implementations for the Path class.
Path is able to return the center path of the track. It takes an array of x,y positions for each wall of the track.
*/

//Constructor
//The positions contained in leftcones and rightcones are relative to the current position of the robot
Path::Path ( ConePosition leftcones, ConePosition rightcones, int size)
:
inputArraySize(size),
forecastRange(2) //default of 2m for now
{
	for ( int i = 0; i < SIZE; i++ )
	{
		distanceToConesSquared[i] = 0;
		pathToTake.x[i] = 0;
		pathToTake.y[i] = 0;
	}
	
	//The rest of the code here modifies pathToTake and entries of distanceToConesSquared. Should be part of the setPathToTake member function.
	int i = 0;
	int r = 0;
	int l = 0;
	
	while ( i < (SIZE-1) && r < SIZE && l < SIZE )
	{
		if ( fabs((leftcones.y[l]*leftcones.y[l] + leftcones.x[l]*leftcones.x[l]) - (rightcones.y[r]*rightcones.y[r] + rightcones.x[r]*rightcones.x[r])) >  0.5 ) //if the distance between corresponding cones on both sides are off by more than 0.5 m
		{
			if  ( (leftcones.y[l]*leftcones.y[l] + leftcones.x[l]*leftcones.x[l]) - (rightcones.y[r]*rightcones.y[r] + rightcones.x[r]*rightcones.x[r]) >  0 ) //if left cone is farther than right cone
			{
				r++;
			}
			else
			{
				l++;
			};
			
			i++;
		}
		else //if distances of rightcones[r] and leftcones[l] to robot is within 0.5 from each other
		{
		distanceToConesSquared[i] = (leftcones.y[i]*leftcones.y[i] + leftcones.x[i]*leftcones.x[i] + rightcones.y[i]*rightcones.y[i] + rightcones.x[i]*rightcones.x[i]) / 2.0;
		
		
		pathToTake.x[i] = ((leftcones.x[i+1] - leftcones.x[i]) + (rightcones.x[i+1] - rightcones.x[i])) / 2.0;
		//The x-component of the path vector is the average of the x-components of the vectors from the ith cone
		//to the (ith + 1) cone of the left and right wall.
		
		pathToTake.y[i] = ((leftcones.y[i+1] - leftcones.y[i]) + (rightcones.y[i+1] - rightcones.y[i])) / 2.0;
		i++;
		}
	}
}
	
//Accessor
//PRE: Path knows the position of the robot and the cones that it can see.
//POST: returns an angle in radians with 0 being straight, negative being left and positive being right
	//that is valid for the robot to take for the next second or so
//The cone positions used will be relative to the robot, which is assumed to be at position 0.
double Path::getDirection( )
{
	double xvect = 0;
	double yvect = 0;
	int count = 0;
	int i = 0;
	
	while ( i < SIZE && distanceToConesSquared[i] < forecastRange*forecastRange )
	{
		if (pathToTake.x[i] == 0 && pathToTake.y[i] == 0)
		{}
		else
		{
			xvect += pathToTake.x[i];
			yvect += pathToTake.y[i];
			count++;
		};
		i++;
	};
	
	yvect /= count;
	xvect /= count;
	
	double theta = atan (xvect/yvect);
	return theta; //in radians. straight ahead is 0, left is negative, right is positive
}

//Accessor
//POST:prints member variables on the screen 
void Path::printInfo()
{
	double direction = this->getDirection();
	std::cout << "Directional heading that is valid for the next " << forecastRange <<" metres: " << direction * 180.0 / 3.14159 << " degrees.\n\n";
	
	std::cout << "Data in distanceToConesSquared:\n i \t distanceToConesSquared[i]\n";
	
	for ( int i = 0; i < SIZE; i++ )
	{
		std::cout << i<< "\t " << distanceToConesSquared[i] << '\n';
	}
	
	std::cout << "\nData in pathToTake:\n";
	std::cout << "i \t pathToTake.x[i] \t pathToTake.y[i] \n";
	for ( int i = 0; i < SIZE; i++ )
	{
		std::cout << i << '\t' << pathToTake.x[i] << "\t \t \t \t " << pathToTake.y[i] << '\n'; 
	} 
}
	