/*
Andy Tan
Oct 23rd

Header file for a node that characterizes a cone. These ConeNodes are used in the TrackWall linked list.

xCoord and yCoord are the coordinates relative to the robot.
xVectorToNextCone and yVectorToNextCone give the direction to the next cone.
nextCone points to the next cone.
Diameter of each cone is set as 20 cm.
*/

#ifndef CONENODE_H
#define CONENODE_H

struct Coordinate
{
	double x;
	double y;

	//Constructor
	//POST: makes an initialized coordinate
	Coordinate(double newx, double newy) :x(newx), y(newy) {} 
};

class ConeNode
{
public:
	ConeNode* next;
	Coordinate conePosition;
	Coordinate vectorToNextCone;
	static const int DIAMETER = 20; //20 cm per cone

	//Constructor
	//POST: a cone is initialized with its conePosition = xpos and ypos
	ConeNode (double xpos, double ypos);

	//Mutator
	//PRE: deltax and deltay are the amount in metres that the robot moves by.
	//POST: the xCoord and yCoord of the cone are updated
	void moveCone( double deltax, double deltay );

	//Destructor
	~ConeNode();
};

#endif 
