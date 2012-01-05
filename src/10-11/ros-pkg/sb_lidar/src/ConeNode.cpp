/*
Andy Tan
Oct 23rd

.cpp for a node that characterizes a cone.

xCoord and yCoord are the coordinates relative to the robot.
xVectorToNextCone and yVectorToNextCone give the direction to the next cone.
nextCone points to the next cone.
Diameter of each cone is set as 20 cm.
*/

#include "ConeNode.h"

	//Constructor
	//POST: a cone is initialized with its conePosition = xpos and ypos
ConeNode::ConeNode (double xpos, double ypos)
	:
	next(0),
	conePosition(xpos, ypos),
	vectorToNextCone(0,0)
	{
	}

	//Mutator
	//PRE: deltax and deltay are the amount in metres that the robot moves by.
	//POST: the xCoord and yCoord of the cone are updated
void ConeNode::moveCone( double deltax, double deltay )
{
	conePosition.x -= deltax;
	conePosition.y -= deltay;
}


	//Destructor
ConeNode::~ConeNode()
{
	delete next;
}

