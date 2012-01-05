/*
Andy Tan
Created: Oct 23rd 2010
Last Modified: Nov 6th 2010

.cpp for a singly linked list of ConeNodes.

Class Invariants: An TrackWall contains a dummy node when empty.
				  head points to the first node of the TrackWall, the dummy node.
				  cursor points to the ConeNode which the robot is currently closest to.
*/

#include "TrackWall.h"
#include "ConeNode.h" 
#include <iostream>

struct Coordinate;

//Default Constructor
//PRE: (none)
//POST: an empty TrackWall is made with a dummy node
TrackWall::TrackWall()
{
	ConeNode* newest = new ConeNode(0,0);		//This makes a dummy node.
	newest->next = 0;
	head = newest;
	tail = newest;
	numCones = 0;
}

//Mutator
//PRE: (none)
//POST: a new ConeNode is added to the end of the TrackWall.
void TrackWall::insertCone ( double xpos, double ypos )
{
	ConeNode* newest = new ConeNode( xpos, ypos );

	tail->next = newest;
	
	newest->next = 0;

	setVectorOf(tail); //where tail is now the 2nd last cone

	tail = newest; //tail is now the last cone
}

//Accessor
//PRE: (none)
//POST: prints out a list of cones in the TrackWall
void TrackWall::printWall()
{
	ConeNode* trackStart = head->next; //Skips the dummy node

	while (trackStart != 0) 
	{
		std::cout << "Cone position: (" << trackStart->conePosition.x << "," << trackStart->conePosition.y << ")" << '\n';
		trackStart = trackStart->next;
	}
}

//Destructor
TrackWall::~TrackWall()
{
	ConeNode* ptrToDelete = head;
	ConeNode* nextPtr = 0;

	while ( ptrToDelete != 0 )
	{
		nextPtr = ptrToDelete->next;
		delete ptrToDelete;
		ptrToDelete = nextPtr;
	}

	head = 0;
	tail = 0;
	ptrToDelete = 0;
	nextPtr = 0;
}
	
//Private methods
//------------------------------------------------------------------------------------

	//Mutator
	//PRE: node's next pointer points to a valid node.
	//POST: node's vectorToNextCone is computed from thisnode's condata and thisnode->next's conedata
void TrackWall::setVectorOf (ConeNode* node )
{
		Coordinate myvector( node->next->conePosition.x - node->conePosition.x, node->next->conePosition.y - node->conePosition.y);
		node->vectorToNextCone = myvector;
}
