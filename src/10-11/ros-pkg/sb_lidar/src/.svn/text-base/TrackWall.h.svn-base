/*
Andy Tan
Created: Oct 23rd 2010
Last Modified: Nov 6th 2010

This is a singly linked list of ConeNodes.

Class Invariants: An TrackWall contains a dummy node when empty.
				  head points to the first node of the TrackWall, the dummy node.
				  cursor points to the ConeNode which the robot is currently closest to.
*/

#ifndef TRACKWALL_H
#define TRACKWALL_H

class ConeNode;

class TrackWall
{
private:
	ConeNode* head;
	ConeNode* tail;
	int numCones;
	
	//Mutator
	//PRE: thisnode's next pointer points to a valid node.
	//POST: thisnode's vectorToNextCone is computed from node's condata and node->next's conedata
	void setVectorOf (ConeNode* node );

public:
	//Default constructor
	//PRE: (none)
	//POST: An empty ActionList is made with a dummy node.
	TrackWall();

	//Mutator
	//PRE: (none)
	//POST: a new ConeNode is added to the end of the TrackWall.
	void insertCone ( double xpos, double ypos );

	//Accessor
	//PRE: (none)
	//POST: prints out a list of cones in the TrackWall
	void printWall ();


	//Destructor
	~TrackWall();
};

#endif