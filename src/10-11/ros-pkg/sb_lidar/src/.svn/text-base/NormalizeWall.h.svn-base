#include <iostream>
#include <vector>
#include "ConePosition.h"
using namespace std;

#ifndef NORMALIZEWALL_H
#define NORMALIZEWALL_H

class NormalizeWall
{
public:

	/*
	 * Take left and right wall
	 * Convert to constant distance depth wall
	 * This will make it easier to do further calcuations
	 * d is the number of m between cones
	 * default is 0.1 m
	 */
	NormalizeWall(vector<Cone> leftwall,vector<Cone> rightwall, double d = 0.1);

	void PrintLeftCones();
	void PrintRightCones();

	void PrintGraphLeftCones();
	void PrintGraphRightCones();

	vector<Cone> normalizedLeftWall;
	vector<Cone> normalizedRightWall;

private:

	//taken from ConePosition
	void addToRightWall(Coordinate point);
	void addToLeftWall(Coordinate point);

	double d;

	/*
	 * These will make you cry
	 * Makes regular distance walls
	 */
	void normalizeLeftWall(vector<Cone> leftwall);
	void normalizeRightWall(vector<Cone> rightwall);
};

#endif
