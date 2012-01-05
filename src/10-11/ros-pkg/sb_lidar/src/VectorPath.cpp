/*
 * VectorPath.cpp
 *
 *  Created on: 2010-12-28
 *      Author: nwadams
 */

#include "VectorPath.h"
#include <iostream>
#include "ConePosition.h"
#include <math.h>
#include <vector>
#define PI 3.14159

using namespace std;

VectorPath::VectorPath(vector<Cone> leftwall, vector<Cone> rightwall) {
	int leftSize = leftwall.size();
	int rightSize = rightwall.size();

	if (leftSize < rightSize)
	{
		int i;
		for (i = 0; i < leftSize; i++)
		{
			Coordinate point;
			point.x = (leftwall[i].x + rightwall[i].x) / 2;
			point.y = leftwall[i].y;
			addToPath(point);
		}
		for (int j = 0; j < (rightSize - leftSize); j++)
		{
			Coordinate point;
			// 1 meter to left of right wall
			point.x = rightwall[i+j].x - rightwall[i].x;
			point.y = rightwall[i+j].y;
			addToPath(point);
		}
	} else {
		int i;
		for (i = 0; i < rightSize; i++)
		{
			Coordinate point;
			point.x = (rightwall[i].x + leftwall[i].x) / 2;
			point.y = rightwall[i].y;
			addToPath(point);
		}
		for (int j = 0; j < (leftSize - rightSize); j++)
		{
			Coordinate point;
			// 1 meter to right of left wall
			point.x = leftwall[i+j].x - leftwall[i].x;
			point.y = leftwall[i+j].y;
			addToPath(point);
		}
	}
}

VectorPath::~VectorPath() {
}

void VectorPath::addToPath(Coordinate point)
{
	//cout << "add to left" << endl;
	Cone cone;
	cone.x = point.x;
	cone.y = point.y;
	cone.distance = sqrt(pow(point.x,2) + pow(point.y,2));
	//TODO angle doesn't work
	double angle = atan(point.y / point.x);
	if (angle > PI/2)
	{
		cone.angle = (PI/2) - angle;
	} else {
		cone.angle = -(PI/2) - angle;
	}
	path.push_back(cone);
}

void VectorPath::printPath()
{
	cout << "============Path Cones============" << endl << endl;
	for (int i = 0; i < path.size(); i++)
	{
		cout << "Cone: " << i+1 << endl;
		cout << "x: " << path[i].x << endl;
		cout << "y: " << path[i].y << endl;
		cout << "distance: " << path[i].distance << endl;
		cout << "angle: " << path[i].angle << endl;
		cout << endl;
	}
}

void VectorPath::printGraph()
{
	cout << "============Path Cones============" << endl << endl;
	for (int i = 0; i < path.size(); i++)
	{

		cout << -path[i].x;
		cout << " " << path[i].y << endl;
	}
}

