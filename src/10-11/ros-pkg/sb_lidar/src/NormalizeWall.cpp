#include <iostream>
#include "NormalizeWall.h"
#include "ConePosition.h"
#include <math.h>
#include <vector>

#define PI 3.14159

using namespace std;

NormalizeWall::NormalizeWall(vector<Cone> leftwall, vector<Cone> rightwall, double distance)
{
	d = distance;
	normalizeLeftWall(leftwall);
	normalizeRightWall(rightwall);
}

void NormalizeWall::addToLeftWall(Coordinate point)
{
	//cout << "add to left" << endl;
	Cone cone;
	cone.x = point.x;
	cone.y = point.y;
	cone.distance = sqrt(pow(point.x,2) + pow(point.y,2));
	//TODO
	double angle = atan(point.y / point.x);
	if (angle > PI/2)
	{
		cone.angle = (PI/2) - angle;
	} else {
		cone.angle = -(PI/2) - angle;
	}
	normalizedLeftWall.push_back(cone);
}

void NormalizeWall::addToRightWall(Coordinate point)
{
	//cout << "add to right" << endl;
	Cone cone;
	cone.x = point.x;
	cone.y = point.y;
	cone.distance = sqrt(pow(point.x,2) + pow(point.y,2));
	//TODO
	double angle = atan(point.y / point.x);
	if (angle > PI/2)
	{
		cone.angle = (PI/2) - angle;
	} else {
		cone.angle = -(PI/2) - angle;
	}
	normalizedRightWall.push_back(cone);
}

void NormalizeWall::PrintLeftCones()
{
	cout << "============Left Cones============" << endl << endl;
	for (int i = 0; i < normalizedLeftWall.size(); i++)
	{
		cout << "Cone: " << i+1 << endl;
		cout << "x: " << normalizedLeftWall[i].x << endl;
		cout << "y: " << normalizedLeftWall[i].y << endl;
		cout << "distance: " << normalizedLeftWall[i].distance << endl;
		cout << "angle: " << normalizedLeftWall[i].angle << endl;
		cout << endl;
	}
}

void NormalizeWall::PrintGraphLeftCones()
{
	cout << "============Left Cones============" << endl << endl;
	for (int i = 0; i < normalizedLeftWall.size(); i++)
	{

		cout << -normalizedLeftWall[i].x;
		cout << " " << normalizedLeftWall[i].y << endl;
	}
}

void NormalizeWall::PrintRightCones()
{
	cout << "============Right Cones============" << endl << endl;
	for (int i = 0; i < normalizedRightWall.size(); i++)
	{
		cout << "Cone: " << i+1 << endl;
		cout << "x: " << normalizedRightWall[i].x << endl;
		cout << "y: " << normalizedRightWall[i].y << endl;
		cout << "distance: " << normalizedRightWall[i].distance << endl;
		cout << "angle:" << normalizedRightWall[i].angle << endl;
		cout << endl;
	}
}

void NormalizeWall::PrintGraphRightCones()
{
	cout << "============Right Cones============" << endl << endl;
	for (int i = 0; i < normalizedRightWall.size(); i++)
	{

		cout << -normalizedRightWall[i].x;
		cout << " " << normalizedRightWall[i].y << endl;
	}
}

void NormalizeWall::normalizeLeftWall(vector<Cone> leftwall)
{
	double carry = 0;
	for(int i = 0; leftwall.size() > i && leftwall.size() > 0; i++)
	{
		Cone current = leftwall[i];
		if(i == 0)
		{
			//first cone
			double count = current.y / d;
			for (int j = 0; j < count; j++)
			{
				Coordinate point;
				point.x = current.x;
				point.y = d*j;
				addToLeftWall(point);
			}
			carry = d - (current.y - (int)count*d);
		}
		if(leftwall.size() - 1 > i)
		{
			Cone next = leftwall[i+1];
			if ((next.x - current.x) != 0)
			{
				double slope = (next.y - current.y) / (next.x - current.x);
				double count = (next.y - current.y) / d;
				for (int j = 0; j < count; j++)
				{
					Coordinate point;
					point.x = current.x + (d*j+carry)/slope;
					point.y = current.y + d*j + carry;
					if(leftwall.size() > i+2 && point.y > next.y)
					{
						break;
					}
					addToLeftWall(point);

				}
				carry = d - ((next.y - current.y - carry)- (int)count*d);
				if(carry >= d)
				{
					carry -= d;
				}
			} else {
				double count = (next.y - current.y) / d;
				for (int j = 0; j < count; j++)
				{
					Coordinate point;
					point.x = current.x;
					point.y = current.y + d*j + carry;
					if (point.y > next.y)
					{
						break;
					}
					addToLeftWall(point);
				}
				carry = d - ((next.y - current.y - carry)- (int)count*d);
				if(carry >= d)
				{
					carry -= d;
				}
			}
		}
	}
}

void NormalizeWall::normalizeRightWall(vector<Cone> rightwall)
{
	double carry = 0;
	for(int i = 0; rightwall.size() > i && rightwall.size() > 0; i++)
	{
		Cone current = rightwall[i];
		if(i == 0)
		{
			//first cone
			double count = current.y / d;
			for (int j = 0; j < count; j++)
			{
				Coordinate point;
				point.x = current.x;
				point.y = d*j;
				addToRightWall(point);
			}
			carry = d - (current.y - (int)count*d);
		}
		if(rightwall.size() - 1 > i)
		{
			Cone next = rightwall[i+1];
			if ((next.x - current.x) != 0)
			{
				double slope = (next.y - current.y) / (next.x - current.x);
				double count = (next.y - current.y) / d;
				for (int j = 0; j < count; j++)
				{
					Coordinate point;
					point.x = current.x + (d*j+carry)/slope;
					point.y = current.y + d*j + carry;
					if(rightwall.size() > i+2 && point.y > next.y)
					{
						break;
					}
					addToRightWall(point);

				}
				carry = d - ((next.y - current.y - carry)- (int)count*d);
				if(carry >= d)
				{
					carry -= d;
				}
			} else {
				double count = (next.y - current.y) / d;
				for (int j = 0; j < count; j++)
				{
					Coordinate point;
					point.x = current.x;
					point.y = current.y + d*j + carry;
					if (point.y > next.y)
					{
						break;
					}
					addToRightWall(point);
				}
				carry = d - ((next.y - current.y - carry)- (int)count*d);
				if(carry >= d)
				{
					carry -= d;
				}
			}
		}
	}
}
