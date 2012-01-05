/*
Navigation 1.4

Code used during Robot Racing 2011 Competition

Name: Edward Han
Date: July 23, 2011

*/

#include <iostream>
#include <string>
#include <vector>
#include <math.h>
#include <iomanip>
#include "ConeLocation.h"
#define PI 3.1415
using namespace std;

ConeLocation::ConeLocation()
{
	coneCount = 0;
	queueCount = 0;
	new_cone = false;
	last_distance = 0;
	last_angle = 0;
	avg_distance = 0;
	avg_angle = 0;
	text_distance = "distance";
	text_angle = "angle";

	inner_view_angle = 14; // 14 for circuit, 20 for drag
	full_view_angle = 78;
	cent = 0;
		
}

double ConeLocation::ToDegrees(double radian)
{
	double degrees;
	degrees = radian * (180/PI);
	return degrees;
}

void ConeLocation::GetConstants(double _maxDistance, double _minDistance, double _maxAngle, double _minAngle, double _angleInc)
{
	maxDistance = 5;  // change to _maxDistance if you need 5.2m range. otherwise 4m
	minDistance = _minDistance;
	maxAngle = ToDegrees(_maxAngle);
	minAngle = ToDegrees(_minAngle);
	angleInc = ToDegrees(_angleInc);
	total = ((maxAngle - minAngle)/(angleInc)); // double to int but that's ok, should use atoi
}

void ConeLocation::GetMyData(double _my_data[])
{
	for (int i = 0; i < total; i++)
	{
		my_data[i] = _my_data[i];
	}
	cout << endl;
}

void ConeLocation::GetDistanceAngle()
{
	for (int i = 0; i < total; i++)
	{
		if (my_data[i] > minDistance && my_data[i] < maxDistance)
		{
			distances.push_back(my_data[i]);
			angles.push_back((maxAngle-minAngle)/2 - angleInc*i );
		}
	}
}

// ----------------------------Point---------------------------

void ConeLocation::IdentifyPoint()
{
	for (int i = 0; i < distances.size(); i++)
	{
		AddToPointList(distances[i], angles[i]);
	}
}

void ConeLocation::AddToPointList(double distance, double angle)
{
	point.distance = distance;
	point.angle = angle;
	point_list.push_back(point);
}

void ConeLocation::SeparatePoint()
{
	for (unsigned int i = 0; i < point_list.size(); i++)
	{ 
		if (point_list[i].angle >= -full_view_angle && point_list[i].angle < -inner_view_angle)
		{
			left.push_back(point_list[i]);
		}
		else if (point_list[i].angle >= -inner_view_angle && point_list[i].angle <= inner_view_angle)
		{
			front.push_back(point_list[i]);
		}
		else if (point_list[i].angle > inner_view_angle && point_list[i].angle <= full_view_angle)
		{
			right.push_back(point_list[i]);
		}
		else 
		{
			//cout << "invalid point: " << point_list[i].distance << " "  << point_list[i].angle << endl;
		}
	}
}

void ConeLocation::PrintPointList()
{
	cout << "-------Print Point List (Distance, Angle)--------" << endl;
	
	for (unsigned int i = 0; i < point_list.size(); i++)
	{
		cout << "Point " << i+1 << ": (" << setprecision(3) << point_list[i].distance << ", " 
                     << setprecision(3) << point_list[i].angle << ")  " << endl;
	}
}

// ----------------------------------End Point-------------------------------------

void ConeLocation::IdentifyCone()
{
	for (unsigned int i = 0; i < distances.size(); i++)
	{
		if (i == 0)
		{
			ItIsTheFirstCone(distances[i], angles[i]); 
			StoreQueue(distances[i], angles[i]);
		}
		else if (i == (distances.size() - 1))
		{
			ItIsTheLastCone(distances[i], angles[i]);
			StoreQueue(distances[i], angles[i]);
			GetAverage(queue_distance, text_distance);
			GetAverage(queue_angle, text_angle);
			AddToConeList(avg_distance, avg_angle);
			ClearQueue();
		}
		else if (ItIsTheSameCone(distances[i], angles[i]))
		{
			StoreQueue(distances[i], angles[i]);
		}
		else if (ItIsANewCone(distances[i], angles[i]))
		{
			GetAverage(queue_distance, text_distance);
			GetAverage(queue_angle, text_angle);
			AddToConeList(avg_distance, avg_angle);
			//PrintQueue(queue_distance);
			ClearQueue();
			StoreQueue(distances[i], angles[i]);
		}
	}	
}

void ConeLocation::ItIsTheFirstCone(double distance, double angle)
{
	coneCount++;
	last_distance = distance;
	last_angle = angle;
}

bool ConeLocation::ItIsTheSameCone(double distance, double angle)
{
	if ((fabs (distance - last_distance) < 0.1) && (fabs(angle - last_angle) < 0.5))
	{
		last_distance = distance;
		last_angle = angle;
		return true;
	}
	else
	{
		return false;
	}
}

bool ConeLocation::ItIsANewCone(double distance, double angle)
{
	if ((fabs (distance - last_distance) >= 0.1) && (fabs(angle - last_angle) >= 0.5))
	{
		coneCount++;
		last_distance = distance;
		last_angle = angle;
		return true;
	}
	else
	{
		return false;
	}
}

void ConeLocation::ItIsTheLastCone(double distance, double angle)
{
	last_distance = distance;
	last_angle = angle;


}

void ConeLocation::GetAverage(vector<double> queue, string text)
{
	double sum = 0;
	double _avg = 0;
	
	for (unsigned int i = 0; i < queue.size(); i++)
	{
		sum += queue[i];
	}

	if (queue.size() == 0)
	{
		cout << "Divide by 0!!" << endl;
	}
	else
	{
		_avg = sum / queue.size();

		if (text == "distance")
		{
			avg_distance = _avg;
		}
		else if (text == "angle")
		{
			avg_angle = _avg;
		}

		//PrintAverage(_avg);
	}
}

void ConeLocation::AddToConeList(double distance, double angle)
{
	cone.distance = distance;
	cone.angle = angle;
	cone_list.push_back(cone);
}

void ConeLocation::SeparateCone()
{
	for (unsigned int i = 0; i < cone_list.size(); i++)
	{
		if (cone_list[i].angle >= -full_view_angle && cone_list[i].angle < -inner_view_angle)
		{
			left.push_back(cone_list[i]);
		}
		else if (cone_list[i].angle >= -inner_view_angle && cone_list[i].angle < cent)
		{
			middle_left.push_back(cone_list[i]);
		}
		else if (cone_list[i].angle >= cent && cone_list[i].angle <= inner_view_angle)
		{
			middle_right.push_back(cone_list[i]);
		}
		else if (cone_list[i].angle > inner_view_angle && cone_list[i].angle <= full_view_angle)
		{
			right.push_back(cone_list[i]);
		}
		else 
		{
			//cout << "invalid cone: " << cone_list[i].distance << " "  << cone_list[i].angle << endl;
		}
	}
}

void ConeLocation::StoreQueue(double distance, double angle)
{
	queue_distance.push_back(distance);
	queue_angle.push_back(angle);
}

void ConeLocation::ClearQueue()
{
	queue_distance.clear();
	queue_angle.clear();
}

void ConeLocation::Clear()
{
	coneCount = 0;
	queueCount = 0;
	distances.clear();
	angles.clear();
	
	point_list.clear();
	cone_list.clear();
	left.clear();
	middle_left.clear();
	middle_right.clear();
	right.clear();
	front.clear();
}

void ConeLocation::PrintConstants()
{
	cout << "=====...Printing Constants...=====" << endl << endl;
	cout << "Max Distance: " << maxDistance << endl;
	cout << "Min Distance: " << minDistance << endl;
	cout << "Max Angle: " << maxAngle << endl;
	cout << "Min Angle: " << minAngle << endl;
	cout << "Angle Increment: " << angleInc << endl;
	cout << "Total Values: " << total << endl << endl;
}

void ConeLocation::PrintDistance()
{
	int c = 0;
	cout << "=====...Printing Distance...=====" << endl << endl;
	
	for (unsigned int i = 0; i < distances.size(); i++)
	{	
		cout << setprecision(3) << distances[i] << " ";
		c++;
		if (c == 5)
		{
			cout << endl;
			c = 0;
		}
	}
	cout << endl << endl;
}

void ConeLocation::PrintAngle()
{
	int c = 0;
	cout << "=====...Printing Angle...=====" << endl << endl;
	
	for (unsigned int i = 0; i < angles.size(); i++)
	{	
		cout << setprecision(3) << angles[i] << " ";
		c++;
		if (c == 5)
		{
			cout << endl;
			c = 0;
		}
	}
	cout << endl << endl;
}

void ConeLocation::PrintDistanceAngle()
{
	cout << "=====...Printing Distance...=====" << endl << endl;
	
	for (unsigned int i = 0; i < distances.size(); i++)
	{	
		cout << distances[i] << " ";
	}
	
	cout << endl << endl;
	
	cout << "=====...Printing Angle...=====" << endl << endl;
	for (unsigned int i = 0; i < angles.size(); i++)
	{	
		cout << angles[i] << " ";
	}
	
	cout << endl << endl;
	
}

void ConeLocation::PrintMyData()
{
	cout << "=====...Printing My Data...=====" << endl << endl;
	for (int i = 0; i < total; i++)
	{
		cout << setprecision(3) << my_data[i] << " ";
	}
	cout << endl << endl;
}

void ConeLocation::PrintConeCount()
{
	cout << "=====...Printing Cone Count...=====" << endl << endl;
	cout << coneCount << endl << endl;
}

void ConeLocation::PrintQueue(vector<double> my_queue)
{
	int c = 0;
	cout << "=====...Printing Queue...=====" << endl << endl;
	for (unsigned int i = 0; i < my_queue.size(); i++)
	{
		cout << setprecision(3) << my_queue[i] << " ";
		c++;
		if (c == 5)
		{
			cout << endl;
			c = 0;
		}
	}
	cout << endl << endl;
}

void ConeLocation::PrintAverage(double _avg)
{
	cout << "=====...Printing Average...=====" << endl << endl;
	cout << _avg;
	cout << endl << endl;
}

void ConeLocation::PrintConeList()
{
	cout << "=====...Printing Cone List...=====" << endl << endl;
	for (unsigned int i = 0; i < cone_list.size(); i++)
	{
		cout << "Cone " << i+1 << ": " << endl;
		cout << "Distance: " << setprecision(3) << cone_list[i].distance << endl;
		cout << "Angle: " << setprecision(3) << cone_list[i].angle << endl;
		cout << endl << endl;
	}
}

void ConeLocation::PrintConeStage()
{
	cout << "============== Printing Cone by Section Stage ==================" << endl;	

	cout << "...Left..." << endl;
	for (unsigned int i = 0; i < left.size(); i++)
	{
		cout << "Cone " << i+1 << ": " << "Distance: " << setprecision(3) << left[i].distance << " Angle: " << setprecision(3) 
                << left[i].angle << endl;
	} 
	cout << endl;

	cout << "...Middle Left..." << endl;
	for (unsigned int i = 0; i < middle_left.size(); i++)
	{
		cout << "Cone " << i+1 << ": " << "Distance: " << setprecision(3) << middle_left[i].distance << " Angle: " 
		<< setprecision(3) << middle_left[i].angle << endl;
	}
	cout << endl;

	cout << "...Middle_Right..." << endl;
	for (unsigned int i = 0; i < middle_right.size(); i++)
	{
		cout << "Cone " << i+1 << ": " << "Distance: " << setprecision(3) << middle_right[i].distance <<  " Angle: " 
                << setprecision(3) << middle_right[i].angle << endl;

	}
	cout << endl;

	cout << "...Right..." << endl;
	for (unsigned int i = 0; i < right.size(); i++)
	{
		cout << "Cone " << i+1 << ": " << "Distance: " << setprecision(3) << right[i].distance <<  " Angle: " << setprecision(3) 
		<< right[i].angle << endl;

	}
	cout << endl;
}

void ConeLocation::PrintCone()
{
	cout << "============== Printing Cone by Section ==================" << endl;	

	cout << "...Left..." << endl;
	for (unsigned int i = 0; i < right.size(); i++)
	{
		cout << "Cone " << i+1 << ": " << "Distance: " << setprecision(3) << right[i].distance << " Angle: " << setprecision(3) 
                << -right[i].angle << endl;
	} 
	cout << endl;

	cout << "...Middle Left..." << endl;
	for (unsigned int i = 0; i < middle_right.size(); i++)
	{
		cout << "Cone " << i+1 << ": " << "Distance: " << setprecision(3) << middle_right[i].distance << " Angle: " 
		<< setprecision(3) << -middle_right[i].angle << endl;
	}
	cout << endl;

	cout << "...Middle_Right..." << endl;
	for (unsigned int i = 0; i < middle_left.size(); i++)
	{
		cout << "Cone " << i+1 << ": " << "Distance: " << setprecision(3) << middle_left[i].distance <<  " Angle: " 
                << setprecision(3) << -middle_left[i].angle << endl;

	}
	cout << endl;

	cout << "...Right..." << endl;
	for (unsigned int i = 0; i < left.size(); i++)
	{
		cout << "Cone " << i+1 << ": " << "Distance: " << setprecision(3) << left[i].distance <<  " Angle: " << setprecision(3) 
		<< -left[i].angle << endl;

	}
	cout << endl;
}
