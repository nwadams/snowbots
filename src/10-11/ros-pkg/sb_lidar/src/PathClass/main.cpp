#include <iostream>
#include <math.h>
#include "Path.h"
#include "ConePosition.h"
using namespace std;

//test program to see if Path really works
int main (int argc, char *argv[])
{
	ConePosition leftcones;
	ConePosition rightcones;
	
	/*
	//Uncomment this block to test that Path navigates a left turn of inner radius innerR, outer radius outerR.
	double coneSpacing = 0.5;
	double innerR = 1;
	double outerR = 2;
	
	double pi = 3.14159;
	double innerDeltaTheta = coneSpacing/innerR;
	double outerDeltaTheta = coneSpacing/outerR;
	double centerOfTurn = -innerR - (outerR-innerR)/2.0;  
	
	for ( int i = 0; i < 10; i++ )
	{
		leftcones.x[i] = innerR * cos( i*innerDeltaTheta) + centerOfTurn;
		leftcones.y[i] = innerR * sin( i*innerDeltaTheta);
		rightcones.x[i] = outerR * cos( i*outerDeltaTheta) + centerOfTurn;
		rightcones.y[i] = outerR * sin( i*outerDeltaTheta);
	}
	*/
	

	/*
	//Uncomment this block to test that Path navigates a straight path correctly.
		for ( int i = 0; i < 10; i++)
		{
			leftcones.x[i] = -1;
			leftcones.y[i] = 0.5*i;
			rightcones.x[i] = 1;
			rightcones.y[i] = 0.5*i;
		}
		*/
	
	cout << "Data in leftcones data is:\n" << "leftcones.x[i] \t leftcones.y[i] \n";
	for ( int i = 0; i < 10; i++ )
	{
		cout << leftcones.x[i] << "\t \t \t " << leftcones.y[i] << '\n';
	}
	
	cout << '\n';
	
	cout << "Data in rightcones data is:\n" << "rightcones.x[i] \t rightcones.y[i] \n";
	for ( int i = 0; i < 10; i++ )
	{
		cout << rightcones.x[i] << "\t \t \t \t " << rightcones.y[i] << '\n';
	}
	
	cout << '\n';
	Path testpath(leftcones, rightcones, 10);
	testpath.printInfo();
}
